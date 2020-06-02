/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ���ѧ��̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       main.c
 * @brief      ɽ��K60 ƽ̨������
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-08-28
 */

#include "common.h"
#include "include.h"
#define ImageOdd   1
#define ImageEven  2
#define ImageWait  0
Site_t site     = {127, 64};                           //��ʾͼ�����Ͻ�λ��
Size_t imgsize  = {CAMERA_W, CAMERA_H};             //ͼ���С

 
int8 ImageStauts = ImageWait;        //ImageStautsΪ 0
int8 ImageLastStatus = ImageEven;    //ImageLastStatusΪ 2 
double speedSet=4;




#define USE_SCREEN              TFT             //ѡ��ʹ�õ� LCD

#define BackGroundColor         RED             //����ɫ          ��ѡ��ɫ RED GREEN BLUE PURPLE YELLOW CYAN ORANGE BLACK WHITE
#define WordColor               BLACK           //����ɫ                   ��  ��    ��   ��     ��     ��   ��    ��    ��






//��������
void PIT0_IRQHandler();  
void PORTA_IRQHandler();
void DMA0_IRQHandler();
void sendimg(uint8 *imgaddr, uint32 imgsize);
 void UART2_RX_IRQHandler(); 
  void UART0_RX_TX_IRQHandler();

  void InitModule();
   void InitControlPara();
   void BuzzerOn();
   void BuzzerOff();
   extern void SmartCarStartRun();
   extern void SmartCarStop();
  
extern int Time_Slice=0;   //����PITʱ��Ƭ��ʱ
void Pit1_IRQHander();
 char buffer[40];
 
 int DIPswitch = 0;
//byte buffer[40];
 
int TFTorOLEDFresh = 0;
 
 /*
 * ������Ļ�ͺ�
 */
#define OLED    1
#define TFT     2 

#define USE_SCREEN              TFT             //ѡ��ʹ�õ� LCD

#define BackGroundColor         RED             //����ɫ          ��ѡ��ɫ RED GREEN BLUE PURPLE YELLOW CYAN ORANGE BLACK WHITE
#define WordColor               BLACK           //����ɫ                   ��   ��    ��    ��     ��    ��    ��    ��     ��
 

void pulse_BZ(int count, int TH,int TL)
{   int i;  
    for(i=0;i<count;i++) 
    { 
      Smart_Car.Buzzer.setOn();  
      DELAY_MS(TH); //��ʱTH��0.5ms 
      Smart_Car.Buzzer.setOff();   
      DELAY_MS(TL); //��ʱTL��0.5ms 
    } 
}


/*
 *  @brief     �����޸ĳ�����
 *  @since      v1.0
 *  @note      
*/
int mode=0,key_out = 0;

int KEY_Scan(void)
{	 
	static uint8_t key_up=1;
	
        if( key_check(KEY_U) == KEY_DOWN ){
          while(key_check(KEY_U) == KEY_DOWN){
          Smart_Car.Buzzer.setOn(); }
          
               Smart_Car.Buzzer.setOff();
         return 1;
        }
           
        else if(key_check(KEY_D) == KEY_DOWN){
          while(key_check(KEY_D) == KEY_DOWN){
           Smart_Car.Buzzer.setOn();}
           Smart_Car.Buzzer.setOff();
          return 2;
        } 
        
         else if(key_check(KEY_L) == KEY_DOWN){
          while(key_check(KEY_L) == KEY_DOWN){
           Smart_Car.Buzzer.setOn();}
           Smart_Car.Buzzer.setOff();
          return 3;
        } 
        
         else if(key_check(KEY_R) == KEY_DOWN){
          while(key_check(KEY_R) == KEY_DOWN){
           Smart_Car.Buzzer.setOn();}
           Smart_Car.Buzzer.setOff();
          return 4;
        } 
         
	  key_up=0;
            
 	return 0;
        
}






void BuzzerOn()
{
	//GPIO_PDOR_REG(GPIOX_BASE(PTD15)) |= (1 << PTn(PTD15));
             gpio_set(PTD15,1);
}

void BuzzerOff()
{
	//GPIO_PDOR_REG(GPIOX_BASE(PTD15)) &= ~(1 << PTn(PTD15));
   gpio_set(PTD15,0);
}

 void camera111_init()    //��Ϊ��ֲ����ͷ��ʱ�����ƫ�����ûʱ���޸���
  {
    
       gpio_init (PTD15, GPO,0);  
    BuzzerOn();
   
    DELAY_MS(5);
    BuzzerOff();
   
    uart_init(UART2,115200);
    uart_init(UART0,115200);
  //  set_vector_handler(UART2_RX_TX_VECTORn,Uart0_IRQHandler);   // �����жϸ�λ�������ж���������

    uart_rx_irq_en (UART2);                                 //�����ڽ����ж�
   uart_putstr(UART2,"hello");
   
   
     uart_rx_irq_en (UART0);                                 //�����ڽ����ж�
     uart_putstr(UART0,"hello");
   
   
   
     pll_init(CORE_CLK);                   //��Ƶ200MHZ 
     camera_init(Image);                   //ӥ�۳�ʼ��
     set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //����LPTMR���жϸ�λ����Ϊ PORTA_IRQHandler
     set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);     //����LPTMR���жϸ�λ����Ϊ PORTA_IRQHandler

     ftm_pwm_init(FTM0,FTM_CH1,14000 ,0);       
     ftm_pwm_init(FTM0,FTM_CH2,14000 ,0);
     ftm_pwm_init(FTM0,FTM_CH3,14000 ,0);
     ftm_pwm_init(FTM0,FTM_CH4,14000 ,0);
     
  ftm_pwm_init(FTM3,FTM_CH3,50,1550);     //�¿� ����ʹ��3��ftm��
     
    //-----------���ռ�ձ�------------------------ 
     ftm_pwm_duty(FTM0,FTM_CH1,0); 
     ftm_pwm_duty(FTM0,FTM_CH2,0);
     ftm_pwm_duty(FTM0,FTM_CH3,0);
     ftm_pwm_duty(FTM0,FTM_CH4,0);
     LCD_P6X8Str(0, 1, "FTM Init Success",WordColor,BackGroundColor);   //��ʾӢ��6*8�ַ���
    
    //-----------��ʼ��FTM��������ģ��--------------	  
     ftm_quad_init(FTM1);                                    //FTM1 ���������ʼ�������õĹܽſɲ� vcan_port_cfg.h ��
     ftm_quad_init(FTM2);                                    //FTM2 ���������ʼ�������õĹܽſɲ� vcan_port_cfg.h ��
      
 
  /*FTM_PWM_Duty(FTM0, FTM_CH1,0 );
  FTM_PWM_Duty(FTM0, FTM_CH2, 5000);
  FTM_PWM_Duty(FTM0, FTM_CH3,0 );
  FTM_PWM_Duty(FTM0, FTM_CH4, 5000);*/
  
  DMA_Count_Init(DMA_CH3, PTC5,  0xffffffff,  DMA_rising_down_keepon);    //DMA�ۼӼ�����ʼ��   
  DMA_Count_Init(DMA_CH4, PTB11, 0xffffffff,  DMA_rising_down_keepon);   //DMA_rising_down_keepon�����ش�����Դ��ַIO�˿��ڲ�������Ŀ�ĵ�ַ���ֲ���
  
 // FTM_QUAD_Init(FTM2);                     //FTM2���������ʼ��

    
    TFT_LCD_init();
    TFT_LCD_CLS();
   // LCD_P6X8Str(0,0,"LTP",CYAN,BLACK);
    
   camera_get_img();  
  }

 void camera_img_pro()
  {
    
       if(ImageStauts==ImageOdd)
      {
          ImageStauts=ImageWait;
          camera_vsync();          
          if( DIPswitch == 15)
           Send_Img();   
              
          camera_get_img();
           
      }
     
      if(ImageStauts==ImageEven)
      {                
          ImageStauts=ImageWait;
          
          
          
                        if( Car_Meet.k60_shibie){ // �ڵ�һ��ʶ��ᳵ�����ֹ����ͷʶ�� 60000���
                                 
                                  Img_Pro();   //ͼ����
                                   
                                  if(ef>=2)
                                    {
                                           ef=0;
                                           BuzzerOn(); 
                                            
                                           Car_Meet.K60_coming_meet = 1;   //����ᳵ����־λ
                                         
                                           Car_Meet.k60_shibie=0;
                                    }
                                   else
                                      
                                           BuzzerOff();
                        }
            
             camera_get_img();
           
       }
       
       
    //------------------------------   ������������   -----------------------
                                if(send_data_contorl == 1)
                                 {     
                                  Send_Begin();
                                  Variable_update();
                                  Send_Variable(16);
                                  }
                                 if(SendPara == 1)
                                 {
                                  Send_Begin();
                                  Send_Parameter();
                                  SendPara = 0;
                                  }
                                   
        
  
    
    
  }


/*!
 *  @brief      main����
 *  @since       
 *  @note       27��Ԥ������     7��27��
 */
    void  main(void)      /////main����
{
  
              DisableInterrupts;   //�ر����ж�    
               /************************ ���� K60 �����ȼ�  ***********************/
              //K60 ��Ĭ�����ȼ� ��Ϊ 0
              //�ο����ӣ������ж�Ƕ�׵����� - ���ܳ������� - Ұ���ѧ123��̳
              //http://www.vcan123.com/thread-499-1-1.html
              NVIC_SetPriorityGrouping(4);            //�������ȼ�����,4bit ��ռ���ȼ�,û�������ȼ�

              //prioԽ�ͣ������ȼ�Խ�� ��4bit��    
             // NVIC_SetPriority(PORTA_IRQn, 1);         //�������ȼ�  ���ж����ȼ����
              //NVIC_SetPriority(DMA0_IRQn, 0);          //�������ȼ�  ����ͷ���ݲɼ����DMA�ж�
                 
              NVIC_SetPriority(PIT0_IRQn, 4);          //�������ȼ�  ֱ�����ȼ�
              NVIC_SetPriority(UART2_RX_TX_IRQn, 0);    //�������ȼ� 
              NVIC_SetPriority(UART0_RX_TX_IRQn, 2); 
              
              NVIC_SetPriority(PORTA_IRQn, 1);         //�������ȼ�  ���ж����ȼ����
              NVIC_SetPriority(DMA0_IRQn, 0); 
                
                //-------------��ʼ�����Ʋ���----------------  
               InitControlPara();
              
               //-------------��ʼ������ģ��----------------- 
               InitModule();
               EnableInterrupts; //ʹ�����ж�
               Smart_Car.Buzzer.setOn();
               DELAY_MS(20);
               Smart_Car.Buzzer.setOff();
               
               DELAY_MS(400);
             
               TFT_LCD_CLS();
               gpio_init (PTD15, GPO,0); 
	//-------------��ȡͼ��-------------------   
               camera111_init();
    
  while(1)
  {   
    camera_img_pro(); 
    if(Car_Meet.fasong == 1){
         if( Car_Meet.distance < 800 ){   //�����������ľ���С��800ʱ��
                               Car_Meet.Receive ++ ;
                               // ������  ��������б�����
                               if(Car_Meet.Receive == 35){    
                               Car_Meet.Nrf_Flag = 1;
                               Car_Meet.Receive = 0;
                               }
                               NRF_TX_Buff[0] = 1;
                               NRF_TX_Buff[1] = 1;  
                               Car_Meet.kea_coming_flag++;  
                               
                       }
                        
          else{ 
                              Car_Meet.Receive_wupan++;
                              // Car_Meet.Nrf_Flag = 2;
                                  if(Car_Meet.Receive_wupan == 150){  
                                        Car_Meet.Receive = 0;
                                        Car_Meet.Receive_wupan = 0; 
                               } 
                               Car_Meet.Nrf_Flag = 0;
                               NRF_TX_Buff[0] = 1;
                               NRF_TX_Buff[1] = 0; 
               }   
                        
          
                        NRF_Send_Packet(NRF_TX_Buff);  
          if(Car_Meet.kea_coming_flag > 15){
             
            BuzzerOff();
           
            Car_Meet.kea_coming_flag_qingdiao = 1;
          
          }
                            if(Car_Meet.kea_coming_flag_qingdiao){
                            
                                                if(Car_Meet.distance > 6000){  // ����ᳵ֮��k60�ľ�������Ĵ���5000�� 
                                                
                                                  Car_Meet.k60_ready_flag ++;
                                                  
                                                }
                            
                            }  
                            if(Car_Meet.k60_ready_flag == 450){    // ��450�εĻ� k60��ʼ����  ������б�־λ
                                                
                                                           //   BuzzerOff();
                               SmartCarStartRun();
                                                              
                                                              
                               Car_Meet.K60_coming_meet = 0; //���ڻᳵ����
                                                                 
                               Car_Meet.position_flag = 0;
                               Car_Meet.K60_coming_meet = 0;  
                               Car_Control.Car_A_Stop = 1 ; 
                               Car_Meet.k60_coming = 0;   
                               Car_Meet.fasong = 0;  // �˱�־λ��nrf���ٷ���  
                               Car_Meet.K60_coming_meet = 0;
                                                              
                                   
                                  //  �ᳵ��Ϊ�������־λ�ı�־λ
                              Car_Meet.kea_coming_flag = 0;
                              Car_Meet.kea_coming_flag_qingdiao = 0;
                              Car_Meet.k60_ready_flag = 0;
                                                           
                                                             
                                                //Car_Meet.huicheqv_flag = 0;
                                              }
            
         }
  
  }
}


/*!
 *  @brief      PORTA�жϷ�����
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
                uint8  n;    //���ź�
                uint32 flag;

              
                flag = PORTA_ISFR;
                PORTA_ISFR  = ~0;                                   //���жϱ�־λ

                n = 29;                                             //���ж�
                if(flag & (1 << n))                                 //PTA29�����ж�
                {
                  chang++;
                  if(chang>50)
                  {
                    chang=0;
                  }
                  
                  if(ImageStauts==ImageWait) 
                  {
                            if(ImageLastStatus==ImageEven)
                              {                      
                                  ImageStauts = ImageOdd;
                                  ImageLastStatus = ImageOdd;
                               }
                            
                            
                    else if(ImageLastStatus==ImageOdd)
                    {
                        ImageStauts = ImageEven;
                        ImageLastStatus = ImageEven;
                    }
                    
                    
                  }
                }
}

/*!
 *  @brief      DMA0�жϷ�����
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}
//����ͼ����λ����ʾ
//��ͬ����λ������ͬ���������ʹ�� yy_����ͷ���ڵ��� ���
//���ʹ��������λ��������Ҫ�޸Ĵ���
/*void SendPicData(UARTn uartn, uint8 * data, uint16 size)
{
  uint8 psize[2] = {0};
  if(size >= 256)
    psize[0] = size / 256;
  psize[1] = size - 256 * (uint16)psize[0];
  
  uart_putbuff(uartn, SendHead, 4);
  uart_putchar(uartn, 250);
  uart_putbuff(uartn, psize, sizeof(psize));
  uart_putbuff(uartn, data, size);
  uart_putchar(uartn, 0x0A);  ////���һλУ��
}*/

void  InitModule(){
    
   //--------------��ʼ��OLEDģ��-------------- 
    TFT_LCD_init(); 
  
    servo_init();
   //---------------��ʼ������ģ��---------------- 
    uart_init(UART2,115200);  
    
    uart_init(UART0,115200); 
    
    set_vector_handler(UART2_RX_TX_VECTORn,UART2_RX_IRQHandler);   // �����жϸ�λ�������ж���������
    LCD_P6X8Str(0,0,"Uart2 Init Success",WordColor,BackGroundColor);   //��ʾӢ��6*8�ַ���
    uart_rx_irq_en(UART2); 
      
    set_vector_handler(UART0_RX_TX_VECTORn,UART0_RX_TX_IRQHandler);   // �����жϸ�λ�������ж���������
    LCD_P6X8Str(0,0,"Uart0 Init Success",WordColor,BackGroundColor);   //��ʾӢ��6*8�ַ���
    uart_rx_irq_en(UART0); 
      
       
       //--------------��ʼ������---------------------     
                          //�����ڰ����ϵ�λ��
                    //           (*)          PTE9
                    //           (*)          PTE10
                    //           (*)          PTE11
                    //           (*)          PTE12
        key_init(KEY_U);    //PTE9
        key_init(KEY_D);    //PTE11
        key_init(KEY_L);    
        key_init(KEY_R);  
        
    
      
    
    
     //-------------��ʼ�����뿪��ģ��----------------
        gpio_init(PTC6,GPI,0);
        port_init_NoALT(PTC6, PULLUP); //����������һ��Ҫ�ӣ����Ӳ��뿪�ض�����׼ȷ
        gpio_init(PTC7,GPI,0);
        port_init_NoALT(PTC7, PULLUP); //����������һ��Ҫ�ӣ����Ӳ��뿪�ض�����׼ȷ
        gpio_init(PTC8,GPI,0);
        port_init_NoALT(PTC8, PULLUP); //����������һ��Ҫ�ӣ����Ӳ��뿪�ض�����׼ȷ
        gpio_init(PTC9,GPI,0);  
        port_init_NoALT(PTC9, PULLUP); //����������һ��Ҫ�ӣ����Ӳ��뿪�ض�����׼ȷ 
                
    //    DIPswitch |= gpio_get(PTC9) ? 0x01 : 0;
	DIPswitch |= gpio_get(PTC8) ? 0x01 : 0;
	DIPswitch |= gpio_get(PTC7) ? 0x02 : 0;
	DIPswitch |= gpio_get(PTC6) ? 0x04 : 0;
    
        Smart_Car.DIPswitch = DIPswitch;
     
    
    
    
      //-----------���  ���  ��ʼ��----------------- 
       ftm_pwm_init(FTM0,FTM_CH1,14000 ,0);       
       ftm_pwm_init(FTM0,FTM_CH2,14000 ,0);
       ftm_pwm_init(FTM0,FTM_CH3,14000 ,0);
       ftm_pwm_init(FTM0,FTM_CH4,14000 ,0);
       
       ftm_pwm_init(FTM3,FTM_CH3,50,1550);     //�¿� ����ʹ��3��ftm��
       
      //-----------���ռ�ձ�------------------------ 
       ftm_pwm_duty(FTM0,FTM_CH1,0); 
       ftm_pwm_duty(FTM0,FTM_CH2,0);
       ftm_pwm_duty(FTM0,FTM_CH3,0);
       ftm_pwm_duty(FTM0,FTM_CH4,0);
       LCD_P6X8Str(0, 1, "FTM Init Success",WordColor,BackGroundColor);   //��ʾӢ��6*8�ַ���
      
      //-----------��ʼ��FTM��������ģ��--------------	  
       ftm_quad_init(FTM1);                                    //FTM1 ���������ʼ�������õĹܽſɲ� vcan_port_cfg.h ��
       ftm_quad_init(FTM2);                                    //FTM2 ���������ʼ�������õĹܽſɲ� vcan_port_cfg.h ��
      
       LCD_P6X8Str(0, 1, "FTM Init Success",WordColor,BackGroundColor);   //��ʾӢ��6*8�ַ���
      
      //------------ADC�ɼ��ڳ�ʼ��------------------- 
        adc_init(AD0 );
        LCD_P6X8Str(0, 2, "AD0 Init Success",WordColor,BackGroundColor);
        adc_init(AD1 );  
        LCD_P6X8Str(0, 2, "AD1 Init Success",WordColor,BackGroundColor);
        adc_init(AD2 );  
        LCD_P6X8Str(0, 2, "AD2 Init Success",WordColor,BackGroundColor);
        adc_init(AD3 );
        LCD_P6X8Str(0, 2, "AD3 Init Success",WordColor,BackGroundColor);
        adc_init(AD4 );
        LCD_P6X8Str(0, 2, "AD4 Init Success",WordColor,BackGroundColor);
       
      
       //------------��ʼ��������ģ��----------------
        Smart_Car.Buzzer.BuzzerPin = PTD15;
        gpio_init (Smart_Car.Buzzer.BuzzerPin, GPO,0); 
        Smart_Car.Buzzer.setOn = BuzzerOn;
        Smart_Car.Buzzer.setOff = BuzzerOff;
        Smart_Car.Buzzer.setOn();
        DELAY_MS(5);//����������
        Smart_Car.Buzzer.setOff();  
          
        Smart_Car.CarStartRun = SmartCarStartRun;
        Smart_Car.CarStop = SmartCarStop;                      
   
      //--------------��вɼ������Сֵ-------------- 
   
        LCD_P6X8Str(0, 3, "Collect is Working",WordColor,BackGroundColor);
        Collect_Init();//�ɼ�
        LCD_P6X8Str(0, 4, "Collect is Success",WordColor,BackGroundColor);
    
        //--------------------nrf2401ͨ�ų�ʼ��--------------
        
        NRF_Dev_Init();
       
        
      //----------------��ʼ��PITģ��------------------- //�����ж�   
        pit_init_ms(PIT0, 1);                                //��ʼ��PIT0����ʱʱ��Ϊ�� 1ms
        set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //����PIT0���жϷ�����Ϊ PIT0_IRQHandler
        enable_irq(PIT0_IRQn);                                 //ʹ��PIT0�ж�
     
      /*  pit_init_ms(PIT1, 30);
        set_vector_handler(PIT1_VECTORn, Pit1_IRQHander);   // ����PIT0�жϸ�λ�������ж���������   
        LCD_P6X8Str(0, 5, "PIT0 Init Success",WordColor,BackGroundColor);   //��ʾӢ��6*8�ַ���
      */
         
      
     } 

void PIT0_IRQHandler(void)
  { 
      
      static uint8  g_n2MSEventCount;
      static uint16 g_n8MSEventCount;
      int i;
     
      g_n2MSEventCount++;//1�����һ����
      g_n8MSEventCount++;
    
       GetSpeed();
  
  switch (g_n2MSEventCount)
  {
     
  case 1:                        
           Date_analyse();
           break;  
  case 2: 
          DirectionControl();
          SpeedControl(); 
          break;
  default:
          g_n2MSEventCount = 0;
     break;
  }
   
   //-----------------�����޸�----------------
    key_out = KEY_Scan();
  if(key_out == 1){
     Car_Meet.kaishihuiche = 1;   //��������
     Car_Control.Car_A_Stop = 1 ; 
  }
  
  if(key_out == 2){ 
    
 //   Car_Speed.DIPspeed = 1;
    
    
    Car_Control.Car_A_Stop = 1 ; 
    Car_Meet.K60_coming_meet = 0;
    Car_Meet.fasong  = 0;
    
  }
  
    
  
   PIT_Flag_Clear(PIT0);       //���жϱ�־λ
    
 }


/*!
*  @brief      ����ͣ������
*  @since      v1.0
*  @note
*/
void SmartCarStartRun()
{
      
     Smart_Car.Status = Run;
     
}

void SmartCarStop()
{
     ftm_pwm_duty(FTM3,FTM_CH3,Car_Control.baseServoPwm); 
     Smart_Car.Status = Stop;
   
} 




/*
 //  @brief         UART3�ж�ִ�к���
 //  @return        void   
 //  @since         v1.0     ������ɿƼ�   ���nrf�ײ�� ������ȥ�����
 //  Sample usage:  ��UART3�����жϹ����ҷ����жϵ�ʱ����Զ�ִ�иú���
 */ 
  uint16 distance;
  uint8 dat[3];
  uint8 num;

  void UART0_RX_TX_IRQHandler(void)
  {
      uart_getchar(UART0,&dat[num]);
          if(dat[0] != 0xa5)num = 0;	//���ͷ֡�Ƿ���ȷ������ȷ�����½���
          else num++;
          
          if(num==3)					//������ɣ���ʼ��������
          {
              num = 0;
              Car_Meet.distance = dat[1]<<8 | dat[2];
          }
          
  }
 
  
  
  
