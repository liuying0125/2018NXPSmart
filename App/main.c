/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外初学论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       main.c
 * @brief      山外K60 平台主程序
 * @author     山外科技
 * @version    v5.0
 * @date       2013-08-28
 */

#include "common.h"
#include "include.h"
#define ImageOdd   1
#define ImageEven  2
#define ImageWait  0
Site_t site     = {127, 64};                           //显示图像左上角位置
Size_t imgsize  = {CAMERA_W, CAMERA_H};             //图像大小

 
int8 ImageStauts = ImageWait;        //ImageStauts为 0
int8 ImageLastStatus = ImageEven;    //ImageLastStatus为 2 
double speedSet=4;




#define USE_SCREEN              TFT             //选择使用的 LCD

#define BackGroundColor         RED             //背景色          可选颜色 RED GREEN BLUE PURPLE YELLOW CYAN ORANGE BLACK WHITE
#define WordColor               BLACK           //字体色                   红  绿    蓝   紫     黄     青   橙    黑    白






//函数声明
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
  
extern int Time_Slice=0;   //用以PIT时间片计时
void Pit1_IRQHander();
 char buffer[40];
 
 int DIPswitch = 0;
//byte buffer[40];
 
int TFTorOLEDFresh = 0;
 
 /*
 * 定义屏幕型号
 */
#define OLED    1
#define TFT     2 

#define USE_SCREEN              TFT             //选择使用的 LCD

#define BackGroundColor         RED             //背景色          可选颜色 RED GREEN BLUE PURPLE YELLOW CYAN ORANGE BLACK WHITE
#define WordColor               BLACK           //字体色                   红   绿    蓝    紫     黄    青    橙    黑     白
 

void pulse_BZ(int count, int TH,int TL)
{   int i;  
    for(i=0;i<count;i++) 
    { 
      Smart_Car.Buzzer.setOn();  
      DELAY_MS(TH); //延时TH个0.5ms 
      Smart_Car.Buzzer.setOff();   
      DELAY_MS(TL); //延时TL个0.5ms 
    } 
}


/*
 *  @brief     按键修改程序函数
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

 void camera111_init()    //因为移植摄像头的时候出现偏差，后来没时间修改了
  {
    
       gpio_init (PTD15, GPO,0);  
    BuzzerOn();
   
    DELAY_MS(5);
    BuzzerOff();
   
    uart_init(UART2,115200);
    uart_init(UART0,115200);
  //  set_vector_handler(UART2_RX_TX_VECTORn,Uart0_IRQHandler);   // 设置中断复位函数到中断向量表里

    uart_rx_irq_en (UART2);                                 //开串口接收中断
   uart_putstr(UART2,"hello");
   
   
     uart_rx_irq_en (UART0);                                 //开串口接收中断
     uart_putstr(UART0,"hello");
   
   
   
     pll_init(CORE_CLK);                   //超频200MHZ 
     camera_init(Image);                   //鹰眼初始化
     set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //设置LPTMR的中断复位函数为 PORTA_IRQHandler
     set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);     //设置LPTMR的中断复位函数为 PORTA_IRQHandler

     ftm_pwm_init(FTM0,FTM_CH1,14000 ,0);       
     ftm_pwm_init(FTM0,FTM_CH2,14000 ,0);
     ftm_pwm_init(FTM0,FTM_CH3,14000 ,0);
     ftm_pwm_init(FTM0,FTM_CH4,14000 ,0);
     
  ftm_pwm_init(FTM3,FTM_CH3,50,1550);     //新库 可以使用3个ftm口
     
    //-----------电机占空比------------------------ 
     ftm_pwm_duty(FTM0,FTM_CH1,0); 
     ftm_pwm_duty(FTM0,FTM_CH2,0);
     ftm_pwm_duty(FTM0,FTM_CH3,0);
     ftm_pwm_duty(FTM0,FTM_CH4,0);
     LCD_P6X8Str(0, 1, "FTM Init Success",WordColor,BackGroundColor);   //显示英文6*8字符串
    
    //-----------初始化FTM正交解码模块--------------	  
     ftm_quad_init(FTM1);                                    //FTM1 正交解码初始化（所用的管脚可查 vcan_port_cfg.h ）
     ftm_quad_init(FTM2);                                    //FTM2 正交解码初始化（所用的管脚可查 vcan_port_cfg.h ）
      
 
  /*FTM_PWM_Duty(FTM0, FTM_CH1,0 );
  FTM_PWM_Duty(FTM0, FTM_CH2, 5000);
  FTM_PWM_Duty(FTM0, FTM_CH3,0 );
  FTM_PWM_Duty(FTM0, FTM_CH4, 5000);*/
  
  DMA_Count_Init(DMA_CH3, PTC5,  0xffffffff,  DMA_rising_down_keepon);    //DMA累加计数初始化   
  DMA_Count_Init(DMA_CH4, PTB11, 0xffffffff,  DMA_rising_down_keepon);   //DMA_rising_down_keepon上升沿触发，源地址IO端口内部下拉，目的地址保持不变
  
 // FTM_QUAD_Init(FTM2);                     //FTM2正交解码初始化

    
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
          
          
          
                        if( Car_Meet.k60_shibie){ // 在第一次识别会车区后禁止摄像头识别 60000秒冲
                                 
                                  Img_Pro();   //图像处理
                                   
                                  if(ef>=2)
                                    {
                                           ef=0;
                                           BuzzerOn(); 
                                            
                                           Car_Meet.K60_coming_meet = 1;   //进入会车区标志位
                                         
                                           Car_Meet.k60_shibie=0;
                                    }
                                   else
                                      
                                           BuzzerOff();
                        }
            
             camera_get_img();
           
       }
       
       
    //------------------------------   波形助手蓝牙   -----------------------
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
 *  @brief      main函数
 *  @since       
 *  @note       27号预赛代码     7月27日
 */
    void  main(void)      /////main函数
{
  
              DisableInterrupts;   //关闭总中断    
               /************************ 配置 K60 的优先级  ***********************/
              //K60 的默认优先级 都为 0
              //参考帖子：急求中断嵌套的例程 - 智能车讨论区 - 野火初学123论坛
              //http://www.vcan123.com/thread-499-1-1.html
              NVIC_SetPriorityGrouping(4);            //设置优先级分组,4bit 抢占优先级,没有亚优先级

              //prio越低，则优先级越高 （4bit）    
             // NVIC_SetPriority(PORTA_IRQn, 1);         //配置优先级  场中断优先级最高
              //NVIC_SetPriority(DMA0_IRQn, 0);          //配置优先级  摄像头数据采集完成DMA中断
                 
              NVIC_SetPriority(PIT0_IRQn, 4);          //配置优先级  直立优先级
              NVIC_SetPriority(UART2_RX_TX_IRQn, 0);    //配置优先级 
              NVIC_SetPriority(UART0_RX_TX_IRQn, 2); 
              
              NVIC_SetPriority(PORTA_IRQn, 1);         //配置优先级  场中断优先级最高
              NVIC_SetPriority(DMA0_IRQn, 0); 
                
                //-------------初始化控制参数----------------  
               InitControlPara();
              
               //-------------初始化各个模块----------------- 
               InitModule();
               EnableInterrupts; //使能总中断
               Smart_Car.Buzzer.setOn();
               DELAY_MS(20);
               Smart_Car.Buzzer.setOff();
               
               DELAY_MS(400);
             
               TFT_LCD_CLS();
               gpio_init (PTD15, GPO,0); 
	//-------------获取图像-------------------   
               camera111_init();
    
  while(1)
  {   
    camera_img_pro(); 
    if(Car_Meet.fasong == 1){
         if( Car_Meet.distance < 800 ){   //超声波读到的距离小于800时候
                               Car_Meet.Receive ++ ;
                               // 防误判  软件增加判别条件
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
                            
                                                if(Car_Meet.distance > 6000){  // 如果会车之后k60的距离读到的大于5000有 
                                                
                                                  Car_Meet.k60_ready_flag ++;
                                                  
                                                }
                            
                            }  
                            if(Car_Meet.k60_ready_flag == 450){    // 有450次的话 k60开始走了  清掉所有标志位
                                                
                                                           //   BuzzerOff();
                               SmartCarStartRun();
                                                              
                                                              
                               Car_Meet.K60_coming_meet = 0; //不在会车区了
                                                                 
                               Car_Meet.position_flag = 0;
                               Car_Meet.K60_coming_meet = 0;  
                               Car_Control.Car_A_Stop = 1 ; 
                               Car_Meet.k60_coming = 0;   
                               Car_Meet.fasong = 0;  // 此标志位让nrf不再发送  
                               Car_Meet.K60_coming_meet = 0;
                                                              
                                   
                                  //  会车区为了清掉标志位的标志位
                              Car_Meet.kea_coming_flag = 0;
                              Car_Meet.kea_coming_flag_qingdiao = 0;
                              Car_Meet.k60_ready_flag = 0;
                                                           
                                                             
                                                //Car_Meet.huicheqv_flag = 0;
                                              }
            
         }
  
  }
}


/*!
 *  @brief      PORTA中断服务函数
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
                uint8  n;    //引脚号
                uint32 flag;

              
                flag = PORTA_ISFR;
                PORTA_ISFR  = ~0;                                   //清中断标志位

                n = 29;                                             //场中断
                if(flag & (1 << n))                                 //PTA29触发中断
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
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}
//发送图像到上位机显示
//不同的上位机，不同的命令，这里使用 yy_摄像头串口调试 软件
//如果使用其他上位机，则需要修改代码
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
  uart_putchar(uartn, 0x0A);  ////最后一位校验
}*/

void  InitModule(){
    
   //--------------初始化OLED模块-------------- 
    TFT_LCD_init(); 
  
    servo_init();
   //---------------初始化蓝牙模块---------------- 
    uart_init(UART2,115200);  
    
    uart_init(UART0,115200); 
    
    set_vector_handler(UART2_RX_TX_VECTORn,UART2_RX_IRQHandler);   // 设置中断复位函数到中断向量表里
    LCD_P6X8Str(0,0,"Uart2 Init Success",WordColor,BackGroundColor);   //显示英文6*8字符串
    uart_rx_irq_en(UART2); 
      
    set_vector_handler(UART0_RX_TX_VECTORn,UART0_RX_TX_IRQHandler);   // 设置中断复位函数到中断向量表里
    LCD_P6X8Str(0,0,"Uart0 Init Success",WordColor,BackGroundColor);   //显示英文6*8字符串
    uart_rx_irq_en(UART0); 
      
       
       //--------------初始化按键---------------------     
                          //按键在板子上的位置
                    //           (*)          PTE9
                    //           (*)          PTE10
                    //           (*)          PTE11
                    //           (*)          PTE12
        key_init(KEY_U);    //PTE9
        key_init(KEY_D);    //PTE11
        key_init(KEY_L);    
        key_init(KEY_R);  
        
    
      
    
    
     //-------------初始化拨码开关模块----------------
        gpio_init(PTC6,GPI,0);
        port_init_NoALT(PTC6, PULLUP); //配置上拉，一定要加，不加拨码开关读数不准确
        gpio_init(PTC7,GPI,0);
        port_init_NoALT(PTC7, PULLUP); //配置上拉，一定要加，不加拨码开关读数不准确
        gpio_init(PTC8,GPI,0);
        port_init_NoALT(PTC8, PULLUP); //配置上拉，一定要加，不加拨码开关读数不准确
        gpio_init(PTC9,GPI,0);  
        port_init_NoALT(PTC9, PULLUP); //配置上拉，一定要加，不加拨码开关读数不准确 
                
    //    DIPswitch |= gpio_get(PTC9) ? 0x01 : 0;
	DIPswitch |= gpio_get(PTC8) ? 0x01 : 0;
	DIPswitch |= gpio_get(PTC7) ? 0x02 : 0;
	DIPswitch |= gpio_get(PTC6) ? 0x04 : 0;
    
        Smart_Car.DIPswitch = DIPswitch;
     
    
    
    
      //-----------电机  舵机  初始化----------------- 
       ftm_pwm_init(FTM0,FTM_CH1,14000 ,0);       
       ftm_pwm_init(FTM0,FTM_CH2,14000 ,0);
       ftm_pwm_init(FTM0,FTM_CH3,14000 ,0);
       ftm_pwm_init(FTM0,FTM_CH4,14000 ,0);
       
       ftm_pwm_init(FTM3,FTM_CH3,50,1550);     //新库 可以使用3个ftm口
       
      //-----------电机占空比------------------------ 
       ftm_pwm_duty(FTM0,FTM_CH1,0); 
       ftm_pwm_duty(FTM0,FTM_CH2,0);
       ftm_pwm_duty(FTM0,FTM_CH3,0);
       ftm_pwm_duty(FTM0,FTM_CH4,0);
       LCD_P6X8Str(0, 1, "FTM Init Success",WordColor,BackGroundColor);   //显示英文6*8字符串
      
      //-----------初始化FTM正交解码模块--------------	  
       ftm_quad_init(FTM1);                                    //FTM1 正交解码初始化（所用的管脚可查 vcan_port_cfg.h ）
       ftm_quad_init(FTM2);                                    //FTM2 正交解码初始化（所用的管脚可查 vcan_port_cfg.h ）
      
       LCD_P6X8Str(0, 1, "FTM Init Success",WordColor,BackGroundColor);   //显示英文6*8字符串
      
      //------------ADC采集口初始化------------------- 
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
       
      
       //------------初始化蜂鸣器模块----------------
        Smart_Car.Buzzer.BuzzerPin = PTD15;
        gpio_init (Smart_Car.Buzzer.BuzzerPin, GPO,0); 
        Smart_Car.Buzzer.setOn = BuzzerOn;
        Smart_Car.Buzzer.setOff = BuzzerOff;
        Smart_Car.Buzzer.setOn();
        DELAY_MS(5);//蜂鸣器测试
        Smart_Car.Buzzer.setOff();  
          
        Smart_Car.CarStartRun = SmartCarStartRun;
        Smart_Car.CarStop = SmartCarStop;                      
   
      //--------------电感采集最大最小值-------------- 
   
        LCD_P6X8Str(0, 3, "Collect is Working",WordColor,BackGroundColor);
        Collect_Init();//采集
        LCD_P6X8Str(0, 4, "Collect is Success",WordColor,BackGroundColor);
    
        //--------------------nrf2401通信初始化--------------
        
        NRF_Dev_Init();
       
        
      //----------------初始化PIT模块------------------- //控制中断   
        pit_init_ms(PIT0, 1);                                //初始化PIT0，定时时间为： 1ms
        set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //设置PIT0的中断服务函数为 PIT0_IRQHandler
        enable_irq(PIT0_IRQn);                                 //使能PIT0中断
     
      /*  pit_init_ms(PIT1, 30);
        set_vector_handler(PIT1_VECTORn, Pit1_IRQHander);   // 设置PIT0中断复位函数到中断向量表里   
        LCD_P6X8Str(0, 5, "PIT0 Init Success",WordColor,BackGroundColor);   //显示英文6*8字符串
      */
         
      
     } 

void PIT0_IRQHandler(void)
  { 
      
      static uint8  g_n2MSEventCount;
      static uint16 g_n8MSEventCount;
      int i;
     
      g_n2MSEventCount++;//1毫秒加一次数
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
   
   //-----------------按键修改----------------
    key_out = KEY_Scan();
  if(key_out == 1){
     Car_Meet.kaishihuiche = 1;   //发车按键
     Car_Control.Car_A_Stop = 1 ; 
  }
  
  if(key_out == 2){ 
    
 //   Car_Speed.DIPspeed = 1;
    
    
    Car_Control.Car_A_Stop = 1 ; 
    Car_Meet.K60_coming_meet = 0;
    Car_Meet.fasong  = 0;
    
  }
  
    
  
   PIT_Flag_Clear(PIT0);       //清中断标志位
    
 }


/*!
*  @brief      开车停车函数
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
 //  @brief         UART3中断执行函数
 //  @return        void   
 //  @since         v1.0     来自逐飞科技   逐飞nrf底层库 有来有去真好用
 //  Sample usage:  当UART3启用中断功能且发生中断的时候会自动执行该函数
 */ 
  uint16 distance;
  uint8 dat[3];
  uint8 num;

  void UART0_RX_TX_IRQHandler(void)
  {
      uart_getchar(UART0,&dat[num]);
          if(dat[0] != 0xa5)num = 0;	//检查头帧是否正确，不正确就重新接收
          else num++;
          
          if(num==3)					//接收完成，开始处理数据
          {
              num = 0;
              Car_Meet.distance = dat[1]<<8 | dat[2];
          }
          
  }
 
  
  
  
