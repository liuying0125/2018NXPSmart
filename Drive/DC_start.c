#include "include.h"
#include "DC_start.h"
#define   NM    3
CarElectric Car_E; 

RoadInfo  Road_Info; 
                           
int16  AD_valu[5],AD_V[5][NM],chazhi,chazhi_Island,chazhi_old = 0;
 
float sensor_to_one[5],cha_zhi;   //��һ���õ��ĵ��ֵ
 
int position ,position_back;   //�ж�С���������ϵ�λ�ý���
int16  max_front=0,max_back,max_side = 0;    //�� ��ǰ�� �� ���� ���ֵ�ĵ��

int16  max_v[5],min_v[5];  //��б궨 �ɼ�ֵ
int16  max_loop[5],min_loop[5];

//int16  Position_transit[5];  //��¼ ���ɵ� ��һ����ֵ
int16  AD_sum[5];   //��ȡ�����ֵʱ������õ�������
int16  AD_MAX_NUM;   
 
float  max_value,AD_0_max,AD_1_max,AD_2_max,AD_3_max;
 
 
 
 //7-1------------���ϲο��������
float sensor_zhongjian , Slope_AD_1   ;
int16 lose_valve = 43;
float Deviation[5];     //ƫ�������� һ���˲�����ֵ
float loop_cheat[5] = {10.0,10.0,10.0,10.0,10.0};



//��������
int16  AD_Ring_max[5],AD_Ring_min[5];
float AD_Ring[5];      //AD��й�һ���ֵ*100 sensor_to_oneһ�ι�һ����ֵ 




float multi_46 = 0.0;
uint8 cross_road_num = 0;   //ʮ�ִ���
uint8 cross_flag = 0;   //��ʮ�ֱ�־ 1Ϊ��ʮ����
uint8 set_cross[5] = {0,0,0,0,0};
uint8 cross_max = 5;
uint8 line_flag=0; //ֱ����־ 1Ϊ��ֱ����
int16 time_ms = 500;
uint8 cross_go=0;
 
  
uint8 loop_flag = 0;
uint8 loop_set_flag = 0;
int16 loop_num = 0;
int16 set_loop[5]={0,0,0,0,0};    //����ĸ�Բ�� 0�� 1��
uint8 loop_inflag = 0;
uint8 loop_last = 0;//1Ϊ��Բ���� 0˵��ʱ�䵽��
int16 loop_lasttime = 0;
//----------------------------------------------------------------


 static float abs_s(float a){

  if(a > 0)
    return a;
  else
    return -a;
  
}



/* 
*  ��������   Collect_Init 
*  ����˵���� ���ֵ����
*  ����˵����         
*  �������أ� ��
*  �޸�ʱ�䣺
*  ��    ע��
 */ 
void Collect_Init(void)
{
   uint16   i,j;
   int16    Position_transit_short[6];
   float   sensor_1,sensor_2,sensor_3,sensor_4,sensor_5,sensor_6 ;
   float Position_transit;
     // -------7��13��----------
   float AD_Island[2];
   
       max_v[0] = max_v[1] = max_v[2] = max_v[3] = max_v[4] = 0 ;
       min_v[0] = min_v[1] = min_v[2] = min_v[3] = min_v[4] = 1023 ; 
          
       for(i=0;i<8000;i++) 
       {
          
           Smoothing_AD() ;
            
           for(j=0;j<5;j++) 
           {      
               if(AD_valu[j] > max_v[j]) 
               {
                   max_v[j] = AD_valu[j];
                    
                                                      if(j==0)   {   Position_transit_short[0] =  AD_valu[1];  //��¼���ɵ� ���ֵ
                                                                     Position_transit_short[4] =  AD_valu[2];}
                                                      if(j==2)   {   Position_transit_short[1] =  AD_valu[1]; 
                                                                     Position_transit_short[5] =  AD_valu[0];}
                                                      if(j==3)    AD_Island[0] = AD_valu[3];  //������¼��ʷ��ֵ
                                                      if(j==4)    AD_Island[1] = AD_valu[4]; 
                 
               }
                 if(AD_valu[j]<min_v[j])
                {
                    min_v[j] = AD_valu[j];
                } 
           }
           
           DELAY_MS(1);           //��ʱ    
       }
       
        for(i = 0;i< 5;i++)
        {
            AD_Ring_max[i] = (int)((float)max_v[i] / 2);
            AD_Ring_min[i] = (int)((float)min_v[i] / 2);
        }
       Position_transit_short[2] = (int)((float)Position_transit_short[0] / 2 );
       Position_transit_short[3] = (int)((float)Position_transit_short[1] / 2 );
       
     //  ------------------------------��¼�Ĺ��ɵ��һ��---------------------------------------------
                                             sensor_1 = (float)(Position_transit_short[0] - min_v[1])/(float)(max_v[1] - min_v[1]); 
                                             if(sensor_1 <= 0.0)  sensor_1 = 0.001;
                                             if(sensor_1 >= 2.0)  sensor_1 = 2.0; 
                                             
                                             sensor_2 = (float)(Position_transit_short[1] - min_v[1])/(float)(max_v[1] - min_v[1]); 
                                             if(sensor_2 <= 0.0)  sensor_2 = 0.001;
                                             if(sensor_2 >= 2.0)  sensor_2 = 2.0; 
                                              
                                              sensor_3 = (float)(Position_transit_short[2] - AD_Ring_min[1])/(float)(AD_Ring_max[1] - AD_Ring_min[1]); 
                                              if(sensor_3 <= 0.0)  sensor_3 = 0.001;
                                              if(sensor_3 >= 2.0)  sensor_3 = 2.0; 
                                              
                                              sensor_4 = (float)(Position_transit_short[3] - AD_Ring_min[1])/(float)(AD_Ring_max[1] - AD_Ring_min[1]); 
                                              if(sensor_4 <= 0.0)  sensor_4 = 0.001;
                                              if(sensor_4 >= 2.0)  sensor_4 = 2.0; 
       
       Car_E.Position_transit[0] = (int16)(100 * sensor_1);
       Car_E.Position_transit[1] = (int16)(100 * sensor_2)  ;                                       
       Car_E.Position_transit[2] = (int16)(100 * sensor_3); 
       Car_E.Position_transit[3] = (int16)(100 * sensor_4); 
                                              
                   //--------------------------------------------ֱ���ᳵ-----------------------------------     
                                              sensor_5 = (float)(Position_transit_short[4] - min_v[2])/(float)(max_v[2] - min_v[2]); 
                                             if(sensor_5 <= 0.0)  sensor_1 = 0.001;
                                             if(sensor_5 >= 2.0)  sensor_1 = 2.0; 
                                             
                                             sensor_6 = (float)(Position_transit_short[5] - min_v[0])/(float)(max_v[0] - min_v[0]); 
                                             if(sensor_6 <= 0.0)  sensor_2 = 0.001;
                                             if(sensor_6 >= 2.0)  sensor_2 = 2.0; 
                                              Car_E.Position_transit[4] = (int16)(100 * sensor_5);
                                              Car_E.Position_transit[5] = (int16)(100 * sensor_6);
                                              
                                              
                                              
                                              
                                              
        //--------------------������߼�¼����̫����---------------------
      
       
       if(abs_s(Car_E.Position_transit[0] - Car_E.Position_transit[1]) >15.0   )
       {
         Position_transit = (Car_E.Position_transit[0] + Car_E.Position_transit[1])/2;
         
         Car_E.Position_transit[0] = Position_transit;
         Car_E.Position_transit[1] = Position_transit;
         
       } 
        
    
                                                                                                 //---------����---------
                                                                                     sensor_5 = (float)((AD_Island[0]- min_v[3] ) / 870.0    ); 
                                                                                     if(sensor_5 <= 0.0)  sensor_5 = 0.001;
                                                                                     if(sensor_5 >= 2.0)  sensor_5 = 2.0; 
                                                                                     Car_E.Side_L = 100*sensor_5;
                                                                                    
                                                                                     sensor_6 = (float)((AD_Island[1]  ) / 870.0    ); 
                                                                                     if(sensor_6 <= 0.0)  sensor_6 = 0.001;
                                                                                     if(sensor_6 >= 2.0)  sensor_6 = 2.0; 
                                                                                     Car_E.Side_R = 100*sensor_6;
                               
       
       
       
   Smart_Car.Buzzer.setOn();
   DELAY_MS(1000);
   Smart_Car.Buzzer.setOff();
   
   }
//======================================================================
//�������� : Smoothing_AD
//�������� : ADֵ����
//������� : 
//������� : 
//����ֵ   : NONE
//��ע     :  
//======================================================================
int16 AD_original[5][5];
void Smoothing_AD(void) 
{  
    uint8 i;   
    AD_original[0][4] = adc_once( AD0,ADC_10bit);
    AD_original[1][4] = adc_once( AD1,ADC_10bit);
    AD_original[2][4] = adc_once( AD2,ADC_10bit);
    AD_original[3][4] = adc_once( AD3,ADC_10bit);
    AD_original[4][4] = adc_once( AD4,ADC_10bit);
    for(i = 0; i < 4; i++)
    {
        AD_original[0][i] = AD_original[0][i+1];
        AD_original[1][i] = AD_original[1][i+1];
        AD_original[2][i] = AD_original[2][i+1];
        AD_original[3][i] = AD_original[3][i+1];
        AD_original[4][i] = AD_original[4][i+1];
    }
    
    
    for(i = 0; i < 5; i++)
    {
        AD_valu[i] = (AD_original[i][0]*1 + AD_original[i][1]*2 + AD_original[i][2]*3 + AD_original[i][3]*4 + AD_original[i][4]*5)/15; 
    }
    
}     

  



/* 
*  ��������   Date_analyse
*  ����˵���� ���ݷ���
*  ����˵����         
*  �������أ� ��
*  �޸�ʱ�䣺
*  ��    ע��
*/
void Date_analyse()
{  
  
  
    int fanghuandao = 1;
    
    
  
    int i;    //���λ��
    
   
    static int16 max_old = 1,max_crosstalk = 1;
    static int16 position_last = 2;
    static int16 Stright_Flag = 0;
    float  sensor_1;
   
    Smoothing_AD();
   // Read_ADC(); //��ȡ�����ֵ
     
        //----------------------��һ������-------------------------
     
     for(i=0;i<5;i++) 
    {
        sensor_to_one[i] = (float)(AD_valu[i] - min_v[i])/(float)(max_v[i] - min_v[i]); 
      //  if(i == 0 || i == 1 || i == 2)
            Car_E.AD_ramp[i] = 100 * sensor_to_one[i];
      
        if(sensor_to_one[i] <= 0.0)  sensor_to_one[i] = 0.001;
        if(sensor_to_one[i] > 2)  sensor_to_one[i] = 2; 
      
          Car_E.AD[i] = 100 * sensor_to_one[i];     //AD[i]Ϊ��һ�����ֵ  ��ΧΪ0-100
    }
    
    
 
      //�����ϵ���ֵ
   for(i=0;i < 5;i++) 
    {
        sensor_to_one[i] = (float)(AD_valu[i] - AD_Ring_min[i])/(float)(AD_Ring_max[i] - AD_Ring_min[i]); 
        if(sensor_to_one[i]<=0.0)  sensor_to_one[i]=0.001;
        if(sensor_to_one[i]>1.0)  sensor_to_one[i]=1.0; 
        
        AD_Ring[i] = 100 * sensor_to_one[i];     //AD[i]Ϊ��һ�����ֵ  ��ΧΪ0-100
    } 
    
    
     
    
      Car_E.Sum = Car_E.AD[0] + Car_E.AD[1] + Car_E.AD[2] ;  
      
      if(Car_E.Sum < 20  ){
      
         Smart_Car.CarStop();
      
      }else
      {
        Smart_Car.CarStartRun();
      }
      
     //------------------�ж�----------------
     
      for(i=0;i<3;i++)                 //�ҳ���ǿ�Ĵ�����
      {  
        if(Car_E.AD[max_front]<Car_E.AD[i] - 2)
          max_front=i;
      } 
        max_value=Car_E.AD[max_front]; 
      
   //���ߴ���
        if(max_value < lose_valve)          
        {
            max_front = max_old;
            max_value = Car_E.AD[max_front]; 
        }
        else
            max_old=max_front;
        
       //------------------��Ⱥͼ���--------------------
     Car_E.Vertical = ((Car_E.AD[1] - Car_E.AD[3])/(Car_E.AD[1] + Car_E.AD[3] )) - ((Car_E.AD[1] - Car_E.AD[4])/(Car_E.AD[1] + Car_E.AD[4] ));  //��ֱ��Ⱥ�
       
     //ˮƽ��Ⱥͼ����µ�˼�룬�����ȶ�
      Car_E.Parallel_New = ( sqrt((double)Car_E.AD[0]) -  sqrt((double)Car_E.AD[2]))   /  ((double)Car_E.AD[0] + (double)Car_E.AD[2]) ;
     //�����㷨 
   /*  Car_E.Zhongnan = 30000/25*(1/Car_E.AD[0]-1/Car_E.AD[2]);
     if(Car_E.Zhongnan < 0){
         Car_E.Zhongnan = -Car_E.Zhongnan; 
       } 
       
     
   */    
   
     //----------------------������λ�ý���------------------
       Car_E.Max_side = (Car_E.AD[3] > Car_E.AD[4]) ? 3:4; 
       
     
       
       
       if( max_front != 1 && Car_Meet.K60_coming_meet && AD_valu[0] + AD_valu[1] +AD_valu[2] > 2000){
       
         Car_Meet.K60_coming_meet = 0;
       
       }
       
       
      

int jishi;

int zuoflag= 0,youflag = 0;

int flag = 0;
       
       /* 
       
       
        //-----------------------------------------------------
     
    
    Car_E.coreChazhi = Car_E.AD_ramp[3] - Car_E.AD_ramp[4];
    
      
     jishi=pit_time_get_ms(PIT1);
      
     if(jishi>500 && jishi < 2300 && (Car_E.AD[3] > 180 || Car_E.AD[4] > 180)){
          
         Road_Info.On_Circle_Flag = 1;
           
         BuzzerOn();
         if(Car_E.coreChazhi < 0){    //��
              
            chazhi= -50 + ( Car_E.AD_ramp[3] - Car_E.AD_ramp[4] )*Road_Info.side*(jishi/1000) ;
          // chazhi =  Car_E.AD[3]*Road_Info.side;
         }
          if(Car_E.coreChazhi>0){
            
             chazhi= 50 + (Car_E.AD_ramp[3] - Car_E.AD_ramp[4]) *Road_Info.side *(jishi/1000);
            // chazhi = -Car_E.AD[4]*Road_Info.side;
          }
         flag12=1;
         
       }else  {
        //  BuzzerOff(); 
          Road_Info.On_Circle_Flag = 0;
       }
       
   
      
       if(jishi > 5000 && jishi < 9000){//�ٶ�4���־ʱ��Ӧ��4000~5000����Ӧ�󻷣�4.2�����Ͽ���3000~4000
           flag12=0;      
           pit_close(PIT1);  
 
       }
       if(Car_E.Sum > 500 && (Car_E.AD[1]+Car_E.AD[2]> 380  || Car_E.AD[1] + Car_E.AD[0] > 380) && flag12==0) {   
          
         if(Car_E.coreChazhi>-90&&Car_E.coreChazhi<90)  {
           if(Car_E.coreChazhi - Car_E.coreLastChazhi1<0)    zuoflag=1;
           if(Car_E.coreChazhi - Car_E.coreLastChazhi1>0){   youflag=1;
        } }
        Car_E.coreLastChazhi1 = Car_E.coreChazhi;
               if(youflag){  //����
                  
                   youflag=0;
                   pit_time_start(PIT1); 
               }
                
               if(zuoflag){  //����
                  
                   zuoflag=0;
                   pit_time_start(PIT1); 
                          
               }
          
       }
       
       
       
        
       
     if(Car_E.AD[0] + Car_E.AD[2] > 370){
     
     
       Road_Info.On_Circle_Flag = 1;
       
     }
       
       
       
        
       
       
       
       int round_turn_flag = 0;
       int round_judge_time  = 1000;   */
       
//  if(Smart_Car.DIPswitch == 13)        {
       Road_Info.tiaobian = Car_E.AD[3] - Car_E.AD[4];
       Road_Info.tiaobian_old = Road_Info.tiaobian;
       
       
       /*
       if(   AD_valu[0] + AD_valu[1] + AD_valu[2] >2100  && flag == 0){
       
         Road_Info.On_Circle_Flag = 1;
         
       
       }


       if(   AD_valu[0] + AD_valu[1] + AD_valu[2] < 2100 ){ 
         
                                                                                       flag = 0;      
                                                                                       pit_close(PIT1); 
                                                                                       Road_Info.On_Circle_Flag = 0;
                                                                                       Road_Info.In_Circle_Flag = 0; 
                                                                                       Road_Info.lock_Ring = 0;
                                                                                       Road_Info.LeftRing = 0;
                                                                                       Road_Info.RightRing = 0;
                                                                                       Road_Info.zhuanxiang = 0;
                                                                                       Smart_Car.Buzzer.setOff(); 
       }        
         
       if(Road_Info.Ring_diatance_flag  == 2){
       
       Road_Info.On_Circle_Flag = 0; 
       
         
       }  */
//}
       
  if(Road_Info.On_Circle_Flag == 1  ){  //---------------------�����ж�---7��-7��----------------  
   
   /*  Car_Speed.fSetSpeed = 0.4;
    
    chazhi = Car_E.Parallel_New * 60 + Car_E.Vertical*100;
    
    
    
    if(Car_E.AD_ramp[3]> 100 || Car_E.AD_ramp[4] > 100){
      
      
    chazhi = (Car_E.AD_ramp[3] - Car_E.AD_ramp[4]);
    Road_Info.zhuanxiang = chazhi;
    Road_Info.lock_Ring ++;
    
    }
    
    if( Road_Info.lock_Ring > 0){
    
      chazhi = Road_Info.zhuanxiang;
      
      if(Road_Info.lock_Ring == 400){
      
        
        Road_Info.Ring_flag = 1;
        Road_Info.lock_Ring = 0;
      
      }
    
    }
    
    
           */
       
   }  //����л���
   
   
    

        
else{  //û�л���
        /*
                  if( Car_Meet.K60_coming_meet == 2 ){            // û�лᳵ��־λ 
                   
                      if(Car_Meet.A_Car_Stop == 2){
                            
                             if( Car_Meet.distance > 6000){
                                      
                                        Car_Meet.k60_ready++ ;
                                        
                                 }
                             if(Car_Meet.k60_ready == 50){
                                      SmartCarStartRun();
                                      Car_Meet.position_flag = 0;
                                      Car_Meet.K60_coming_meet = 0; 
                             }
                            
                            }
              }
                   */
                      
  
  
  //------------------�������뿪���ٶȵ�λ-----------
  if(  !Car_Meet.K60_coming_meet ) {    
     
              if(Smart_Car.DIPswitch == 1){
              
                 Car_Speed.fSetSpeed = 2.5;              
              }
              
              else if(Smart_Car.DIPswitch == 2){
              
                 Car_Speed.fSetSpeed = 2.4;
                
              }
              else if(Smart_Car.DIPswitch == 3){
              
                Car_Speed.fSetSpeed = 2.4;
                
              }
              
              else if(Smart_Car.DIPswitch == 4){
              
                Car_Speed.fSetSpeed = 2.4;
              
              }
              else{
                Car_Speed.fSetSpeed =  2.4;
              
              }
                                                  //--------------------------ǰ��λ�ý���---------------------------
                                     
                                                    if(max_front==0 && (Car_E.AD[1] <= Car_E.Position_transit[0] - 1))  //�Ѿ�ƫ��0�Ŵ�����   
                                                    {
                                                        position=0;  
                                                      }   
                                                      else if((max_front==0 && (Car_E.AD[1] > Car_E.Position_transit[0] + 1)) || (max_front == 1 && (Car_E.AD[0] - Car_E.AD[2]) > 1))  //���λ��  0-1�Ŵ�����֮��
                                                    {    
                                                      position=1; 
                                                      AD_0_max = Car_E.AD[0];  //��¼�´�ʱ��0�Ŵ�������ֵ
                                                      }
                                                      else if((max_front==1 && (Car_E.AD[2] - Car_E.AD[0]) > 1) || (max_front==2 && (Car_E.AD[1] > Car_E.Position_transit[1] + 1)))  //�Ҳ�λ��  1-2�Ŵ�����֮��
                                                    {
                                                      position=2;
                                                      AD_2_max = Car_E.AD[2];  //��¼�´�ʱ��2�Ŵ�������ֵ
                                                      }
                                                      else if(max_front==2 && (Car_E.AD[1] <= Car_E.Position_transit[1] - 1))  //�Ѿ�ƫ��2�Ŵ�����    
                                                      {        
                                                        position = 3;  
                                                      } 
                                      
                                                     if(max_front==1 && (abs_s(Car_E.AD[0]-Car_E.AD[2]) <12))
                                                      {
                                                      Stright_Flag = 1;
                                                        }
                                   
                                                      if(abs_s(position - position_last) == 2)  //λ�÷�����
                                                            position = position_last;
                                                            position_last = position;
                                                
                                                    //--------����ں͵��߼нǹ����º���ת���㣬��ʱ���ɵ�ǿ������---------
                                                      if(position == 0 && AD_0_max < 85)
                                                      {
                                                      AD_0_max = 85 + abs_s(85 - AD_0_max);    
                                                        }                                               
                                                      else if(position == 3 && AD_2_max <85)
                                                       {
                                                      AD_2_max = 85 + abs_s(85 - AD_2_max);  
                                                        }    
                                            
                                         //------------����ƫ����--------------- 
                                         
                                    
                                        
                                                               if(position == 0)   //��ඪ��  
                                                            {     
                                                                  chazhi = -((int16)((Car_E.AD[1] - abs_s(AD_0_max-Car_E.AD[0]) - AD_0_max)*1.5) - 6 );
                                                            }
                                          
                                                               else if(position == 1 || position == 2) //�����м�λ��  ����Ҳ��position = 1/2 ֮��
                                                            { 
                                          
                                                                 //  chazhi =   (int16)( Car_E.AD[0] - Car_E.AD[2] ) ; 
                                                                   chazhi = Car_E.Parallel_New*Road_Info.E_offset;
                                                                 // chazhi = Car_E.Zhongnan;
                                                             }       
                                        
                                                               else if(position == 3)  //�Ҳඪ��
                                                            {    
                                                                 chazhi = -((int16)((abs_s(AD_2_max - Car_E.AD[2]) + AD_2_max - Car_E.AD[1] )*1.5) + 6  );    
                                                
                                                             } 
                                        
                                        
                                                               Deviation[4] = chazhi;
                                         
                                                                 Car_E.Parallel_6_1= (sqrt(Car_E.AD[2]) - sqrt(Car_E.AD[0]))*1000/(Car_E.AD[0] + Car_E.AD[2]);
                                                                      if(Car_E.Parallel_6_1 > 95)
                                                                       Car_E.Parallel_6_1 = 95;
                                                                      else if(Car_E.Parallel_6_1 < -95)
                                                                       Car_E.Parallel_6_1 = -95;
                                         
                                               
                                             
                                                    if ( Car_E.AD[0]+Car_E.AD[1]+Car_E.AD[2]< 75 )  
                                                    {     //�ٶȹ��� ת�䶪�� 
                                                         
                                                          Road_Info.lost = 1;
                                                          chazhi =  1.2*(Car_E.AD[0] - Car_E.AD[2])   ;
                                                          if( abs_s(Car_E.AD[3] - Car_E.AD[4]) > 30){ 
                                                            chazhi = chazhi + (Car_E.AD[4] - Car_E.AD[3])/1.5 ; 
                                                          }
                                                          
                                                       /*        Car_Speed.fSafeSpeed = (Car_E.AD[0] - Car_E.AD[2])/50;
                                                                     if( Car_Speed.fSafeSpeed < 0){
                                                                      Car_Speed.fSafeSpeed = -  Car_Speed.fSafeSpeed;
                                                                                                   } 
                                                                      Car_Speed.fSetSpeed =  Car_Speed.fSetSpeed -   Car_Speed.fSafeSpeed  ;
                                                          
                                                                            if(Car_Speed.fSetSpeed  < 0.6 )
                                                                                Car_Speed.fSetSpeed  = 0.6;
                                                          
                                                                                Stright_Flag = 0;*/
                                                                               }
                                        
                                             
                                                 
                                                                                  if(Stright_Flag)
                                                                              { 
                                                                                    chazhi = (int16)(Car_E.Parallel_New*100);  //��ֱ����б�ʿ��ƶ��   
                                                                                  // chazhi = Turn_Out_Filter(chazhi); //�ж�ֱ��chazhi�˲�
                                                                                   // Car_Speed.fSetSpeed = 2.8;
                                                                                    Stright_Flag = 0;
                                                                               } 
                                         
                                                                //-----------ͣ������-------------
                                                                        if(Smart_Car.Status == Stop){ 
                                        
                                                                             Car_Speed.fSetSpeed = 0; 
                                                                             Car_Control.Car_A_Stop = 0 ;
                                                                                  }  
                                                                           
                                                                   if(Car_Control.Car_A_Stop == 0){   //-------ͣ����������---------

                                                                       Car_Speed.fSetSpeed = 0;
                                            
                                                                   } 
  
  
}
                  
                  
 else if(  Car_Meet.K60_coming_meet == 1 && !Car_Meet.K60_meet_over_distance_flag   ){    //�����ᳵ��־λ  -----------------------------
   
                              Car_Speed.fSetSpeed = 0.8;
                               
                             /*  
                            if((max_front == 0 && (Car_E.AD[1] > Car_E.Position_transit[0] + 1)) || (max_front == 1 && (Car_E.AD[0] - Car_E.AD[2]) > 1))  //�Ҳ�λ��  1 2�Ŵ�����֮��
                            {    
                                   position=1; 
                                  
                            }
                            else if((max_front == 1 && (Car_E.AD[2] - Car_E.AD[0]) > 1) || (max_front == 2 && (Car_E.AD[1] > Car_E.Position_transit[1] + 1)))  //���λ��  0 1�Ŵ�����֮��
                            {
                                        position=2;
                                   
                            }
                         
                            else if(max_front == 2 && (Car_E.AD[1] <= Car_E.Position_transit[1] - 1))  //�Ѿ�ƫ��2�Ŵ�����    
                            {           
                                          position = 3;  
                                   
                            } 
                            else if(max_front == 2 && (Car_E.AD[0] <= Car_E.Position_transit[5] - 2) && (Car_E.AD[2] < 90 )){
                                    
                                          Car_Meet.position_flag ++; 
                                          position = 4;  
                                    
                            } 
                            else if( max_front == 2 && Car_E.AD[2] < 50){
                            
                                          position = 5; 
                            
                                 }
                                */ 
                              
                              
                             // - �µ�λ�ý��㣬�ó�ƫ��һ��
                              
                              if((max_front == 0 && (Car_E.AD[1] > Car_E.Position_transit[0] + 1)) || (max_front == 1 && (Car_E.AD[0] - Car_E.AD[2]) > 1))  //�Ҳ�λ��  1 2�Ŵ�����֮��
                            {    
                                   position=1; 
                                  
                            }
                            else if((max_front == 1 && (Car_E.AD[2] - Car_E.AD[0]) > 1) || (max_front == 2 && (Car_E.AD[1] > Car_E.Position_transit[1] + 1)))  //���λ��  0 1�Ŵ�����֮��
                            {
                                        position=2;
                                   
                            }
                         
                            else if(max_front == 2 && (Car_E.AD[1] <= Car_E.Position_transit[1] - 1))  //�Ѿ�ƫ��2�Ŵ�����    
                            {           
                                          position = 3;  
                                   
                            } 
                            else if(max_front == 2 && (Car_E.AD[0] <= Car_E.Position_transit[5] - 4) && (Car_E.AD[2] < 100 )){
                                    
                                          Car_Meet.position_flag ++; 
                                          position = 4;  
                                    
                            } 
                            else if( max_front == 2 && Car_E.AD[0] < Car_E.Position_transit[5] / 1.5){
                            
                                          position = 5; 
                            
                                 }
                                
                       
                         
                              //---------------------����positionд�µ�λ�ý���--------------------
                                  if(position == 1 || position == 2){
                          
                                   chazhi = Car_E.AD[1] - Car_E.AD[2] + 60;
                                                                                               
                                 } 
                                 
                                  else  if(  position  == 3){
                                    
                                     chazhi = -10.16;
                                      
                                  
                                  }
                                  else if(position == 4){
                                  
                                    chazhi = - Car_E.AD[1]; 
                                  
                                    }
                                  else if(position == 5){
                                  
                                    chazhi = Car_E.AD[1] - Car_E.AD[2];
                                  
                                  }
                          
                    
                            
                            
                              if(Car_Meet.position_flag == Car_E.flag || Car_Meet.k60_distance_flag_stop == 1 ){    //��k60ͣ���о������̬��������
                                                         
                                
                                                          Car_Meet.over ++ ;
                                                          SmartCarStop();
                                                          
                                                          Car_Meet.fasong  = 1 ;
                                                           
                                                        //  Car_Meet.K60_meet_over_distance_flag = 1;
                                                          
                                                          Car_Meet.position_flag = 0;
                                                          Car_Meet.k60_distance_flag_stop = 0;
                                                           
                                                      
                                                   }
                               

                                  //-----------ͣ������--------------------
                            
                                  if(Car_Meet.over_flag == 3){
                                  
                                    Smart_Car.Status == Stop;
                                    
                                  } 

                                                            if(Smart_Car.Status == Stop){ 
                                                                 Car_Speed.fSetSpeed = 0; 
                                                                 Car_Control.Car_A_Stop = 0 ;
                                                             }  
                                                                     
                                                       if(Car_Control.Car_A_Stop == 0){

                                                           Car_Speed.fSetSpeed = 0;
                                                     } 
                              
           }    //��ʱ�ᳵ
      
   
  /*
   if( Car_Meet.K60_coming_meet == 2 ){            // û�лᳵ��־λ 
                   
                                                
  
                         if(Car_Meet.A_Car_Stop == 2){
                                                            
                                                             if( Car_Meet.distance > 6000){
                                                                      
                                                                        Car_Meet.k60_ready++ ;
                                                                        
                                                                 }
                                                             if(Car_Meet.k60_ready == 50){
                                                                      SmartCarStartRun();
                                                                      Car_Meet.position_flag = 0;
                                                                      Car_Meet.K60_coming_meet = 0; 
                                                             }
                                                            
                                                          }
                         
                          
              }*/
    
   
  
  
  
                      
  
       }  //���û�л���
  
                     if(Deviation[4] > 130)
                            Deviation[4] = 130;
                      else if(Deviation[4] < -130)
                            Deviation[4] = -130;
  
    
                      
                      
    
}//���������ó�chazhi����