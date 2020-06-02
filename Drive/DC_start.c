#include "include.h"
#include "DC_start.h"
#define   NM    3
CarElectric Car_E; 

RoadInfo  Road_Info; 
                           
int16  AD_valu[5],AD_V[5][NM],chazhi,chazhi_Island,chazhi_old = 0;
 
float sensor_to_one[5],cha_zhi;   //归一化得到的电感值
 
int position ,position_back;   //判断小车在赛道上的位置解算
int16  max_front=0,max_back,max_side = 0;    //判 断前排 和 后排 最大值的电感

int16  max_v[5],min_v[5];  //电感标定 采集值
int16  max_loop[5],min_loop[5];

//int16  Position_transit[5];  //记录 过渡点 归一化的值
int16  AD_sum[5];   //读取电感数值时候过滤用到的数组
int16  AD_MAX_NUM;   
 
float  max_value,AD_0_max,AD_1_max,AD_2_max,AD_3_max;
 
 
 
 //7-1------------早上参考代码更新
float sensor_zhongjian , Slope_AD_1   ;
int16 lose_valve = 43;
float Deviation[5];     //偏差的输出量 一阶滤波缓冲值
float loop_cheat[5] = {10.0,10.0,10.0,10.0,10.0};



//环岛参数
int16  AD_Ring_max[5],AD_Ring_min[5];
float AD_Ring[5];      //AD电感归一后的值*100 sensor_to_one一次归一化的值 




float multi_46 = 0.0;
uint8 cross_road_num = 0;   //十字次数
uint8 cross_flag = 0;   //入十字标志 1为在十字内
uint8 set_cross[5] = {0,0,0,0,0};
uint8 cross_max = 5;
uint8 line_flag=0; //直道标志 1为在直道内
int16 time_ms = 500;
uint8 cross_go=0;
 
  
uint8 loop_flag = 0;
uint8 loop_set_flag = 0;
int16 loop_num = 0;
int16 set_loop[5]={0,0,0,0,0};    //最多四个圆环 0左 1右
uint8 loop_inflag = 0;
uint8 loop_last = 0;//1为出圆环后 0说明时间到了
int16 loop_lasttime = 0;
//----------------------------------------------------------------


 static float abs_s(float a){

  if(a > 0)
    return a;
  else
    return -a;
  
}



/* 
*  函数名称   Collect_Init 
*  功能说明： 最大值采样
*  参数说明：         
*  函数返回： 无
*  修改时间：
*  备    注：
 */ 
void Collect_Init(void)
{
   uint16   i,j;
   int16    Position_transit_short[6];
   float   sensor_1,sensor_2,sensor_3,sensor_4,sensor_5,sensor_6 ;
   float Position_transit;
     // -------7月13日----------
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
                    
                                                      if(j==0)   {   Position_transit_short[0] =  AD_valu[1];  //记录过渡点 电感值
                                                                     Position_transit_short[4] =  AD_valu[2];}
                                                      if(j==2)   {   Position_transit_short[1] =  AD_valu[1]; 
                                                                     Position_transit_short[5] =  AD_valu[0];}
                                                      if(j==3)    AD_Island[0] = AD_valu[3];  //环岛记录历史峰值
                                                      if(j==4)    AD_Island[1] = AD_valu[4]; 
                 
               }
                 if(AD_valu[j]<min_v[j])
                {
                    min_v[j] = AD_valu[j];
                } 
           }
           
           DELAY_MS(1);           //延时    
       }
       
        for(i = 0;i< 5;i++)
        {
            AD_Ring_max[i] = (int)((float)max_v[i] / 2);
            AD_Ring_min[i] = (int)((float)min_v[i] / 2);
        }
       Position_transit_short[2] = (int)((float)Position_transit_short[0] / 2 );
       Position_transit_short[3] = (int)((float)Position_transit_short[1] / 2 );
       
     //  ------------------------------记录的过渡点归一化---------------------------------------------
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
                                              
                   //--------------------------------------------直道会车-----------------------------------     
                                              sensor_5 = (float)(Position_transit_short[4] - min_v[2])/(float)(max_v[2] - min_v[2]); 
                                             if(sensor_5 <= 0.0)  sensor_1 = 0.001;
                                             if(sensor_5 >= 2.0)  sensor_1 = 2.0; 
                                             
                                             sensor_6 = (float)(Position_transit_short[5] - min_v[0])/(float)(max_v[0] - min_v[0]); 
                                             if(sensor_6 <= 0.0)  sensor_2 = 0.001;
                                             if(sensor_6 >= 2.0)  sensor_2 = 2.0; 
                                              Car_E.Position_transit[4] = (int16)(100 * sensor_5);
                                              Car_E.Position_transit[5] = (int16)(100 * sensor_6);
                                              
                                              
                                              
                                              
                                              
        //--------------------如果两边记录点差别太大了---------------------
      
       
       if(abs_s(Car_E.Position_transit[0] - Car_E.Position_transit[1]) >15.0   )
       {
         Position_transit = (Car_E.Position_transit[0] + Car_E.Position_transit[1])/2;
         
         Car_E.Position_transit[0] = Position_transit;
         Car_E.Position_transit[1] = Position_transit;
         
       } 
        
    
                                                                                                 //---------环岛---------
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
//函数名称 : Smoothing_AD
//功能描述 : AD值采样
//输入参数 : 
//输出参数 : 
//返回值   : NONE
//备注     :  
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
*  函数名称   Date_analyse
*  功能说明： 数据分析
*  参数说明：         
*  函数返回： 无
*  修改时间：
*  备    注：
*/
void Date_analyse()
{  
  
  
    int fanghuandao = 1;
    
    
  
    int i;    //电感位置
    
   
    static int16 max_old = 1,max_crosstalk = 1;
    static int16 position_last = 2;
    static int16 Stright_Flag = 0;
    float  sensor_1;
   
    Smoothing_AD();
   // Read_ADC(); //读取电感数值
     
        //----------------------归一化处理-------------------------
     
     for(i=0;i<5;i++) 
    {
        sensor_to_one[i] = (float)(AD_valu[i] - min_v[i])/(float)(max_v[i] - min_v[i]); 
      //  if(i == 0 || i == 1 || i == 2)
            Car_E.AD_ramp[i] = 100 * sensor_to_one[i];
      
        if(sensor_to_one[i] <= 0.0)  sensor_to_one[i] = 0.001;
        if(sensor_to_one[i] > 2)  sensor_to_one[i] = 2; 
      
          Car_E.AD[i] = 100 * sensor_to_one[i];     //AD[i]为归一化后的值  范围为0-100
    }
    
    
 
      //环岛上的数值
   for(i=0;i < 5;i++) 
    {
        sensor_to_one[i] = (float)(AD_valu[i] - AD_Ring_min[i])/(float)(AD_Ring_max[i] - AD_Ring_min[i]); 
        if(sensor_to_one[i]<=0.0)  sensor_to_one[i]=0.001;
        if(sensor_to_one[i]>1.0)  sensor_to_one[i]=1.0; 
        
        AD_Ring[i] = 100 * sensor_to_one[i];     //AD[i]为归一化后的值  范围为0-100
    } 
    
    
     
    
      Car_E.Sum = Car_E.AD[0] + Car_E.AD[1] + Car_E.AD[2] ;  
      
      if(Car_E.Sum < 20  ){
      
         Smart_Car.CarStop();
      
      }else
      {
        Smart_Car.CarStartRun();
      }
      
     //------------------判断----------------
     
      for(i=0;i<3;i++)                 //找出最强的传感器
      {  
        if(Car_E.AD[max_front]<Car_E.AD[i] - 2)
          max_front=i;
      } 
        max_value=Car_E.AD[max_front]; 
      
   //丢线处理
        if(max_value < lose_valve)          
        {
            max_front = max_old;
            max_value = Car_E.AD[max_front]; 
        }
        else
            max_old=max_front;
        
       //------------------差比和计算--------------------
     Car_E.Vertical = ((Car_E.AD[1] - Car_E.AD[3])/(Car_E.AD[1] + Car_E.AD[3] )) - ((Car_E.AD[1] - Car_E.AD[4])/(Car_E.AD[1] + Car_E.AD[4] ));  //竖直差比和
       
     //水平差比和加入新的思想，更加稳定
      Car_E.Parallel_New = ( sqrt((double)Car_E.AD[0]) -  sqrt((double)Car_E.AD[2]))   /  ((double)Car_E.AD[0] + (double)Car_E.AD[2]) ;
     //中南算法 
   /*  Car_E.Zhongnan = 30000/25*(1/Car_E.AD[0]-1/Car_E.AD[2]);
     if(Car_E.Zhongnan < 0){
         Car_E.Zhongnan = -Car_E.Zhongnan; 
       } 
       
     
   */    
   
     //----------------------侧面电感位置解算------------------
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
         if(Car_E.coreChazhi < 0){    //左
              
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
       
   
      
       if(jishi > 5000 && jishi < 9000){//速度4清标志时间应在4000~5000以适应大环，4.2及以上可用3000~4000
           flag12=0;      
           pit_close(PIT1);  
 
       }
       if(Car_E.Sum > 500 && (Car_E.AD[1]+Car_E.AD[2]> 380  || Car_E.AD[1] + Car_E.AD[0] > 380) && flag12==0) {   
          
         if(Car_E.coreChazhi>-90&&Car_E.coreChazhi<90)  {
           if(Car_E.coreChazhi - Car_E.coreLastChazhi1<0)    zuoflag=1;
           if(Car_E.coreChazhi - Car_E.coreLastChazhi1>0){   youflag=1;
        } }
        Car_E.coreLastChazhi1 = Car_E.coreChazhi;
               if(youflag){  //进右
                  
                   youflag=0;
                   pit_time_start(PIT1); 
               }
                
               if(zuoflag){  //进左
                  
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
       
  if(Road_Info.On_Circle_Flag == 1  ){  //---------------------环岛判断---7月-7日----------------  
   
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
       
   }  //如果有环岛
   
   
    

        
else{  //没有环岛
        /*
                  if( Car_Meet.K60_coming_meet == 2 ){            // 没有会车标志位 
                   
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
                      
  
  
  //------------------比赛拨码开关速度档位-----------
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
                                                  //--------------------------前排位置解算---------------------------
                                     
                                                    if(max_front==0 && (Car_E.AD[1] <= Car_E.Position_transit[0] - 1))  //已经偏离0号传感器   
                                                    {
                                                        position=0;  
                                                      }   
                                                      else if((max_front==0 && (Car_E.AD[1] > Car_E.Position_transit[0] + 1)) || (max_front == 1 && (Car_E.AD[0] - Car_E.AD[2]) > 1))  //左侧位置  0-1号传感器之间
                                                    {    
                                                      position=1; 
                                                      AD_0_max = Car_E.AD[0];  //记录下此时的0号传感器的值
                                                      }
                                                      else if((max_front==1 && (Car_E.AD[2] - Car_E.AD[0]) > 1) || (max_front==2 && (Car_E.AD[1] > Car_E.Position_transit[1] + 1)))  //右侧位置  1-2号传感器之间
                                                    {
                                                      position=2;
                                                      AD_2_max = Car_E.AD[2];  //记录下此时的2号传感器的值
                                                      }
                                                      else if(max_front==2 && (Car_E.AD[1] <= Car_E.Position_transit[1] - 1))  //已经偏离2号传感器    
                                                      {        
                                                        position = 3;  
                                                      } 
                                      
                                                     if(max_front==1 && (abs_s(Car_E.AD[0]-Car_E.AD[2]) <12))
                                                      {
                                                      Stright_Flag = 1;
                                                        }
                                   
                                                      if(abs_s(position - position_last) == 2)  //位置防跳变
                                                            position = position_last;
                                                            position_last = position;
                                                
                                                    //--------弯道内和导线夹角过大导致后面转向不足，此时过渡点强制增大---------
                                                      if(position == 0 && AD_0_max < 85)
                                                      {
                                                      AD_0_max = 85 + abs_s(85 - AD_0_max);    
                                                        }                                               
                                                      else if(position == 3 && AD_2_max <85)
                                                       {
                                                      AD_2_max = 85 + abs_s(85 - AD_2_max);  
                                                        }    
                                            
                                         //------------计算偏移量--------------- 
                                         
                                    
                                        
                                                               if(position == 0)   //左侧丢线  
                                                            {     
                                                                  chazhi = -((int16)((Car_E.AD[1] - abs_s(AD_0_max-Car_E.AD[0]) - AD_0_max)*1.5) - 6 );
                                                            }
                                          
                                                               else if(position == 1 || position == 2) //处于中间位置  环岛也在position = 1/2 之间
                                                            { 
                                          
                                                                 //  chazhi =   (int16)( Car_E.AD[0] - Car_E.AD[2] ) ; 
                                                                   chazhi = Car_E.Parallel_New*Road_Info.E_offset;
                                                                 // chazhi = Car_E.Zhongnan;
                                                             }       
                                        
                                                               else if(position == 3)  //右侧丢线
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
                                                    {     //速度过快 转弯丢线 
                                                         
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
                                                                                    chazhi = (int16)(Car_E.Parallel_New*100);  //长直道用斜率控制舵机   
                                                                                  // chazhi = Turn_Out_Filter(chazhi); //判断直道chazhi滤波
                                                                                   // Car_Speed.fSetSpeed = 2.8;
                                                                                    Stright_Flag = 0;
                                                                               } 
                                         
                                                                //-----------停车程序-------------
                                                                        if(Smart_Car.Status == Stop){ 
                                        
                                                                             Car_Speed.fSetSpeed = 0; 
                                                                             Car_Control.Car_A_Stop = 0 ;
                                                                                  }  
                                                                           
                                                                   if(Car_Control.Car_A_Stop == 0){   //-------停车锁死程序---------

                                                                       Car_Speed.fSetSpeed = 0;
                                            
                                                                   } 
  
  
}
                  
                  
 else if(  Car_Meet.K60_coming_meet == 1 && !Car_Meet.K60_meet_over_distance_flag   ){    //产生会车标志位  -----------------------------
   
                              Car_Speed.fSetSpeed = 0.8;
                               
                             /*  
                            if((max_front == 0 && (Car_E.AD[1] > Car_E.Position_transit[0] + 1)) || (max_front == 1 && (Car_E.AD[0] - Car_E.AD[2]) > 1))  //右侧位置  1 2号传感器之间
                            {    
                                   position=1; 
                                  
                            }
                            else if((max_front == 1 && (Car_E.AD[2] - Car_E.AD[0]) > 1) || (max_front == 2 && (Car_E.AD[1] > Car_E.Position_transit[1] + 1)))  //左侧位置  0 1号传感器之间
                            {
                                        position=2;
                                   
                            }
                         
                            else if(max_front == 2 && (Car_E.AD[1] <= Car_E.Position_transit[1] - 1))  //已经偏离2号传感器    
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
                              
                              
                             // - 新的位置解算，让车偏向一边
                              
                              if((max_front == 0 && (Car_E.AD[1] > Car_E.Position_transit[0] + 1)) || (max_front == 1 && (Car_E.AD[0] - Car_E.AD[2]) > 1))  //右侧位置  1 2号传感器之间
                            {    
                                   position=1; 
                                  
                            }
                            else if((max_front == 1 && (Car_E.AD[2] - Car_E.AD[0]) > 1) || (max_front == 2 && (Car_E.AD[1] > Car_E.Position_transit[1] + 1)))  //左侧位置  0 1号传感器之间
                            {
                                        position=2;
                                   
                            }
                         
                            else if(max_front == 2 && (Car_E.AD[1] <= Car_E.Position_transit[1] - 1))  //已经偏离2号传感器    
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
                                
                       
                         
                              //---------------------根据position写新的位置解算--------------------
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
                          
                    
                            
                            
                              if(Car_Meet.position_flag == Car_E.flag || Car_Meet.k60_distance_flag_stop == 1 ){    //让k60停车有距离和姿态两个限制
                                                         
                                
                                                          Car_Meet.over ++ ;
                                                          SmartCarStop();
                                                          
                                                          Car_Meet.fasong  = 1 ;
                                                           
                                                        //  Car_Meet.K60_meet_over_distance_flag = 1;
                                                          
                                                          Car_Meet.position_flag = 0;
                                                          Car_Meet.k60_distance_flag_stop = 0;
                                                           
                                                      
                                                   }
                               

                                  //-----------停车程序--------------------
                            
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
                              
           }    //此时会车
      
   
  /*
   if( Car_Meet.K60_coming_meet == 2 ){            // 没有会车标志位 
                   
                                                
  
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
    
   
  
  
  
                      
  
       }  //如果没有环岛
  
                     if(Deviation[4] > 130)
                            Deviation[4] = 130;
                      else if(Deviation[4] < -130)
                            Deviation[4] = -130;
  
    
                      
                      
    
}//分析赛道得出chazhi结束