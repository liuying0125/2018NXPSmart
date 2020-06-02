#ifndef _DC_START_H_
#define _DC_START_H_

//------------------ADC接口-----------------------
#define   AD0    ADC1_SE4a     // PTE0 
#define   AD1    ADC1_SE5a     // PTE1
#define   AD2    ADC1_SE6a     // PTE2
#define   AD3    ADC1_SE7a     // PTE3
#define   AD4    ADC0_SE15      // PTC1   
 
#define NM 3  //滑动平滑滤波
 


typedef struct{
      int16  Position_transit[6];  //记录过渡点归一化的值
      
      int Max_side;
        
      float AD[5]; 
      
      float AD_ramp[5];
      
      float Loop[5];
      
      float AD_Island[5];
      
      float Sum;
      
      double Part[2];
      
      float Parallel;
      float Vertical;
      
      double Parallel_New;
      
      double Parallel_6_1;
  
      double Parallel_Old;
      
      double Zhongnan;
      
      float All_Old;
     
      float Side_L;
      float Side_R;
      
      float flag;
      
      
      
      
       
      
      float coreChazhi;
      float coreChazhi1;
      float coreLastChazhi1;
      
      
 
}CarElectric;

extern CarElectric Car_E;
 
 
 void Smoothing_AD(void) ;
 

extern int16  Position_transit[5];  //记录过渡点归一化的值

extern int16  max_v[5],min_v[5];    //电感标定 采集值
extern int16 max_loop[5],min_loop[5];


extern float  AD[5],sensor_to_one[5];  //归一化之后的值
 
extern int16  AD_sum[5]; 

extern int16  max_front,max_back,max_side; 


extern int16  AD_MAX_NUM;   


extern int16  AD_valu[5],AD_V[5][NM],chazhi,chazhi_Island;
 
 
extern int position ,position_back,Stright_Flag;
extern float  max_value,AD_0_max,AD_1_max,AD_2_max,AD_3_max;
  

//------------2018-3-29-----更新算法-----------------
extern int AD_judge;
extern int chazhi_History[50];
extern float AD_front_sum;  //前排三个电感总和
extern int  x;
extern int decide;
extern int RingRoad,RingRoad_old,RingRoad_max;


 
 //5.24早上参考代码更新
extern float sensor_zhongjian , Slope_AD_1  ;
extern float Deviation[5];     //偏差的输出量 一阶滤波缓冲值
extern float AD_Ring[5];


//--------------------函数---------------------
extern void Date_analyse();   
extern void SC_black_Init();


#endif