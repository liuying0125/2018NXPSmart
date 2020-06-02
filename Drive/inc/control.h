#ifndef _CONTROL_H_
#define _CONTROL_H_

  
extern float UartSetAngle;  //蓝牙转角

extern int Nrf_Flag;

typedef struct  
{
	//PID控制参数
	float P;//P参数即比例参数
	float I;//I参数即积分参数
	float D;//D参数即微分参数
       //控制输出
	double pOut;//比例控制输出
	double iOut;//积分控制输出
	double dOut;//微分控制输出
	//总输出
	double Out;//PID总输出
        float P_Meet;
	 
}PID;

typedef struct 
{
	PTXn_e BuzzerPin;
	void (*setOn)(void);
	void (*setOff)(void);
}Buzzer; 


 
   
typedef struct{
     
     
     int On_Circle_Flag;
     int In_Circle_Flag;
     int Out_Circle_Flag;
     
     int RightRing;
     int LeftRing;
      
     int Stright_Flag;
      
     float Car_Old;
 
     int lost;
     
     int time;  //时间片
     float E_offset;
     
     float side;
     
     
     float tiaobian;
     float tiaobian_old;
     
     int lock_Ring;
     
     float tiao[3];
     int zhuanxiang;
     int jishi;
     
     int Ring_flag;
     int Ring_distance;
     int Ring_diatance_flag;
     
     
     
}RoadInfo;
extern RoadInfo  Road_Info; 

typedef struct 
{
    int Stop_Flag;
    int A_Car_Run;
    float A_Car_Stop;
    int B_Car_Run;
    int B_Car_Stop;
    int16 distance;
    float Nrf_Flag;
    int Receive;
    int Receive_wupan;
    
    //-----------7月10日
    int kea_coming;
    int kea_stop;
    int k60_ready;
    int k60_coming;
    int k60_stop;
    
   
    int kea_keep_flag;
    
    int fasong;
      
    int position_flag;
     
    
    
    int k60_distance_flag;
  
    int kea_coming_flag;
    int kea_coming_flag_qingdiao;
    int k60_ready_flag;
   
    int k60_distance_flag_stop;
   
   
   
   //--------------------------------
    int K60_coming_meet;
    int K60_length;
    int K60_meet_over_distance_flag;
    int  K60_meet_over_distance;
   
    int  k60_shibie;
    int k60_shibie_length;
   

    
    int kaishihuiche;
    int kaishihuiche_diatance;
    
    int zuizhongting;
    int zuizhongting_distance;
    int zuizhongting_flag;
    
    int over;
    int over_distance;
    int over_flag;
    
    

}CarMeet;

extern CarMeet Car_Meet;



#define Run 1
#define Stop 0

typedef struct
{
  
	int Status;//车模状态   
        void (*CarStartRun)(void);
	void (*CarStop)(void);
        
        double Distance;//行驶距离
	double RunTime;//运行时间
	double CarSpeed;//瞬时车速
	//模块
	//Motor Motor;
	Buzzer Buzzer;
        int DIPswitch;
}SmartCar;
extern SmartCar Smart_Car;



//舵机 电机类
typedef struct Student
{
	PID Server;
        PID Speed;
        
        int baseServoPwm;   //舵机中值
        int LeftAngle;     //最左边
        int RightAngle;    //最右边
        int SetAngle;     
        int SetAngle_Last; 
        int Car_A_Stop;
        
}CarControl;

extern struct Student /*CarControl*/ Car_Control;


//速度控制类
typedef struct 
{
  
  
  
        float DIPspeed;
	 
	double fCarSpeed;//车实际速度
        
        double CarSpeed_L;
        double CarSpeed_R;
        
	float fSetSpeed;    //车设定速度
        
        double fSafeSpeed;  //车过弯环岛的安全速度
        
	double fMaxSpeed; //车设定最大速度
	
        double fMinSpeed; //车设定最大速度
	 
        int nLeftMotorSpeed;//编码器获得左轮的速度
	int nRightMotorSpeed;//编码器获得右轮的速度
        
         
        double MotorPwmChangeR;
        double MotorPwmfR;
        double MotorPwmfR_old;
        int MotorPwmfR_int;
        
        double MotorPwmChangeL;
        double MotorPwmfL;
        double MotorPwmfL_old;
        int MotorPwmfL_int;
      
        
        double err_L;
        double err_L_1;
        double err_L_2;
        
        double err_R;
  
        double err_R_1;
        double err_R_2;
        
        int xL;
        int xR;
        
        int chasu_Left;
        int chasu_Right;
        
        double chasu_P;
        
        double Distance;
        
	 
	double fSpeedControlIntegral;//速度控制积分输出
	double fSpeedControlOutNew;//当前速度控制输出值
	double fSpeedControlOutOld;//上次速度控制输出值
	double fHill_Slow_Ratio;//坡道速度减小比例
	//平滑输出
	double fSpeedControlOut;//交给电机速度控制输出---经过平滑输出
	int SPEED_CONTROL_PERIOD;
	int nSpeedControlPeriod;//速度平滑输出控制周期
 
	 
}Speed_Control;

extern Speed_Control Car_Speed;

//方向控制类
typedef struct
{
	//方向控制
	PID Direction_Control;
	int nUseLine[4];//图像使用行
	int nUseLineErr[4];//权值，即更信任Err值大的行
	int nOffSet[4];//图像使用行与中线的偏差
	double fSumOffSet;//通过加权处理获得的总偏差
	double fLastSumOffSet;//上一次总偏差
	 
	 
	double fDirectionControlOutNew;//当前速度控制输出值
	double fDirectionControlOutOld;//上次速度控制输出值
	//平滑输出
	double fDirectionControlOut;//交给电机方向控制输出---经过平滑输出
	int DIRECTION_CONTROL_PERIOD;
	int nDirectionControlPeriod;//方向平滑输出控制周期
	 
	//速度控制限幅
	int nDirectionControlOutMax;
	int nDirectionControlOutMin;
}Direction_Control;

extern Direction_Control Car_Direction;







typedef struct {
    float SetPoint;     //设定值 
    float Proportion;   //比例项 
    float Derivative;   //差分项 
    float Error[20];
    
    float  dis1cm_err_store[25];                //1cm道路偏差
    float  distance_err_max_val;                //最大单位偏差 ,遥控可修改
    float  distance_err_d_max_val;              //最大偏差率(偏差微分),可修改
    float  max_dis_err;                         // 最大偏差变化
    float  max_dis_err_d;                       // 最大偏差变化率 
    
    uint16 mode_select;
} PID_Sever;

extern PID_Sever ServoPID;
 
extern PID Servo_L;
extern PID Servo_R;
extern PID Servo_LC;            //十字
extern PID Servo_RC;
extern PID Servo_L_LOOP;
extern PID Servo_R_LOOP;
extern PID Servo_LOOP_Back;
extern PID Servo_LOOP_Small;
extern PID Sever;
extern PID Servo_ramp;


         

extern double LeftMotorOut,RightMotorOut;   //电机输出量

   
extern double speedSet;
extern int Speed_L;
extern int Speed_R;
extern int dead_r;
extern int dead_l;
extern int16 err2;
extern int16 err;
extern int16 SetAngle;
  
extern double g_fSpeedControlOut;
extern double mP;
extern double mI;
extern double mD;
extern int flag;
extern int16 baseServoPwm;


extern double g_fSpeedControlOutNew;
extern double g_fSpeedControlOutOld;

extern int g_nSpeedControlPeriod;
extern int SPEED_CONTROL_PERIOD;

extern double Gyro_Ratio;
extern double DIRECTION_CONTROL_P;
extern double DIRECTION_CONTROL_D;
extern double g_fDirectionControlOut;

extern double g_fDirectionControlOutNew;
extern double g_fDirectionControlOutOld;

extern int g_nDirectionControlPeriod;
extern int DIRECTION_CONTROL_PERIOD;


extern int g_nUseLine[5];
extern int g_nUseLineErr[4];
extern int g_nOffSet[5];
extern double g_fSumOffSet;
extern int offset;

 
extern void AngleControl(void);
extern void SpeedControl();
extern void SpeedControlOutput();
extern int MotorSpeedOut(double anglePWM,double dirPWM );
extern void DirectionControl();
extern void DirectionControlOutput();

#endif