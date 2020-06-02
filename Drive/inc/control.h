#ifndef _CONTROL_H_
#define _CONTROL_H_

  
extern float UartSetAngle;  //����ת��

extern int Nrf_Flag;

typedef struct  
{
	//PID���Ʋ���
	float P;//P��������������
	float I;//I���������ֲ���
	float D;//D������΢�ֲ���
       //�������
	double pOut;//�����������
	double iOut;//���ֿ������
	double dOut;//΢�ֿ������
	//�����
	double Out;//PID�����
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
     
     int time;  //ʱ��Ƭ
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
    
    //-----------7��10��
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
  
	int Status;//��ģ״̬   
        void (*CarStartRun)(void);
	void (*CarStop)(void);
        
        double Distance;//��ʻ����
	double RunTime;//����ʱ��
	double CarSpeed;//˲ʱ����
	//ģ��
	//Motor Motor;
	Buzzer Buzzer;
        int DIPswitch;
}SmartCar;
extern SmartCar Smart_Car;



//��� �����
typedef struct Student
{
	PID Server;
        PID Speed;
        
        int baseServoPwm;   //�����ֵ
        int LeftAngle;     //�����
        int RightAngle;    //���ұ�
        int SetAngle;     
        int SetAngle_Last; 
        int Car_A_Stop;
        
}CarControl;

extern struct Student /*CarControl*/ Car_Control;


//�ٶȿ�����
typedef struct 
{
  
  
  
        float DIPspeed;
	 
	double fCarSpeed;//��ʵ���ٶ�
        
        double CarSpeed_L;
        double CarSpeed_R;
        
	float fSetSpeed;    //���趨�ٶ�
        
        double fSafeSpeed;  //�����价���İ�ȫ�ٶ�
        
	double fMaxSpeed; //���趨����ٶ�
	
        double fMinSpeed; //���趨����ٶ�
	 
        int nLeftMotorSpeed;//������������ֵ��ٶ�
	int nRightMotorSpeed;//������������ֵ��ٶ�
        
         
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
        
	 
	double fSpeedControlIntegral;//�ٶȿ��ƻ������
	double fSpeedControlOutNew;//��ǰ�ٶȿ������ֵ
	double fSpeedControlOutOld;//�ϴ��ٶȿ������ֵ
	double fHill_Slow_Ratio;//�µ��ٶȼ�С����
	//ƽ�����
	double fSpeedControlOut;//��������ٶȿ������---����ƽ�����
	int SPEED_CONTROL_PERIOD;
	int nSpeedControlPeriod;//�ٶ�ƽ�������������
 
	 
}Speed_Control;

extern Speed_Control Car_Speed;

//���������
typedef struct
{
	//�������
	PID Direction_Control;
	int nUseLine[4];//ͼ��ʹ����
	int nUseLineErr[4];//Ȩֵ����������Errֵ�����
	int nOffSet[4];//ͼ��ʹ���������ߵ�ƫ��
	double fSumOffSet;//ͨ����Ȩ�����õ���ƫ��
	double fLastSumOffSet;//��һ����ƫ��
	 
	 
	double fDirectionControlOutNew;//��ǰ�ٶȿ������ֵ
	double fDirectionControlOutOld;//�ϴ��ٶȿ������ֵ
	//ƽ�����
	double fDirectionControlOut;//�����������������---����ƽ�����
	int DIRECTION_CONTROL_PERIOD;
	int nDirectionControlPeriod;//����ƽ�������������
	 
	//�ٶȿ����޷�
	int nDirectionControlOutMax;
	int nDirectionControlOutMin;
}Direction_Control;

extern Direction_Control Car_Direction;







typedef struct {
    float SetPoint;     //�趨ֵ 
    float Proportion;   //������ 
    float Derivative;   //����� 
    float Error[20];
    
    float  dis1cm_err_store[25];                //1cm��·ƫ��
    float  distance_err_max_val;                //���λƫ�� ,ң�ؿ��޸�
    float  distance_err_d_max_val;              //���ƫ����(ƫ��΢��),���޸�
    float  max_dis_err;                         // ���ƫ��仯
    float  max_dis_err_d;                       // ���ƫ��仯�� 
    
    uint16 mode_select;
} PID_Sever;

extern PID_Sever ServoPID;
 
extern PID Servo_L;
extern PID Servo_R;
extern PID Servo_LC;            //ʮ��
extern PID Servo_RC;
extern PID Servo_L_LOOP;
extern PID Servo_R_LOOP;
extern PID Servo_LOOP_Back;
extern PID Servo_LOOP_Small;
extern PID Sever;
extern PID Servo_ramp;


         

extern double LeftMotorOut,RightMotorOut;   //��������

   
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