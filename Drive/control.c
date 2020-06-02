#include "common.h"
#include "include.h"
#include "math.h"


//------------����ṹ�庯��----------------
SmartCar Smart_Car;
CarControl Car_Control;   //������ �����庯��
Speed_Control Car_Speed;
Direction_Control Car_Direction;
CarMeet Car_Meet;
  

PID_Sever ServoPID;
  PID Servo_L;
  PID Servo_R;
  PID Servo_LC;            //ʮ��
  PID Servo_RC;
  PID Servo_L_LOOP;
  PID Servo_R_LOOP;
  PID Servo_LOOP_Back;
  PID Servo_LOOP_Small;
  PID Sever;
  PID Servo_ramp;

//------------------ģ������------------------
double  Fuzzy_Kp;
double  Fuzzy_Kd;
 

double  LeftMotorOut,RightMotorOut;   //��������



/*!
*  @brief      ���Ʋ�����ʼ��
*  @since      v1.0
*/
void InitControlPara(){
     
  
   Car_Control.Server.P_Meet = 2.7;
   Car_Control.Server.P = 95;//86 ; //4��21�ռ���     //���ø���֮��52  //�ٶ�0.9ʱ��4.9  //�ٶ�1��ʱ��4.96  
   Car_Control.Server.D = 10; 
   Car_Control.Speed.P = 2;//0.5;
   Car_Control.Speed.I = 0.4;
   Car_Control.Speed.D = 0.9;
 
   Car_Speed.MotorPwmfR = 1.5 ;
   Car_Speed.MotorPwmChangeR = 1;
   Car_Speed.MotorPwmChangeL = 1;
   Car_Speed.err_R = 0;
   Car_Speed.err_R_1 = 0;
   Car_Speed.err_R_2 = 0;
   Car_Speed.err_L = 0;
   Car_Speed.err_L_1 = 0;
   Car_Speed.err_L_2 = 0;
   Car_Speed.MotorPwmfL_int = 0;
   Car_Speed.MotorPwmfR_int = 0;
   Car_Speed.fSetSpeed = 2;
   Car_Speed.fCarSpeed = 0;
   Car_Speed.fMaxSpeed = 4;
  
  //--------����P-------
   Car_Speed.chasu_P = 0.7;
   Car_Speed.CarSpeed_L = 0;
   Car_Speed.CarSpeed_R = 0;
        
 
   Car_Control.LeftAngle =  1730;//870;   //����ǰ�����һ�����������󣬼�С���� 
      
   Car_Control.baseServoPwm = 1544;//730;//678; 
   
   Car_Control.RightAngle = 1380;// 465; 
   
   Car_Control.SetAngle = 0;
   Car_Control.SetAngle_Last = 0;
    
   Car_Direction.fDirectionControlOutNew = 0;
   Car_Direction.fDirectionControlOutOld = 0;
   Car_Direction.fDirectionControlOut = 0;
     
   Car_Meet.A_Car_Stop =0;
   
   
   Road_Info.On_Circle_Flag = 0;
   Road_Info.In_Circle_Flag = 0;
   Road_Info.Out_Circle_Flag = 0;
   
   //----------ģ�����Ʋ���-----------
   Servo_L.P = 0.012;
   Servo_L.I = 5.3;
   Servo_L.D = 0.066;
   
   Servo_R.P = 0.012 ;
   Servo_R.I = 5.3;  //�ӵ�ģ�����ƺ��Servo_P
   Servo_R.D = 0.066;
   
   Servo_L_LOOP.P = 0.25;
   Servo_L_LOOP.I = 0;
   Servo_L_LOOP.D = 0.75;
   
   Servo_R_LOOP.P = 0.25;
   Servo_R_LOOP.I = 0;
   Servo_R_LOOP.D = 0.75;
   
   Road_Info.E_offset = 770;
   
  
   Car_Meet.Nrf_Flag == 0;
   Car_Meet.Receive = 0;
   Car_Meet.Receive_wupan = 0;
   
   Car_Meet.K60_coming_meet = 0;
   
    
   
   Car_E.flag = 3;
   Car_Meet.K60_meet_over_distance_flag = 0;
   Car_Meet.k60_distance_flag = 0;
   Car_Meet.K60_length = 0;
   Car_Meet.K60_length = 1;
    
    
    Car_Meet.k60_shibie=1;
    Car_Meet.k60_shibie_length=0;
    Car_Meet.k60_ready_flag = 0;
   
    Car_Control.Car_A_Stop = 0;
    
    
   Road_Info.side = 0.5;
   
     
}
 
float UartSetAngle;
int judge ;   //���ڵ�������ֱ��Dֵ��һ�� 
  
   //Car_Control.baseServoPwm = 800;//500;
  // Car_Control.LeftAngle = 990;//600;   //����ǰ�����һ�����������󣬼�С���� 
  // Car_Control.RightAngle = 610;//400;
 
int16 UFF_L[7] = { 0,15,30,50,80,120,130   };
/*�����˵������ӵĹ����,�����ʱƫ�����,С���ʱƫ�����仯*/

float Fuzzy_L(float P, float D)/*ģ����������*/
{
     float PFF[4] = {  0,10,20,30 };
    /*������D����ֵ������*/
    float DFF[4] = { 0,5,9,14};
    /*�����U����ֵ������*/
    //int UFF_L[7] = { 0,150,300,450,600,750,900 };
    /*�����˵������ӵĹ����,�����ʱƫ�����,С���ʱƫ�����仯*/
    /*a0=0.3,a1=0.55,a2=0.74,a3=0.89*/
  /*  int rule[7][7] = {
//�仯�� -3,-2,-1, 0, 1, 2, 3 //���
        { -6,-6,-6,-5,-5,-5,-4, },  //-3
        { -5,-4,-4,-3,-2,-2,-1, },  //-2 
        { -4,-3,-2,-2, 0, 1, 2, },  //-1 
        { -4,-3,-1, 1, 1, 3, 4, },  //0 
        { -2,-1, 0, 2, 2, 3, 4, },  //1 
        { 1, 2, 2, 3, 4, 4, 5, },  //2 
        { 4, 5, 5, 5, 6, 6, 6 } };  //3
    */ 
      
      int rule[7][7] = {                         //��һ��ģ������
//�仯�� -3,-2,-1, 0, 1, 2, 3 //���
        { -6,-6,-6,-5,-5,-5,-4, },  //-3
        { -6,-4,-3,-3,-2,-2,-1, },  //-2 
        { -4,-3,-2,-2, 0, 1, 2, },  //-1 
        { -4,-3,-1, 1, 1, 3, 4, },  //0 
        { -4,-1, 0, 2, 2, 3, 4, },  //1 
        { 1, 2, 2, 3, 4, 4, 5, },  //2 
        { 4, 5, 5, 5, 6, 6, 6 } };  //3
 
 
        
      
    /**********************************************************/
    
    float  U;  /*ƫ��,ƫ��΢���Լ����ֵ�ľ�ȷ��*/
    float PF[2], DF[2], UF[4];  /*ƫ��,ƫ��΢���Լ����ֵ��������*/
    int Pn, Dn, Un[4];
    float temp1, temp2;
    
    /*�����ȵ�ȷ��*/
    /*����PD��ָ������ֵ�����Ч������*/
    if (P>-PFF[3] && P<PFF[3])
    {
        if (P <= -PFF[2])
        {
            Pn = -2;
            PF[0] = (-PFF[2] - P) / (PFF[3] - PFF[2]);
        }
        else if (P <= -PFF[1])
        {
            Pn = -1;
            PF[0] = (-PFF[1] - P) / (PFF[2] - PFF[1]);
        }
        else if (P <= PFF[0])
        {
            Pn = 0;
            PF[0] = (-PFF[0] - P) / (PFF[1] - PFF[0]);
        }
        else if (P <= PFF[1])
        {
            Pn = 1; PF[0] = (PFF[1] - P) / (PFF[1] - PFF[0]);
        }
        else if (P <= PFF[2])
        {
            Pn = 2; PF[0] = (PFF[2] - P) / (PFF[2] - PFF[1]);
        }
        else if (P <= PFF[3])
        {
            Pn = 3; PF[0] = (PFF[3] - P) / (PFF[3] - PFF[2]);
        }
    }
    else if (P <= -PFF[3])
    {
        Pn = -2; PF[0] = 1;
    }
    else if (P >= PFF[3])
    {
        Pn = 3; PF[0] = 0;
    }
    PF[1] = 1 - PF[0];
    if (D>-DFF[3] && D<DFF[3])
    {
        if (D <= -DFF[2])
        {
            Dn = -2;DF[0] = (-DFF[2] - D) / (DFF[3] - DFF[2]);
        }
        else if (D <= -DFF[1])
        {
            Dn = -1;
            DF[0] = (-DFF[1] - D) / (DFF[2] - DFF[1]);
        }
        else if (D <= DFF[0])
        {
            Dn = 0;
            DF[0] = (-DFF[0] - D) / (DFF[1] - DFF[0]);
        }
        else if (D <= DFF[1])
        {
            Dn = 1;
            DF[0] = (DFF[1] - D) / (DFF[1] - DFF[0]);
        }
        else if (D <= DFF[2])
        {
            Dn = 2; DF[0] = (DFF[2] - D) / (DFF[2] - DFF[1]);
        }
        else if (D <= DFF[3])
        {
            Dn = 3; DF[0] = (DFF[3] - D) / (DFF[3] - DFF[2]);
        }
    }
    else if (D <= -DFF[3])
    {
        Dn = -2;
        DF[0] = 1;
    }
    else if (D >= DFF[3])
    {
        Dn = 3;
        DF[0] = 0;
    }
    DF[1] = 1 - DF[0];
    /*ʹ����Χ�Ż���Ĺ����rule[7][7]*/
    /*���ֵʹ��13����������,����ֵ��UFF_L[7]ָ��*/
    /*һ�㶼���ĸ�������Ч*/
    Un[0] = rule[Pn - 1 + 3][Dn - 1 + 3];
    Un[1] = rule[Pn + 3][Dn - 1 + 3];
    Un[2] = rule[Pn - 1 + 3][Dn + 3];
    Un[3] = rule[Pn + 3][Dn + 3];
    if (PF[0] <= DF[0])
        UF[0] = PF[0];
    else
        UF[0] = DF[0];
    if (PF[1] <= DF[0])
        UF[1] = PF[1];
    else
        UF[1] = DF[0];
    if (PF[0] <= DF[1])
        UF[2] = PF[0];
    else
        UF[2] = DF[1];
    if (PF[1] <= DF[1])
        UF[3] = PF[1];
    else
        UF[3] = DF[1];
    /*ͬ���������������ֵ���*/
    
    if (Un[0] == Un[1])
    {
        if (UF[0]>UF[1])
            UF[1] = 0;
        else
            UF[0] = 0;
    }
    if (Un[0] == Un[2])
    {
        if (UF[0]>UF[2])
            UF[2] = 0;
        else
            UF[0] = 0;
    }
    if (Un[0] == Un[3])
    {
        if (UF[0]>UF[3])
            UF[3] = 0;
        else
            UF[0] = 0;
    }
    if (Un[1] == Un[2])
    {
        if (UF[1]>UF[2])
            UF[2] = 0;
        else
            UF[1] = 0;
    }
    if (Un[1] == Un[3])
    {
        if (UF[1]>UF[3])
            UF[3] = 0;
        else
            UF[1] = 0;
    }
    if (Un[2] == Un[3])
    {
        if (UF[2]>UF[3])
            UF[3] = 0;
        else
            UF[2] = 0;
    }
    
    /*���ķ���ģ��*/
    /*Un[]ԭֵΪ�������������ţ�ת��Ϊ��������ֵ*/
    if (Un[0] >= 0)
        Un[0] = UFF_L[Un[0]];
    else
        Un[0] = -UFF_L[-Un[0]];
    if (Un[1] >= 0)
        Un[1] = UFF_L[Un[1]];
    else
        Un[1] = -UFF_L[-Un[1]];
    if (Un[2] >= 0)
        Un[2] = UFF_L[Un[2]];
    else
        Un[2] = -UFF_L[-Un[2]];
    if (Un[3] >= 0)
        Un[3] = UFF_L[Un[3]];
    else
        Un[3] = -UFF_L[-Un[3]];
    
    temp1 = UF[0] * Un[0] + UF[1] * Un[1] + UF[2] * Un[2] + UF[3] * Un[3];
    temp2 = UF[0] + UF[1] + UF[2] + UF[3];
    U = temp1 / temp2;
    return U;
}

int16 UFF_R[7] = {0,15,30,50,80,120,130   };
/*�����˵������ӵĹ����,�����ʱƫ�����,С���ʱƫ�����仯*/

float Fuzzy_R(float P, float D)/*ģ����������*/
{
  //-330 -180 -90 0 90 180 330    float PFF[4] = { 100,166,232,300 };
    float PFF[4] = { 0,10,20,30 };
    /*������D����ֵ������*/
    float DFF[4] = { 0,5,9,14};
    /*�����U����ֵ������*/
    //int UFF_R[7] = { 0,150,300,450,600,750,900 };
    /*�����˵������ӵĹ����,�����ʱƫ�����,С���ʱƫ�����仯*/
    /*a0=0.3,a1=0.55,a2=0.74,a3=0.89*/
 /*  int rule[7][7] = {
//�仯�� -3,-2,-1, 0, 1, 2, 3 //     ���
        { -6,-6,-6,-5,-5,-5,-4, },  //-3
        { -5,-4,-4,-3,-2,-2,-1, },  //-2 
        { -4,-3,-2,-2, 0, 1, 2, },  //-1 
        { -4,-3,-1, 1, 1, 3, 4, },  //0 
        { -2,-1, 0, 2, 2, 3, 4, },  //1 
        { 1, 2, 2, 3, 4, 4, 5, },  //2 
        { 4, 5, 5, 5, 6, 6, 6 } };  //3
    
  */
    /*
   int rule[7][7] = {                         //��2��ģ������
//�仯�� -3,-2,-1, 0, 1, 2, 3 //���
        { -6,-6,-6,-5,-5,-5,-4, },  //-3
        { -6,-4,-3,-3,-2,-2,-1, },  //-2 
        { -4,-3,-2,-2, 0, 1, 2, },  //-1 
        { -4,-3,-1, 1, 1, 3, 4, },  //0 
        { -2,-1, 0, 2, 2, 3, 4, },  //1 
        { 1, 2, 2, 3, 4, 4, 5, },  //2 
        { 3, 5, 5, 5, 6, 6, 6 } };  //3
   
   */
         int rule[7][7] = {                         //��2��ģ������
//�仯�� -3,-2,-1, 0, 1, 2, 3 //���
         { -6,-6,-6,-5,-5,-5,-4, },  //-3
        { -6,-4,-3,-3,-2,-2,-1, },  //-2 
        { -4,-3,-2,-2, 0, 1, 2, },  //-1 
        { -4,-3,-1, 1, 1, 3, 4, },  //0 
        { -4,-1, 0, 2, 2, 3, 4, },  //1 
        { 1, 2, 2, 3, 4, 4, 5, },  //2 
        { 4, 5, 5, 5, 6, 6, 6 } };  //3
   
    /**********************************************************/
    
    float  U;  /*ƫ��,ƫ��΢���Լ����ֵ�ľ�ȷ��*/
    float PF[2], DF[2], UF[4];  /*ƫ��,ƫ��΢���Լ����ֵ��������*/
    int Pn, Dn, Un[4];
    float temp1, temp2;
    
    /*�����ȵ�ȷ��*/
    /*����PD��ָ������ֵ�����Ч������*/
    if (P>-PFF[3] && P<PFF[3])
    {
        if (P <= -PFF[2])
        {
            Pn = -2;
            PF[0] = (-PFF[2] - P) / (PFF[3] - PFF[2]);
        }
        else if (P <= -PFF[1])
        {
            Pn = -1;
            PF[0] = (-PFF[1] - P) / (PFF[2] - PFF[1]);
        }
        else if (P <= PFF[0])
        {
            Pn = 0;
            PF[0] = (-PFF[0] - P) / (PFF[1] - PFF[0]);
        }
        else if (P <= PFF[1])
        {
            Pn = 1; PF[0] = (PFF[1] - P) / (PFF[1] - PFF[0]);
        }
        else if (P <= PFF[2])
        {
            Pn = 2; PF[0] = (PFF[2] - P) / (PFF[2] - PFF[1]);
        }
        else if (P <= PFF[3])
        {
            Pn = 3; PF[0] = (PFF[3] - P) / (PFF[3] - PFF[2]);
        }
    }
    else if (P <= -PFF[3])
    {
        Pn = -2; PF[0] = 1;
    }
    else if (P >= PFF[3])
    {
        Pn = 3; PF[0] = 0;
    }
    PF[1] = 1 - PF[0];
    if (D>-DFF[3] && D<DFF[3])
    {
        if (D <= -DFF[2])
        {
            Dn = -2;DF[0] = (-DFF[2] - D) / (DFF[3] - DFF[2]);
        }
        else if (D <= -DFF[1])
        {
            Dn = -1;
            DF[0] = (-DFF[1] - D) / (DFF[2] - DFF[1]);
        }
        else if (D <= DFF[0])
        {
            Dn = 0;
            DF[0] = (-DFF[0] - D) / (DFF[1] - DFF[0]);
        }
        else if (D <= DFF[1])
        {
            Dn = 1;
            DF[0] = (DFF[1] - D) / (DFF[1] - DFF[0]);
        }
        else if (D <= DFF[2])
        {
            Dn = 2; DF[0] = (DFF[2] - D) / (DFF[2] - DFF[1]);
        }
        else if (D <= DFF[3])
        {
            Dn = 3; DF[0] = (DFF[3] - D) / (DFF[3] - DFF[2]);
        }
    }
    else if (D <= -DFF[3])
    {
        Dn = -2;
        DF[0] = 1;
    }
    else if (D >= DFF[3])
    {
        Dn = 3;
        DF[0] = 0;
    }
    DF[1] = 1 - DF[0];
    /*ʹ����Χ�Ż���Ĺ����rule[7][7]*/
    /*���ֵʹ��13����������,����ֵ��UFF_R[7]ָ��*/
    /*һ�㶼���ĸ�������Ч*/
    Un[0] = rule[Pn - 1 + 3][Dn - 1 + 3];
    Un[1] = rule[Pn + 3][Dn - 1 + 3];
    Un[2] = rule[Pn - 1 + 3][Dn + 3];
    Un[3] = rule[Pn + 3][Dn + 3];
    if (PF[0] <= DF[0])
        UF[0] = PF[0];
    else
        UF[0] = DF[0];
    if (PF[1] <= DF[0])
        UF[1] = PF[1];
    else
        UF[1] = DF[0];
    if (PF[0] <= DF[1])
        UF[2] = PF[0];
    else
        UF[2] = DF[1];
    if (PF[1] <= DF[1])
        UF[3] = PF[1];
    else
        UF[3] = DF[1];
    /*ͬ���������������ֵ���*/
    
    if (Un[0] == Un[1])
    {
        if (UF[0]>UF[1])
            UF[1] = 0;
        else
            UF[0] = 0;
    }
    if (Un[0] == Un[2])
    {
        if (UF[0]>UF[2])
            UF[2] = 0;
        else
            UF[0] = 0;
    }
    if (Un[0] == Un[3])
    {
        if (UF[0]>UF[3])
            UF[3] = 0;
        else
            UF[0] = 0;
    }
    if (Un[1] == Un[2])
    {
        if (UF[1]>UF[2])
            UF[2] = 0;
        else
            UF[1] = 0;
    }
    if (Un[1] == Un[3])
    {
        if (UF[1]>UF[3])
            UF[3] = 0;
        else
            UF[1] = 0;
    }
    if (Un[2] == Un[3])
    {
        if (UF[2]>UF[3])
            UF[3] = 0;
        else
            UF[2] = 0;
    }
    
    /*���ķ���ģ��*/
    /*Un[]ԭֵΪ�������������ţ�ת��Ϊ��������ֵ*/
    if (Un[0] >= 0)
        Un[0] = UFF_R[Un[0]];
    else
        Un[0] = -UFF_R[-Un[0]];
    if (Un[1] >= 0)
        Un[1] = UFF_R[Un[1]];
    else
        Un[1] = -UFF_R[-Un[1]];
    if (Un[2] >= 0)
        Un[2] = UFF_R[Un[2]];
    else
        Un[2] = -UFF_R[-Un[2]];
    if (Un[3] >= 0)
        Un[3] = UFF_R[Un[3]];
    else
        Un[3] = -UFF_R[-Un[3]];
    
    temp1 = UF[0] * Un[0] + UF[1] * Un[1] + UF[2] * Un[2] + UF[3] * Un[3];
    temp2 = UF[0] + UF[1] + UF[2] + UF[3];
    U = temp1 / temp2;
    return U;
}

 
static float abs_self(float in){

  if(in > 0)
    return in;
  else
    return -in;
  
}


void servo_init(void)
{
  uint8 i = 0;

  ServoPID.Proportion = 0.00;
  ServoPID.SetPoint = 0.00;
  ServoPID.Derivative = 0.00;
  for(i=0;i<20;i++)
  {
      ServoPID.Error[i] = 0.0;
  }
   
}
   

          //          Deviation[4]                 3
 
int PID_ServoAngle(float Angle, unsigned char Radius) {   
    int OUT_Steer=0;   //������������
    int i;
    
    ServoPID.Error[0] = Angle;           // ƫ��
    for(i = 19; i > 0; i --)
        ServoPID.Error[i] = ServoPID.Error[i-1];
    
    if(ServoPID.Error[0] < 0)
    {
        switch (Radius) {
        case 1:
            ServoPID.Proportion = abs_self(  Fuzzy_R(     -ServoPID.Error[0],(ServoPID.Error[9]-ServoPID.Error[0])    )      )*Servo_R.P + Servo_R.I;
            ServoPID.Derivative = abs_self(  Fuzzy_R(     -ServoPID.Error[0],(ServoPID.Error[9]-ServoPID.Error[0])    )      )*Servo_R.D;
            
            // ServoPID.Derivative = Servo_R.D;
            break;
        case 2: 
            ServoPID.Proportion = abs_self(Fuzzy_R(-ServoPID.Error[0],(ServoPID.Error[9]-ServoPID.Error[0])))*Servo_RC.P/1000 + Servo_R.I;
            ServoPID.Derivative = Servo_RC.D;
            break;
        case 3: //Բ��
            ServoPID.Proportion = abs_self(Fuzzy_R(-ServoPID.Error[0],(ServoPID.Error[9]-ServoPID.Error[0])))*Servo_R_LOOP.P/10 + Servo_R.I;
            ServoPID.Derivative = Servo_R_LOOP.D;
            break;
        case 4: //Բ����
            ServoPID.Proportion = abs_self(Fuzzy_R(-ServoPID.Error[0],(ServoPID.Error[9]-ServoPID.Error[0])))*Servo_LOOP_Back.P/1000 + Servo_R.I;
            ServoPID.Derivative = Servo_R.D;
            break;
        case 5: //Բ����xiao
            ServoPID.Proportion = abs_self(Fuzzy_R(-ServoPID.Error[0],(ServoPID.Error[9]-ServoPID.Error[0])))*Servo_LOOP_Small.P/1000 + Servo_R.I;
            ServoPID.Derivative = Servo_R.D;
            break;
        case 6: //�µ�
            ServoPID.Proportion = Servo_ramp.P;
            ServoPID.Derivative = Servo_R.D;
            break;
        }
    }
    else
    {
        switch (Radius) {
        case 1:
            ServoPID.Proportion = abs_self(Fuzzy_L(   ServoPID.Error[0],(   ServoPID.Error[0] - ServoPID.Error[9]   )    )   )*Servo_L.P + Servo_L.I;
            ServoPID.Derivative = abs_self(Fuzzy_L(   ServoPID.Error[0],(   ServoPID.Error[0] - ServoPID.Error[9]   )    )   )*Servo_L.D;
           // ServoPID.Derivative = Servo_L.D;
            break;
        case 2: 
            ServoPID.Proportion = abs_self(Fuzzy_L(ServoPID.Error[0],(ServoPID.Error[0]-ServoPID.Error[9])))*Servo_LC.P/10 + Servo_L.I;
            ServoPID.Derivative = Servo_LC.D;
            break;
        case 3: //Բ��
            ServoPID.Proportion = abs_self(Fuzzy_L(ServoPID.Error[0],(ServoPID.Error[0]-ServoPID.Error[9])))*Servo_L_LOOP.P/10 + Servo_L.I;
            ServoPID.Derivative = Servo_L_LOOP.D;
            break;
        case 4: //Բ�����
            ServoPID.Proportion = abs_self(Fuzzy_L(ServoPID.Error[0],(ServoPID.Error[0]-ServoPID.Error[9])))*Servo_LOOP_Back.P/1000 + Servo_L.I;
            ServoPID.Derivative = Servo_L.D;
            break;
        case 5: //Բ����С
            ServoPID.Proportion = abs_self(Fuzzy_L(ServoPID.Error[0],(ServoPID.Error[0]-ServoPID.Error[9])))*Servo_LOOP_Small.P/1000 + Servo_L.I;
            ServoPID.Derivative = Servo_L.D;
            break;
        case 6: //�µ�
            ServoPID.Proportion = Servo_ramp.P;
            ServoPID.Derivative = Servo_L.D;
            break;
        }
    }
    
    OUT_Steer =  (int)(ServoPID.Proportion * ServoPID.Error[0]       // ������
            + ServoPID.Derivative * (ServoPID.Error[0]-ServoPID.Error[9]));      // ΢����
   
   
    return OUT_Steer;
}



 
 
/*!
*  @brief      �������
*  @since      v1.0
*/
 void DirectionControl()
{  
    
  //--------------2018-6-9����---��ſ���ϵͳ--//5.3-----  
   
  
  if(Smart_Car.Status == Stop || Car_Meet.kaishihuiche){
  
  Car_Control.SetAngle = Car_Control.baseServoPwm + 3;
  }
   
  
  else if(   Road_Info.zhuanxiang){
      Car_Control.SetAngle = Car_Control.baseServoPwm + Car_Control.Server.P*chazhi+ Car_Control.Server.D*(Car_Control.SetAngle - Car_Control.SetAngle_Last);
      
       
  }
  else if(Car_Meet.K60_coming_meet){
    
    if(Car_Meet.fasong ){
          Car_Control.SetAngle = Car_Control.baseServoPwm;
    }
  else
     Car_Control.SetAngle = Car_Control.baseServoPwm + Car_Control.Server.P_Meet*chazhi+ Car_Control.Server.D*(Car_Control.SetAngle - Car_Control.SetAngle_Last);
  
  }
   
    
  else   {
    
   Car_Control.SetAngle =  Car_Control.baseServoPwm + PID_ServoAngle(  Deviation[4] , 1 ) ;  
   
    // Car_Control.SetAngle = Car_Control.baseServoPwm + (Car_E.Parallel_New + 0.01*Car_E.Vertical)*Car_Control.Server.P*chazhi+ Car_Control.Server.D*(Car_Control.SetAngle - Car_Control.SetAngle_Last) ;
  }
  
  
    //----------------��ֹת�ǳ�����Χ--------------
    if(Car_Control.SetAngle > Car_Control.LeftAngle){
        Car_Control.SetAngle = Car_Control.LeftAngle;
      }
     
      if(Car_Control.SetAngle < Car_Control.RightAngle){
        Car_Control.SetAngle = Car_Control.RightAngle;
      }
       
      UartSetAngle = (float)Car_Control.SetAngle;
      
   
      ftm_pwm_duty(FTM3, FTM_CH3, Car_Control.SetAngle);        
      
      Car_Control.SetAngle_Last = Car_Control.SetAngle;
      
       
 }
 
  
double  Speed_Turn_Out(double turn_out)    //�ٶȿ�������˲�      
{
  double Turn_Out_Filtered; 
  static double Pre1_Error[4]; 
  Pre1_Error[3]=Pre1_Error[2];
  Pre1_Error[2]=Pre1_Error[1];
  Pre1_Error[1]=Pre1_Error[0];
  Pre1_Error[0]=turn_out;
  Turn_Out_Filtered=Pre1_Error[0]*0.4+Pre1_Error[1]*0.3+Pre1_Error[2]*0.2+Pre1_Error[3]*0.1;
  return Turn_Out_Filtered;
} 
  
  
/*!
*  @brief      �ٶȻ��
*  @since      v1.0
*/
float old_speed;
void GetSpeed()
{
  
         Car_Speed.nLeftMotorSpeed =   ftm_quad_get(FTM1);//��ȡFTM2 �������� ��������(������ʾ������)
	 ftm_quad_clean(FTM1);
	  
	 Car_Speed.nRightMotorSpeed = -ftm_quad_get(FTM2);//��ȡFTM2 �������� ��������(������ʾ������)
	 ftm_quad_clean(FTM2);
        
         Car_Speed.xL += Car_Speed.nLeftMotorSpeed;
         Car_Speed.xR += Car_Speed.nRightMotorSpeed;
        
       /*  if(Car_Meet.K60_meet_over_distance_flag == 1){   
         
              Car_Meet.K60_meet_over_distance += Car_Speed.nLeftMotorSpeed ;
           
           
                           if(  Car_Meet.K60_meet_over_distance  >  10000){      
                                
                             Car_Meet.K60_meet_over_distance_flag = 0 ;
                             Car_Meet.K60_meet_over_distance = 0;
                           
                            } 
         }*/
         
         
         if(Road_Info.Ring_flag){
         
            Road_Info.Ring_distance += (Car_Speed.nLeftMotorSpeed + Car_Speed.nRightMotorSpeed)/2;
            if(Road_Info.Ring_distance < 6000){
            
              
              Road_Info.Ring_diatance_flag = 2;
            }
           
           
         
         }
         
         if(Car_Meet.kaishihuiche){    //��ʼ����  ������k60����ֵ
         
           Car_Meet.kaishihuiche_diatance += Car_Speed.nLeftMotorSpeed;
           if(Car_Meet.kaishihuiche_diatance >4210){
           Car_Meet.kaishihuiche_diatance = 0;
           Car_Meet.kaishihuiche = 0;
       //    Car_Meet.k60_shibie = 0;
           Car_Meet.k60_shibie = 1;
            Car_Meet.over = 1;  
            
            
            
            Car_Meet.K60_coming_meet = 0;
           
             
           }
           
           
           if(Car_Meet.kaishihuiche_diatance < 7000){
           
             Car_Meet.k60_shibie = 0;
           
           }
           
         
         }
         
         if(Car_Meet.K60_coming_meet == 1 ){   // ����ᳵ����־λ    
                                               //k60�����лᳵ�����û���жϵ���̬��־λ���� 4500 ����ͣ 
                     Car_Meet.K60_length += Car_Speed.nRightMotorSpeed;
                      
                                   if(Car_Meet.K60_length >= 4400){
                                   
                                     Car_Meet.K60_length = 0;
                                      
                                     Car_Meet.k60_distance_flag = 0;
                                     Car_Meet.k60_distance_flag_stop = 1;
                                   } 
         
         }
         
         if(  ! Car_Meet.k60_shibie)
         {  
            Car_Meet.k60_shibie_length+= Car_Speed.nLeftMotorSpeed;
             if( Car_Meet.k60_shibie_length >= 20000)     // ��main������ �ڵ�һ��ʶ��ᳵ�����ֹ����ͷʶ�� 60000��� ������Ծ�����ʶȣ�
             {
                  Car_Meet.k60_shibie_length=0; 
                  Car_Meet.k60_shibie= 1;
             }
              
         }
         
         
         if( Car_Meet.over == 3){
         
           Car_Meet.over_distance +=   Car_Speed.nRightMotorSpeed;
           if(Car_Meet.over_distance > 6200){
           
             Smart_Car.CarStop();
             Car_Meet.over_distance = 0;
             Car_Control.Car_A_Stop = 0 ;
              Car_Meet.over_flag = 1;
              }
            
         
         }
           

         Car_Speed.CarSpeed_L = Car_Speed.nLeftMotorSpeed  / 11.608;
         Car_Speed.CarSpeed_R = Car_Speed.nRightMotorSpeed  / 11.608;
	    
 
//����һ�ף����5804������
	// Smart_Car.Distance += Car_Speed.nRightMotorSpeed  / 5804;
        
//�ٶ�1ms�ɼ�һ�� 5804 * 0.001 = 5.804
    //   Car_Speed.fCarSpeed = Car_Speed.fCarSpeed*0.1 + ((Car_Speed.nRightMotorSpeed ) / 11.608)*0.9;
         Car_Speed.fCarSpeed = Car_Speed.fCarSpeed*0.1 + (( Car_Speed.nRightMotorSpeed  + Car_Speed.nLeftMotorSpeed) /11.608)*0.9;  //��ó�ģ�ٶ�	        
   
         if(Car_Speed.fCarSpeed > Car_Speed.fMaxSpeed){
           Car_Speed.fCarSpeed = Car_Speed.fMaxSpeed;
         }
        
}

 
/*!
*  @  brief      �ٶȿ���
*  @since      v1.0
*/
void SpeedControl()
{   
  
  
  
  if(Car_Meet.kaishihuiche){
     
      Car_Speed.fSetSpeed = 0.7;
      
    }
  
  
  
      //------------------------�������Ʋ���--------------------
      Car_Speed.err_L = Car_Speed.fSetSpeed - Car_Speed.fCarSpeed;
        
      Car_Speed.MotorPwmChangeL =  Car_Control.Speed.P * (Car_Speed.err_L - Car_Speed.err_L_1) +  Car_Control.Speed.I * Car_Speed.err_L + 
      Car_Control.Speed.D*( Car_Speed.err_L + Car_Speed.err_L_2 - 2*Car_Speed.err_L_1);  //PID��ʽ����
       
      Car_Speed.MotorPwmfL += Car_Speed.MotorPwmChangeL;
       
      Car_Speed.MotorPwmfL = Car_Speed.MotorPwmfL*0.9 + Car_Speed.MotorPwmfL_old*0.1;  //���ֵ��ͨ�˲�
 
      Car_Speed.MotorPwmfL_old = Car_Speed.MotorPwmfL;
    
     if(Car_Speed.MotorPwmfL > Car_Speed.fMaxSpeed )
        Car_Speed.MotorPwmfL = Car_Speed.fMaxSpeed ;
       
     if (Car_Speed.MotorPwmfL < -Car_Speed.fMaxSpeed) {
        
	 Car_Speed.MotorPwmfL = -Car_Speed.fMaxSpeed;
	}
     
      Car_Speed.err_L_2 = Car_Speed.err_L_1;
      Car_Speed.err_L_1 = Car_Speed.err_L;
  
      Car_Speed.MotorPwmfL_int = (int)(  Car_Speed.MotorPwmfL/Car_Speed.fMaxSpeed * 9900   ) ;  //������ֵ
 
      
      
      
      
      //--------------------------�ҵ�����Ʋ���-----------------------------
      Car_Speed.err_R = Car_Speed.fSetSpeed - Car_Speed.fCarSpeed;
        
      Car_Speed.MotorPwmChangeR =  Car_Control.Speed.P * (Car_Speed.err_R - Car_Speed.err_R_1) +  Car_Control.Speed.I * Car_Speed.err_R + 
      Car_Control.Speed.D*( Car_Speed.err_R + Car_Speed.err_R_2 - 2*Car_Speed.err_R_1);  //PID��ʽ����
       
      Car_Speed.MotorPwmfR += Car_Speed.MotorPwmChangeR;
        
     Car_Speed.MotorPwmfR = Car_Speed.MotorPwmfR*0.9 + Car_Speed.MotorPwmfR_old*0.1;  //���ֵ��ͨ�˲�
  
     Car_Speed.MotorPwmfR_old = Car_Speed.MotorPwmfR;
    
     if(Car_Speed.MotorPwmfR > Car_Speed.fMaxSpeed )
        Car_Speed.MotorPwmfR = Car_Speed.fMaxSpeed ;
       
     if (Car_Speed.MotorPwmfR < -Car_Speed.fMaxSpeed) {
        
	 Car_Speed.MotorPwmfR = -Car_Speed.fMaxSpeed;
	}
     
      Car_Speed.err_R_2 = Car_Speed.err_R_1;
      Car_Speed.err_R_1 = Car_Speed.err_R;
      
      Car_Speed.MotorPwmfR_int = (int)(  Car_Speed.MotorPwmfR/Car_Speed.fMaxSpeed * 9900   ) ;  //������ֵ
    
      
    //-----------�������޷�------------------  
      
      if(Car_Speed.MotorPwmfL_int > 9900)
      
        Car_Speed.MotorPwmfL_int = 9900;
     
      if(Car_Speed.MotorPwmfL_int < -9900)
      
        Car_Speed.MotorPwmfL_int = -9900;
      
     
     
      if(Car_Speed.MotorPwmfR_int > 9900)
        
        Car_Speed.MotorPwmfR_int = 9900;
     
      if(Car_Speed.MotorPwmfR_int < -9900)
        
        Car_Speed.MotorPwmfR_int = -9900;
    
    
 //---------------���Ƶ��----------------------
      
      if(Car_Speed.MotorPwmfL_int > 0  ){
      
      ftm_pwm_duty(FTM0,FTM_CH1,0);       
      ftm_pwm_duty(FTM0,FTM_CH2,Car_Speed.MotorPwmfL_int);
        
      }
       else{
       
      ftm_pwm_duty(FTM0,FTM_CH1,- Car_Speed.MotorPwmfL_int);       
      ftm_pwm_duty(FTM0,FTM_CH2,0);
       
       }
     
      if(Car_Speed.MotorPwmfR_int > 0    ){
       
       ftm_pwm_duty(FTM0,FTM_CH3,0);       
      ftm_pwm_duty(FTM0,FTM_CH4,Car_Speed.MotorPwmfR_int);
      
      }
    
      else{
      
      ftm_pwm_duty(FTM0,FTM_CH3,- Car_Speed.MotorPwmfR_int);
      ftm_pwm_duty(FTM0,FTM_CH4 ,0); 
      }
     
     
      
      
      
} 
 


/*!
*  @brief      �˲��㷨
*  @since      v1.0
*  @means  
*/
int  Turn_Out_Filter(int turn_out)    //ת���������˲�      
{
  int Turn_Out_Filtered; 
  static int Pre1_Error[4]; 
  Pre1_Error[3]=Pre1_Error[2];
  Pre1_Error[2]=Pre1_Error[1];
  Pre1_Error[1]=Pre1_Error[0];
  Pre1_Error[0]=turn_out;
  Turn_Out_Filtered=Pre1_Error[0]*0.4+Pre1_Error[1]*0.3+Pre1_Error[2]*0.2+Pre1_Error[3]*0.1;
  return Turn_Out_Filtered;
}
    

/*!
*  @brief      ʮ�ֻᳵ 
*  @since      v1.0
*  @means  
*/
 void CarTenMeet(){
       
    if(Car_E.AD[3] + Car_E.AD[4] > 180 && Car_E.AD[3] > 90 && Car_E.AD[4] > 90 ){
        
      if(NRF_RX_Buff[1] == 1){      //A����
        Car_Speed.fSetSpeed = 1.6;
        Car_Meet.Nrf_Flag = 0;
         
      } 
      else{                        //A����
        Car_Speed.fSetSpeed  = 0;
        Car_Meet.B_Car_Stop = 1;
        Car_Meet.Nrf_Flag = 1;
        
      }
    }

} 



/*!
*  @brief      ��··���жϣ����Ľ���
*  @since      v1.0
*  @means  
*/   
int  JudgeRoad(float turn_in)    
{
  int RoadMessage;
  
  
  if(turn_in > 120){
  Road_Info.time ++ ;
    
  }
 
  if(Road_Info.time > 50){
 
  RoadMessage = 1;
  Road_Info.time = 0;
  
  } 
  return RoadMessage;
}
 







 
/*!
*  @brief      �����ж�
*  @since      v1.0
*  @means  
*/
void Road_Judge(){
  
  if( Car_E.AD_ramp[1] >120 &&Car_E.AD_ramp[0]>130 && Car_E.AD_ramp[2] > 130 ){
        
          Road_Info.On_Circle_Flag = 1;
  
  }
  
   // if(Car)
     
   
    
   
    
  
  
}



 
 

 
 



