#include "common.h"
#include "include.h"
 
double Variable[20];

uint8 Uart_Send=0,SendPara,stop_contorl,send_data_contorl=0,SendSD,SD_Save=0,beep=0,SD_Write_info=0;
 
 
float *Control_Para[14] =
{
    &Servo_R.P,
    &Servo_R.I,
    &Servo_R.D,
    &Servo_L.P,
    &Servo_L.I,
    &Servo_L.D,
    &Road_Info.E_offset,
    &Car_Meet.Nrf_Flag,
    &Car_Meet.A_Car_Stop,
    &Road_Info.side,
    
    
};


void my_putchar(char temp)
{
      uart_putchar(UART2,temp); //����ʵ�ʵĴ��ں����޸�
}
/*����֪ͨ��λ���µ�һ�����ݿ�ʼ��Ҫ�������ݱ��뷢����*/
void Send_Begin()
{
  my_putchar(0x55);
  my_putchar(0xaa);
  my_putchar(0xae);
}


 

void Variable_update()
{
 
  
  
  Variable[0] = AD_valu[0]+AD_valu[1]+AD_valu[2];
  Variable[1] = Car_E.AD[0];
  Variable[2] = Car_E.AD[1];
  Variable[3] = 100*( Car_E.AD_ramp[2]-Car_E.AD_ramp[4])/( Car_E.AD[2]+Car_E.AD_ramp[4]) ;
  Variable[4] = 100*( Car_E.AD_ramp[0]-Car_E.AD_ramp[3])/( Car_E.AD[0]+Car_E.AD_ramp[3]) ; 
  
  
  Variable[5] = Car_E.AD[2] ;
  Variable[6] = Road_Info.RightRing;//Car_Meet.kea_coming_flag_qingdiao; 
  Variable[7] = Road_Info.LeftRing;//Car_Meet.k60_distance_flag_stop; 
  Variable[8] = Road_Info.On_Circle_Flag ;//Car_Meet.distance; 
  Variable[9] = Road_Info.lock_Ring ; //Car_Meet.k60_ready_flag;
  
  Variable[10] =  Car_Meet.K60_length  ;
  
  
  Variable[11] =  Car_Meet.k60_distance_flag_stop ;
  Variable[12] =  Car_Speed.fSetSpeed; 
  Variable[13] =  Car_Speed.fCarSpeed ;
  Variable[14] =  Car_Speed.CarSpeed_L ;
  Variable[15] =  Car_Speed.CarSpeed_R ;
  
  
  
  
  
  
}


void Send_Variable(int Variable_num)
{
  uint8 i=0,ch=0;
  float temp=0;  
  my_putchar(0x55);
  my_putchar(0xaa);
  my_putchar(0xad);
  my_putchar(Variable_num);
 for(i=0;i<Variable_num;i++)
  {
    temp=Variable[i];
    ch=BYTE0(temp);
      my_putchar(ch);
    ch=BYTE1(temp);
      my_putchar(ch);
    ch=BYTE2(temp);
      my_putchar(ch);
    ch=BYTE3(temp);
      my_putchar(ch);
  }
     my_putchar(0x0d);
}


 


void Modify_Parameter(uint8 *buff)
{
   uint8 i=0,addr=0;
   float temp;
   uint8 Parameter_num= 10; //14���ɸĲ���
  /*          �޸Ĳ�������         */
   for(i=0;i<Parameter_num;i++)
  {
       BYTE0(temp)=*(uint8*)(buff+addr);
       addr++;
       BYTE1(temp)=*(uint8*)(buff+addr);
       addr++;
       BYTE2(temp)=*(uint8*)(buff+addr);
       addr++;
       BYTE3(temp)=*(uint8*)(buff+addr);
       addr++;
       *Control_Para[i]=temp;
   }
   
}



void Send_Parameter()
{
  uint8 i=0,ch=0;
  float temp=0;
  uint8 Parameter_num=10;  //14���ɸĲ���
  
 
  my_putchar(0x55);
  my_putchar(0xaa);
  my_putchar(0xab);
  my_putchar(Parameter_num);
  for(i=0;i<Parameter_num;i++)
  { 
    temp=*Control_Para[i];
    ch=BYTE0(temp);
    my_putchar(ch);
    ch=BYTE1(temp);
    my_putchar(ch);
    ch=BYTE2(temp);
    my_putchar(ch);
    ch=BYTE3(temp);
    my_putchar(ch);
  }
    my_putchar(0X0b);//֡β
}

 


void UART2_RX_IRQHandler()
{
	static uint8 recv;
	static uint8 data_cnt = 0;
	static uint8 Recv_Buff[100];
	static uint8 Data_Receiving = FALSE, Block_Index_Receiving = FALSE;
	if (uart_query(UART2) == 1)  uart_getchar(UART2, (char*)(&recv));  //����ʵ�ʵĴ������޸�
	/**********�������ڽ���������λ���Ĳ�������*********/
	if (Data_Receiving || Block_Index_Receiving)
	{
		if (Data_Receiving)
		{
			if (data_cnt < 56)
			{
				Recv_Buff[data_cnt] = recv;
				data_cnt++;
			}
			else
			{
				data_cnt = 0;    //�ﵽ֡��
				Data_Receiving = FALSE;
				if (recv == 0xAB)  //֡β
				{
					Modify_Parameter(Recv_Buff);
					SendPara = 1;      //�����ش���ȷ�ϲ����޸����
				}
			}
		}
		if (Block_Index_Receiving)
		{
			if (data_cnt < 1)
			{
				Recv_Buff[data_cnt] = recv;
				data_cnt++;
			}
			else
			{
				Recv_Buff[data_cnt] = recv;
				// BYTE0(Block_Index)=*(uint8*)(Recv_Buff);
				// BYTE1(Block_Index)=*(uint8*)(Recv_Buff+1);
				SendSD = TRUE;

				data_cnt = 0;    //
				//Block_Index_Receiving=FALSE;
			}
		}
	}
	else
	{
		beep = 1;

		switch (recv)         //�жϹ�����
		{
		case 0x30:           //��ģ��ͣ���� �յ����ݺ���Կ���С����������ֹͣ
			if (Smart_Car.Status == Stop)
				Smart_Car.CarStartRun();//Smart_Car.Status = Run;//Car_Speed.nSetSpeed = Car_Speed.nMaxSpeed;
			/***********�����������ֹͣС��������ٶ�����Ϊ�㣬�رյ����PWM��************/
			else
				Smart_Car.CarStop();//Smart_Car.Status = Stop;//Car_Speed.nSetSpeed = 0;
			/***********���������������С����ʹС����������************/
			break;

		case 0x31:           //���ݷ��Ϳ���
			if (send_data_contorl == 0)
				send_data_contorl = 1;
			else
				send_data_contorl = 0;
			/*********�������ݷ���ʱҪռ�ò���CPU��Դ������ʱ�䣬С������ʱ�����鷢�����ݣ����Կ��Թرշ���*********/
			break;

		case 0x32:           //��ȡ����
			if (SendPara == 0)
				SendPara = 1;
			break;
		case 0x33:             //�޸Ĳ���
			Data_Receiving = TRUE;
			data_cnt = 0;
			break;
		case 0x34:             //�������
			//EEPROM_Save();
			break;
		default:           //
			break;
		}
	}
	uart_rx_irq_en(UART2);//ʹ�ܴ��ڽ����жϣ���ֹ�������жϱ��ر�     
}
