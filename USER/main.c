//���ߣ�����
//������Ŀ�ģ�2017������ʡ���и߼���-���泵����
//ʱ�䣺2017.11.4



#include "sys.h"
#include "delay.h"
#include "dbus.h"
#include "led.h"
#include "can.h"
#include "usart.h"
#include "timer.h"
#include "Mydefine.h"
#include "pid.h"
#include "task.h"


int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);    //��ʼ����ʱ����
	
	//PID��ʼ��
	PID_Init(&CM1_Speed_Pid,AM_SPEEDPID_P,AM_SPEEDPID_I,AM_SPEEDPID_D,AM_SPEEDPID_DEAD,AM_SPEEDPID_MER,AM_SPEEDPID_I_MAX,AM_SPEEDPID_MAXOUT);
	PID_Init(&CM2_Speed_Pid,AM_SPEEDPID_P,AM_SPEEDPID_I,AM_SPEEDPID_D,AM_SPEEDPID_DEAD,AM_SPEEDPID_MER,AM_SPEEDPID_I_MAX,AM_SPEEDPID_MAXOUT);
	PID_Init(&CM3_Speed_Pid,AM_SPEEDPID_P,AM_SPEEDPID_I,AM_SPEEDPID_D,AM_SPEEDPID_DEAD,AM_SPEEDPID_MER,AM_SPEEDPID_I_MAX,AM_SPEEDPID_MAXOUT);
	PID_Init(&CM4_Speed_Pid,AM_SPEEDPID_P,AM_SPEEDPID_I,AM_SPEEDPID_D,AM_SPEEDPID_DEAD,AM_SPEEDPID_MER,AM_SPEEDPID_I_MAX,AM_SPEEDPID_MAXOUT);
	
	LED_Init();					//��ʼ��LED 
	Date_Init();
	DBUS_Init();
	CAN1_Init();
	RC_Calibration();	//ң�������ݼ��
	
	CAN2_Init();
	
	TIM5_PWM_Init(20000-1,84-1);	//����20000��20ms
	TIM3_Int_Init(20-1,8400-1);	//��ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����20��Ϊ2ms  
	TIM2_PWM_Init(20000-1,84-1);	//����20000��20ms
 	
	
while(1)
	{
	
//		delay_ms(500);
//		GREEN_LED_OFF(); 
//		delay_ms(500);

	}
}



