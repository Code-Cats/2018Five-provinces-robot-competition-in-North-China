//作者：余鑫
//程序功能目的：2017华北五省空中高级组-地面车程序
//时间：2017.11.4



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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);    //初始化延时函数
	
	//PID初始化
	PID_Init(&CM1_Speed_Pid,AM_SPEEDPID_P,AM_SPEEDPID_I,AM_SPEEDPID_D,AM_SPEEDPID_DEAD,AM_SPEEDPID_MER,AM_SPEEDPID_I_MAX,AM_SPEEDPID_MAXOUT);
	PID_Init(&CM2_Speed_Pid,AM_SPEEDPID_P,AM_SPEEDPID_I,AM_SPEEDPID_D,AM_SPEEDPID_DEAD,AM_SPEEDPID_MER,AM_SPEEDPID_I_MAX,AM_SPEEDPID_MAXOUT);
	PID_Init(&CM3_Speed_Pid,AM_SPEEDPID_P,AM_SPEEDPID_I,AM_SPEEDPID_D,AM_SPEEDPID_DEAD,AM_SPEEDPID_MER,AM_SPEEDPID_I_MAX,AM_SPEEDPID_MAXOUT);
	PID_Init(&CM4_Speed_Pid,AM_SPEEDPID_P,AM_SPEEDPID_I,AM_SPEEDPID_D,AM_SPEEDPID_DEAD,AM_SPEEDPID_MER,AM_SPEEDPID_I_MAX,AM_SPEEDPID_MAXOUT);
	
	LED_Init();					//初始化LED 
	Date_Init();
	DBUS_Init();
	CAN1_Init();
	RC_Calibration();	//遥控器数据检查
	
	CAN2_Init();
	
	TIM5_PWM_Init(20000-1,84-1);	//计数20000次20ms
	TIM3_Int_Init(20-1,8400-1);	//定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数20次为2ms  
	TIM2_PWM_Init(20000-1,84-1);	//计数20000次20ms
 	
	
while(1)
	{
	
//		delay_ms(500);
//		GREEN_LED_OFF(); 
//		delay_ms(500);

	}
}



