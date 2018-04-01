//作者：余鑫
//程序功能目的：2017华北五省空中高级组-地面车程序
//时间：2017.11.5
//task.c  控制任务处理

#include "task.h"
#include "Mydefine.h"
#include "can.h"
#include "dbus.h"
#include "timer.h"
#include "pid.h"


s16 Speed_X=0;
s16 Speed_Y=0;
s16 Speed_W=0;


s16 CM1_Speed=0;
s16 CM2_Speed=0;
s16 CM3_Speed=0;
s16 CM4_Speed=0;


void Control_Task(void)
{
	Remove_Task();
}




void Remove_Task(void)	//2ms进一次中断
{
	Speed_X=RC_Ctl.rc.ch1-1024;//将拨杆中位定义为速度的零值
	Speed_Y=RC_Ctl.rc.ch0-1024;
	Speed_W=(RC_Ctl.rc.ch2-1024)/2;
	
	CM1_Speed=(int)((Speed_X+Speed_Y+Speed_W)*K_Speed);//将3向速度值赋到4个电机
	CM2_Speed=(int)((-Speed_X+Speed_Y+Speed_W)*K_Speed);
	CM3_Speed=(int)((Speed_X-Speed_Y+Speed_W)*K_Speed);
	CM4_Speed=(int)((-Speed_X-Speed_Y+Speed_W)*K_Speed);
	
	PID_Speed_Calc(&CM1_Speed_Pid,CM1_Speed,CM1_Feedback.speed);
	PID_Speed_Calc(&CM2_Speed_Pid,CM2_Speed,CM2_Feedback.speed);
	PID_Speed_Calc(&CM3_Speed_Pid,CM3_Speed,CM3_Feedback.speed);
	PID_Speed_Calc(&CM4_Speed_Pid,CM4_Speed,CM4_Feedback.speed);
	
Set_CAN2_1((int)CM1_Speed_Pid.output,(int)CM2_Speed_Pid.output,(int)CM3_Speed_Pid.output,(int)CM4_Speed_Pid.output);
////Set_CAN1_1((int)CM1_Speed_Pid.output,(int)CM2_Speed_Pid.output,(int)CM3_Speed_Pid.output,(int)CM4_Speed_Pid.output);

}



//舵机封装函数	//输入值为цs
void Steer1_set(u16 set1)
{
	TIM_SetCompare1(TIM5,ESC_CYCLE-set1);
}

void Steer2_set(u16 set2)
{
	TIM_SetCompare2(TIM5,ESC_CYCLE-set2);
}

void Steer3_set(u16 set3)
{
	TIM_SetCompare3(TIM2,ESC_CYCLE-set3);
}

void Steer4_set(u16 set4)
{
	TIM_SetCompare4(TIM2,ESC_CYCLE-set4);
}



void RC_Calibration(void)	//上电检测遥控器接收值并与默认参数比较，判断是否正常，否则软复位
{													//注：必须放在遥控器接收初始化后（此处为判断串口数值）
	if((RC_Ctl.rc.ch0+RC_Ctl.rc.ch1+RC_Ctl.rc.ch2+RC_Ctl.rc.ch3-1024*4)>2||(RC_Ctl.rc.ch0+RC_Ctl.rc.ch1+RC_Ctl.rc.ch2+RC_Ctl.rc.ch3-1024*4)<-2)
	{
		NVIC_SystemReset();
	}
}


void Date_Init(void)
{
	RC_Ctl.rc.ch0=1024;
	RC_Ctl.rc.ch1=1024;
	RC_Ctl.rc.ch2=1024;
	RC_Ctl.rc.ch3=1024;
	
	CM1_Feedback.count=0;
	CM2_Feedback.count=0;
	CM3_Feedback.count=0;
	CM4_Feedback.count=0;
}





