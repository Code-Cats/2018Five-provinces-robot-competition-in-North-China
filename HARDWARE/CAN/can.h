#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"

typedef struct
{
	 s16 speed;
	 s16 position[3];
	 s16 ecd_position;
	 u8 count;
} ReceiveTypeDef;

					    						    
										 							 				    
void CAN1_Init(void);//CAN1初始化
void CAN2_Init(void);//CAN2初始化

//CAN1发送数据
void Set_CAN1_1(s16 motor1,s16 motor2,s16 motor3,s16 motor4);
void Set_CAN1_2(s16 motor5,s16 motor6,s16 motor7,s16 motor8);
//CAN2发送数据,剩下几个不知道能用来干啥0_0,先写再说
void Set_CAN2_1(s16 motor1,s16 motor2,s16 motor3,s16 motor4);
void Set_CAN2_2(s16 motor5,s16 motor6,s16 motor7,s16 motor8);
//CAN1和CAN2接收并处理数据
//void CAN1_Motor_Data_Receive(CanRxMsg *msg);
void CAN2_Motor_Data_Receive(CanRxMsg *msg);

void Recive_Data_deal(ReceiveTypeDef *Receive,CanRxMsg * msg);

extern ReceiveTypeDef CM1_Feedback;
extern ReceiveTypeDef CM2_Feedback;
extern ReceiveTypeDef CM3_Feedback;
extern ReceiveTypeDef CM4_Feedback;

#endif


