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

					    						    
										 							 				    
void CAN1_Init(void);//CAN1��ʼ��
void CAN2_Init(void);//CAN2��ʼ��

//CAN1��������
void Set_CAN1_1(s16 motor1,s16 motor2,s16 motor3,s16 motor4);
void Set_CAN1_2(s16 motor5,s16 motor6,s16 motor7,s16 motor8);
//CAN2��������,ʣ�¼�����֪����������ɶ0_0,��д��˵
void Set_CAN2_1(s16 motor1,s16 motor2,s16 motor3,s16 motor4);
void Set_CAN2_2(s16 motor5,s16 motor6,s16 motor7,s16 motor8);
//CAN1��CAN2���ղ���������
//void CAN1_Motor_Data_Receive(CanRxMsg *msg);
void CAN2_Motor_Data_Receive(CanRxMsg *msg);

void Recive_Data_deal(ReceiveTypeDef *Receive,CanRxMsg * msg);

extern ReceiveTypeDef CM1_Feedback;
extern ReceiveTypeDef CM2_Feedback;
extern ReceiveTypeDef CM3_Feedback;
extern ReceiveTypeDef CM4_Feedback;

#endif


