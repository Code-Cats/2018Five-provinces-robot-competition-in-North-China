#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "pid.h"


ReceiveTypeDef CM1_Feedback;
ReceiveTypeDef CM2_Feedback;
ReceiveTypeDef CM3_Feedback;
ReceiveTypeDef CM4_Feedback;

/////////////////////////////CAN1的初始化//////////////////////////////
void CAN1_Init(void)
{
	
	   CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);

    //初始化GPIO
	  gpio.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_AF;//复用功能
    gpio.GPIO_OType = GPIO_OType_PP;//推挽输出
    gpio.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    gpio.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOA, &gpio);//初始化PA11,PA12
    
    nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    CAN_DeInit(CAN1);
    CAN_StructInit(&can);
    
    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = ENABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = DISABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_9tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 3;   //CAN BaudRate 90/(1+8+6)*3=1Mbps
    CAN_Init(CAN1, &can);

	  can_filter.CAN_FilterNumber=0;
	  can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
	  can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
	  can_filter.CAN_FilterIdHigh=0x0000;
	  can_filter.CAN_FilterIdLow=0x0000;
	  can_filter.CAN_FilterMaskIdHigh=0x0000;
	  can_filter.CAN_FilterMaskIdLow=0x0000;
	  can_filter.CAN_FilterFIFOAssignment=0;    //the message which pass the filter save in fifo0
	  can_filter.CAN_FilterActivation=ENABLE;
	  CAN_FilterInit(&can_filter);
    
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
				
}   
 

void CAN1_RX0_IRQHandler(void)
{
	  CanRxMsg rx_message;	
   if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	  {
      CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
		    CAN_ClearFlag(CAN1, CAN_FLAG_FF0); 		
		    CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
//				  CAN1_Motor_Data_Receive(&rx_message);
   }
}


////////////////////////CAN1发送数据//////////////////////////////////////
void Set_CAN1_1(s16 motor1,s16 motor2,s16 motor3,s16 motor4)//底盘速度发送
{	
  CanTxMsg TxMessage;
  TxMessage.StdId=0x200;	 
  TxMessage.IDE=CAN_Id_Standard;		  
  TxMessage.RTR=CAN_RTR_Data;		  
  TxMessage.DLC=0x08;							
  TxMessage.Data[0] = (uint8_t)(motor1 >> 8);
  TxMessage.Data[1] = (uint8_t)motor1;
  TxMessage.Data[2] = (uint8_t)(motor2 >> 8);
  TxMessage.Data[3] = (uint8_t)motor2;   
	 TxMessage.Data[4] = (uint8_t)(motor3 >> 8);
  TxMessage.Data[5] = (uint8_t)motor3;
  TxMessage.Data[6] = (uint8_t)(motor4 >> 8);
  TxMessage.Data[7] = (uint8_t)motor4;	
  CAN_Transmit(CAN1, &TxMessage);   	
}

void Set_CAN1_2(s16 motor5,s16 motor6,s16 motor7,s16 motor8)//抓取速度发送
{
    CanTxMsg TxMessage;    
    TxMessage.StdId = 0x1FF;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 0x08;  
    TxMessage.Data[0] = (uint8_t)(motor5 >> 8);
    TxMessage.Data[1] = (uint8_t)motor5;
    TxMessage.Data[2] = (uint8_t)(motor6 >> 8);
    TxMessage.Data[3] = (uint8_t)motor6;
    TxMessage.Data[4] = (uint8_t)(motor7>>8);
    TxMessage.Data[5] = (uint8_t)motor7;
    TxMessage.Data[6] = (uint8_t)(motor8>>8);
    TxMessage.Data[7] = (uint8_t)motor8;
    CAN_Transmit(CAN1,&TxMessage);
}


/***********************************************************************************/
/////////////////////////CAN2的初始化/////////////////////////
void CAN2_Init(void)
{
CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);

    //初始化GPIO
	  gpio.GPIO_Pin = GPIO_Pin_12| GPIO_Pin_13;
    gpio.GPIO_Mode = GPIO_Mode_AF;//复用功能
    gpio.GPIO_OType = GPIO_OType_PP;//推挽输出
    gpio.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    gpio.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &gpio);//初始化PB5,PB6
    
    nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
				
//				nvic.NVIC_IRQChannel = CAN2_TX_IRQn;
//    nvic.NVIC_IRQChannelPreemptionPriority = 0;
//    nvic.NVIC_IRQChannelSubPriority = 1;
//    nvic.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&nvic);
    
    CAN_DeInit(CAN2);
    CAN_StructInit(&can);
    
    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = DISABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = ENABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_9tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 3;   //CAN BaudRate 90/((1+8+6)*3)=1Mbps
    CAN_Init(CAN2, &can);

	  can_filter.CAN_FilterNumber=14;
	  can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
	  can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
	  can_filter.CAN_FilterIdHigh=0x0000;
	  can_filter.CAN_FilterIdLow=0x0000;
	  can_filter.CAN_FilterMaskIdHigh=0x0000;
	  can_filter.CAN_FilterMaskIdLow=0x0000;
	  can_filter.CAN_FilterFIFOAssignment=0;    //the message which pass the filter save in fifo0
	  can_filter.CAN_FilterActivation=ENABLE;
	  CAN_FilterInit(&can_filter);
    
    CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
				CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
				
}   
 

void CAN2_RX0_IRQHandler(void)
{
	  CanRxMsg rx_message;	
    if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET)
	   {
      CAN_ClearITPendingBit(CAN2, CAN_IT_FF0);
		    CAN_ClearFlag(CAN2, CAN_FLAG_FF0); 		
		    CAN_Receive(CAN2, CAN_FIFO0, &rx_message);
				  CAN2_Motor_Data_Receive(&rx_message);
    }
}


////////////////////////CAN2发送数据//////////////////////////////////////
void Set_CAN2_1(s16 motor1,s16 motor2,s16 motor3,s16 motor4)
{
	CanTxMsg TxMessage;
  TxMessage.StdId=0x200;	 
  TxMessage.IDE=CAN_Id_Standard;		  
  TxMessage.RTR=CAN_RTR_Data;		  
  TxMessage.DLC=0x08;							
  TxMessage.Data[0] = (uint8_t)(motor1 >> 8);
  TxMessage.Data[1] = (uint8_t)motor1;
  TxMessage.Data[2] = (uint8_t)(motor2 >> 8);
  TxMessage.Data[3] = (uint8_t)motor2;   
	 TxMessage.Data[4] = (uint8_t)(motor3 >> 8);
  TxMessage.Data[5] = (uint8_t)motor3;
  TxMessage.Data[6] = (uint8_t)(motor4 >> 8);
  TxMessage.Data[7] = (uint8_t)motor4;	
  CAN_Transmit(CAN2, &TxMessage);   	
}
void Set_CAN2_2(s16 motor5,s16 motor6,s16 motor7,s16 motor8)
{
	   CanTxMsg TxMessage;    
    TxMessage.StdId = 0x1FF;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 0x08;  
    TxMessage.Data[0] = (uint8_t)(motor5 >> 8);
    TxMessage.Data[1] = (uint8_t)motor5;
    TxMessage.Data[2] = (uint8_t)(motor6 >> 8);
    TxMessage.Data[3] = (uint8_t)motor6;
    TxMessage.Data[4] = (uint8_t)(motor7>>8);
    TxMessage.Data[5] = (uint8_t)motor7;
    TxMessage.Data[6] = (uint8_t)(motor8>>8);
    TxMessage.Data[7] = (uint8_t)motor8;
    CAN_Transmit(CAN2,&TxMessage);
}

void CAN2_Motor_Data_Receive(CanRxMsg *msg)
{		
//	CAN_Receive(CAN1, CAN_FIFO0, msg);//读取数据	
	switch(msg->StdId)
	{
		case 0x201:
		{
			Recive_Data_deal(&CM1_Feedback,msg);
			break;	
		}
		case 0x202:
		{
			Recive_Data_deal(&CM2_Feedback,msg);
			break;
		}
		case 0x203:
		{
			Recive_Data_deal(&CM3_Feedback,msg);
			break;
		}
		case 0x204:
		{
			Recive_Data_deal(&CM4_Feedback,msg);
		  break;
		}
		case 0x205:
		{		
			
			break;			
		}
		case 0x206:
		{
			
			break;			
		}
		case 0x207:
		{
   break;
		}
		default:
		{
			break;
		}
	}
}

//对接收到的数据做解码处理
void Recive_Data_deal(ReceiveTypeDef *Receive,CanRxMsg * msg)
{
	if(Receive->count<1)
	{
		Receive->position[NOW]=(msg->Data[0]<<8)|msg->Data[1];
		if(Receive->position[NOW]-Receive->position[LAST]>7500)	//一次跨越太大，说明倒着转了一圈
		{
			Receive->ecd_position=Receive->position[NOW]-8192;
		}
		else if(Receive->position[NOW]-Receive->position[LAST]<-7500)
		{
			Receive->ecd_position=Receive->position[NOW]+8192;
		}
		else
		{
			Receive->ecd_position=Receive->position[NOW];
		}
			
		Receive->speed=Receive->ecd_position-Receive->position[LAST];
		
		Receive->position[LAST]=Receive->position[NOW];
		Receive->count=2;
	}
	Receive->count--;
}
