
/*

主频 72MHz

注意protocol中因内存对齐而改变成员顺序的问题（上传时可能对顺序有要求）


*/
#include "includes.h"
#include "tiza_include.h"

//任务堆栈大小定义
#define TASK0_TCB_SIZE 128
#define TASK1_TCB_SIZE 128
//任务堆栈定义
u32 TASK0_STK[TASK0_TCB_SIZE];
u32 TASK1_STK[TASK1_TCB_SIZE];
//任务优先级定义
#define Task0_Prio 0
#define Task1_Prio 1



/******************************************************
??????????
******************************************************/
void System_Mode_Init(void)
{
	//-Tz1000_Init();
	
	GpioInit();
	//-IwdgInit();	
	PvdInit();
	FeedWtd();
	UsartInit(LOCAL_USART,LOCAL_USART_BPR,USART_DATA_8B,USART_STOPBITS_1,USART_PARITY_NO);
	UsartInit(GPS_USART  ,GPS_USART_BPR  ,USART_DATA_8B,USART_STOPBITS_1,USART_PARITY_NO);
	
	FeedWtd();
	ExteFlashInit();
	RtcInit();
	AdcInit();
////	TIM3_Int_Init(1999,7199);//10Khz?????,???2000?200ms  
	FeedWtd();
	LocalUartFixedLenSend((uint8*)"Device start...\r\n",StrLen((uint8*)"Device start...\r\n",0));
	__NOP();__NOP();__NOP();__NOP();
}





void Task0(void *pdata)
{
	u8 i = 0;
	while(1)
	{
		i++;
//		PAout(8) ^= 1;
		OSTimeDly(500);		
	}
}

void Task1(void *pdata)
{
	while(1)
	{
//		PDout(2) ^= 1;
		OSTimeDly(250);
	}
}



/******************************************************
Main函数
******************************************************/
int main(void)
{	
		delay_init();
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		FeedWtd();
	
		OSTaskCreate(Task0, (void *)0, (u32 *)&TASK0_STK[TASK0_TCB_SIZE-1], Task0_Prio);
		OSTaskCreate(Task1, (void *)0, (u32 *)&TASK1_STK[TASK1_TCB_SIZE-1], Task1_Prio);
		OSStart();
		return 0;
//		System_Mode_Init(); 
//	
//	
//		while(1)
//		{
//		
//		}
		
}
	


