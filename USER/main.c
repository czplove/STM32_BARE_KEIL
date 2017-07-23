
/*

��Ƶ 72MHz

ע��protocol�����ڴ������ı��Ա˳������⣨�ϴ�ʱ���ܶ�˳����Ҫ��


*/
#include "includes.h"
#include "tiza_include.h"

//�����ջ��С����
#define TASK0_TCB_SIZE 128
#define TASK1_TCB_SIZE 128
//�����ջ����
u32 TASK0_STK[TASK0_TCB_SIZE];
u32 TASK1_STK[TASK1_TCB_SIZE];
//�������ȼ�����
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
Main����
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
	


