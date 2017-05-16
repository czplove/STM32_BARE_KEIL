
/*

主频 72MHz

注意protocol中因内存对齐而改变成员顺序的问题（上传时可能对顺序有要求）


*/
#include "includes.h"
#include "tiza_include.h"



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


/******************************************************
Main函数
******************************************************/
int main(void)
{	
		delay_init();
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		FeedWtd();
	
	
		System_Mode_Init(); 
	
	
		while(1)
		{
		
		}
		
}
	


