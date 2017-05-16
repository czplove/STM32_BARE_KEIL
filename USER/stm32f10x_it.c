
//#include "stm32f10x_it.h" 
#include "tiza_include.h"


//运行系统时系统占用
/*void SysTick_Handler(void)
//{
//	sys_misc_run_struct.sys_tick_ms_counter++;
//	
//	if(gsm_misc_struct.gsm_ring_low_flag)
//	{
//		if(!RING_STATE())
//		{
//			gsm_misc_struct.gsm_ring_low_ms_counter++;
//		}
//		else
//		{
//			if(gsm_misc_struct.gsm_ring_low_ms_counter < 50)
//			{
//				gsm_misc_struct.ring_low_counter = 0;
//				gsm_misc_struct.gsm_ring_low_flag = FALSE;
//				gsm_misc_struct.gsm_ring_low_ms_counter = 0;
//			}
//		}
//	}
}
*/
//定时器3中断服务程序
void TIM3_IRQHandler(void)   //TIM3中断
{
	//OSIntEnter();
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET){  //检查TIM3更新中断发生与否
		
//		g_sysmiscrun_struct.time3_200ms_count = (g_sysmiscrun_struct.time3_200ms_count+1)%60000;
		
		
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  		//清除TIMx更新中断标志 
	}
	//OSIntExit();
}

void RTC_IRQHandler(void)
{	
	//OSIntEnter();
	if (RTC_GetITStatus(RTC_IT_SEC) != RESET){
		
		RTC_ClearITPendingBit(RTC_IT_SEC);
	}

	//OSIntExit();
}

void PVD_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line16);
	if (PWR_GetFlagStatus(PWR_FLAG_PVDO))
	{
		while(1);
	}
}
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	//-CanRx();
}



#if 0
void USART1_IRQHandler(void)///本地串口
{
	uint8 delay;
	
	//OSIntEnter();
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		g_local_uart_struct.rx_buf[g_local_uart_struct.rx_head] = USART_ReceiveData(USART1);
		g_local_uart_struct.rx_head = (g_local_uart_struct.rx_head+1) % LOCAL_UART_BUF_LEN;
		
		if(g_local_uart_struct.rx_head == g_local_uart_struct.rx_tail)
		{//buff满，尾也向前移动
			g_local_uart_struct.rx_tail = (g_local_uart_struct.rx_tail+1) % LOCAL_UART_BUF_LEN;
		}
		
	}
	//OSIntExit();
}
#endif 
#if 0
void USART2_IRQHandler(void)///GPS
{	
	//OSIntEnter();
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		g_gps_uart_struct.rx_buf[g_gps_uart_struct.rx_head] = USART_ReceiveData(USART2);
		g_gps_uart_struct.rx_head = (g_gps_uart_struct.rx_head+1) % GPS_UART_BUF_LEN;
		
		if(g_gps_uart_struct.rx_head == g_gps_uart_struct.rx_tail)
		{//buff满，尾也向前移动
			g_gps_uart_struct.rx_tail = (g_gps_uart_struct.rx_tail+1) % GPS_UART_BUF_LEN;
		}
		
	}
	//OSIntExit();
}
#endif
#if 0
void USART3_IRQHandler(void)///GPRS
{
	//OSIntEnter();
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
//		USART_ClearITPendingBit(USART3,USART_IT_RXNE); //清中断标志
		g_gprs_uart_struct.rx_buf[g_gprs_uart_struct.rx_head] = USART_ReceiveData(USART3);
		g_gprs_uart_struct.rx_head = (g_gprs_uart_struct.rx_head+1) % GPRS_UART_BUF_LEN;
		
		if(g_gprs_uart_struct.rx_head == g_gprs_uart_struct.rx_tail)
		{//buff满，尾也向前移动
			g_gprs_uart_struct.rx_tail = (g_gprs_uart_struct.rx_tail+1) % GPRS_UART_BUF_LEN;
		}
		
	}
	
	//OSIntExit();
}
#endif
void UART4_IRQHandler(void)///METER仪表
{
	uint8 delay;
	
	//OSIntEnter();
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	{
		//-g_meter_uart_struct.rx_buf[g_meter_uart_struct.rx_head] = USART_ReceiveData(UART4);
		//-g_meter_uart_struct.rx_head = (g_meter_uart_struct.rx_head+1) % METER_UART_BUF_LEN;
		
		//-if(g_meter_uart_struct.rx_head == g_meter_uart_struct.rx_tail)
		//-{//buff满，尾也向前移动
		//-	g_meter_uart_struct.rx_tail = (g_meter_uart_struct.rx_tail+1) % METER_UART_BUF_LEN;
		//-}
		
	}
	//OSIntExit();
	
}

void UART5_IRQHandler(void)///485
{
	uint8 delay;
	
	//OSIntEnter();
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
	{
		//-g_rs485_uart_struct.rx_buf[g_rs485_uart_struct.rx_head] = USART_ReceiveData(UART5);
		
		//-g_rs485_uart_struct.rx_head = (g_rs485_uart_struct.rx_head+1) % RS485_UART_BUF_LEN;		
		//-if(g_rs485_uart_struct.rx_head == g_rs485_uart_struct.rx_tail)
		//-{//buff满，尾也向前移动
		//-	g_rs485_uart_struct.rx_tail = (g_rs485_uart_struct.rx_tail+1) % RS485_UART_BUF_LEN;
		//-}

	}
	//OSIntExit();
	
}


//=======================================================================================//
 
void NMI_Handler(void)
{
	//OSIntEnter();
	//OSIntExit();
}
 
void HardFault_Handler(void)
{
	//OSIntEnter();
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
	//OSIntExit();  
}
 
void MemManage_Handler(void)
{
	//OSIntEnter();
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
	//OSIntExit();  
}

 
void BusFault_Handler(void)
{
	//OSIntEnter();
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
	//OSIntExit();  
}
 
void UsageFault_Handler(void)
{
	//OSIntEnter();
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
	//OSIntExit();  
}
 
void SVC_Handler(void)
{
	//OSIntEnter();
	//OSIntExit();  
}
 
void DebugMon_Handler(void)
{
	//OSIntEnter();
	//OSIntExit();  
}

 
 

