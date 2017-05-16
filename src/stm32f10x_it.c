#include "include.h"

void SysTick_Handler(void)
{
	sys_misc_run_struct.sys_tick_ms_counter++;
	
	if(gsm_misc_struct.gsm_ring_low_flag)
	{
		if(!RING_STATE())
		{
			gsm_misc_struct.gsm_ring_low_ms_counter++;
		}
		else
		{
			if(gsm_misc_struct.gsm_ring_low_ms_counter < 50)
			{
				gsm_misc_struct.ring_low_counter = 0;
				gsm_misc_struct.gsm_ring_low_flag = FALSE;
				gsm_misc_struct.gsm_ring_low_ms_counter = 0;
			}
		}
	}
}
void RTC_IRQHandler(void)
{	
	if (RTC_GetITStatus(RTC_IT_SEC) != RESET)
	{
		sys_misc_run_struct.sys_rtc_sec_counter++;
		RTC_ClearITPendingBit(RTC_IT_SEC);
	}
	
	sys_work_para_struct.lsnal_sys_must_reset_sec_counter++;
	if(sys_work_para_struct.lsnal_sys_must_reset_sec_counter >= 14400)
	{
		sys_work_para_struct.lsnal_sys_must_reset_sec_counter = 0;
		sys_work_para_struct.term_self_reset_flag = TRUE;
		if(!sys_misc_run_struct.term_enter_sleep_flag)
		{
			SysKeyDataSave();
			while(1)
			{
				NVIC_DISABLE();
			}
		}
	}		
	sys_misc_run_struct.wdg_counter = 0;
}
void USART1_IRQHandler(void)///本地串口
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		if(g_local_uart_struct.rx_counter >= LOCAL_UART_BUF_LEN)
		{
			g_local_uart_struct.rx_counter -= 1;
		}
		g_local_uart_struct.rx_buf[g_local_uart_struct.rx_counter++] = USART_ReceiveData(USART1);
		g_local_uart_struct.rx_delay_time_counter = UART_RX_50_MS_DELAY;
	}
}
void USART2_IRQHandler(void)///GPS
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{	
		if(g_gps_uart_struct.rx_counter >= LOCAL_UART_BUF_LEN)
		{
			g_gps_uart_struct.rx_counter -= 1;
		}
		g_gps_uart_struct.rx_buf[g_gps_uart_struct.rx_counter++] = USART_ReceiveData(USART2);
		g_gps_uart_struct.rx_delay_time_counter = UART_RX_50_MS_DELAY;
	}
}


void USART3_IRQHandler(void)///GPRS
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		g_gprs_uart_struct.rx_buf[g_gprs_usart_struct.rx_head_point] = USART_ReceiveData(USART3);
		
		g_gprs_usart_struct.rx_head_point = (g_gprs_usart_struct.rx_head_point+1)%GPRS_UART_BUF_LEN;
		if(g_gprs_usart_struct.rx_head_point == g_gprs_usart_struct.rx_tail_point)
		{
			g_gprs_usart_struct.rx_tail_point = (g_gprs_usart_struct.rx_tail_point+1)%GPRS_UART_BUF_LEN;
		}
		
		if(gsm_misc_struct.cur_mode <= AT_INIT_MODE)
		{
			g_gprs_uart_struct.rx_delay_time_counter = UART_RX_500_MS_DELAY;
		}
		else
		{
			g_gprs_uart_struct.rx_delay_time_counter = UART_RX_5_MS_DELAY;
		}
	}
}

void UART4_IRQHandler(void)///METER仪表
{
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	{
		if(g_meter_uart_struct.rx_counter >= METER_UART_BUF_LEN)
		{
			g_meter_uart_struct.rx_counter -= 1;
		}
		
		g_meter_uart_struct.rx_buf[g_meter_uart_struct.rx_counter++] = USART_ReceiveData(UART4);

		if(g_meter_uart_struct.rx_counter >= METER_UART_BUF_LEN)
		{
			g_meter_uart_struct.rx_counter -= 1;
		}
		g_meter_uart_struct.rx_delay_time_counter = UART_RX_50_MS_DELAY;
	}
}

void DMA1_Channel1_IRQHandler(void)
{
	static uint32 adc_val[4] = {0x00,0x00,0x00,0x00};
	static uint8 s_conv_counter = 0;
	
	if(DMA_GetFlagStatus(DMA1_FLAG_TC1) != RESET)
	{
		DMA_ClearITPendingBit(DMA1_IT_TC1);
		
		adc_val[0] += sys_misc_run_struct.adc_conv_buf[0];///PWR_C
		adc_val[3] += sys_misc_run_struct.adc_conv_buf[3];///BAT_VOLTAGE
	    s_conv_counter++;
		if(s_conv_counter >= 5)
		{
			s_conv_counter = 0x00;
			adc_val[0] = adc_val[0] / 5;
			adc_val[0] = adc_val[0]*303/4096;						///电压值，单位0.1V
			sys_misc_run_struct.power_voltage_val = adc_val[0];
	
			adc_val[3] = adc_val[3] / 5;
			adc_val[3] = adc_val[3]*48/4096;							///电压值，单位0.1V
			sys_misc_run_struct.battery_voltage_val = adc_val[3];	///加0.3V,校正
		}
	}
}

void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line9) != RESET)
	{ 
		EXTI_ClearITPendingBit(EXTI_Line9);
	}
	if(EXTI_GetITStatus(EXTI_Line8) != RESET)
	{  
		EXTI_ClearITPendingBit(EXTI_Line8);
		if(gsm_misc_struct.cur_mode >= AT_INIT_MODE)
		{
			gsm_misc_struct.ring_low_counter++;
			gsm_misc_struct.gsm_ring_low_flag = TRUE;
			gsm_misc_struct.gsm_ring_low_ms_counter = 0;
		}
	}
	if(EXTI_GetITStatus(EXTI_Line7) != RESET)
	{ 
		EXTI_ClearITPendingBit(EXTI_Line7);
	}
	if(EXTI_GetITStatus(EXTI_Line6) != RESET)
	{   
		EXTI_ClearITPendingBit(EXTI_Line6);
	}
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{ 
		EXTI_ClearITPendingBit(EXTI_Line5);
	}
	
	sys_misc_run_struct.wake_up_by_exit_flag = TRUE;
}
void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line15) != RESET)
	{ 
		EXTI_ClearITPendingBit(EXTI_Line15);
	}
	if(EXTI_GetITStatus(EXTI_Line14) != RESET)
	{  
		EXTI_ClearITPendingBit(EXTI_Line14);
	}
	if(EXTI_GetITStatus(EXTI_Line13) != RESET)
	{ 
		EXTI_ClearITPendingBit(EXTI_Line13);
	///	sys_misc_run_struct.wake_up_by_exit_flag = TRUE;
	}
	if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	{   
		EXTI_ClearITPendingBit(EXTI_Line12);
	}
	if(EXTI_GetITStatus(EXTI_Line11) != RESET)
	{ 
		EXTI_ClearITPendingBit(EXTI_Line11);
	}
	if(EXTI_GetITStatus(EXTI_Line10) != RESET)
	{ 
		EXTI_ClearITPendingBit(EXTI_Line10);
	}
}
void PVD_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line16);
	if (PWR_GetFlagStatus(PWR_FLAG_PVDO))
	{
		while(1);
	}
}
void RTCAlarm_IRQHandler(void)
{
	RTC_GetITStatus(RTC_IT_ALR);
	EXTI_ClearITPendingBit(EXTI_Line17);
	RTC_WaitForLastTask();   
	RTC_ClearITPendingBit(RTC_IT_ALR);
	RTC_WaitForLastTask();
}
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	CanRx();
}
void NMI_Handler(void)
{
	while (1)
	{
	}
}
/***
__asm void wait()
{
	BX lr
}
***/
void HardFault_Handler(void)
{
	while (1)
	{
	}
	///wait();
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}
void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
