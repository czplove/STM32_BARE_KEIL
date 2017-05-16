
/*

主频 72MHz

注意protocol中因内存对齐而改变成员顺序的问题（上传时可能对顺序有要求）


*/
#include "includes.h"
#include "tiza_include.h"


 

/////////////////////////UCOSII任务设置///////////////////////////////////
//START 任务
//设置任务优先级
#define START_TASK_PRIO      			11 //开始任务的优先级设置为最低
#define LED0_TASK_PRIO       			5//7 
#define L218_TASK_PRIO      			6//5 // 8//
#define PERIOD_TASK_PRIO      		7//6 // 9//
#define TEST_TASK_PRIO      			10 
//设置任务堆栈大小
#define START_STK_SIZE  					64
#define LED0_STK_SIZE  		    		128
#define L218_STK_SIZE  						1024
#define PERIOD_STK_SIZE  					1024
#define TEST_STK_SIZE  						256
//任务堆栈	
OS_STK START_TASK_STK[START_STK_SIZE];
OS_STK LED0_TASK_STK[LED0_STK_SIZE];
OS_STK L218_TASK_STK[L218_STK_SIZE];
OS_STK PERIOD_TASK_STK[PERIOD_STK_SIZE];
OS_STK TEST_TASK_STK[TEST_STK_SIZE];
//任务函数
void start_task(void *pdata);	
void led0_task(void *pdata);
void L218_task(void *pdata);
void Period_task(void *pdata);
void Test_task(void *pdata);



/******************************************************
系统一些模块的初始化
******************************************************/
void System_Mode_Init(void)
{
	Tz1000_Init();
	
	GpioInit();
	IwdgInit();	
	PvdInit();
	FeedWtd();
	UsartInit(LOCAL_USART,LOCAL_USART_BPR,USART_DATA_8B,USART_STOPBITS_1,USART_PARITY_NO);
	UsartInit(GPS_USART  ,GPS_USART_BPR  ,USART_DATA_8B,USART_STOPBITS_1,USART_PARITY_NO);
	
	FeedWtd();
	ExteFlashInit();
	RtcInit();
	AdcInit();
////	TIM3_Int_Init(1999,7199);//10Khz的计数频率，计数到2000为200ms  
	FeedWtd();
	LocalUartFixedLenSend((uint8*)"Device start...\r\n",StrLen((uint8*)"Device start...\r\n",0));
	__NOP();__NOP();__NOP();__NOP();
}

/******************************************************
Main函数
******************************************************/
int main(void)
{	
	delay_init();	    //延时函数初始化	 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	FeedWtd();
	
	OSInit();   
 	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
	OSStart();	  	 
}
	

/******************************************************
//开始任务
******************************************************/
void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;	
	pdata = pdata; 
	
	System_Mode_Init(); 
	
	OS_ENTER_CRITICAL();						//进入临界区(无法被中断打断)    
	//-------------- 任务指针	---------------------堆栈--------------------------------------------优先级     
#if 1		//LED0任务
	OSTaskCreate(led0_task,			 	(void *)0,(OS_STK*)&LED0_TASK_STK[LED0_STK_SIZE-1],						LED0_TASK_PRIO);	
#endif	
#if 1		//L218任务
	OSTaskCreate(L218_task,				(void *)0,(OS_STK*)&L218_TASK_STK[L218_STK_SIZE-1],						L218_TASK_PRIO);
#endif
#if 1		//周期处理任务
	OSTaskCreate(Period_task,			(void *)0,(OS_STK*)&PERIOD_TASK_STK[PERIOD_STK_SIZE-1],				PERIOD_TASK_PRIO);
#endif
#if 1 	//测试任务	
	OSTaskCreate(Test_task,				(void *)0,(OS_STK*)&TEST_TASK_STK[TEST_STK_SIZE-1],				TEST_TASK_PRIO);
#endif
	
	OSTaskSuspend(START_TASK_PRIO);	//挂起起始任务.
	OS_EXIT_CRITICAL();							//退出临界区(可以被中断打断)
}






//LED0任务
void led0_task(void *pdata)
{	 	
	uint32 ftpflshadd;
	
#if 1	
	uint16 count, i;
	static uint32 adc_val[2] = {0x00,0x00};
	static uint8  s_conv_counter = 0;
	static uint16 times;
	while(1)
	{
		OSTimeDlyHMSM(0, 0, 0, 5);
///UART3 接收		
		dma_uart3_buff_head = (DMA_UART3_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Channel3)) % DMA_UART3_BUF_LEN;	
		if(dma_uart3_buff_tail != dma_uart3_buff_head){
			count = (DMA_UART3_BUF_LEN + dma_uart3_buff_head - dma_uart3_buff_tail) % DMA_UART3_BUF_LEN;
			for(i=0; i < count; i++){
				gprs_uart_buf[g_gprs_uart_struct.rx_head] = dma_uart3_buff[dma_uart3_buff_tail];
				
				g_gprs_uart_struct.rx_head = (g_gprs_uart_struct.rx_head+1) % GPRS_UART_BUF_LEN;
				if(g_gprs_uart_struct.rx_head == g_gprs_uart_struct.rx_tail)
				{//buff满，尾也向前移动
					g_gprs_uart_struct.rx_tail = (g_gprs_uart_struct.rx_tail+1) % GPRS_UART_BUF_LEN;
				}
				
				dma_uart3_buff_tail = (dma_uart3_buff_tail + 1) % DMA_UART3_BUF_LEN;				
			}
		}

/////UART3 发送
//	if(DMA_GetFlagStatus(DMA1_FLAG_TC2)!=RESET){  //-一定要保证内容发送出去了,再使用DMA发送数据
//		if(uart3_dma_buff_head2 != uart3_dma_buff_tail2){
//			count = (UART3_DMA_BUF_LEN2 + uart3_dma_buff_head2 - uart3_dma_buff_tail2)%UART3_DMA_BUF_LEN2;
//			for(i=0; i < count; i++){
//				uart3_dma_buff[i] = uart3_dma_buff2[uart3_dma_buff_tail2];
//				uart3_dma_buff_tail2 = (uart3_dma_buff_tail2+1) % UART3_DMA_BUF_LEN2;
//			}
//			printf("\r\n**************count = %d*********\r\n", count);
//			DMA_Cmd(DMA1_Channel2, DISABLE);
//			DMA1_Channel2->CNDTR = count; 							//-传输数量寄存器,指示剩余的待传输字节数目
//			DMA_Cmd(DMA1_Channel2, ENABLE);
//			
//			DMA_ClearFlag(DMA1_FLAG_TC2);
//		}
//  }		
		
		
	if(times % 100 == 0){
		
		FeedWtd();										///喂狗

	}
		
	
	if(++times >= 200)
	{		
		times = 0;
//		FeedWtd();										///喂狗
		
		
/// ADC		
		adc_val[0] += adc_conv_buf[0];///PWR_C
		adc_val[1] += adc_conv_buf[1];///BAT_ADC
	  s_conv_counter++;
		if(s_conv_counter >= 5)
		{
			s_conv_counter = 0x00;
			adc_val[0] = adc_val[0] / 5;
			adc_val[0] = adc_val[0]*303/4096;						///电压值，单位0.1V
			adc_result[0] = adc_val[0];
	
			adc_val[1] = adc_val[1] / 5;
			adc_val[1] = adc_val[1]*48/4096;						///电压值，单位0.1V    3.3/(220/320)
			adc_result[1] = adc_val[1];									///
		}
	}
		
	}
	
#else	
	
	while(1)
	{
//		ON_WORK_LED();
//		OFF_GPS_LED();
//		CPL_ERR_LED();
		OSTimeDlyHMSM(0, 0, 1, 0);		
		FeedWtd();

	
		if(g_propostion_union.Item.status.bit.B0 ==1){
			OFF_GPS_LED();	//未定位
		}
		else{
			ON_GPS_LED();
		}
//		if(g_pro_struct.login_center_flag == TRUE){	//已经连接
//			ON_WORK_LED();
//		}
//		else{
//			OFF_WORK_LED();	//网络连接标志
//		}
		if(g_pro_struct.try_login_statu == 3){	//已经登录	
			ON_ERR_LED();
		}
		else{
			OFF_ERR_LED();	//网络连接标志
		}
		
		
		
//		OFF_WORK_LED();
//		ON_GPS_LED();
		OSTimeDlyHMSM(0, 0, 1, 0);
		FeedWtd();
		
	}
	
	#endif
	
}







