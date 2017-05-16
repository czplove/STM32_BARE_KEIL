
/*

��Ƶ 72MHz

ע��protocol�����ڴ������ı��Ա˳������⣨�ϴ�ʱ���ܶ�˳����Ҫ��


*/
#include "includes.h"
#include "tiza_include.h"


 

/////////////////////////UCOSII��������///////////////////////////////////
//START ����
//�����������ȼ�
#define START_TASK_PRIO      			11 //��ʼ��������ȼ�����Ϊ���
#define LED0_TASK_PRIO       			5//7 
#define L218_TASK_PRIO      			6//5 // 8//
#define PERIOD_TASK_PRIO      		7//6 // 9//
#define TEST_TASK_PRIO      			10 
//���������ջ��С
#define START_STK_SIZE  					64
#define LED0_STK_SIZE  		    		128
#define L218_STK_SIZE  						1024
#define PERIOD_STK_SIZE  					1024
#define TEST_STK_SIZE  						256
//�����ջ	
OS_STK START_TASK_STK[START_STK_SIZE];
OS_STK LED0_TASK_STK[LED0_STK_SIZE];
OS_STK L218_TASK_STK[L218_STK_SIZE];
OS_STK PERIOD_TASK_STK[PERIOD_STK_SIZE];
OS_STK TEST_TASK_STK[TEST_STK_SIZE];
//������
void start_task(void *pdata);	
void led0_task(void *pdata);
void L218_task(void *pdata);
void Period_task(void *pdata);
void Test_task(void *pdata);



/******************************************************
ϵͳһЩģ��ĳ�ʼ��
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
////	TIM3_Int_Init(1999,7199);//10Khz�ļ���Ƶ�ʣ�������2000Ϊ200ms  
	FeedWtd();
	LocalUartFixedLenSend((uint8*)"Device start...\r\n",StrLen((uint8*)"Device start...\r\n",0));
	__NOP();__NOP();__NOP();__NOP();
}

/******************************************************
Main����
******************************************************/
int main(void)
{	
	delay_init();	    //��ʱ������ʼ��	 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	FeedWtd();
	
	OSInit();   
 	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//������ʼ����
	OSStart();	  	 
}
	

/******************************************************
//��ʼ����
******************************************************/
void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;	
	pdata = pdata; 
	
	System_Mode_Init(); 
	
	OS_ENTER_CRITICAL();						//�����ٽ���(�޷����жϴ��)    
	//-------------- ����ָ��	---------------------��ջ--------------------------------------------���ȼ�     
#if 1		//LED0����
	OSTaskCreate(led0_task,			 	(void *)0,(OS_STK*)&LED0_TASK_STK[LED0_STK_SIZE-1],						LED0_TASK_PRIO);	
#endif	
#if 1		//L218����
	OSTaskCreate(L218_task,				(void *)0,(OS_STK*)&L218_TASK_STK[L218_STK_SIZE-1],						L218_TASK_PRIO);
#endif
#if 1		//���ڴ�������
	OSTaskCreate(Period_task,			(void *)0,(OS_STK*)&PERIOD_TASK_STK[PERIOD_STK_SIZE-1],				PERIOD_TASK_PRIO);
#endif
#if 1 	//��������	
	OSTaskCreate(Test_task,				(void *)0,(OS_STK*)&TEST_TASK_STK[TEST_STK_SIZE-1],				TEST_TASK_PRIO);
#endif
	
	OSTaskSuspend(START_TASK_PRIO);	//������ʼ����.
	OS_EXIT_CRITICAL();							//�˳��ٽ���(���Ա��жϴ��)
}






//LED0����
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
///UART3 ����		
		dma_uart3_buff_head = (DMA_UART3_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Channel3)) % DMA_UART3_BUF_LEN;	
		if(dma_uart3_buff_tail != dma_uart3_buff_head){
			count = (DMA_UART3_BUF_LEN + dma_uart3_buff_head - dma_uart3_buff_tail) % DMA_UART3_BUF_LEN;
			for(i=0; i < count; i++){
				gprs_uart_buf[g_gprs_uart_struct.rx_head] = dma_uart3_buff[dma_uart3_buff_tail];
				
				g_gprs_uart_struct.rx_head = (g_gprs_uart_struct.rx_head+1) % GPRS_UART_BUF_LEN;
				if(g_gprs_uart_struct.rx_head == g_gprs_uart_struct.rx_tail)
				{//buff����βҲ��ǰ�ƶ�
					g_gprs_uart_struct.rx_tail = (g_gprs_uart_struct.rx_tail+1) % GPRS_UART_BUF_LEN;
				}
				
				dma_uart3_buff_tail = (dma_uart3_buff_tail + 1) % DMA_UART3_BUF_LEN;				
			}
		}

/////UART3 ����
//	if(DMA_GetFlagStatus(DMA1_FLAG_TC2)!=RESET){  //-һ��Ҫ��֤���ݷ��ͳ�ȥ��,��ʹ��DMA��������
//		if(uart3_dma_buff_head2 != uart3_dma_buff_tail2){
//			count = (UART3_DMA_BUF_LEN2 + uart3_dma_buff_head2 - uart3_dma_buff_tail2)%UART3_DMA_BUF_LEN2;
//			for(i=0; i < count; i++){
//				uart3_dma_buff[i] = uart3_dma_buff2[uart3_dma_buff_tail2];
//				uart3_dma_buff_tail2 = (uart3_dma_buff_tail2+1) % UART3_DMA_BUF_LEN2;
//			}
//			printf("\r\n**************count = %d*********\r\n", count);
//			DMA_Cmd(DMA1_Channel2, DISABLE);
//			DMA1_Channel2->CNDTR = count; 							//-���������Ĵ���,ָʾʣ��Ĵ������ֽ���Ŀ
//			DMA_Cmd(DMA1_Channel2, ENABLE);
//			
//			DMA_ClearFlag(DMA1_FLAG_TC2);
//		}
//  }		
		
		
	if(times % 100 == 0){
		
		FeedWtd();										///ι��

	}
		
	
	if(++times >= 200)
	{		
		times = 0;
//		FeedWtd();										///ι��
		
		
/// ADC		
		adc_val[0] += adc_conv_buf[0];///PWR_C
		adc_val[1] += adc_conv_buf[1];///BAT_ADC
	  s_conv_counter++;
		if(s_conv_counter >= 5)
		{
			s_conv_counter = 0x00;
			adc_val[0] = adc_val[0] / 5;
			adc_val[0] = adc_val[0]*303/4096;						///��ѹֵ����λ0.1V
			adc_result[0] = adc_val[0];
	
			adc_val[1] = adc_val[1] / 5;
			adc_val[1] = adc_val[1]*48/4096;						///��ѹֵ����λ0.1V    3.3/(220/320)
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
			OFF_GPS_LED();	//δ��λ
		}
		else{
			ON_GPS_LED();
		}
//		if(g_pro_struct.login_center_flag == TRUE){	//�Ѿ�����
//			ON_WORK_LED();
//		}
//		else{
//			OFF_WORK_LED();	//�������ӱ�־
//		}
		if(g_pro_struct.try_login_statu == 3){	//�Ѿ���¼	
			ON_ERR_LED();
		}
		else{
			OFF_ERR_LED();	//�������ӱ�־
		}
		
		
		
//		OFF_WORK_LED();
//		ON_GPS_LED();
		OSTimeDlyHMSM(0, 0, 1, 0);
		FeedWtd();
		
	}
	
	#endif
	
}







