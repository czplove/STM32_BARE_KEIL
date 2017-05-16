/*
			中断优先级配置表
			
	00   	低压中断
	02		TIME3				
	03		UART3				GPRS

	10    UART2				GPS
	11		UART5				485
	12		CAN
	13		RTC
	13		RTC ALARM
	
	20		DMA      		处理adc
	
	33		UART4				仪表
	33		UART5				本地调试串口

*/





#ifndef __DRIVER_H
#define __DRIVER_H

	#ifdef DRIVER_GLOBAL
		#define DRIVER_EXTERN
	#else
		#define DRIVER_EXTERN extern
	#endif
	
	#include "stm32f10x.h"
	
	///#define LOCAL_DEBUG
	
	#define NVIC_DISABLE()     (__disable_irq())
	#define NVIC_ENABLE()      (__enable_irq())
///输入
	#define LOW_STATE			0
	#define HIGH_STATE		1
	#define CHECK_POW()			(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6))///总电源监测 1有电
	
///输出		
	#define ON_WORK_LED()			(GPIO_ResetBits(GPIOB, GPIO_Pin_12))///灯
	#define OFF_WORK_LED()		(GPIO_SetBits(GPIOB, GPIO_Pin_12))
	#define CPL_WORK_LED()		(GPIOB->ODR ^= GPIO_Pin_12)
	
	#define ON_GPS_LED()			(GPIO_ResetBits(GPIOD, GPIO_Pin_13))
	#define OFF_GPS_LED()			(GPIO_SetBits(GPIOD, GPIO_Pin_13))
	#define CPL_GPS_LED()			(GPIOD->ODR ^= GPIO_Pin_13)
	
	#define ON_ERR_LED()			(GPIO_ResetBits(GPIOE, GPIO_Pin_3))
	#define OFF_ERR_LED()			(GPIO_SetBits(GPIOE, GPIO_Pin_3))
	#define CPL_ERR_LED()			(GPIOE->ODR ^= GPIO_Pin_3)
	
	#define FEED_WDT()       	(GPIOD->ODR ^= GPIO_Pin_8)	
	
	#define ON_485232_PWR()		(GPIO_SetBits(GPIOC, GPIO_Pin_7))///485
	#define OFF_485232_PWR()	(GPIO_ResetBits(GPIOC, GPIO_Pin_7))
	#define ENABLE_TX485()		(GPIO_SetBits(GPIOD, GPIO_Pin_4))
	#define ENABLE_RX485()		(GPIO_ResetBits(GPIOD, GPIO_Pin_4))

	#define ON_GPRS_PWR()			(GPIO_SetBits(GPIOC, GPIO_Pin_4))///L218
	#define OFF_GPRS_PWR()		(GPIO_ResetBits(GPIOC, GPIO_Pin_4))
	#define LOW_GPRS_IGT()		(GPIO_SetBits(GPIOE, GPIO_Pin_15))
	#define HIGH_GPRS_IGT()		(GPIO_ResetBits(GPIOE, GPIO_Pin_15))
	#define LOW_GPRS_DTR()		(GPIO_SetBits(GPIOC, GPIO_Pin_2))
	#define HIGH_GPRS_DTR()		(GPIO_ResetBits(GPIOC, GPIO_Pin_2))

	#define ON_CT5V_PWR()			(GPIO_SetBits(GPIOB, GPIO_Pin_13))///CAN电源、电池充电
	#define OFF_CT5V_PWR()		(GPIO_ResetBits(GPIOB, GPIO_Pin_13))
	#define ON_CAN_PWR()			ON_CT5V_PWR()
	#define OFF_CAN_PWR()			OFF_CT5V_PWR()
	
	DRIVER_EXTERN void GpioInit(void);
	
	#define LOCAL_USART_BPR		115200 //9600///本地串口
	#define GPS_USART_BPR			115200
	#define GPRS_USART_BPR		115200
	#define METER_USART_BPR		9600///仪表串口
	#define RS485_USART_BPR		9600///485串口

	#define LOCAL_USART				((USART_TypeDef *) USART1_BASE)
	#define LOCAL_USART_BASE	USART1_BASE
	#define GPS_USART					((USART_TypeDef *) USART2_BASE)
	#define GPS_USART_BASE		USART2_BASE
	#define GPRS_USART				((USART_TypeDef *) USART3_BASE)
	#define GPRS_USART_BASE		USART3_BASE
	#define METER_USART				((USART_TypeDef *) UART4_BASE)
	#define METER_USART_BASE	UART4_BASE	
	#define RS485_USART				((USART_TypeDef *) UART5_BASE)
	#define RS485_USART_BASE	UART5_BASE
		
	#define USART_PARITY_NO      ((uint16_t)0x0000)
	#define USART_PARITY_EVEN    ((uint16_t)0x0400)
	#define USART_PARITY_ODD     ((uint16_t)0x0600) 	
	#define USART_STOPBITS_1     ((uint16_t)0x0000)
	#define USART_STOPBITS_2     ((uint16_t)0x2000)
	#define USART_DATA_8B        ((uint16_t)0x0000)
	#define USART_DATA_9B        ((uint16_t)0x1000)
	
//	#define LOCAL_TEST_DEBUG   		1
//	#define LOCAL_PPP_DEBUG   		2
//	#define LOCAL_PRO_DEBUG   		3
//	#define LOCAL_GPS_DEBUG   		4

	#define LOCAL_UART_BUF_LEN 		64
	#define GPS_UART_BUF_LEN   		300//1000//
	#define GPRS_UART_BUF_LEN  		1500
	#define METER_UART_BUF_LEN   	256
	#define RS485_UART_BUF_LEN   	256	
	
	typedef struct
	{
		uint8  rx_flag;
		uint16 rx_head;
		uint16 rx_tail;
		uint8 *rx_buf;
	}USART_STRUCT;
	
	typedef struct
	{
		uint16 rx_head_point;
		uint16 rx_tail_point;
	}GPRS_USART_STRUCT;	
	
	DRIVER_EXTERN USART_STRUCT g_local_uart_struct;
	DRIVER_EXTERN USART_STRUCT g_gps_uart_struct;
	DRIVER_EXTERN USART_STRUCT g_gprs_uart_struct;
	DRIVER_EXTERN USART_STRUCT g_meter_uart_struct;
	DRIVER_EXTERN USART_STRUCT g_rs485_uart_struct;
	
	DRIVER_EXTERN uint8 local_uart_buf[LOCAL_UART_BUF_LEN];
	DRIVER_EXTERN uint8 gps_uart_buf[GPS_UART_BUF_LEN];
	DRIVER_EXTERN uint8 gprs_uart_buf[GPRS_UART_BUF_LEN];
	DRIVER_EXTERN uint8 meter_uart_buf[METER_UART_BUF_LEN];
	DRIVER_EXTERN uint8 rs485_uart_buf[RS485_UART_BUF_LEN];
	
	DRIVER_EXTERN void LocalUartFixedLenSend(uint8 data[],uint16 len);
	DRIVER_EXTERN void GpsUartFixedLenSend(uint8 data[],uint16 len);
	DRIVER_EXTERN void GprsUartFixedLenSend(uint8 data[],uint16 len);
	DRIVER_EXTERN void MeterUartFixedLenSend(uint8 data[],uint16 len);
	DRIVER_EXTERN void RS485UartFixedLenSend(uint8 data[],uint16 len) ;
	DRIVER_EXTERN void UsartGprsDeInit(void);///Gprs串口引脚反初始化
	DRIVER_EXTERN void UsartInit(USART_TypeDef* USARTx, u32 uart_bpr,uint8 data_bits,uint8 stop_bits,uint8 parity_check);
	
//	DRIVER_EXTERN GPRS_USART_STRUCT g_gprs_usart_struct;
//	#define UART_RX_5_MS_DELAY 		5
//	#define UART_RX_50_MS_DELAY 	50
//	#define UART_RX_500_MS_DELAY 	500
//	
//	#define SYS_TASK_SEC_TIMER 						60 ///系统任务执行周期
//	#define SYS_AHEAD_WAKEUP_SEC_TIMER 		120 ///GPS实现定位，休眠提前2分钟唤醒
//	#define SYS_DELAY_SLEEP_SEC_TIMER     180 ///ACC_OFF时，上传工作参数后，3分钟唤醒再次休眠，
	
	#define DMA_UART3_BUF_LEN			2048
	#define UART3_DMA_BUF_LEN			1024
	#define UART3_DMA_BUF_LEN2		1024
	DRIVER_EXTERN uint8  dma_uart3_buff[DMA_UART3_BUF_LEN];
	DRIVER_EXTERN	uint16 dma_uart3_buff_head;
	DRIVER_EXTERN uint16 dma_uart3_buff_tail;
	DRIVER_EXTERN uint8  uart3_dma_buff[UART3_DMA_BUF_LEN];
	DRIVER_EXTERN uint8  uart3_dma_buff2[UART3_DMA_BUF_LEN2];
	DRIVER_EXTERN	uint16 uart3_dma_buff_head2;
	DRIVER_EXTERN uint16 uart3_dma_buff_tail2;
	
	DRIVER_EXTERN void RtcConfiguration(void);
	DRIVER_EXTERN void RtcSetCalendarTime(uint8 data[]);
	DRIVER_EXTERN void RtcInit(void);
	DRIVER_EXTERN void SysClkConfigStop(void);
	DRIVER_EXTERN void TIM3_Int_Init(uint16 arr,uint16 psc);
	DRIVER_EXTERN void GetCalendarTime(uint8 date_time[],uint8 flag);
	
	DRIVER_EXTERN void IwdgInit(void);
	DRIVER_EXTERN void FeedWtd(void);
	DRIVER_EXTERN void SysReset(void);
	DRIVER_EXTERN void PvdInit(void);
	DRIVER_EXTERN void AdcInit(void);
	
	DRIVER_EXTERN uint16 adc_conv_buf[2];
	DRIVER_EXTERN uint16 adc_result[2];
	
#endif
