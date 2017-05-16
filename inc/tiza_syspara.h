#ifndef __SYS_PARA_H
#define __SYS_PARA_H

#include "stdint.h"

	#ifdef SYS_PARA_STRUCT_GLOBAL
		#define EXTERN_SYS_PARA
	#else
		#define EXTERN_SYS_PARA extern
	#endif
	
	
		
	#define SYSM_CANRX_TIMEOUT   60		//60个计数(1s调用就60s)	
	typedef struct{//系统程序块超时计时
		uint16 canrx_count;									///can长期收不到数据计数
		
		uint16 SavePeri_count;							///周期保存计时
		uint16 NorUpPeri_count;							///正常周期上传信息计时
		uint16 AlrUpPeri_count;							///报警上传间隔计时
//		uint16 TAckTim_count;								///终端应答超时时间					FF无效		
		uint16 PAckTim_count;								///平台应答超时时间					FF无效
		uint16 NLogTim_count;								///三次登录失败后重新登录时间间隔	
		uint8  upheart_count;								///系统上传心跳计时
	}SYSM_TIMEOUT_STRUCT;
	
	#define SYSM_ON							1
	#define SYSM_OFF						0
	typedef struct{//系统程序块是否启用 开关
		uint8 canrx_switch;										//接收中断
		uint8 GPRSPacketTx_switch;						//GPRS无需应答式发送 或 打包给周期发送
		uint8 GPRSPeriodTx_switch;						//GPRS需要应答式发送
		uint8 ADC_switch;											//ADC转换中断
		uint8 extflash_w_switch;							//片外flash擦写
		uint8 flash_w_switch;									//片内flash擦写
	}SYSM_ON_OFF_STRUCT;
	
	EXTERN_SYS_PARA SYSM_TIMEOUT_STRUCT g_sysm_timeout_struct;
	EXTERN_SYS_PARA SYSM_ON_OFF_STRUCT 	g_sysm_on_off_struct;
	
	EXTERN_SYS_PARA void Tz1000_Init(void);
	


#endif
