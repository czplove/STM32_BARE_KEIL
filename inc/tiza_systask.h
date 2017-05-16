#ifndef __SYS_TASK_H
#define __SYS_TASK_H

#include "stdint.h"

	#ifdef SYS_TASK_GLOBAL
		#define EXTERN_SYS_TASK
	#else
		#define EXTERN_SYS_TASK extern
	#endif

	
	#define BEFORE_ALARM_MAXINDEX				32	//40
	#define SYS_ALARM_MAXINDEX					80
	#define ALARM_PAGE_SIZE 						0x200						//512byte		
	#define SYS_ALARM_START_ADDR				FLASH_ALARM_START_ADDR	
	#define SYS_ALARM_END_ADDR					FLASH_ALARM_END_ADDR		//与SYS_ALARM_MAXINDEX对应
	#define SYS_ALARM_INDEXPAGE					FLASH_ALARM_INDEXPAGE		
	#define SYS_SAVEPARA_INDEXPAGE			FLASH_SAVEPARA_INDEXPAGE	
	#define SYS_SAVEVAILDPOS_INDEXPAGE	FLASH_SAVEVAILDPOS_INDEXPAGE
	#define SYS_SAVEVIN_INDEXPAGE				FLASH_SAVEVIN_INDEXPAGE
	
	#define SYS_SAVEALARM_NUMB					3				//系统未出现报警时 保留INTFLAH的记录条数  
	#define SYS_HAVEALARM_COUNT					35//40			//出现报警最大倒计时时间 s
	typedef struct
	{
		uint16 beforeheadindex;							  //报警前的存储下标号 RAM
		uint16 beforetailindex;							  //报警前的存储下标号
		
		uint32 afterheadindex;							  //包号(每块16段，每段4Kbety)
		uint32 aftertailindex;								//包号(每块16段，每段4Kbety)
	}SYS_ALARM_STRUCT;
		
			
	typedef struct
	{					
		uint8  need_check_time_flag;						///需要校验系统日期时间     01从GPS   02从平台  00无效
		uint8  have_sysAlarm_flag;							///系统出现报警             01有报警	02检测有无报警盲区 03无报警盲区	00无报警
		uint8  stop_sysAlarm_flag;							///在发送报警数据过程中 报警取消标志  00初始 01报警取消
		uint8  send_sysAlarm_numb;							///系统出现报警时 发送条数  
		PRO_TIME_UNION send_sysAlarm_time[2];			///系统出现报警时 发送使用时间
//		uint8  save_sysAlarm_numb;							///系统未出现报警时 保留INTFLAH的记录条数  
		uint8  have_sysAlarm_count;							///系统出现报警倒计时    

		uint8  main_power_statu;						///主电源状态     01上电1次  02断电  03再上电
		uint8  assist_gps_flag;							///辅助定位标志		02初始     00之前定位有效   01之前定位无效
		
		uint8  ProgramUpgrade_flag;					///固件升级标志		00初始		 01有要更新升级    02执行固件下载   03写BOOT标志  04
		uint8  GPRSreconnect2plat;					///模块断开服务器连接再重连接		00初始 01需要断开 02需要重连 03等待重连OK 04重连OK 05重启模块
		
	}SYS_MISC_RUN_STRUCT;///系统运行参数

	
	EXTERN_SYS_TASK SYS_ALARM_STRUCT 				g_sysalarm_struct;
	EXTERN_SYS_TASK SYS_MISC_RUN_STRUCT 		g_sysmiscrun_struct;		
	
	EXTERN_SYS_TASK	uint8 g_BeforeAlarmData[BEFORE_ALARM_MAXINDEX][512];	
			
			
	EXTERN_SYS_TASK void SetGPRSNetPara(void);
	EXTERN_SYS_TASK void ProPutIntoAlarm(uint8 data[], uint16 len, uint8 cmd);
	EXTERN_SYS_TASK void ReadWriteOldVaildPostion(uint8 rw);
	EXTERN_SYS_TASK void AlarmHaveJudge(uint8 flag);
	EXTERN_SYS_TASK void ReadyBeforeUpgrade(void);
#endif
