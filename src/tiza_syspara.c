#define SYS_PARA_STRUCT_GLOBAL

#include "tiza_include.h"

void SysPara_Init(void)
{
	//======================================//
	g_sysm_timeout_struct.SavePeri_count 	= 0;							///周期保存计时
	g_sysm_timeout_struct.NorUpPeri_count = 0;							///正常周期上传信息计时
	g_sysm_timeout_struct.AlrUpPeri_count	= 0;							///报警上传间隔计时
	g_sysm_timeout_struct.upheart_count		= 0;							///系统上传心跳计时
//	sysm_timeout_struct.TAckTim_count		= 0;						///终端应答超时时间		
	g_sysm_timeout_struct.PAckTim_count		= 0xFF;						///平台应答超时时间		
	
	//======================================//
	g_sysm_on_off_struct.canrx_switch 				= SYSM_ON;
	g_sysm_on_off_struct.GPRSPacketTx_switch	= SYSM_ON;	
	g_sysm_on_off_struct.GPRSPeriodTx_switch 	= SYSM_ON;
	g_sysm_on_off_struct.ADC_switch 					= SYSM_ON;
	g_sysm_on_off_struct.extflash_w_switch		= SYSM_ON;
	g_sysm_on_off_struct.flash_w_switch				= SYSM_ON;
	
	//======================================//
	g_sysmiscrun_struct.have_sysAlarm_flag 	 = 0;						///系统报警标志 同时send_sysAlarm_count=0
	g_sysmiscrun_struct.stop_sysAlarm_flag 	 = 0;						//在发送报警数据过程中 报警取消标志  00初始 01报警取消
	g_sysmiscrun_struct.need_check_time_flag = 1;						///需要校验系统日期时间     01从GPS   02从平台  00无效
	g_sysmiscrun_struct.have_sysAlarm_count  = 0;						///系统出现报警倒计时
//	g_sysmiscrun_struct.save_sysAlarm_numb   = SYS_SAVEALARM_NUMB;		///未出现报警时 保留INTFLAH的记录条数  
	g_sysmiscrun_struct.assist_gps_flag      = 2;						///辅助定位标志		02初始     00之前定位有效   01之前定位无效
	g_sysmiscrun_struct.GPRSreconnect2plat   = 0;						///模块断开服务器连接再重连接		00初始 01需要断开 02需要重连 03等待重连OK 04重连OK 05重启模块


}



void Tz1000_Init(void)
{
	ProPara_Init();
	SysPara_Init();
}




