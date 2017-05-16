#define BUSINESS_GLOABL
#define BUSINESS_DEBUG

#include "tiza_include.h"

//测试发送数据
//uint8  GPRStestdata[13] = {0x26,0x50,0x05,0x18,0x60,0x49,0xaa,0x00,0x16,0x55,0xaa,0xbb,	0x62};
//uint8  GPRStestdata[600];
uint32 GPRStestcount = 1;
/***************************模块复位控制函数***********
//模块复位启动函数
******************************************************/
void BGprsResetPara(void)
{
	g_gprs_data_struct.setnetparaok_flag 		= 0;
	SetGPRSNetPara();
//	g_propostion_union.Item.status.byte  		= 0x01;
	g_gps_struct.gpsinform.subitem.statu.byte = 0x01;									///初始化定位无效标志
	g_pro_struct.updata_sending 				= 0;
	g_pro_struct.updata_noacksend 			= 0;
	g_pro_struct.try_login_statu  			= 0;
	g_pro_struct.login_center_flag  		= FALSE;
	g_sysm_timeout_struct.NLogTim_count = 0xFFFF;		///三次登录失败后重新登录时间间隔	
	
//初始化模块状态
	g_gprs_data_struct.initStatus 		= GPRS_INIT_NO;
	g_gprs_data_struct.netLinkStatus 	= GPRS_NETLINK_NO;
	g_gprs_data_struct.MserLinkStatus = GPRS_SERLINK_NO;
	g_gprs_data_struct.sendDataStatus = GPRS_SENDDATA_ZERO;
	g_gprs_data_struct.GpsOpenStatus 	= GPS_OPEN_NO;
//设置状态机
	g_gprs_ctr_struct.business = GPRS_INIT;
	g_gprs_ctr_struct.ope = AT_INDEX;
	
	
//业务上逻辑
	g_business_struct.usemodule		= _YES;
	g_business_struct.needlinkser = _YES;
	g_business_struct.needopengps = _YES;//_NO;//
	
}


/******************************************************
//模块发送前准备
输出：3：准备完成,其他：失败
******************************************************/
static uint8 BusiSendReady(void)
{
	uint8 step = 0;
	
	if(g_business_struct.needlinkser == _NO){
		return step;
	}
	
	switch(step)
	{
		case 0:{
			if(g_gprs_data_struct.initStatus != GPRS_INIT_OK){
				g_gprs_ctr_struct.business = GPRS_INIT;
				g_gprs_ctr_struct.ope = AT_INDEX;
				break;
			}
			else
				step = 1;
		}
		case 1:{
			if(g_gprs_data_struct.netLinkStatus != GPRS_NETLINK_OK){
				g_gprs_ctr_struct.business = GPRS_CONNECT_NET;
				g_gprs_ctr_struct.ope = AT_NETCLOSE_INDEX;//AT_CIPMUX_INDEX;//
				break;
			}
			else
				step = 2;
		}
		case 2:{
			if(g_gprs_data_struct.MserLinkStatus != GPRS_SERLINK_OK){
				g_gprs_ctr_struct.business = GPRS_CONNECT_SERVER;
				break;
			}
			else
				step = 3;
		}
		default: break;
	}
	return step;
}

/******************************************************
//断开模块与服务器连接
输出：
******************************************************/
void BusiDisconSer(void)
{
	g_pro_struct.updata_sending 				= 0;
	g_pro_struct.updata_noacksend 			= 0;
	g_pro_struct.try_login_statu  			= 0;
	g_pro_struct.login_center_flag  		= FALSE;
	g_sysm_timeout_struct.NLogTim_count = 0xFFFF;		///三次登录失败后重新登录时间间隔	
	//关需要连接标志
	g_business_struct.needlinkser = _NO;
	//清状态
	g_gprs_data_struct.sendDataStatus = GPRS_SENDDATA_ZERO;
	g_gprs_data_struct.MserLinkStatus = GPRS_SERLINK_NO;
	//执行业务
	g_gprs_ctr_struct.business 				= GPRS_DISCONNECT_SERVER;
}
/******************************************************
//启用模块与服务器连接
输出：
******************************************************/
void BusiConSer(void)
{
	//开需要连接标志
	g_business_struct.usemodule		= _YES;
	g_business_struct.needlinkser = _YES;
	//清状态
	g_gprs_data_struct.sendDataStatus = GPRS_SENDDATA_ZERO;
	g_gprs_data_struct.MserLinkStatus = GPRS_SERLINK_NO;
	//执行业务
	g_gprs_ctr_struct.business 				= GPRS_NULL;
}
/******************************************************
//关闭GPS数据
输出：
******************************************************/
void BusiDisconGps(void)
{
	//关需要连接标志
	g_business_struct.needopengps = _NO;
	//清状态
	g_gprs_data_struct.GpsOpenStatus = GPS_OPEN_NO;
	//执行业务
	g_gprs_ctr_struct.business 			 = GPS_CLOSE_NET;
}
/******************************************************
//开启GPS数据
输出：
******************************************************/
void BusiConGps(void)
{
	//开需要连接标志
	g_business_struct.usemodule		= _YES;
	g_business_struct.needopengps = _YES;//_NO;//
	//清状态
	g_gprs_data_struct.GpsOpenStatus = GPS_OPEN_NO;
	//执行业务
	g_gprs_ctr_struct.business 			 = GPS_OPEN_NET;
}
/******************************************************
//复位模块
输出：
******************************************************/
void BusiResetModule(void)
{
	//关启用模块标志
	g_business_struct.usemodule = _NO;
	//重启模块电源	L218Reset();
	g_gprs_ctr_struct.business 	= GPRS_RESET;
	ModlueCalledProcess();
	//重置参数
	BGprsResetPara();
}


/******************************************************
//模块重连平台程序
//如果重连接后3次登录失败就重启模块 在周期发送操作
******************************************************/
static void GPRS_Reconnect2Platform(void)
{
	switch(g_sysmiscrun_struct.GPRSreconnect2plat){
		case 1:{
			BusiDisconSer();
			g_sysmiscrun_struct.GPRSreconnect2plat = 2;
		break;
		}
		case 2:{
			if(g_gprs_data_struct.netLinkStatus == GPRS_NETLINK_NO){
				BusiConSer();
				g_sysmiscrun_struct.GPRSreconnect2plat = 3;
			}
			else if(g_gprs_data_struct.netLinkStatus == GPRS_NETLINK_ERR){
				#ifdef BUSINESS_DEBUG
				LocalUartFixedLenSend((uint8*)"\r\n redisconnect fail",StrLen((uint8*)"\r\n redisconnect fail",0));
				#endif
				g_sysmiscrun_struct.GPRSreconnect2plat = 1;
			}
		break;
		}
		case 3:{
			if(g_gprs_data_struct.netLinkStatus == GPRS_NETLINK_OK){
				g_sysmiscrun_struct.GPRSreconnect2plat = 4;
			}
			else if(g_gprs_data_struct.netLinkStatus == GPRS_NETLINK_ERR){
				#ifdef BUSINESS_DEBUG
				LocalUartFixedLenSend((uint8*)"\r\n reconnect fail",StrLen((uint8*)"\r\n reconnect fail",0));
				#endif
				g_sysmiscrun_struct.GPRSreconnect2plat = 2;
			}
		break;
		}
		case 5:{ //重启模块
			g_sysmiscrun_struct.GPRSreconnect2plat = 0;
			BusiResetModule();
		break;
		}
		default :{	break;		}
	}
}
/******************************************************
//模块执行任务调用
输出：
******************************************************/
void ExecuteModuleTask(void)
{	
		if(g_business_struct.usemodule == _NO){
			return;
		}
		///模块调度函数
		ModlueCalledProcess();
			
		if(g_sysmiscrun_struct.ProgramUpgrade_flag > 1){ // 0 1
			return ;
		}
		if(g_gprs_data_struct.initStatus != GPRS_INIT_OK){	
			g_pro_struct.link_center_flag = 0;			
			return;
		}
		g_pro_struct.link_center_flag = 1;		

		///           不启用GPS               或               GPS正确打开(判断考虑有效定位标志) 
		if(g_business_struct.needopengps==_NO || g_gprs_data_struct.GpsOpenStatus==GPS_OPEN_OK)
		{	
			if(g_gprs_data_struct.setnetparaok_flag != 1){
				return;		//参数未设置好，退出
			}
			if(GpsAssistProcess() == 2){
				return;
			}

			//准备使用GPRS发送数据		
			switch(g_gprs_data_struct.sendDataStatus)
			{
				case GPRS_SENDDATA_ZERO:{
					if(BusiSendReady() == 3){
						g_gprs_data_struct.sendDataStatus = GPRS_SENDDATA_IDLE;
					}
					break;
				}
				case GPRS_SENDDATA_IDLE:{//空闲或发送成功
					if(g_pro_struct.try_login_statu == 0){		//置登录标志，只一次
						g_pro_struct.try_login_statu = 1;
					}
					//上传ACK（上发不需要应答的报文）
					if(g_pro_struct.updata_noacksend == 1){
						g_pro_struct.updata_noacksend = 2;
						g_gprs_ctr_struct.business = GPRS_NULL;
						g_gprs_data_struct.sendDataStatus = GPRS_SENDDATA_OUT;
						break;
					}
					else if(g_pro_struct.updata_noacksend == 2){
						g_gprs_ctr_struct.business = GPRS_NULL;
						g_pro_struct.updata_sending = 0;		
						g_pro_struct.updata_noacksend = 0;
						break;
					}
					//更新升级
					if(g_sysmiscrun_struct.ProgramUpgrade_flag == 1){
						ReadyBeforeUpgrade();
						break;
					}
				
					g_gprs_ctr_struct.business = GPRS_NULL;
					g_pro_struct.updata_sending = 0;				
					break;
				}
				case GPRS_SENDDATA_OUT:{//有数据要发送
					g_gprs_ctr_struct.business = GPRS_SEND_DATA;
					break;
				}
				case GPRS_SENDDATA_BUSY:{//阻塞或发送失败
//					#ifdef BUSINESS_DEBUG
//						printf("\r\n L218 send data fail,we will reset l218 module\r\n");	
//					#endif
//					BusiResetModule();

					#ifdef BUSINESS_DEBUG
					LocalUartFixedLenSend((uint8*)"\r\n L218 send data fail,we will reconnect to platform now;\r\n",StrLen((uint8*)"\r\n L218 send data fail,we will reconnect to platform now;\r\n",0));
					#endif
					g_sysmiscrun_struct.GPRSreconnect2plat   = 1; //置重连平台标志
//					g_gprs_data_struct.sendDataStatus = GPRS_SENDDATA_IDLE;
					break;
				}
				default:					break;
			}
 		}
		///                   启用GPS               且               GPS尚未成功打开 
		else if(g_business_struct.needopengps==_YES && g_gprs_data_struct.GpsOpenStatus!=GPS_OPEN_OK)
		{
			if(g_gprs_data_struct.GpsOpenStatus != GPS_OPEN_ING){
				g_gprs_ctr_struct.business = GPS_OPEN_NET;
				g_gprs_ctr_struct.ope = AT_GETGPS_INDEX;
			}
		}
}


/******************************************************
//L218任务
******************************************************/
void L218_task(void *pdata)
{	 	
	L218PowerInit();	
	BGprsResetPara();
	#ifdef L218_DEBUG
	LocalUartFixedLenSend((uint8*)"\r\n##### L218 POWER OK #####\r\n",StrLen((uint8*)"\r\n##### L218 POWER OK #####\r\n",0));
	#endif	
	

/////IP：220.169.30.122 端口：9876
//	g_gprs_data_struct.IP[0] = 220;
//	g_gprs_data_struct.IP[1] = 169;
//	g_gprs_data_struct.IP[2] = 30;
//	g_gprs_data_struct.IP[3] = 122;
//	g_gprs_data_struct.Port = 9876;

	SetGPRSNetPara();
//	ProParaInit();
	
	while(1)
	{
//		if(g_sysmiscrun_struct.have_sysAlarm_flag==1 || g_sysmiscrun_struct.have_sysAlarm_flag==2){
//			OSTimeDlyHMSM(0, 0, 0, 200);
//		}
//		else{
//			if(g_propara_union.Item.NorUpPeri < 5){				
//				OSTimeDlyHMSM(0, 0, 0, 200);
//			}
//			else if(g_propara_union.Item.NorUpPeri < 10){				
//				OSTimeDlyHMSM(0, 0, 0, 500);
//			}
//			else{
//				OSTimeDlyHMSM(0, 0, 1, 0);
//			}
//		}
		OSTimeDlyHMSM(0, 0, 0, 200);	
		GPRS_Reconnect2Platform();
		ExecuteModuleTask();
	}
}


