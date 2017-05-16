
#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "stdint.h"

	#ifdef PROTOCOL_GLOBAL
		#define PROTOCOL_EXTERN
	#else
		#define PROTOCOL_EXTERN extern
	#endif
/*
驱动电机个数 1
储能子系统个数  1
单体电池总数		96   /4 = 24
探针个数				24	 /8 = 3
*/
	///上行命令ID	
	#define PRO_UP_LOGIN_ID						0X01		///车辆登入
	#define PRO_UP_REAL_INFO_ID				0X02		///实时信息上报
	#define PRO_UP_LSNAL_INFO_ID			0X03		///补发信息上报
	#define PRO_UP_LOGOUT_ID					0X04		///车辆登出
	#define PRO_UP_HEARTBEAT_ID				0X07		///心跳
	#define PRO_UP_CHECKTIME_ID				0X08		///终端校时
	//0x05~0x06 平台传输数据占用    自定义
	//0x09~0x7F 上行数据系统预留    
	
	///下行命令ID
	#define PRO_DOWN_INQUIRY_ID				0X80		///查询命令
	#define PRO_DOWN_SET_ID						0X81		///设置命令
	#define PRO_DOWN_CONTROL_ID				0X82		///车载终端控制命令
	//0x83~0xBF 下行数据系统预留    
	//0xC0~0xFE 平台交换自定义数据    自定义
	
	///命令单元-应答标志
	#define PRO_ACK_SUCCEED_FLAG			0X01		///接收到的信息正确
	#define PRO_ACK_FAILED_FLAG				0X02		///设置失败
	#define PRO_ACK_VIN_ERR_FLAG			0X03		///VIN重复错误
	#define PRO_ACK_CMD_FLAG					0XFE		///表示本包为命令包，而非应答包
	
	///数据加密方式
	#define PRO_DISENCRYT_MODE				0X01		///不加密
	#define PRO_RSAENCRYT_MODE				0X02		///RSA加密
	#define PRO_AES128ENCRYT_MODE			0X03		///AES128加密
	
	///信息类型标志
	#define PRO_VEHICLE_FLAG					0X01		///整车数据
	#define PRO_MOTOR_FLAG						0X02		///驱动电机数据
	#define PRO_FUELCELL_FLAG					0X03		///燃料电池数据
	#define PRO_ENGINE_FLAG						0X04		///发动机数据
	#define PRO_POSITION_FLAG					0X05		///车辆位置
	#define PRO_EXTREME_FLAG					0X06		///极值数据
	#define PRO_ALARM_FLAG						0X07		///报警数据
	#define PRO_BVOLTAGE_FLAG					0X08		///可充电储能装置电压数据
	#define PRO_BTEMPERATURE_FLAG			0X09		///可充电储能装置温度数据
	//0x0A~0x2F   平台交换协议数据
	//0x30~0x7F   预留
	//0x80~0xFE   用户自定义
	
	///
	#define MAX9999999	9999999
	#define MAX65531		65531
	#define MAX60000		60000
	#define MAX253			253
	#define MAX250			250
	///时间格式CMT+8
	typedef union {
		uint8 arry[6];
		
		struct{
			uint8 year;			//0-99
			uint8 month;		//1-12
			uint8 day;			//1-31
			uint8 hour;			//0-23
			uint8 minute;		//0-59
			uint8 second;		//0-59
		}Item;
	}PRO_TIME_UNION;
	
	///车辆登入数据格式
	typedef union{
//		uint8 arry[34];
		uint8 arry[24];
		struct{
//			PRO_TIME_UNION time_union;					//数据采集时间
			uint16 login_number;								//登入流水号	+1 最大65531
			uint8  SIM_ICCID[20];								//终端从SIM获得的ICCID
			uint8  storage_num;									//可充电储能子系统数n=0~250
			uint8  storage_len;									//可充电储能子系统编码长度m=0~50;0表示不上传该编码
//			uint8* storage_code;								//可充电储能系统编码，终端从获取
			
		}Item;
	}PRO_LOGIN_FORM_UNION;
	
	///实时信息上报格式
#if 1	//信息体	
	///整车数据
	typedef union {
		uint8 arry[20];
		
		struct{
			uint32 mileage;				//累计里程			0~9999999(0~999999.9km)    FFFFFFFE异常  FFFFFFFF无效
			uint16 speed;					//车速					0~2200(0~220km/h) FFFE异常 FFFF无效
			uint16 voltage;				//总电压				0~10000(0~1000V)  FFFE异常 FFFF无效
			uint16 current;				//总电流				0~20000(-1000~+1000A)      FFFE异常      FFFF无效
			uint16 resistance;		//绝缘电阻			0~60000(0~60000kO)
			uint8  vehicle;				//车辆状态			01启动   02熄火   03其他   FE异常 FF无效
			uint8  charge;				//充电状态			01停车充 02行驶充 03未充态 04充完 FE异常 FF无效
			uint8  mode;					//运行模式			01纯电   02混动   03燃油   FE异常 FF无效
			uint8  SOC;						//SOC						0~100(0%~100%)		FE异常   FF无效
			uint8  DC_statu;			//DC/DC状态			01工作   02断开   FE异常   FF无效
			union08 stall;				//档位					b5:1/0有/无驱动力  b4:1/0有/无制动力 L:0空 1~C档 D倒 E自动d档 F停车p档 
			uint8  Accelerate;		//加速踏板行程值0~100(0%~100%)    FE异常   FF无效
			uint8  braking;				//制动踏板状态	0~100(0%~100%)    00制动关 65制动有效    FE异常  FF无效
		}Item;
	}PRO_VEHICLE_UNION;
	///驱动电机数据
	#define PRO_MOTOR_NUMBER					1				//驱动电机数据——驱动电机个数	
	PROTOCOL_EXTERN uint8  g_motor_number;										//驱动电机个数	1~253
	typedef union{//驱动电机总成信息列表
		uint8 arry[12];
		
		struct{
			uint8  serial;										//驱动电机序号	1~253
			uint8  status;										//状态					01耗电 02发电 03关闭态 04准备态 FE异常 FF无效
			uint8  ctr_tem;										//控制器温度		0~250(-40~+210℃)								FE异常 FF无效
			uint8  motor_tem;									//驱动电机温度	0~250(-40~+210℃)								FE异常 FF无效
			uint16 rpm;												//转速					0~65531(-20000~+45531r/min)	FFFE异常 FFFF无效
			uint16 torque;										//转矩					0~65531(-2000~+4553.1Nm)		FFFE异常 FFFF无效
			uint16 ctr_in_vol;								//控制器输入电压0~60000(0~6000V)						FFFE异常 FFFF无效
			uint16 ctr_bus_cur;								//控制器母线电流0~20000(-1000~+1000A)				FFFE异常 FFFF无效
		}Item;
	}PRO_MOTOR_UNION;
	
	///燃料电池数据
	#define PRO_FTPROBE_NUMBER					24				//燃料电池数据——温度探针个数	
	PROTOCOL_EXTERN uint8 g_TProbe_val[PRO_FTPROBE_NUMBER];		//探针温度值	0~240(-40~+200℃)
	typedef union{
		uint8 arry[18];
//		uint8 arry[18+PRO_FTPROBE_NUMBER];
		
		struct{
			uint16 voltage;										//电压					0~20000(0~2000V)				FFFE异常 FFFF无效
			uint16 current;										//电流					0~20000(0~2000A)				FFFE异常 FFFF无效
			uint16 consumpt;									//消耗率				0~60000(0~600kg/100km)	FFFE异常 FFFF无效
			uint16 tprobe_num;								//温度探针个数	0~65531									FFFE异常 FFFF无效
//注意改arry	uint8 g_TProbe_val[PRO_FTPROBE_NUMBER];		//探针温度值	0~240(-40~+200℃)
			uint16 maxtem_val;								//氢系统最高温度0~2400(-40~+200℃)			FFFE异常 FFFF无效
			uint16 maxconcent;								//氢气最高浓度	0~60000(0~50000mg/kg)		FFFE异常 FFFF无效
			uint16 maxpre_val;								//氢气最高压力	0~1000(0~100MPa)				FFFE异常 FFFF无效
			uint8  maxtem_id;								  //最高温度探针号0~252										FE异常 		FF无效
			uint8  maxconcent_id;						  //最高浓度传感号0~252										FE异常 		FF无效
			uint8  maxpre_id;									//最高压力传感号0~252										FE异常 		FF无效
			uint8  preDCstatu;								//高压DCDC状态	01工作 02断开 FE异常 FF无效
			}Item;
	}PRO_FUELCELL_UNION;	
	
	///发动机数据
	typedef union{
		uint8 arry[5];
		
		struct{
			uint16 rpm;										//曲轴转速			0~60000(0~60000r/min)		FFFE异常 FFFF无效
			uint16 consumpt;							//消耗率				0~60000(0~600L/100km)		FFFE异常 FFFF无效
			uint8  status;								//发动机状态		01启动态 02关闭态 FE异常 FF无效								
			}Item;
	}PRO_ENGINE_UNION;	
	
	///车辆位置
	typedef union{
		uint8 arry[9];
		
		struct{		
			uint32 longitude;							//经度			以度为单位*10^6，精确到百万分之一度
			uint32 latitude;							//纬度			同上
			union08  status;							//定位状态	b0:1/0无/有效定位	b1:1/0南/北纬	b2:1/0西/东经		
			}Item;
	}PRO_POSTION_UNION;	
	
	///极值数据
	typedef union{
		uint8 arry[14];
		
		struct{
			uint8  maxvolsys_id;								//最高电压电池子系统号	1~250										FE异常 		FF无效
			uint8  maxvolbat_id;								//最高电压电池单体号		1~250										FE异常 		FF无效
			uint16 maxvolbat_val;								//最高电压电池单体值		0~15000(0~15V)				FFFE异常 FFFF无效
			uint8  minvolsys_id;								//最低电压电池子系统号	1~250										FE异常 		FF无效
			uint8  minvolbat_id;								//最低电压电池单体号		1~250										FE异常 		FF无效
			uint16 minvolbat_val;								//最低电压电池单体值		0~15000(0~15V)				FFFE异常 FFFF无效
			uint8  maxtemsys_id;								//最高温度子系统号			1~250										FE异常 		FF无效
			uint8  maxtempro_id;								//最高温度探针号				1~250										FE异常 		FF无效
			uint8  maxtempro_val;								//最高温度探针值				0~250(-40~+210℃)				FE异常 		FF无效
			uint8  mintemsys_id;								//最低温度子系统号			1~250										FE异常 		FF无效
			uint8  mintempro_id;								//最低温度探针号				1~250										FE异常 		FF无效
			uint8  mintempro_val;								//最低温度探针值				0~250(-40~+210℃)				FE异常 		FF无效
			}Item;
	}PRO_EXTREME_UNION;	
	
	///报警数据
	#define PRO_ASTORAGE_MAXNUMBER				4					//报警数据——可充电储能装置故障个数	
	#define PRO_AMOTOR_MAXNUMBER					4					//报警数据——驱动电机故障个数	
//	#define PRO_AENFINE_MAXNUMBER					0					//报警数据——发动机故障个数	
	#define PRO_AOTHERS_MAXNUMBER					4					//报警数据——其他故障个数	
	PROTOCOL_EXTERN uint32 g_storage_alr[PRO_ASTORAGE_MAXNUMBER];	//可充电储能装置故障码	自定义
	PROTOCOL_EXTERN uint32 g_motro_alr[PRO_AMOTOR_MAXNUMBER];			//驱动电机故障码				自定义
//	PROTOCOL_EXTERN uint32 g_engine_alr[PRO_AENFINE_MAXNUMBER];		//发动机故障码					自定义
	PROTOCOL_EXTERN uint32 g_others_alr[PRO_AOTHERS_MAXNUMBER];		//其他故障码						自定义
	typedef union{
		uint8 arry[9];
		
		struct{
			union32 flag;					//通用报警标志					b0~b18:0正常/1报警   b19~b31:预留
			uint8  rank;					//最高报警等级					00无故障 1~3:几级故障 FE异常 FF无效		等级故障自定义
			uint8  storagenum;		//可充电储能装置故障数	0~252										FE异常 		FF无效
			uint8  motornum;			//驱动电机故障数				0~252										FE异常 		FF无效
			uint8  enginenum;			//发动机故障数					0~252										FE异常 		FF无效
			uint8  othersnum;			//其他故障数						0~252										FE异常 		FF无效
		}Item;
	}PRO_ALARM_UNION;	
	
	///可充电电池电压数据
	#define PRO_BATTSYS_NUMBER						1						//可充电储能装置电压数据格式——可充电储能子系统个数	
	#define PRO_SIGBATT_MAXNUMBER					96					//本帧单体电池总数
	PROTOCOL_EXTERN uint16 g_sigbatt_val[PRO_BATTSYS_NUMBER][PRO_SIGBATT_MAXNUMBER];		//单体电池电压		0~60000(0~60.000V)	FFFE异常 FFFF无效
	typedef union{//可充电储能子系统电压信息列表
		uint8 arry[10];
		
		struct{
			uint8  serial;								//子系统号					1~250
			uint8  framebatt_num;					//本帧单体电池总数	1~200										FE异常 FF无效
			uint16 voltage;								//装置电压					0~10000(0~1000V)			FFFE异常 FFFF无效
			uint16 current;								//装置电流					0~20000(-1000~+1000A)	FFFE异常 FFFF无效
			uint16 sigbatt_num;						//单体电池总数			1~65531								FFFE异常 FFFF无效
			uint16 framebatt_id;					//本帧起始电池号		1~65531								FFFE异常 FFFF无效	单帧最多200个
		}Item;
	}PRO_VBATTSYS_UNION;
	
	///可充电电池温度数据
	/*可充电储能子系统个数	同电压数据上所示*/
	#define PRO_BTPROBE_MAXNUMBER					24				//可充电温度探针个数
	PROTOCOL_EXTERN uint8 g_btprobe_val[PRO_BATTSYS_NUMBER][PRO_BTPROBE_MAXNUMBER];		//各探针温度值		0~250(-40~+210℃)		FE异常 FF无效
	typedef union{//可充电储能子系统温度信息列表
		uint8 arry[3];
		
		struct{
			uint16 btprobe_num;						//可充电温度探针个数1~65531					FFFE异常 FFFF无效
			uint8  serial;								//子系统号					1~250
		}Item;
	}PRO_TBATTSYS_UNION;

	///自定义数据
	#define PRO_SEFDAT_LEN	2						//1~65531		自定义数据长度
	//PROTOCOL_EXTERN uint16 g_sefdata_len;								//1~65531		自定义数据长度
	//PROTOCOL_EXTERN uint8  g_sefdata[PRO_SEFDAT_LEN];		//自定义
	
	///车辆登出数据格式
	typedef union{
		uint8 arry[8];
		
		struct{
			PRO_TIME_UNION time_union;					//数据采集时间
			uint16 logout_number;								//登出流水号  要与登入流水号一致
		}Item;
	}PRO_LOGOUT_FORM_UNION;
	
#endif		
	
	
	
	#define PRO_CMD_INDEX						 2	//命令标志下标
	#define PRO_ACK_INDEX						 3	//应答标志下标
	#define PRO_DATALEN_INDEX				22	//数据长度下标
	#define PRO_DATA_INDEX					24	//数据开始下标
	#define PRO_TIME_INDEX1					24	//时间开始下标1
	
	PROTOCOL_EXTERN uint8 g_VIN[17];		//车辆唯一VIN
	PROTOCOL_EXTERN PRO_TIME_UNION 				g_protime_union;
	PROTOCOL_EXTERN PRO_LOGIN_FORM_UNION	g_prologin_union;
	PROTOCOL_EXTERN PRO_VEHICLE_UNION			g_provehice_union;
	PROTOCOL_EXTERN PRO_MOTOR_UNION				g_promotor_union[PRO_MOTOR_NUMBER];
	PROTOCOL_EXTERN PRO_FUELCELL_UNION		g_profullcell_union;
	PROTOCOL_EXTERN PRO_ENGINE_UNION			g_proengine_union;
	PROTOCOL_EXTERN PRO_POSTION_UNION			g_propostion_union;
	PROTOCOL_EXTERN PRO_EXTREME_UNION			g_proextreme_union;
	PROTOCOL_EXTERN PRO_ALARM_UNION				g_proalarm_union;
	PROTOCOL_EXTERN PRO_VBATTSYS_UNION		g_provbattsys_union;
	PROTOCOL_EXTERN PRO_TBATTSYS_UNION		g_protbattsys_union;
	
	
	
	
	///////////////////////////////////////////////////////////////////////////////
	
	///参数查询设置ID
	#define PRO_PARA_SAVEPERI_ID		0X01			///终端本地保存周期
	#define PRO_PARA_NORUPPERI_ID		0X02			///正常时上传周期
	#define PRO_PARA_ALRUPPERI_ID		0X03			///报警时上传周期
	#define PRO_PARA_DOMAINLEN_ID		0X04			///平台域名长度
	#define PRO_PARA_DOMAIN_ID			0X05			///平台域名
	#define PRO_PARA_PORT_ID				0X06			///平台端口
	#define PRO_PARA_HDVERSION_ID		0X07			///终端硬件版本 		R
	#define PRO_PARA_SFVERSION_ID		0X08			///终端固件版本			R
	#define PRO_PARA_HEARTPERI_ID		0X09			///终端心跳发送周期
	#define PRO_PARA_T_ACK_TIM_ID		0X0A			///终端应答超时时间
	#define PRO_PARA_P_ACK_TIM_ID		0X0B			///平台应答超时时间
	#define PRO_PARA_N_LOG_TIM_ID		0X0C			///3次失败登录下次登录时间间隔
	#define PRO_PARA_PDOMAINLEN_ID	0X0D			///公共平台域名长度
	#define PRO_PARA_PDOMAIN_ID			0X0E			///公共平台域名
	#define PRO_PARA_PPORT_ID				0X0F			///公共平台端口
	#define PRO_PARA_MONITOR_ID			0X10			///是否抽样检测
	//0x11~0x7F		预留
	//0x80~0xFE		用户自定义
	
	///参数值
	#define PRO_DOMAINLEN_MAX			30											///平台或公共平台域名最大长度 暂定
	#define PRO_PARA_UNION_LEN		90  //19+2*PRO_DOMAINLEN_MAX+11		///参数设置结构总长度

	typedef union{
		uint8 arry[PRO_PARA_UNION_LEN];
		
		struct{
			uint16 SavePeri;									//终端本地保存周期							0~60000(0~60000ms)			FFFE异常 FFFF无效
			uint16 NorUpPeri;									//正常时上传周期								1~600(1~600s)						FFFE异常 FFFF无效
			uint16 AlrUpPeri;									//报警时上传周期								0~60000(0~60000ms)			FFFE异常 FFFF无效
			uint8  DomainLen;									//平台域名长度									 										
			uint8  Domain[PRO_DOMAINLEN_MAX];	//平台域名											 
			uint8  HeartPeri;									//终端心跳发送周期							1~240(1~240s)						FE异常 		FF无效
			uint16 Port;											//平台端口											0~65531(0~15V)					FFFE异常 FFFF无效
			uint16 TAckTim;										//终端应答超时时间							1~600(1~600s)						FFFE异常 FFFF无效
			uint16 PAckTim;										//平台应答超时时间							1~600(1~600s)						FFFE异常 FFFF无效
			uint8  NLogTim;										//3次失败登录下次登录时间间隔		1~240(1~24min)					FE异常 		FF无效
			uint8  PDomainLen;								//公共平台域名长度							
			uint8  PDomain[PRO_DOMAINLEN_MAX];//公共平台域名									
			uint16 PPort;											//公共平台端口									0~65531(0~15V)					FFFE异常 FFFF无效
			uint8  Monitor;										//是否抽样检测中								01抽样检测中 02不在抽样检测中	FE异常 		FF无效
			
			uint8  g_para_HDVers[5];		//硬件版本号	R只读			
			uint8  g_para_SFVers[5];		//固件版本号	R只读
			uint8  check;							//校验码  保存读取用
			}Item;
	}PRO_PARA_UNION;
	
	///控制命令ID
	#define PRO_CTR_UPGRADE_ID			0X01			///远程升级
	#define PRO_CTR_OFFTERM_ID			0X02			///终端关机
	#define PRO_CTR_RSTTERM_ID			0X03			///终端复位
	#define PRO_CTR_FACTRST_ID			0X04			///终端恢复出厂设置
	#define PRO_CTR_DISLINKSER_ID		0X05			///断开数据通信链路
	#define PRO_CTR_ALRTERM_ID			0X06			///终端报警、预警
	#define PRO_CTR_MONITOR_ID			0X07			///开启抽样检测链路
	//0x08~0x7F		预留
	//0x80~0xFE		用户自定义
	
	///远程升级命令格式
	#define PRO_UPGRADE_PARA_NUM		10				///升级参数个数
	typedef struct{		
		uint8  dial_APN[6];			//拨号点名称
		uint8  user_name[16];		//拨号用户名
		uint8  pass_word[16];		//拨号密码
		uint8  ip_domain[6];		//如果是IPV4则前两位为0;
		uint16 port;						//端口号
		uint32 terminal_id;			//终端ID
		uint8  HDVers[5];//硬件版本号*5
		uint8  SFVers[5];//软件版本号*5
		uint8  file_path[64];		//升级URL地址
		uint16 overtime;				//连接到升级服务器时限min
		
		uint8 flag;							//自添加，1需要升级 2正在升级 0无效
	}PRO_UPGREAD_STRUCT;
	
	
	
	PROTOCOL_EXTERN PRO_PARA_UNION 			g_propara_union;
	PROTOCOL_EXTERN PRO_UPGREAD_STRUCT	g_proupgread_struct;
	
	
	
////////////////////////////////////////////////////////////////////////////////	
	
	#define PRO_MAX_TX_BUF_ARRAY 					4			///最大4个发送缓存
	#define PRO_MAX_TX_BUF_COUNTER 				520		///每个发送缓存最大600字节
	#define PRO_NORMAL_RETX_TIME					8			///8秒重发机制
	#define PRO_OVER3_RELOGIN_TIME				1800	///三次登录失败后30min重新登录
	
	typedef struct
	{
		uint8  re_tx_buf[PRO_MAX_TX_BUF_ARRAY][PRO_MAX_TX_BUF_COUNTER];
		uint16 re_tx_len[PRO_MAX_TX_BUF_ARRAY];
		uint8  re_tx_counter[PRO_MAX_TX_BUF_ARRAY];
		uint8  re_tx_sec_counter[PRO_MAX_TX_BUF_ARRAY];						//重发间隔秒计数
		uint8  re_tx_time[PRO_MAX_TX_BUF_ARRAY];									//重发时间间隔	
		uint8  re_tx_full_flag[PRO_MAX_TX_BUF_ARRAY];							//有数据标志
	}TX_STRUCT;
	typedef struct
	{
		uint8  link_center_flag;							///连接平台标志		1已连接		0未连接
		uint8  updata_sending;								///数据上传标志		1正在上传 0空闲  
		uint8  updata_noacksend;							///上传不需要ACK数据		1需要上传 2正在发送 0空闲  
		
		uint8  try_login_statu;								///登录状态标志 0初始 1使能 2正在登录 3已登录 4正在登出 5已登出 6无效
		uint8  login_center_flag;							///登录中心成功标志 即try_login_statu = 3

		uint8  tx_lsnal_data_flag;						///上一条指令应答成功使能检测有无盲区数据发送   1:使能
		uint8  tx_one_lsnal_falg;							///从EXTFLASH中读出的一条盲区记录正在发送   		1:在发送	0无效

		TX_STRUCT tx_struct;
	}PRO_STRUCT;
	
	
	#define LSNAL_PAGE_SIZE 					0x200				///暂定512，每段放8个,<<9
	#define SYS_LSNAL_SPIMAXINDEX			12288				///96*16*8
	#define SYS_LSNAL_SPISTART_ADDR		SPIFLASH_LSNAL_START_ADD
	#define SYS_LSNAL_SPIEND_ADDR			SPIFLASH_LSNAL_END_ADD
	#define SYS_LSNAL_SPIINDEX_ADDR		SPIFLASH_LSNAL_ADD	///最后1M放其他
	#define SYS_LSNAL_SPIINDEXSECTOR	SPIFLASH_LSNAL_SEC	///页码下标保存起始段号
	
	#define TMP_BUFF_MAXLEN						LSNAL_PAGE_SIZE	
	typedef struct
	{
		uint8  data[LSNAL_PAGE_SIZE];		//一包盲区数据大小
		uint32 headindex;							  //包号(每块16段，每段4Kbety)
		uint32 tailindex;								//包号(每块16段，每段4Kbety)
	}SYS_LSNAL_STRUCT;
		
	
	PROTOCOL_EXTERN SYS_LSNAL_STRUCT g_syslsnal_struct;
	PROTOCOL_EXTERN PRO_STRUCT			 g_pro_struct;
		

	PROTOCOL_EXTERN uint16 Pro_RealTime_Data(uint8 data[]);
	PROTOCOL_EXTERN void ProPara_Init(void);	
	PROTOCOL_EXTERN void SetPara2FactoryReset(void);
	PROTOCOL_EXTERN void ProWrite_SysPara(void);
	PROTOCOL_EXTERN void ProRead_SysPara(void);
	PROTOCOL_EXTERN void ProLsnalSysExit(void);
	PROTOCOL_EXTERN void ProLsnalHeadTailSave(void);
	PROTOCOL_EXTERN void ProUpRealFormation(void);
	PROTOCOL_EXTERN void ProUpLogout(void);
	PROTOCOL_EXTERN void ProUpHeartBeat(void);
	PROTOCOL_EXTERN void ProUpCheckTime(void);
	PROTOCOL_EXTERN void ProConstructFrameHead(uint8 data[],uint16 tx_len,uint8 cmd);
	PROTOCOL_EXTERN void ProConstructFrameTail(uint8 data[],uint16 tx_len);
	PROTOCOL_EXTERN void ProPacket(uint8 tx_data[],uint16 tx_len,uint8 tx_cmd,uint8 ack_flag);
	PROTOCOL_EXTERN void ProPeriodTx(uint16 past_sec);
	PROTOCOL_EXTERN void ProProcess(uint8 data[], uint16 len, uint16 colon);
	
#endif
