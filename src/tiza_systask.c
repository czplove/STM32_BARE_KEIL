
#define SYS_TASK_GLOBAL
#define SYS_TASK_DEBUG

#include "tiza_include.h"
	#include "includes.h"
	#include <os_cfg.h>
	#include <os_cpu.h>

static void ProAlarmHeadTailSave(void);


/******************************************************
//设置GPRS发送所用参数
******************************************************/
void SetGPRSNetPara(void){
//Port = 20000;//27055;//9903;//
	
	g_gprs_data_struct.SendDataLen=0;
  g_gprs_data_struct.SendData=(uint8 *)GPRStestdata;
//	//企业平台 9903
//	g_gprs_data_struct.IP[0] = g_propara_union.Item.Domain[0];
//	g_gprs_data_struct.IP[1] = g_propara_union.Item.Domain[1];
//	g_gprs_data_struct.IP[2] = g_propara_union.Item.Domain[2];
//	g_gprs_data_struct.IP[3] = g_propara_union.Item.Domain[3];
//	g_gprs_data_struct.Port = g_propara_union.Item.Port;
	//公共平台 20000
	g_gprs_data_struct.IP[0] = g_propara_union.Item.PDomain[0];
	g_gprs_data_struct.IP[1] = g_propara_union.Item.PDomain[1];
	g_gprs_data_struct.IP[2] = g_propara_union.Item.PDomain[2];
	g_gprs_data_struct.IP[3] = g_propara_union.Item.PDomain[3];
	g_gprs_data_struct.Port = g_propara_union.Item.PPort;
	
	g_gprs_data_struct.setnetparaok_flag = 1;
}

/******************************************************
//读盲区下标 片外flash
******************************************************/
static void Read_Lsnal_Index(void)
{
	uint8  data[9];
	union32 tmp;
	
	ExteFlashRead(SYS_LSNAL_SPIINDEX_ADDR, data, 9);
	
	if(data[8] == U8SumCheck(data,8)){		
		tmp.byte.HH = data[0];
		tmp.byte.HL = data[1];
		tmp.byte.LH = data[2];
		tmp.byte.LL = data[3];
		g_syslsnal_struct.headindex = tmp.dword;
		tmp.byte.HH = data[4];
		tmp.byte.HL = data[5];
		tmp.byte.LH = data[6];
		tmp.byte.LL = data[7];
		g_syslsnal_struct.tailindex = tmp.dword;
	}
	else {
		#ifdef SYS_TASK_DEBUG
		LocalUartFixedLenSend((uint8*)"ExteFlashRead index ERROR\r\n",StrLen((uint8*)"ExteFlashRead index ERROR\r\n",0));
		#endif
		g_syslsnal_struct.headindex = 0;
		g_syslsnal_struct.tailindex = 0;
		ProLsnalHeadTailSave();
	}
}

/******************************************************
//读报警下标 偏内flash
******************************************************/
static void Read_Alarm_Index(void)
{
	uint8  data[9];
	union32 tmp;
	
	FlashRead(SYS_ALARM_INDEXPAGE, data, 9);
	
	if(data[8] == U8SumCheck(data,8)){		
		tmp.byte.HH = data[0];
		tmp.byte.HL = data[1];
		tmp.byte.LH = data[2];
		tmp.byte.LL = data[3];
		g_sysalarm_struct.afterheadindex = tmp.dword;
		tmp.byte.HH = data[4];
		tmp.byte.HL = data[5];
		tmp.byte.LH = data[6];
		tmp.byte.LL = data[7];
		g_sysalarm_struct.aftertailindex = tmp.dword;
	}
	else {
		#ifdef SYS_TASK_DEBUG
		LocalUartFixedLenSend((uint8*)"IntFlashRead index ERROR\r\n",StrLen((uint8*)"IntFlashRead index ERROR\r\n",0));
		#endif
		g_sysalarm_struct.afterheadindex = 0;
		g_sysalarm_struct.aftertailindex = 0;
		ProAlarmHeadTailSave();
	}
	g_sysalarm_struct.beforeheadindex = 0;
	g_sysalarm_struct.beforetailindex = 0;
}


/******************************************************
//写报警时记录 偏内flash
******************************************************/
///保存页码
static void ProAlarmHeadTailSave(void)
{
	uint8  data[9],res;
	
	if(g_sysm_on_off_struct.flash_w_switch	== SYSM_OFF){	
		return ;
	}
	data[0] = (uint8)(g_sysalarm_struct.afterheadindex >> 24) & 0xFF;
	data[1] = (uint8)(g_sysalarm_struct.afterheadindex >> 16) & 0xFF;
	data[2] = (uint8)(g_sysalarm_struct.afterheadindex >>  8) & 0xFF;
	data[3] = (uint8) g_sysalarm_struct.afterheadindex        & 0xFF;
	data[4] = (uint8)(g_sysalarm_struct.aftertailindex >> 24) & 0xFF;
	data[5] = (uint8)(g_sysalarm_struct.aftertailindex >> 16) & 0xFF;
	data[6] = (uint8)(g_sysalarm_struct.aftertailindex >>  8) & 0xFF;
	data[7] = (uint8) g_sysalarm_struct.aftertailindex        & 0xFF;
	
	data[8] = U8SumCheck(data,8);
	res = FlashErase(SYS_ALARM_INDEXPAGE);	
	if(!res){// 失败
		return;
	}
	FlashWrite(SYS_ALARM_INDEXPAGE, data, 9);
}
static void ProAlarmPageSave(uint8 data[],uint16 len)
{
	uint8  res;
	uint16 page;
	uint32 addr;
	
	if(g_sysm_on_off_struct.flash_w_switch	== SYSM_OFF){	
		return ;
	}
	
	//满一页
	if((g_sysalarm_struct.afterheadindex&0x03)==0){	//FLASH擦除 2K字节/段 即512*4
		addr = SYS_ALARM_START_ADDR + g_sysalarm_struct.afterheadindex*ALARM_PAGE_SIZE;
		res = FlashErase(addr);
		if(!res){// 失败
			goto RETURN_LAB;
		}
	}
	
	//满一包
	addr = SYS_ALARM_START_ADDR + g_sysalarm_struct.afterheadindex*ALARM_PAGE_SIZE ;
	res = FlashWrite(addr, data, len);
	if(!res){// 失败
		goto RETURN_LAB;
	}
	
	g_sysalarm_struct.afterheadindex = (g_sysalarm_struct.afterheadindex+1)%SYS_ALARM_MAXINDEX;
	if(g_sysalarm_struct.afterheadindex == g_sysalarm_struct.aftertailindex){
		//tail前移8包即整段擦除时舍掉其余7包数据；若要不舍需另外开一段临时保存旧数据，这样会增加擦除次数
		g_sysalarm_struct.aftertailindex = (g_sysalarm_struct.aftertailindex+4)%SYS_ALARM_MAXINDEX;
	}
	//正常保留SYS_SAVEALARM_NUMB条实时数据
	if(g_sysmiscrun_struct.have_sysAlarm_flag==0 || g_sysmiscrun_struct.have_sysAlarm_flag==3){	//无报警 或 无报警盲区
		page = (SYS_ALARM_MAXINDEX + g_sysalarm_struct.afterheadindex - g_sysalarm_struct.aftertailindex)%SYS_ALARM_MAXINDEX;
		if(page > SYS_SAVEALARM_NUMB){//g_sysmiscrun_struct.save_sysAlarm_numb){	//只保留规定条数
			page = page - SYS_SAVEALARM_NUMB;//g_sysmiscrun_struct.save_sysAlarm_numb;
			g_sysalarm_struct.aftertailindex = (g_sysalarm_struct.aftertailindex+page)%SYS_ALARM_MAXINDEX;
		}
	}
	ProAlarmHeadTailSave();
	
RETURN_LAB:
	return;
}
static void ProBeforeAlarmSave(uint8 data[],uint16 len)
{
	g_sysalarm_struct.beforeheadindex = (g_sysalarm_struct.beforeheadindex+1)%BEFORE_ALARM_MAXINDEX;
	memcpy(g_BeforeAlarmData[g_sysalarm_struct.beforeheadindex], data, len);
	if(g_sysalarm_struct.beforeheadindex == g_sysalarm_struct.beforetailindex){
		g_sysalarm_struct.beforetailindex = (g_sysalarm_struct.beforetailindex+1)%BEFORE_ALARM_MAXINDEX;
	}	
}
void ProPutIntoAlarm(uint8 data[], uint16 len, uint8 cmd)
{
	uint8 tmpdata[512];
	uint16 tmplen;
	
	if(len == 0 ){
		tmplen = Pro_RealTime_Data(&tmpdata[4]);
	}
	else{
		memcpy(&tmpdata[4],data,len);
		tmplen = len;
	}	
	
	//可以在这里读取下FLASH  来区分要不要擦除
//	if(g_sysalarm_struct.data[0] != VALID_VAL_2A){		
		tmpdata[0] = VALID_VAL_2A;
		tmpdata[1] = tmplen >> 8;
		tmpdata[2] = tmplen & 0xFF;
		tmpdata[3] = cmd;//PRO_UP_LSNAL_INFO_ID;//PRO_UP_REAL_INFO_ID;//cmd;
		
		tmpdata[tmplen+4] = U8SumCheck(tmpdata,tmplen+4);
		if(cmd == PRO_UP_LSNAL_INFO_ID){
			ProBeforeAlarmSave(tmpdata,tmplen+5);	
			
			tmpdata[3] = PRO_UP_REAL_INFO_ID;
			tmpdata[tmplen+4] = U8SumCheck(tmpdata,tmplen+4);
			ProAlarmPageSave(tmpdata,tmplen+5);		//02与03指令衔接时无漏包使用 否则注释
		}
		else {
			ProAlarmPageSave(tmpdata,tmplen+5);	
		}
//	}
	
}

/******************************************************
//终端定时保存实时数据
******************************************************/
static void ChargeSendTime(uint8 flag)
{
	//报警30+30+10使用，暂计时到天（一般不会在夜里0过检）
	if(flag==1){
		g_sysmiscrun_struct.send_sysAlarm_time[1].Item.second++;
		if(g_sysmiscrun_struct.send_sysAlarm_time[1].Item.second >= 60){
			g_sysmiscrun_struct.send_sysAlarm_time[1].Item.second = 0;
			g_sysmiscrun_struct.send_sysAlarm_time[1].Item.minute++;
		}
		if(g_sysmiscrun_struct.send_sysAlarm_time[1].Item.minute >= 60){
			g_sysmiscrun_struct.send_sysAlarm_time[1].Item.minute = 0;
			g_sysmiscrun_struct.send_sysAlarm_time[1].Item.hour++;
		}
		if(g_sysmiscrun_struct.send_sysAlarm_time[1].Item.hour >= 24){
			g_sysmiscrun_struct.send_sysAlarm_time[1].Item.hour = 0;
			g_sysmiscrun_struct.send_sysAlarm_time[1].Item.day++;
		}
	}
	else{
		g_sysmiscrun_struct.send_sysAlarm_time[0].Item.second++;
		if(g_sysmiscrun_struct.send_sysAlarm_time[0].Item.second >= 60){
			g_sysmiscrun_struct.send_sysAlarm_time[0].Item.second = 0;
			g_sysmiscrun_struct.send_sysAlarm_time[0].Item.minute++;
		}
		if(g_sysmiscrun_struct.send_sysAlarm_time[0].Item.minute >= 60){
			g_sysmiscrun_struct.send_sysAlarm_time[0].Item.minute = 0;
			g_sysmiscrun_struct.send_sysAlarm_time[0].Item.hour++;
		}
		if(g_sysmiscrun_struct.send_sysAlarm_time[0].Item.hour >= 24){
			g_sysmiscrun_struct.send_sysAlarm_time[0].Item.hour = 0;
			g_sysmiscrun_struct.send_sysAlarm_time[0].Item.day++;
		}	
			
//		g_sysmiscrun_struct.send_sysAlarm_time[0].Item.second--;
//		if(g_sysmiscrun_struct.send_sysAlarm_time[0].Item.second == 0xff){
//			g_sysmiscrun_struct.send_sysAlarm_time[0].Item.second = 59;
//			g_sysmiscrun_struct.send_sysAlarm_time[0].Item.minute--;
//		}
//		if(g_sysmiscrun_struct.send_sysAlarm_time[0].Item.minute == 0xff){
//			g_sysmiscrun_struct.send_sysAlarm_time[0].Item.minute = 59;
//			g_sysmiscrun_struct.send_sysAlarm_time[0].Item.hour--;
//		}
//		if(g_sysmiscrun_struct.send_sysAlarm_time[0].Item.hour == 0xff){
//			g_sysmiscrun_struct.send_sysAlarm_time[0].Item.hour = 23;
//			g_sysmiscrun_struct.send_sysAlarm_time[0].Item.day--;
//		}
	
	}
	
}
static void MarkSendTime(void)
{
	uint8 i;
	g_sysmiscrun_struct.send_sysAlarm_numb = 1;
	g_sysmiscrun_struct.send_sysAlarm_time[1] = g_protime_union; 
	g_sysmiscrun_struct.send_sysAlarm_time[0] = g_protime_union; 
	
	for(i=0; i < 31; i++){ // 后推31秒
			g_sysmiscrun_struct.send_sysAlarm_time[0].Item.second--;
		if(g_sysmiscrun_struct.send_sysAlarm_time[0].Item.second == 0xff){
			g_sysmiscrun_struct.send_sysAlarm_time[0].Item.second = 59;
			g_sysmiscrun_struct.send_sysAlarm_time[0].Item.minute--;
		}
		if(g_sysmiscrun_struct.send_sysAlarm_time[0].Item.minute == 0xff){
			g_sysmiscrun_struct.send_sysAlarm_time[0].Item.minute = 59;
			g_sysmiscrun_struct.send_sysAlarm_time[0].Item.hour--;
		}
		if(g_sysmiscrun_struct.send_sysAlarm_time[0].Item.hour == 0xff){
			g_sysmiscrun_struct.send_sysAlarm_time[0].Item.hour = 23;
			g_sysmiscrun_struct.send_sysAlarm_time[0].Item.day--;
		}
	}
}
static void TerminalSavePeriod(uint16 past_ms)
{
	if(g_sysm_on_off_struct.flash_w_switch	== SYSM_OFF){	
		return ;
	}
	
	if(g_sysmiscrun_struct.have_sysAlarm_flag == 1){//有报警
//		ProPutIntoAlarm(NULL, 0, PRO_UP_REAL_INFO_ID);		//1s执行一次保存
		
		//报警超时 停止1s报警上传转为正常上传  若仍需1s报警上传则把以下条件代码注释
		if(++g_sysmiscrun_struct.have_sysAlarm_count >= SYS_HAVEALARM_COUNT){
			g_sysmiscrun_struct.have_sysAlarm_count = SYS_HAVEALARM_COUNT;
			g_sysmiscrun_struct.have_sysAlarm_flag = 2;				//超时 置检测有无报警盲区
		}
	}
	else if(g_sysmiscrun_struct.have_sysAlarm_flag==0 || g_sysmiscrun_struct.have_sysAlarm_flag==3){	//无报警 或 无报警盲区
		ProPutIntoAlarm(NULL, 0, PRO_UP_LSNAL_INFO_ID);
	}
}

static void TerminalUpLsnalAlrm(void)
{
	uint8  rx_buf[TMP_BUFF_MAXLEN-25],data[5];	//buff考虑帧头尾-25
	uint8  sum=0;
	uint16 len;
	uint32 addr;	
	#ifdef SYS_TASK_DEBUG
		char str_ch[10];
		uint8 str_len;
	#endif
	
	if(g_sysmiscrun_struct.have_sysAlarm_flag != 1 && g_sysmiscrun_struct.have_sysAlarm_flag != 2){		
		MarkSendTime();
		return;												//不是报警  也  没有报警盲区
	}
/*	
	len = (ALARM_PAGE_SIZE+g_sysalarm_struct.afterheadindex-g_sysalarm_struct.aftertailindex )%ALARM_PAGE_SIZE;
//	if(g_sysalarm_struct.aftertailindex != g_sysalarm_struct.afterheadindex){ // 发送报警后的数据
	if(len > SYS_SAVEALARM_NUMB){ // 发送报警后的数据
		addr = SYS_ALARM_START_ADDR + g_sysalarm_struct.aftertailindex*ALARM_PAGE_SIZE ;
		FlashRead(addr, data, 4);
		
		if(data[0]==VALID_VAL_2A && (data[3]==PRO_UP_LSNAL_INFO_ID || data[3]==PRO_UP_REAL_INFO_ID)){
			len = (data[1]<<8) + data[2];
			sum = U8SumCheck(data, 4);			
			
			FlashRead(addr+4, &rx_buf[PRO_DATA_INDEX], len+1);	
			sum += U8SumCheck(&rx_buf[PRO_DATA_INDEX], len);	
			if(rx_buf[PRO_DATA_INDEX + len] == sum){//
				ProConstructFrameHead(rx_buf, len, data[3]);
				len += PRO_DATA_INDEX;
				ProConstructFrameTail(rx_buf, len);
				
				g_sysalarm_struct.aftertailindex = (g_sysalarm_struct.aftertailindex+1)%SYS_ALARM_MAXINDEX;
				ProPacket(rx_buf, len+1, data[3], FALSE);//TRUE);
			}
			else{
				goto RETURN_LAB1;
			}
		}
		else{
			goto RETURN_LAB1;
		}
		
	}
	*/
	if(g_sysmiscrun_struct.send_sysAlarm_numb <= 30 ){	//使用发送时间确保报警报文时间连续 170504start
		len = Pro_RealTime_Data(&rx_buf[PRO_DATA_INDEX]);
		ProConstructFrameHead(rx_buf, len, PRO_UP_REAL_INFO_ID);	
		len += PRO_DATA_INDEX;	
		memcpy(&rx_buf[PRO_DATA_INDEX], g_sysmiscrun_struct.send_sysAlarm_time[1].arry, 6);		//发送时间		
		ProConstructFrameTail(rx_buf, len);
		ProPacket(rx_buf, len+1, PRO_UP_REAL_INFO_ID, FALSE);
		
		ChargeSendTime(1);												//时间加1
		g_sysmiscrun_struct.send_sysAlarm_numb++;	//条数加1
	}																										//																	170504end
	else{		
//		g_sysmiscrun_struct.have_sysAlarm_flag = 2;
		if(g_sysalarm_struct.beforetailindex != g_sysalarm_struct.beforeheadindex){ //发送报警前的数据				
			if(g_BeforeAlarmData[g_sysalarm_struct.beforetailindex][0]==VALID_VAL_2A && g_BeforeAlarmData[g_sysalarm_struct.beforetailindex][3]==PRO_UP_LSNAL_INFO_ID){
				len = (g_BeforeAlarmData[g_sysalarm_struct.beforetailindex][1]<<8) + g_BeforeAlarmData[g_sysalarm_struct.beforetailindex][2];
				sum = U8SumCheck(g_BeforeAlarmData[g_sysalarm_struct.beforetailindex], len+4);			
				
				if(g_BeforeAlarmData[g_sysalarm_struct.beforetailindex][len+4] == sum){//
					if(g_sysmiscrun_struct.send_sysAlarm_numb%10 !=0){ //判断是否要插入发10s实时数据
						ProConstructFrameHead(rx_buf, len, PRO_UP_LSNAL_INFO_ID);
						memcpy(&rx_buf[PRO_DATA_INDEX], &g_BeforeAlarmData[g_sysalarm_struct.beforetailindex][4], len);
						len += PRO_DATA_INDEX;
						ChargeSendTime(0);																																		//时间减1  170504
						memcpy(&rx_buf[PRO_DATA_INDEX], g_sysmiscrun_struct.send_sysAlarm_time[0].arry, 6);		//发送时间 170504
						ProConstructFrameTail(rx_buf, len);
						
						g_sysalarm_struct.beforetailindex = (g_sysalarm_struct.beforetailindex+1)%BEFORE_ALARM_MAXINDEX;
						ProPacket(rx_buf, len+1, PRO_UP_LSNAL_INFO_ID, FALSE);//TRUE);
					}
					else{//插入10秒一条实时数据   170504start
						len = Pro_RealTime_Data(&rx_buf[PRO_DATA_INDEX]);
						ProConstructFrameHead(rx_buf, len, PRO_UP_REAL_INFO_ID);	
						len += PRO_DATA_INDEX;	
						ProConstructFrameTail(rx_buf, len);
						ProPacket(rx_buf, len+1, PRO_UP_REAL_INFO_ID, FALSE);
					}
					g_sysmiscrun_struct.send_sysAlarm_numb++;	//条数加1
					if(g_sysmiscrun_struct.send_sysAlarm_numb >= 64){
						g_sysalarm_struct.beforetailindex = g_sysalarm_struct.beforeheadindex;
					}											//			170504end
					
				}
				else{
					goto RETURN_LAB2;
				}
			}
			else{
				goto RETURN_LAB2;
			}				
		}
		else{//超时报警盲区数据发完
			if(g_sysmiscrun_struct.have_sysAlarm_flag == 2){
					g_sysmiscrun_struct.have_sysAlarm_flag = 3;	
			}
		}

		
	}
	#ifdef SYS_TASK_DEBUG
		LocalUartFixedLenSend((uint8*)"\r\n g_sysmiscrun_struct.send_sysAlarm_numb = ",StrLen((uint8*)"\r\n g_sysmiscrun_struct.send_sysAlarm_numb = ",0));
		str_len = sprintf(str_ch,"%d ",g_sysmiscrun_struct.send_sysAlarm_numb);
		LocalUartFixedLenSend((uint8*)str_ch,str_len);
		LocalUartFixedLenSend((uint8*)"\r\n g_sysmiscrun_struct.send_sysAlarm_time[0].Item.second = ",StrLen((uint8*)"\r\n g_sysmiscrun_struct.send_sysAlarm_time[0].Item.second = ",0));
		str_len = sprintf(str_ch,"%d ",g_sysmiscrun_struct.send_sysAlarm_time[0].Item.second);
		LocalUartFixedLenSend((uint8*)str_ch,str_len);
		LocalUartFixedLenSend((uint8*)"\r\n g_sysmiscrun_struct.send_sysAlarm_time[1].Item.second = ",StrLen((uint8*)"\r\n g_sysmiscrun_struct.send_sysAlarm_time[1].Item.second = ",0));
		str_len = sprintf(str_ch,"%d ",g_sysmiscrun_struct.send_sysAlarm_time[1].Item.second);
		LocalUartFixedLenSend((uint8*)str_ch,str_len);
	#endif
	
	return;
	
RETURN_LAB1:	
	g_sysalarm_struct.aftertailindex = (g_sysalarm_struct.aftertailindex+1)%SYS_ALARM_MAXINDEX;
	return;
RETURN_LAB2:	
	g_sysalarm_struct.beforetailindex = (g_sysalarm_struct.beforetailindex+1)%BEFORE_ALARM_MAXINDEX;
	return;
}

/*static void TerminalAlrmUpPeriod(uint16 past_ms)
{
	if(g_sysmiscrun_struct.have_sysAlarm_flag == 1){//有报警		
		g_sysm_timeout_struct.AlrUpPeri_count += past_ms;
		if(g_sysm_timeout_struct.AlrUpPeri_count >= g_propara_union.Item.AlrUpPeri){
			
			ProUpRealFormation();
			g_sysm_timeout_struct.AlrUpPeri_count = 0;
		}
	}
	else if(g_sysmiscrun_struct.have_sysAlarm_flag == 0){//无报警		
		g_sysmiscrun_struct.have_sysAlarm_count = 0;
	}
}*/

//=======================================================================================================//
/******************************************************
//其他逻辑函数
******************************************************/
static void TerminalMiscPeriod(uint8 past_sec)
{		
	if(g_propara_union.Item.PAckTim!=0xFF && g_sysm_timeout_struct.PAckTim_count!=0xFF){
		if(++g_sysm_timeout_struct.PAckTim_count >= g_propara_union.Item.PAckTim){
			LocalUartFixedLenSend((uint8*)"平台超时\r\n",StrLen((uint8*)"平台超时\r\n",0));
			g_sysm_timeout_struct.PAckTim_count = 0xFF;
		}
	}
}

static void TerminalNorUpPeriod(uint8 past_sec)
{		
	if(g_sysmiscrun_struct.have_sysAlarm_flag==0 || g_sysmiscrun_struct.have_sysAlarm_flag==3){//没有报警	或超时 
		if(++g_sysm_timeout_struct.NorUpPeri_count >= g_propara_union.Item.NorUpPeri){
			ProUpRealFormation();
			g_sysm_timeout_struct.NorUpPeri_count = 0;
		}
	}	
}

static void TerminalHeart(uint8 past_sec)
{
	if(++g_sysm_timeout_struct.upheart_count > g_propara_union.Item.HeartPeri){	// 1s累加，发送心跳
		ProUpHeartBeat();
		g_sysm_timeout_struct.upheart_count = 0;
	}
}


static void TerminalWakeUp(void)
{
	ErrorStatus HSEStartUpStatus;
	//HSE重新配置
	RCC_HSEConfig(RCC_HSE_ON); 														/*HSES使能*/  
	HSEStartUpStatus = RCC_WaitForHSEStartUp(); 					/*等待*/
	if(HSEStartUpStatus == SUCCESS) { 
		RCC_PLLCmd(ENABLE);																	/*使能*/
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)== RESET); 	/*等待PLL有效*/      
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);					/*将PLL作为系统时钟*/
		while(RCC_GetSYSCLKSource() != 0x08);								/*等待*/
	} 
	RCC->AHBENR = 0x14;
	RCC->APB1ENR = 0x18000000;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO, ENABLE);	
	
	//开外电
	ON_485232_PWR();
	ON_CT5V_PWR();
//	ON_GPRS_PWR();  //模块开
	HIGH_GPRS_DTR(); 	//唤醒L218
	//重初始化	
	delay_init();	    //初始化systick
	PvdInit();
	UsartInit(LOCAL_USART,LOCAL_USART_BPR,USART_DATA_8B,USART_STOPBITS_1,USART_PARITY_NO);
	UsartInit(GPS_USART  ,GPS_USART_BPR  ,USART_DATA_8B,USART_STOPBITS_1,USART_PARITY_NO);	
	ExteFlashInit();
	RtcInit();
	AdcInit();
//	BusiResetModule();
	
	if(0){//若唤醒出问题   复位
		SysReset();
	}
}
static void TerminalIntoSleep(void)
{
	uint8 rtc_counter;
	
	if(CHECK_POW() == HIGH_STATE){//有外电，不休眠
		return;
	}			

	//保存
	
	//关外电
	OFF_485232_PWR();
	OFF_CT5V_PWR();
//	OFF_GPRS_PWR();
		LOW_GPRS_DTR(); 	//休眠L218
	ADC_Cmd(ADC1, DISABLE);

	//进休眠
	rtc_counter = RTC_GetCounter();
	while(1){
		FeedWtd();
		RTC_WaitForLastTask();
		RTC_SetAlarm(rtc_counter++);	
		RTC_WaitForLastTask();
		PWR_EnterSTOPMode(PWR_Regulator_ON,PWR_STOPEntry_WFI);

		if(CHECK_POW() == HIGH_STATE){//有外电，唤醒
			break;
		}
	}
	FeedWtd();
	TerminalWakeUp();
	
}

/******************************************************
//终端断外电再上外电重启
//
******************************************************/
static void TerminalMainPowerStatu(void)
{
	switch(g_sysmiscrun_struct.main_power_statu){
		case 0x01:{//首次上电
			if(CHECK_POW() == LOW_STATE){
				g_sysmiscrun_struct.main_power_statu = 2;
			}
			break;
		}
		case 0x02:{//断电
			if(CHECK_POW() == HIGH_STATE){
				//再次上电，复位
				SysReset();
			}			
			break;
		}
		default:
			if(CHECK_POW() == HIGH_STATE){
				g_sysmiscrun_struct.main_power_statu = 1;				
			}			
		break;
	}
}

/******************************************************
//报警禁用功能
//发送过程中模块重登陆时，不能关掉周期发送函数
******************************************************/
static void AlarmDisableFunction(void)
{
///报警	
	if(g_sysmiscrun_struct.have_sysAlarm_flag==0 || g_sysmiscrun_struct.have_sysAlarm_flag==3){
		//无报警  报警无盲区
		g_sysm_on_off_struct.GPRSPeriodTx_switch 	= SYSM_ON;
		
	}
	else if(g_sysmiscrun_struct.have_sysAlarm_flag==1 || g_sysmiscrun_struct.have_sysAlarm_flag==2){
		//有报警 或 有报警盲区
		if(g_pro_struct.login_center_flag == TRUE){	//已经登录	
			g_sysm_on_off_struct.GPRSPeriodTx_switch 	= SYSM_OFF;
		}
		else {
			g_sysm_on_off_struct.GPRSPeriodTx_switch 	= SYSM_ON;
		}
	}	
}
/******************************************************
//报警判断
//输入：报警标志  00无 01有
******************************************************/
void AlarmHaveJudge(uint8 flag)
{		
	if(flag==1 ){			//报警
		g_sysmiscrun_struct.stop_sysAlarm_flag 	 = 0;	//恢复 报警取消标志
		if(g_sysmiscrun_struct.have_sysAlarm_flag==0){// || g_sysmiscrun_struct.have_sysAlarm_flag==3){
//			g_sysalarm_struct.aftertailindex = g_sysalarm_struct.afterheadindex;
			ProLsnalHeadTailSave();
			g_sysmiscrun_struct.have_sysAlarm_flag = 1;
			g_sysmiscrun_struct.have_sysAlarm_count  = 0;	//清报警发送计数
		}
	}
	else if(flag==0){	//正常
		g_sysmiscrun_struct.stop_sysAlarm_flag 	 = 1;	//置报警取消标志
		if(g_sysmiscrun_struct.have_sysAlarm_flag!=0 && g_sysmiscrun_struct.have_sysAlarm_flag!=3){
			g_sysmiscrun_struct.have_sysAlarm_flag = 2;  //标志检查有无盲区数据
		}
		else{
			g_sysmiscrun_struct.have_sysAlarm_flag = 0;				
		}
	}
}

/******************************************************
//读取或写入上次有效定位信息
//输入 ：00读取 01写入
//头2A + 经度纬度状态 + 校验  = 1+(4+4+1)+1 = 11
******************************************************/
void ReadWriteOldVaildPostion(uint8 rw)	
{
	uint8 data[15];
	union32 tmp;	
	#ifdef SYS_TASK_DEBUG
		char str_ch[10];
		uint8 str_len;
	#endif
	
	if(rw == 0){		//读取		
		FlashRead(SYS_SAVEVAILDPOS_INDEXPAGE, data, 11);
		if(data[0]==0x2A && data[10]==U8SumCheck(data, 10)){
			tmp.byte.HH = data[1];
			tmp.byte.HL = data[2];
			tmp.byte.LH = data[3];
			tmp.byte.LL = data[4];
			g_gps_struct.gpsinform.subitem.longitude = tmp.dword;
			tmp.byte.HH = data[5];
			tmp.byte.HL = data[6];
			tmp.byte.LH = data[7];
			tmp.byte.LL = data[8];
			g_gps_struct.gpsinform.subitem.latitude = tmp.dword;
			g_gps_struct.gpsinform.subitem.statu.byte = data[8]|0x01;	//- - A(invalid)
//			printf("flash read postion = %d \r\n",g_gps_struct.gpsinform.subitem.statu.byte);
		}
		else {			//内存没有位置数据时 使用南京天泽的经纬度作为默认初始值
			g_gps_struct.gpsinform.subitem.longitude 	= 118733477;
			g_gps_struct.gpsinform.subitem.latitude  	= 31993519;
			g_gps_struct.gpsinform.subitem.statu.byte	= 0x01;		//E N A(invalid)
			
	#ifdef SYS_TASK_DEBUG
		LocalUartFixedLenSend((uint8*)"\r\n flash read TIZA postion = ",StrLen((uint8*)"\r\n flash read TIZA postion = ",0));
		str_len = sprintf(str_ch,"%d ",g_gps_struct.gpsinform.subitem.statu.byte);
		LocalUartFixedLenSend((uint8*)str_ch,str_len);
	#endif
			
			goto RET_WRITE;
		}
	}
	else{						//写入
		if(g_gps_struct.gpsinform.subitem.statu.bit.B0 == 0x00){//有效定位
		RET_WRITE:
			data[0] = 0x2A;
			tmp.dword = g_gps_struct.gpsinform.subitem.longitude;
			data[1] = tmp.byte.HH;
			data[2] = tmp.byte.HL;
			data[3] = tmp.byte.LH;
			data[4] = tmp.byte.LL;
			tmp.dword = g_gps_struct.gpsinform.subitem.latitude;
			data[5] = tmp.byte.HH;
			data[6] = tmp.byte.HL;
			data[7] = tmp.byte.LH;
			data[8] = tmp.byte.LL;
			data[9] = g_gps_struct.gpsinform.subitem.statu.byte & 0x07;
			data[10] = U8SumCheck(data, 10);
			FlashErase(SYS_SAVEVAILDPOS_INDEXPAGE);
			FlashWrite(SYS_SAVEVAILDPOS_INDEXPAGE, data, 11);		
		}
	}
}

/******************************************************
//CAN报文对终端的动作
//
******************************************************/
static void CAN2Terminal_Action(void)
{			
/*	if(g_proalarm_union.Item.rank == 0x03){ 
				AlarmHaveJudge(1);
			}
			else{
				AlarmHaveJudge(0);
			}
	}
*/
	//发送过程中出现报警取消 且发送完成  恢复报警标志
	if(g_sysmiscrun_struct.stop_sysAlarm_flag==1 && g_sysmiscrun_struct.have_sysAlarm_flag==3){
			g_sysmiscrun_struct.have_sysAlarm_flag = 0;	
	}
}

/******************************************************
//升级前的准备
******************************************************/	
void ReadyBeforeUpgrade(void)
{
	uint32 ftpflshadd;
	
///升级	
	if(g_sysmiscrun_struct.ProgramUpgrade_flag == 1){	
//		LocalUartFixedLenSend((uint8*)"正在升级\r\n",StrLen((uint8*)"正在升级\r\n",0));
		//CAN
		OFF_CAN_PWR();			
		CanDeInit();
		g_sysm_on_off_struct.canrx_switch 				= SYSM_OFF;
		//周期发送
		g_sysm_on_off_struct.GPRSPeriodTx_switch 	= SYSM_OFF;
		g_sysm_on_off_struct.flash_w_switch				= SYSM_OFF;
		
		for(ftpflshadd=FlASH_STORE_START_ADD; ftpflshadd < FLASH_STORE_END_ADD;){
			OSTimeDlyHMSM(0, 0, 0, 20);
			FlashErase(ftpflshadd);
			ftpflshadd += 0x800;
		}	
		ftp_struct.ftp_upgrade_flag 						= 1;	
		g_sysmiscrun_struct.ProgramUpgrade_flag = 2;
	}
}
/******************************************************
//应用程序任务
******************************************************/
void OS_Timer0(OS_TMR *ptmr, void *p_arg)
{
	//1s软定时
	ProPeriodTx(1);															//周期发送函数	
	if(g_pro_struct.login_center_flag == TRUE){	//已经登录		
		TerminalNorUpPeriod(1);
		TerminalHeart(1);
	}			
	
	CPL_WORK_LED();	
}
//周期处理任务
void Period_task(void *pdata)
{
	OS_TMR *pTimer0 = 0;
	uint8   err = 0; 
	uint16 count_time,period_time;
	static uint8  old_second;
	
//	memcpy(g_VIN,"A1234567890123456",17);		//暂保留
//	memcpy(g_VIN,"A0000013776516481",17);		//周姐测2 
	memcpy(g_VIN,"A0000013515127369",17);		//周姐测1  
//	memcpy(g_VIN,"A0000013585107257",17);		//赵达军  
//	memcpy(g_VIN,"A0000013601464223",17);		//钱利军
//	memcpy(g_VIN,"A0000015261409343",17);		//2017.04.21发给长沙
		
//	ProRead_VIN();																	//读取VIN码
	ProRead_SysPara();
	OSTimeDlyHMSM(0, 0, 0, 500);	
	Read_Lsnal_Index();
	OSTimeDlyHMSM(0, 0, 0, 500);
	Read_Alarm_Index();
	OSTimeDlyHMSM(0, 0, 0, 500);	
	
	//创建软定时器
	 pTimer0 = OSTmrCreate(0,
                      OS_TMR_CFG_TICKS_PER_SEC,
                      OS_TMR_OPT_PERIODIC,
                      //OS_TMR_OPT_ONE_SHOT, 
                      (OS_TMR_CALLBACK)OS_Timer0,
                      (uint8*)0,
                      (uint8*)"OSTIMER0",
                      &err);
	OSTmrStart(pTimer0, &err); //启动定时器										
	while(1){
		OSTimeDlyHMSM(0, 0, 0, 200);	
		count_time = (count_time+1)%60000;
		period_time = count_time;		
		
		GetCalendarTime(g_protime_union.arry, 0x01);		//读取时间 从GPS
		if(old_second != g_protime_union.Item.second){
			TerminalSavePeriod(1000);										//实时数据保存 未登录也要存
			if(g_pro_struct.login_center_flag == TRUE){	//已经登录	
				TerminalUpLsnalAlrm();										//报警时发送数据
			}			
			old_second = g_protime_union.Item.second;
		}
		

		if(period_time%600 == 2){//2min	
			if(g_sysmiscrun_struct.have_sysAlarm_flag==0 || g_sysmiscrun_struct.have_sysAlarm_flag==3){
				g_sysmiscrun_struct.need_check_time_flag = 0x01;			//无报警或报警超时  从GPS校时		
			}
			ReadWriteOldVaildPostion(1);								//保存定位有效时的位置
		}

		if(period_time%5 == 3){//1							
			g_propostion_union.Item.status.byte = g_gps_struct.gpsinform.subitem.statu.byte & 0x07;	//定位状态
			g_propostion_union.Item.longitude   = g_gps_struct.gpsinform.subitem.longitude;
			g_propostion_union.Item.latitude    = g_gps_struct.gpsinform.subitem.latitude;
			
			CAN2Terminal_Action();											//CAN报文对终端的动作
			AlarmDisableFunction();											//报警禁用功能			
//			ProPeriodTx(1);															//周期发送函数	
//			if(g_pro_struct.login_center_flag == TRUE){	//已经登录		
//				TerminalNorUpPeriod(1);
//				TerminalHeart(1);
//			}				
			CanMonitor(1);															//CAN通信监视
			TerminalMainPowerStatu();										//外电断开在上电复位
		}
			
	}
}

