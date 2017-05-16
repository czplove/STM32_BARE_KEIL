#define TEST_STRUCT_GLOBAL
#define TEST_DEBUG
#include "tiza_include.h"

#define FTP_LINK_EXAMP "ftp://Vehicle:Vehicle#*@202.102.090.179:21/TIZA_XGDL_SBJ_V803_170306.bin\x00"

#define TEST_LOCALCMD	
//#define TEST_READLSNAL	
//#define TEST_232_485	
#define TEST_FTP	

//#define TEST_LOGINOUT		1
//#define TEST_CHECKTIME		2
uint8 Test_Step = 0;

uint8 test_loginout_count =0;




static void test_rs232_485(void)
{
	uint16 count,index,j;
	uint8  data[256];
	uint8  dataip[4];
	uint16 dataport;
	#ifdef TEST_DEBUG
		char str_ch[50];
		uint8 str_len;
	#endif
	
	#ifdef TEST_LOCALCMD
	//uart1 本地串口	
	g_local_uart_struct.rx_head = (LOCAL_UART_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Channel5)) % LOCAL_UART_BUF_LEN;
	if(g_local_uart_struct.rx_head != g_local_uart_struct.rx_tail)
	{
		count = (LOCAL_UART_BUF_LEN + g_local_uart_struct.rx_head - g_local_uart_struct.rx_tail) % LOCAL_UART_BUF_LEN;
		for(index=0; index < count; index++){
			data[index] = g_local_uart_struct.rx_buf[((index+g_local_uart_struct.rx_tail) % LOCAL_UART_BUF_LEN)];
		}
		g_local_uart_struct.rx_tail = (g_local_uart_struct.rx_tail + count)%LOCAL_UART_BUF_LEN;
		
		if(SubMatch((uint8*)"CMD ALARM",9,data,count))						{ //报警
			g_sysmiscrun_struct.stop_sysAlarm_flag 	 = 0;
			if(g_sysmiscrun_struct.have_sysAlarm_flag==0){// || g_sysmiscrun_struct.have_sysAlarm_flag==3){
//				g_sysalarm_struct.aftertailindex = g_sysalarm_struct.afterheadindex;
				ProLsnalHeadTailSave();
				g_sysmiscrun_struct.have_sysAlarm_flag = 1;
			}
		}
		else if(SubMatch((uint8*)"CMD NORMAL",10,data,count))			{	//正常
			g_sysmiscrun_struct.stop_sysAlarm_flag 	 = 1;
			if(g_sysmiscrun_struct.have_sysAlarm_flag!=0 && g_sysmiscrun_struct.have_sysAlarm_flag!=3){
				//标志检查有无盲区数据
				g_sysmiscrun_struct.have_sysAlarm_flag = 2;  
			}
			else{
				g_sysmiscrun_struct.have_sysAlarm_flag = 0;				
			}
		}
		else if(SubMatch((uint8*)"CMD REST",8,data,count))				{	//复位终端
//			g_syslsnal_struct.tailindex = g_syslsnal_struct.headindex;
//			ProLsnalHeadTailSave();
			SysReset();
		}
		else if(SubMatch((uint8*)"CMD UPGRADE",11,data,count))		{	//固件审计
			g_proupgread_struct.ip_domain[2] = 202;
			g_proupgread_struct.ip_domain[3] = 102;
			g_proupgread_struct.ip_domain[4] = 90;
			g_proupgread_struct.ip_domain[5] = 179;
			g_proupgread_struct.port         = 21;
			memcpy(g_proupgread_struct.user_name,"Vehicle",7);
			g_proupgread_struct.user_name[7] = 0;
			memcpy(g_proupgread_struct.pass_word,"Vehicle#*",9);
			g_proupgread_struct.pass_word[9] = 0;
			memcpy(g_proupgread_struct.file_path,"/TIZA_FTP.bin",13);
			g_proupgread_struct.file_path[13] = 0;
			
//			ftp_struct.ftp_upgrade_flag 						= 1;	
			g_sysmiscrun_struct.ProgramUpgrade_flag = 1;
		}
		else if(SubMatch((uint8*)"CMD VERSION",11,data,count))		{	//版本号硬件加软件
			LocalUartFixedLenSend((uint8*)"\r\n hardware version is:  ",StrLen((uint8*)"\r\n hardware version is:  ",0));
			LocalUartFixedLenSend(g_propara_union.Item.g_para_HDVers,5);
			LocalUartFixedLenSend((uint8*)"\r\n software version is:  ",StrLen((uint8*)"\r\n software version is:  ",0));
			LocalUartFixedLenSend(g_propara_union.Item.g_para_SFVers,5);
			LocalUartFixedLenSend((uint8*)"\r\n",StrLen((uint8*)"\r\n",0));
		}
		else if(SubMatch((uint8*)"CMD IPPORT",10,data,count)){// && data[count]=='#' && count>27)		{	//更改IP PORT
			//CMD IPPORT192.168.001.002:21#
			index = SubMatch((uint8*)"CMD IPPORT",10,data,count);
			dataip[0]=(data[index]-0x30)*100 + (data[index+1]-0x30)*10 + (data[index+2]-0x30);
			dataip[1]=(data[index+4]-0x30)*100 + (data[index+5]-0x30)*10 + (data[index+6]-0x30);
			dataip[2]=(data[index+8]-0x30)*100 + (data[index+9]-0x30)*10 + (data[index+10]-0x30);
			dataip[3]=(data[index+12]-0x30)*100 + (data[index+13]-0x30)*10 + (data[index+14]-0x30);
			//i = SubMatch((uint8*)"**",1,data,count);
			
			dataport = 0;
			for(j=index+16;j < count;j++){
				if(data[j] == '*') break;
				dataport *= 10;
				dataport += data[j] - 0x30;
			}
			
		#ifdef TEST_DEBUG
			str_len = sprintf(str_ch,"\r\n ipport= %d.%d.%d.%d:%d",dataip[0],dataip[1],dataip[2],dataip[3],dataport);
			LocalUartFixedLenSend((uint8*)str_ch,str_len);
		#endif
			
			g_propara_union.Item.PDomain[0] = dataip[0];								//公共平台域名	
			g_propara_union.Item.PDomain[1] = dataip[1];
			g_propara_union.Item.PDomain[2] = dataip[2];
			g_propara_union.Item.PDomain[3] = dataip[3];	
			g_propara_union.Item.PPort			= dataport;//27055;//		//公共平台端口	
			ProWrite_SysPara();
		}
		
	}
	#endif
	#ifdef TEST_232_485
	//uart4
	
	if(g_meter_uart_struct.rx_head != g_meter_uart_struct.rx_tail)
	{
		count = (METER_UART_BUF_LEN + g_meter_uart_struct.rx_head - g_meter_uart_struct.rx_tail) % METER_UART_BUF_LEN;
		for(index=0; index < count; index++){
			data[index] = g_meter_uart_struct.rx_buf[((index+g_meter_uart_struct.rx_tail) % METER_UART_BUF_LEN)];
		}
		g_meter_uart_struct.rx_tail = (g_meter_uart_struct.rx_tail + count)%METER_UART_BUF_LEN;
		MeterUartFixedLenSend(data,count);
	}
	//uart5
	if(g_rs485_uart_struct.rx_head != g_rs485_uart_struct.rx_tail)
	{
		count = (RS485_UART_BUF_LEN + g_rs485_uart_struct.rx_head - g_rs485_uart_struct.rx_tail) % RS485_UART_BUF_LEN;
		for(index=0; index < count; index++){
			data[index] = g_rs485_uart_struct.rx_buf[((index+g_rs485_uart_struct.rx_tail) % RS485_UART_BUF_LEN)];
		}
		g_rs485_uart_struct.rx_tail = (g_rs485_uart_struct.rx_tail + count)%RS485_UART_BUF_LEN;
		RS485UartFixedLenSend(data,count);
	}
		
	#endif	

}

void Test_task(void *pdata)
{	
	#ifdef TEST_232_485
		UsartInit(METER_USART  ,METER_USART_BPR  ,USART_DATA_8B,USART_STOPBITS_1,USART_PARITY_NO);	
		UsartInit(RS485_USART  ,RS485_USART_BPR  ,USART_DATA_8B,USART_STOPBITS_1,USART_PARITY_NO);
	#endif
	
//	#ifdef TEST_READLSNAL
//		j= (SYS_LSNAL_SPIMAXINDEX+g_syslsnal_struct.headindex-g_syslsnal_struct.tailindex)%SYS_LSNAL_SPIMAXINDEX;
//		for(i=0;i<j;i++){
//			addr = SYS_LSNAL_SPISTART_ADDR + (g_syslsnal_struct.tailindex+i) * LSNAL_PAGE_SIZE; 
//			ExteFlashRead(addr, tmpdata, 250);
//			printf("ExteFlashRead 0 is:\r\n");
//			for(times=0; times < 250;times++){
//				printf("%.2X ",tmpdata[times]);
//			}
//			printf("\r\n");
//		}
//	#endif
	
	while(1){
		OSTimeDlyHMSM(0, 0, 1, 0);

		test_rs232_485();		
//	  printf("\r\n g_sysmiscrun_struct.NLogTim_count = %d g_pro_struct.try_login_statu = %d\r\n",g_sysmiscrun_struct.NLogTim_count,g_pro_struct.try_login_statu);
//		printf(" g_sysalarm_struct.afterheadindex = %d   g_sysalarm_struct.aftertailindex = %d\r\n",g_sysalarm_struct.afterheadindex,g_sysalarm_struct.aftertailindex);
//		printf(" g_sysalarm_struct.beforeheadindex = %d   g_sysalarm_struct.beforetailindex = %d\r\n",g_sysalarm_struct.beforeheadindex,g_sysalarm_struct.beforetailindex);
//		printf(" g_syslsnal_struct.headindex = %d   g_syslsnal_struct.tailindex = %d\r\n",g_syslsnal_struct.headindex,g_syslsnal_struct.tailindex);
//		printf(" g_sysmiscrun_struct.have_sysAlarm_flag = %d \r\n",g_sysmiscrun_struct.have_sysAlarm_flag);
//		printf("\r\n PWR_C = %d BAT_ADC = %d\r\n",adc_result[0],adc_result[1]);

		
	/*	
		
		switch(Test_Step){
			#ifdef 	TEST_LOGINOUT
			case TEST_LOGINOUT:{
				if((g_pro_struct.try_login_statu  == 3)&&(test_loginout_count < 5)){	//已经登入
					OSTimeDlyHMSM(0, 0, 30, 0);		//延时30s准备登出
					ProUpLogout();	
					g_pro_struct.try_login_statu  = 4;			
					test_loginout_count++;
				}
				if(g_pro_struct.try_login_statu == 5){
					OSTimeDlyHMSM(0, 0, 30, 0);		//延时30s准备登入
					g_pro_struct.try_login_statu  = 0;
					g_sysmiscrun_struct.NLogTim_count = 0xFFFF;
					g_pro_struct.login_center_flag  = FALSE;
				}
				break;
			}
			#endif
			#ifdef TEST_CHECKTIME
			case TEST_CHECKTIME:{
				if(++times >= 60){
					ProUpCheckTime();
					times = 0;
				}				
				break;
			}
			#endif
			case 3:{
				break;
			}
			
			default : break;
		}
		*/
		
	}
}

