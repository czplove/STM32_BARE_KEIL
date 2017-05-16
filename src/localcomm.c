#include "include.h"




//void LocalCommFtpState(uint8 ftp_state,uint32 rt_size,uint32 rc_size)///升级状态提示
//{
//	uint8 tx_uart_data[256];
//	
//	tx_uart_data[LOCAL_COMM_DATA_INDEX+0] = ftp_state;
//	Uint32ToU8(tx_uart_data+LOCAL_COMM_DATA_INDEX+1,rt_size);
//	Uint32ToU8(tx_uart_data+LOCAL_COMM_DATA_INDEX+5,rc_size);
//	LocalCommTx(tx_uart_data,9,LOCAL_COMM_CMD_FTP_PROGRAME+LOCAL_COMM_CMD_ACK,111111);
//}
//uint16 LocalCommGetSysState(uint8 data[])
//{
//	uint8 *p = data;
//	uint8 i,tmp_len;
//	
//	tmp_len = GetGpsInfo(p);
//	
//	for(i=0;i<4;i++)
//	{
//		p[i+STATUS_INDEX] = 0x00;
//	}
//	
//	if(sys_work_para_struct.term_run_status_word & STATE_A_GPS)
//	{
//		data[3+STATUS_INDEX] |= LOCAL_COMM_STATE_A_GPS;
//	}
//	if(sys_work_para_struct.term_run_status_word & STATE_N_LAT)
//	{
//		data[3+STATUS_INDEX] |= LOCAL_COMM_STATE_N_LAT;
//	}
//	if(sys_work_para_struct.term_run_status_word & STATE_E_LONG)
//	{
//		data[3+STATUS_INDEX] |= LOCAL_COMM_STATE_E_LONG;
//	}
//	if(sys_work_para_struct.term_run_status_word & STATE_ACC_ON)
//	{
//		data[3+STATUS_INDEX] |= LOCAL_COMM_STATE_ACC_ON;
//	}
//	if(sys_work_para_struct.term_run_status_word & STATE_METER_LOCK)
//	{
//		data[3+STATUS_INDEX] |= LOCAL_COMM_STATE_METER_LOCK;
//	}
//	if(sys_work_para_struct.term_run_status_word & STATE_METER_COMM_BREAKDOWN)
//	{
//		data[3+STATUS_INDEX] |= LOCAL_COMM_STATE_METER_COMM_BREAKDOWN;
//	}
//	if(sys_work_para_struct.term_run_status_word & STATE_PWR_DOWN)
//	{
//		data[3+STATUS_INDEX] |= LOCAL_COMM_STATE_PWR_DOWN;
//	}
//	if(sys_work_para_struct.term_run_status_word & STATE_LOW_VOLTAGE)
//	{
//		data[3+STATUS_INDEX] |= LOCAL_COMM_STATE_LOW_VOLTAGE;
//	}
//	if(sys_work_para_struct.term_run_status_word & STATE_SIMCARD_CHANGE)
//	{
//		data[2+STATUS_INDEX] |= LOCAL_COMM_STATE_SIMCARD_CHANGE;
//	}
//	if(sys_work_para_struct.term_run_status_word & STATE_GPS_BREAKDOWN)
//	{
//		data[2+STATUS_INDEX] |= LOCAL_COMM_STATE_GPS_BREAKDOWN;
//	}
//	if(sys_work_para_struct.term_run_status_word & STATE_GPS_ANT_OFF)
//	{
//		data[2+STATUS_INDEX] |= LOCAL_COMM_STATE_GPS_ANT_OFF;
//	}
//	if(sys_work_para_struct.term_run_status_word & STATE_OVER_SPEED)
//	{
//		data[2+STATUS_INDEX] |= LOCAL_COMM_STATE_OVER_SPEED;
//	}
//	if(sys_work_para_struct.term_run_status_word & STATE_TRAILER)
//	{
//		data[2+STATUS_INDEX] |= LOCAL_COMM_STATE_TRAILER;
//	}
//	if(sys_work_para_struct.term_run_status_word & STATE_SHELL_OFF)
//	{
//		data[2+STATUS_INDEX] |= LOCAL_COMM_STATE_SHELL_OFF;
//	}
//	
//	p += tmp_len;
//	
//	*p++ = sys_data_struct.gps_info_of_gprs[SAT_INDEX];
//	*p++ = sys_misc_run_struct.gsm_csq_val;
//	
//	for(i=0;i<4;i++)
//	{
//		*p++ = gsm_misc_struct.gsm_mode_exe_flag[i];
//	}
//	
//	MemCpy(p,sys_misc_run_struct.link_center_ip,4);
//	p += 4;
//	*p++ = sys_misc_run_struct.power_voltage_val >> 8;
//	*p++ = sys_misc_run_struct.power_voltage_val;
//	
//	if(sys_work_para_struct.no_simcard_falg)
//	{
//		*p++ = 1;///无SIM卡
//	}
//	else
//	{
//		*p++ = 0;
//	}
//	
//	return (p-data);
//}
//void LocalCommTx(uint8 data[],uint16 len,uint8 cmd,uint32 pwd)
//{
//	uint8 *p;
//	
//	p = data;
//	*p++ = '*';
//	*p++ = (len + 8) >> 8;
//	*p++ = len + 8;
//	*p++ = cmd;
//	Uint32ToU8(p,pwd);

//	data[len+8] = U8SumCheck(data+1,len+7);
//	data[len+9] = '#';
//	LocalUartFixedLenSend(data,len+10);
//}	
//void LocalCommRx(void)
//{
//	uint8 tx_data[600],cmd,res;
//	uint8 rx_data[LOCAL_UART_BUF_LEN];
//	uint16 tx_len,rx_len,total_len;
//	uint32 pwd;
//		
//	///*,len_h,len_l,cmd,terminal_pass_word(4字节，高字节在前，默认00 01 B2 07),data,add_check,#
//	total_len = GetLocalUartRxData(rx_data);
//	
//	if(total_len == 0)
//	{
//		goto RETURN_LAB;
//	}
//	
//	
//	if(('*' != rx_data[0])||('#' != rx_data[total_len-1])) 
//	{
//		goto RETURN_LAB;	
//	}
//	
//	rx_len = (rx_data[LOCAL_COMM_LEN_INDEX] << 8) + rx_data[LOCAL_COMM_LEN_INDEX+1];
//	if(rx_len != total_len - 2)
//	{
//		goto RETURN_LAB;
//	}
//	
//	if(U8SumCheck(rx_data+1,rx_len-1) != rx_data[rx_len])
//	{
//		goto RETURN_LAB;
//	}
//	
//	pwd = U8ToUint32(rx_data+LOCAL_COMM_PASSWORD_INDEX);
//	if(pwd != 0x1B207)
//	{
//		goto RETURN_LAB;
//	}
//	
//	rx_len = rx_len - 8;		///内容长度
//	
//	switch(rx_data[LOCAL_COMM_CMD_INDEX])
//	{
//		case LOCAL_COMM_CMD_RESET:
//		{
//			TermReset();
//			break;
//		}
//		case LOCAL_COMM_CMD_QUERY_SYS_PARA:
//		{
//			if((rx_len % 2)!= 0X00)
//			{
//				goto RETURN_LAB;
//			}
//			tx_len = QuerySysPrivatePara(tx_data+LOCAL_COMM_DATA_INDEX,rx_data+LOCAL_COMM_DATA_INDEX,rx_len,TRUE);
//			cmd = LOCAL_COMM_CMD_ACK + LOCAL_COMM_CMD_QUERY_SYS_PARA;
//			break;
//		}
//		case LOCAL_COMM_CMD_SET_SYS_PARA:
//		{
//			res = SetSysPrivatePara(rx_data+LOCAL_COMM_DATA_INDEX,rx_len,TRUE);
//			tx_len = 1;
//			tx_data[LOCAL_COMM_DATA_INDEX] = res;
//			cmd = LOCAL_COMM_CMD_ACK + LOCAL_COMM_CMD_SET_SYS_PARA;
//			break;
//		}
//		case LOCAL_COMM_CMD_QUERY_SYS_STATE:
//		{
//			tx_len = LocalCommGetSysState(tx_data+LOCAL_COMM_DATA_INDEX);
//			cmd = LOCAL_COMM_CMD_ACK + LOCAL_COMM_CMD_QUERY_SYS_STATE;
//			break;
//		}
//		default: 
//		{
//			goto RETURN_LAB;
//		}
//	}
//	
//	LocalCommTx(tx_data,tx_len,cmd,pwd);
//	if(sys_boot_para_struct.sys_para_init_flag == VALID_VAL_DWORD_AA)
//	{
//		SysBootParaWrite();
//		LocalCommDelay(2*WAIT_1S);
//		SysReset();
//	}
//	if(sys_misc_run_struct.term_reset_falg == VALID_VAL_AA)
//	{
//		LocalCommDelay(2*WAIT_1S);
//		TermReset2();
//	}
//RETURN_LAB:
//	return;
//}


void LocalCommMain(void)
{
	
}
