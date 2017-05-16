
#define SMS_GLOBAL
#include "include.h"

/***
+CMGL: 1,1,,81
08 91 683108200505F0
00 
0D 91 685151503712F6

00 00 41018231135023452A331DAE7BBDAC65747ACC2EEBAC65747ACC2E8F5440194CE68AC1642E580EE68ADD723A59EC454D6A835F2674F8B5C65CB06F8C069BC56A2E71DA3D02
+CMGL: 2,1,,55
0891683108200505F0040D91685151503712F600004101823113832328AA6170D87416A9A6A40C268BC164B01C2C66B399A038182E67A2CE603093310693C146
OK***/
uint8 SmsParaLen(uint8 p_in[],uint8 max_count)
{
	uint8 len;
	
	len = 0x00;
	while((*p_in != '#')&&(*p_in != '&'))
	{
		p_in ++;
		len ++;
		if(max_count != 0)
		{
			if(len > max_count)
			{
				len = 0;
				break;
			}
		}
	}
	return len;
}
void SmsAckPacket(uint8 *p_in,uint8 *p_out,uint8 ack_flag)
{
	uint8 gps_data[GPS_INFO_LEN],len;
	char str_ch[256];
	uint8 str_len;
	uint32 tmp_val_1,tmp_val_2;
	
	*p_out++ = '*';
	*p_out++ = ack_flag;
	len = StrLen(p_in,0)-2;
	MemCpy(p_out,p_in+1,len);
	p_out += len;
	*p_out++ = '&';
	GetGpsInfo(gps_data);
	str_len = sprintf(str_ch,"%02d%02d%02d",gps_data[TIME_INDEX],gps_data[TIME_INDEX+1],gps_data[TIME_INDEX+2]);
	MemCpy(p_out,(uint8*)str_ch,str_len);
	p_out += str_len;
	
	tmp_val_1  = gps_data[LAT_INDEX] << 24;
	tmp_val_1 += gps_data[LAT_INDEX+1] << 16;
	tmp_val_1 += gps_data[LAT_INDEX+2] << 8;
	tmp_val_1 += gps_data[LAT_INDEX+3];
	
	tmp_val_2  = gps_data[LONG_INDEX] << 24;
	tmp_val_2 += gps_data[LONG_INDEX+1] << 16;
	tmp_val_2 += gps_data[LONG_INDEX+2] << 8;
	tmp_val_2 += gps_data[LONG_INDEX+3];
	
	str_len = sprintf(str_ch,"%08u%09u%02d%02d%02d%02d%02d",tmp_val_1,tmp_val_2,gps_data[SPEED_INDEX],
	                  gps_data[DIR_INDEX],gps_data[DATE_INDEX+2],gps_data[DATE_INDEX+1],gps_data[DATE_INDEX]);
	MemCpy(p_out,(uint8*)str_ch,str_len);
	p_out += str_len;
	
	str_len = sprintf(str_ch,"%02x%02x%02x%02x",gps_data[STATUS_INDEX],gps_data[STATUS_INDEX+1],gps_data[STATUS_INDEX+2],gps_data[STATUS_INDEX+3]);
	MemCpy(p_out,(uint8*)str_ch,str_len);
	p_out += str_len;
	*p_out++ = '#';
	*p_out++ = '\0';
}
uint8 SmsParaReInit(uint8 p_in[],uint8 p_out[])
{
	uint8 res,ack_flag = 'N';
	
	sys_boot_para_struct.sys_para_init_flag = VALID_VAL_DWORD_AA;
	
	res = SysBootParaWrite();
	if(res)
	{
		ack_flag = 'Y';
	}
	sys_boot_para_struct.sys_para_init_flag = INVALID_VAL_DWORD_55;
	
	SmsAckPacket(p_in,p_out,ack_flag);
	return res;
}
void SmsParaQuery(uint8 p_out[])
{
	uint8 i,len;
	uint8 str_len;
	char str_ch[20];
	uint16 tmp_val;
	
	*p_out++ = '*';
	*p_out++ = 'Q';
	*p_out++ = 'C';
	*p_out++ = '1';
	len = StrLen(sys_private_para_struct.sms_alarm_center_num,LEN_16);
	str_len = sprintf(str_ch,"%d",len);
	MemCpy(p_out,(uint8*)str_ch,str_len);
	p_out += str_len;
	MemCpy(p_out,sys_private_para_struct.sms_alarm_center_num,len);
	p_out += len;
	
	*p_out++ = 'M';///本机号码
	str_len = sprintf(str_ch,"%d",11);
	MemCpy(p_out,(uint8*)str_ch,str_len);
	p_out += str_len;
	SimTermIdToAsciiNum(sys_private_para_struct.terminal_id,p_out);
	p_out += 11;
	
	*p_out++ = 'A';///APN
	len = StrLen(sys_private_para_struct.apn,LEN_16);
	str_len = sprintf(str_ch,"%d",len);
	MemCpy(p_out,(uint8*)str_ch,str_len);
	p_out += str_len;
	MemCpy(p_out,sys_private_para_struct.apn,len);
	p_out += len;
	
	*p_out++ = 'I';///主IP
	*p_out++ = '1';
	*p_out++ = '2';
	for(i=0;i<4;i++)
	{
		str_len = sprintf(str_ch,"%03d",sys_private_para_struct.master_ip_dns[1+i]);
		MemCpy(p_out,(uint8*)str_ch,str_len);
		p_out += str_len;
	}
	
	*p_out++ = 'i';///副IP
	*p_out++ = '1';
	*p_out++ = '2';
	for(i=0;i<4;i++)
	{
		str_len = sprintf(str_ch,"%03d",sys_private_para_struct.slaver_ip_dns[1+i]);
		MemCpy(p_out,(uint8*)str_ch,str_len);
		p_out += str_len;
	}
	
	*p_out++ = 'P';///端口
	tmp_val = sys_private_para_struct.port[0] << 8;
	tmp_val += sys_private_para_struct.port[1];
	str_len = sprintf(str_ch,"%d",tmp_val);
	*p_out++ = str_len + 0x30;///长度
	MemCpy(p_out,(uint8*)str_ch,str_len);
	p_out += str_len;
	
	*p_out++ = 'R';///软件版本号
	len = StrLen((uint8*)sys_const_para_struct.software_version,LEN_32);
	str_len = sprintf(str_ch,"%d",len);
	MemCpy(p_out,(uint8*)str_ch,str_len);
	p_out += str_len;
	MemCpy(p_out,(uint8*)sys_const_para_struct.software_version,len);
	p_out += len;
	
	*p_out++ = 'Y';///硬件版本号
	len = StrLen((uint8*)sys_serial_struct.hardware_version,LEN_12);
	if(!IsValidNumOrCharOrDot(sys_serial_struct.hardware_version,len))
	{
		len = 0;
	}
	str_len = sprintf(str_ch,"%d",len);
	MemCpy(p_out,(uint8*)str_ch,str_len);
	p_out += str_len;
	MemCpy(p_out,sys_serial_struct.hardware_version,len);
	p_out += len;
	
	*p_out++ = 'X';///终端序列号
	len = StrLen(sys_serial_struct.terminal_serial_num,LEN_20);
	if(!IsValidAscii(sys_serial_struct.terminal_serial_num,len))
	{
		len = 0;
	}
	str_len = sprintf(str_ch,"%d",len);
	MemCpy(p_out,(uint8*)str_ch,str_len);
	p_out += str_len;
	MemCpy(p_out,sys_serial_struct.terminal_serial_num,len);
	p_out += len;
	*p_out++ = '#';
	*p_out++ = '\0';
}
void SmsErrorAnalysis(uint8 p_in[],uint8 p_out[])
{
	uint8 i,tmp_ack_data[256],*p_tmp,tmp_len;
	uint8 str_len;
	char str_ch[20];
//	uint32 can_esr_reg;
	
	p_tmp = tmp_ack_data;
	*p_out++ = '*';
	*p_out++ = 'D';
	*p_out++ = p_in[2];
	
	if(p_in[2] == 'N')
	{
		if(gsm_misc_struct.gsm_state_array[AT_CGATT_INDEX] == AT_EXE_SUCESS)
		{
			*p_tmp++ = 0x01;
		}
		else
		{
			*p_tmp++ = 0x00;
		}
		if(gsm_misc_struct.gsm_state_array[AT_ATD_INDEX] == AT_EXE_SUCESS)
		{
			*p_tmp++ = 0x01;
		}
		else
		{
			*p_tmp++ = 0x00;
		}
		*p_tmp++ = g_ppp_struct.sms_ack_flag[0];
		*p_tmp++ = g_ppp_struct.sms_ack_flag[1];
		*p_tmp++ = g_ppp_struct.sms_ack_flag[2];

	}
	else if(p_in[2] == 'G')
	{
		if(gps_struct.sms_gprmc_ack[0] != VALID_VAL_2A)
		{
			gps_struct.sms_gprmc_ack[0] = '\0';
		}
		
		tmp_len = StrLen(gps_struct.sms_gprmc_ack+1,200);
		MemCpy(p_out,gps_struct.sms_gprmc_ack+1,tmp_len);
		p_out += tmp_len;
		goto RETURN_LAB;
	}
	else if(p_in[2] == 'C')
	{
	/**	can_esr_reg = CAN1->ESR;
		Uint32ToU8(p_tmp,can_esr_reg);
		p_tmp += 4;
		//
		*p_tmp++ = (uint8)(CAN_R_1 >> 24);
		*p_tmp++ = (uint8)(CAN_R_1 >> 24);
		*p_tmp++ = (uint8)(CAN_R_1 >> 8);
		*p_tmp++ = (uint8)(CAN_R_1 & 0xFF);
	
		MemCpy(p_tmp,can_struct.rx_can_buf,ONE_PACKET_LEN);
		p_tmp += ONE_PACKET_LEN;**/
	}
	/**
	else if(p_in[2] == 'L')
	{
		if(lock_record_struct.flag_2a == VALID_VAL_2A)
		{
			MemCpy(p_tmp,(uint8*)&lock_record_struct.lock_flag,sizeof(LOCK_RECORD_STRUCT)-1);
			p_tmp += sizeof(LOCK_RECORD_STRUCT)-1;
		}
		else
		{
			goto RETURN_LAB;
		}
	}
	**/
	for(i=0;i<p_tmp-tmp_ack_data;i++)
	{
		str_len = sprintf(str_ch,"%02x",tmp_ack_data[i]);
		MemCpy(p_out,(uint8*)str_ch,str_len);
		p_out += str_len;
	}
RETURN_LAB:
	*p_out++ = '#';
	*p_out++ = '\0';
}
void SmsParaSet(uint8 p_in[],uint8 p_out[])
{
	uint8 *p_temp = p_in+2,len,tmp_len,res,ack_flag = 'N';
	uint8 i,j,tmp_data[4];
	uint16 tmp_val;
	uint8 tmp_id[11];
	
	len = StrLen(p_in,0);
	len -= 3;
	while(len > 0)
	{
		switch(*p_temp)
		{
			case '1':
			{
				p_temp++;
				len--;
				tmp_len = SmsParaLen(p_temp,LEN_16);
				if(tmp_len > 0)
				{
					res = SetSmsAlarmCenterNum(p_temp,tmp_len);
					if(res != SUCCESS_ACK)
					{
						goto RETURN_LAB;
					}
					sms_struct.change_sms_center_num_flag = VALID_VAL_AA;
				}
				else
				{
					goto RETURN_LAB;
				}
				len -= tmp_len;
				p_temp += tmp_len;
				break;
			}
			case 'M':
			{
				p_temp++;
				len--;
				tmp_len = SmsParaLen(p_temp,11);
				if(tmp_len == 11)
				{
					res = SimNumToTermId(p_temp,tmp_id);
					if(!res)
					{
						goto RETURN_LAB;
					}
					
					res = SimSetSimNum(p_temp);
					if(!res)
					{
						goto RETURN_LAB;
					}
					MemCpy(sys_private_para_struct.terminal_id,tmp_id,5);
					sys_misc_run_struct.para_change_re_login_flag = TRUE;
					SysPrivateParaWrite();
				}
				else
				{
					goto RETURN_LAB;
				}
				len -= tmp_len;
				p_temp += tmp_len;
				break;
			}
			case 'A':
			{
				p_temp++;
				len--;
				tmp_len = SmsParaLen(p_temp,LEN_32);
				if(tmp_len > 0)
				{
					res = SetApn(p_temp,tmp_len);
					if(res != SUCCESS_ACK)
					{
						goto RETURN_LAB;
					}
				}
				else
				{
					goto RETURN_LAB;
				}
				len -= tmp_len;
				p_temp += tmp_len;
				break;
			}
			case 'I':
			{
				p_temp++;
				len--;
				tmp_len = SmsParaLen(p_temp,12);
				if((tmp_len == 12)&&(IsValidNum(p_temp,tmp_len)))
				{
					for(i=0;i<4;i++) 
					{
						tmp_data[i] = 0;
						for(j=0;j<3;j++) 
						{
							tmp_data[i] = tmp_data[i]*10+(p_temp[i*3+j]&0x0f);
						}
					}
					res = SetMasterIp(tmp_data,4);
					if(res != SUCCESS_ACK)
					{
						goto RETURN_LAB;
					}
				}
				else
				{
					goto RETURN_LAB;
				}
				len -= tmp_len;
				p_temp += tmp_len;
				break;
			}
			case 'i':
			{
				p_temp++;
				len--;
				tmp_len = SmsParaLen(p_temp,12);
				if((tmp_len == 12)&&(IsValidNum(p_temp,tmp_len)))
				{
					for(i=0;i<4;i++) 
					{
						tmp_data[i] = 0;
						for(j=0;j<3;j++) 
						{
							tmp_data[i] = tmp_data[i]*10+(p_temp[i*3+j]&0x0f);
						}
					}
					res = SetSlaverIp(tmp_data,4);
					if(res != SUCCESS_ACK)
					{
						goto RETURN_LAB;
					}
				}
				else
				{
					goto RETURN_LAB;
				}
				len -= tmp_len;
				p_temp += tmp_len;
				break;
			}
			case 'P':
			{
				p_temp++;
				len--;
				tmp_len = SmsParaLen(p_temp,5);
				if((tmp_len > 0)&&(IsValidNum(p_temp,tmp_len)))
				{
					tmp_val = 0;
					for(i=0;i<tmp_len;i++) 
					{
						tmp_val = tmp_val * 10;
						tmp_val += p_temp[i]&0x0f;	
					}
					tmp_data[0] = tmp_val >> 8;
					tmp_data[1] = tmp_val;
					res = SetPort(tmp_data,2);
					if(res != SUCCESS_ACK)
					{
						goto RETURN_LAB;
					}
				}
				else
				{
					goto RETURN_LAB;
				}
				len -= tmp_len;
				p_temp += tmp_len;
				break;
			}
			
			default:
			{
				goto RETURN_LAB;
			}
		}
		
		if((*p_temp == '#')||(len == 0))
		{
			ack_flag = 'Y';
			break;
		}
		if(len > 0)
		{
			p_temp++;
			len--;
		}
	}
	
RETURN_LAB:
	SmsAckPacket(p_in,p_out,ack_flag);
}

//
uint8 SmsLock(uint8 p_in[],uint8 p_out[])
{
//	uint8 res,ack_flag = 'N';
//	
//	uint8 data[RS232_LOCK_DATA_LEN] = {0x00,0x00,0x4c,0x04,0xae,0x08};
//	
//	MemCpy((uint8*)&rs232_lock_struct,data,RS232_LOCK_DATA_LEN);
//	
//	res = Rs232DataWrite();
//	if(res)
//	{
//		ack_flag = 'Y';
//	}
//	
//	SmsAckPacket(p_in,p_out,ack_flag);
//	return res;
	return 0;
}
uint8 SmsUnlock(uint8 p_in[],uint8 p_out[])
{
	uint8 res,ack_flag = 'N';
	
	sys_work_para_struct.rs232_stop_heart_min_counter = 0;
	sys_work_para_struct.lsnal_min_counter = 0;
	SysWorkParaWrite();
	
	res = Rs232DataInit();
	if(res)
	{
		ack_flag = 'Y';
	}
	
	SmsAckPacket(p_in,p_out,ack_flag);
	return res;
}
uint8 SmsProgramUpgrade(uint8 p_in[],uint8 p_out[])
{
	uint8 in_len,res;
	uint8 ack_flag = 'N';
	
	in_len = StrLen(p_in,0);
	res = FtpAddrAnalysis(p_in+1,in_len-2);
	if(res)
	{
		ack_flag = 'Y';
	}
	SmsAckPacket(p_in,p_out,ack_flag);
	return res;
}
void SmsTxMsg(uint8 msg[])
{
	uint8 exe_res;
	enum GSM_CUR_MODE tmp_mode = gsm_misc_struct.cur_mode;

	exe_res = GprsSwitchAtMode(tmp_mode);
	
	if(exe_res)
	{
		MemCpy(sms_struct.sms_center_num,"8613800250500",13);
		sms_struct.sms_center_num[14] = 0x00;
		MemCpy(sms_struct.sms_tx_num,"8615150573216",13);
		sms_struct.sms_tx_num[14] = 0x00;
		sms_struct.dcs = 0x00;
		sms_struct.pid = 0x00;
		SmsTx(msg);
	
		GprsSwitchGprsDataMode(tmp_mode);
	}
}
void SmsTx(uint8 p_in[])
{
	uint8 tx_data[512];
	uint16 tx_len = 0;
	uint8 str_len;
	char str_ch[20];
	uint16 tmp_len;
	
UP_LAB:	
	tmp_len = SmsEncodePdu(p_in,StrLen(p_in,255),tx_data,&tx_len);
	str_len = sprintf(str_ch,"%d\r",tmp_len);
	gsm_misc_struct.gsm_state_array[AT_CMGS_INDEX] = AT_NOT_EXE;
	GprsSendAtCmd(AT_CMGS_INDEX,(uint8*)str_ch,str_len,">",1);
	if(gsm_misc_struct.gsm_state_array[AT_CMGS_INDEX] == AT_EXE_SUCESS)
	{
		tx_data[tx_len] = 0x1a;///CTRL_Z
		GprsUartFixedLenSend(tx_data,tx_len+1);
	}
	gsm_misc_struct.gsm_state_array[AT_NULL_INDEX] = AT_NOT_EXE;
	GprsSendAtCmd(AT_NULL_INDEX,NULL,0,"OK",2);
	
	if(sms_struct.change_sms_center_num_flag == VALID_VAL_AA)
	{
		sms_struct.change_sms_center_num_flag = 0x00;
		MemCpy(sms_struct.sms_tx_num,sys_private_para_struct.sms_alarm_center_num,StrLen(sys_private_para_struct.sms_alarm_center_num,LEN_16)+1);
		goto UP_LAB;
	}
}
void SmsAnalysis(uint8 data[])
{
	uint8 tmp_len,res,mat_index = 0,tel_index = 0;
	uint8 tmp_data[256],exe_flag[3]={0,0,0};
	uint8 upgrade_success[] = "upgrade program success";
	uint8 upgrade_fail[] = "upgrade program fail";
	uint8 upgrade_flag;
	
	tmp_len = StrLen(data,0);
	if((data[0] != '*')||(data[tmp_len-1] != '#'))
	{
		goto RETURN_LAB;
	}
	
	mat_index = SubMatch(sys_private_para_struct.sms_alarm_center_num,StrLen(sys_private_para_struct.sms_alarm_center_num,LEN_16),
	               sms_struct.sms_tx_num,StrLen(sms_struct.sms_tx_num,LEN_32));
	if(mat_index > 0)
	{
		tel_index = 1;
		goto TEL_LAB;
	}
	mat_index = SubMatch((uint8*)sys_const_para_struct.second_sms_center_num,StrLen((uint8*)sys_const_para_struct.second_sms_center_num,LEN_32),
	                sms_struct.sms_tx_num,StrLen(sms_struct.sms_tx_num,LEN_32));
	if(mat_index > 0)
	{
		tel_index = 2;
		goto TEL_LAB;
	}
	mat_index = SubMatch((uint8*)sys_const_para_struct.third_sms_cetner_num,StrLen((uint8*)sys_const_para_struct.third_sms_cetner_num,LEN_32),
	               sms_struct.sms_tx_num,StrLen(sms_struct.sms_tx_num,LEN_32));
	if(mat_index > 0)
	{
		tel_index = 3;
	}
TEL_LAB:	
	if(tel_index == 0)
	{
		goto RETURN_LAB;
	}
	
	res = MemCmp("*U#",data,StrLen("*U#",0));
	if(res)
	{
		exe_flag[0] =SmsParaReInit(data,tmp_data);
		goto ACK_LAB;
	}
	
	res = MemCmp("*kq#",data,StrLen("*kq#",0));
	if(res)
	{
		SmsParaQuery(tmp_data);
		goto ACK_LAB;
	}
	if(data[1] == 'D')
	{
		if((data[2] == 'N')||(data[2] == 'G')||(data[2] == 'C')||(data[2] == 'L'))
		{
			SmsErrorAnalysis(data,tmp_data);
			goto ACK_LAB;
		}
		else
		{
			goto RETURN_LAB;
		}
	}
	
	if(data[1] == 'C')
	{
		SmsParaSet(data,tmp_data);
		goto ACK_LAB;
	}
	
	res = MemCmp("*L#",data,StrLen("*L#",0));
	if(res)
	{
		SmsLock(data,tmp_data);
		goto ACK_LAB;
	}
	
	res = MemCmp("*J#",data,StrLen("*J#",0));
	if(res)
	{
		SmsUnlock(data,tmp_data);
		goto ACK_LAB;
	}
	
	mat_index = SubMatch("*ftp://",StrLen("*ftp://",0),data,tmp_len);
	if(mat_index > 0)
	{
		exe_flag[2] = SmsProgramUpgrade(data,tmp_data);
		goto ACK_LAB;
	}
	goto RETURN_LAB;
/**
	tmp_data[0] = '*';
	tmp_data[1] = 'g';
	tmp_data[2] = 'z';
	tmp_data[3] = 'a';
	tmp_data[4] = '#';
	tmp_data[5] = '\0';
**/
ACK_LAB:///短信回复
	SmsTx(tmp_data);
	
	if(exe_flag[0])
	{
		GprsDelay(5,FALSE);
		TermReset();
	}
	else if(exe_flag[2])
	{
		upgrade_flag = FALSE;
		
		res = GprsSwitchGprsDataMode((uint8)UDP_MODE);
		
		if(res)
		{
			upgrade_flag = FtpMain();
			
			gsm_misc_struct.cur_mode = AT_INIT_MODE;
			res = GprsSwitchAtMode((uint8)UDP_MODE);	
		}
		
		if(gsm_misc_struct.cur_mode == AT_INIT_MODE)
		{
			if(upgrade_flag)
			{
				SmsTx(upgrade_success);
			}
			else
			{
				SmsTx(upgrade_fail);
			}
		}
		
		if(upgrade_flag)
		{
			SysReset();///置程序更新标志，复位；
		}
	}
RETURN_LAB:
	return;
}
void SmsNumDecode(uint8 rev_num[],uint8 num_len,uint8 normal_num[])///字符反转
{
	uint8 i;

	for(i=0;i<num_len;i+=2)
	{
		normal_num[i+1] = rev_num[i];
		normal_num[i] = rev_num[i+1];
	}
	
	if(normal_num[num_len-1] == 'F')
	{
		normal_num[num_len-1] = '\0';
	}
	else
	{
		normal_num[num_len-2] = '\0';
	}
}
uint8 SmsNumEncode(uint8 normal_num[],uint8 num_len,uint8 rev_num[])
{
	uint8 i,j;

	j = num_len/2;
	for(i=0;i<j;i++)
	{
		rev_num[2*i+1] = normal_num[2*i];
		rev_num[2*i] = normal_num[2*i+1];
	}
	
	if(num_len % 2)
	{
		rev_num[2*j] = 'F';
		rev_num[2*j+1] = normal_num[num_len-1];
	}
	
	return (2*(j+1));
}
uint8 SmsDecode7bit(uint8 *p_src,uint8 src_len,uint8 *p_dst)   
{   
	uint8 i,j,byte_num,left_num;  

	i = 0;   
	j = 0;   
	byte_num = 0;   
	left_num = 0;   

	while(i<src_len)   
	{   
		*p_dst = ((*p_src << byte_num) | left_num) & 0x7f;    
		left_num = *p_src >> (7-byte_num);     
		p_dst++;   
		j++;      
		byte_num++;   

		if(byte_num == 7)   
		{   
			*p_dst = left_num;      
			p_dst++;   
			j++;   
			byte_num = 0;   
			left_num = 0;   
		}   
		p_src++;   
		i++;   
	}   
	*p_dst = 0;  
	
	return j;     
} 
uint8 SmsEncode7bit(uint8 *p_src,uint16 src_len,uint8 *p_dst)   
{   
	uint8 i,j,byte_num,right_num;

	i = 0;   
	j = 0;   
	byte_num = 0;   
	right_num = 0;     

	while(i<src_len+1)   
	{    
		byte_num = i & 7;   

		if(byte_num == 0)   
		{     
			right_num = *p_src;   
		}   
		else   
		{       
			*p_dst = (*p_src << (8-byte_num)) | right_num;      
			right_num = *p_src >> byte_num;       
			p_dst++;   
			j++;    
		}  

		p_src++; 
		i++;   
	} 

	return j;        
}
uint8 SmsDecodeUcs2(uint8 *p_src,uint8 src_len,uint8 *p_dst)   
{   
	uint8 i;  

	///UNICODE    
	for(i=0;i<src_len;i+=2)   
	{    
		p_src++;   
		*p_dst++= *p_src++;   
	}   

	return (i >> 1);   
}   
uint8 SmsEncodeUcs2(uint8 *p_src,uint8 src_len,uint8 *p_dst)   
{   
	uint8 i;   

	for(i=0;i<src_len;i++)   
	{      
		*p_dst++ = 0x00;    
		*p_dst++ = * p_src++;   
	}   

	return (i << 1);   
} 
uint8 SmsDecode8bit(uint8 *p_src,uint8 src_len,uint8 *p_dst)   
{    
	uint8 i;
	
	for(i=0;i<src_len;i++)
	{
		*p_dst++=*p_src++; 
	}
	
	return i;   
} 
uint8 SmsEncode8bit(uint8 *p_src,uint16 src_len,uint8 *p_dst)   
{   
	uint8 i;

	for(i=0;i<src_len;i++)   
	{
		*p_dst++=*p_src++; 
	}

	return src_len;   
}
uint8 SmsTextDecode(uint8 *p_src,uint16 len,uint8 *p_dst,uint8 dcs_flag)
{
	uint16 tmp_len;
	uint8 tmp_buf[256];
	
	tmp_len = AsciiToHex(p_src,len,tmp_buf);
	if(dcs_flag == GSM_7BIT)
	{
		tmp_len = SmsDecode7bit(tmp_buf,tmp_len,p_dst);
	}
	else if(dcs_flag == GSM_UCS2)
	{
		tmp_len = SmsDecodeUcs2(tmp_buf,tmp_len,p_dst);     
	}
	else
	{
		tmp_len = SmsDecode8bit(tmp_buf,tmp_len,p_dst);  
	}

	return tmp_len;
}
uint8 SmsTextEncode(uint8 *p_src,uint8 len,uint8 *p_dst,uint8 dcs_flag)
{
	uint16 tmp_len;
	uint8 tmp_buf[256];

	if(dcs_flag == GSM_7BIT)
	{
		tmp_buf[0] = len;
		tmp_len = SmsEncode7bit(p_src,len,tmp_buf+1);
	}
	else if(dcs_flag == GSM_UCS2)
	{
		tmp_buf[0] = 2*len;
		tmp_len = SmsEncodeUcs2(p_src,len,tmp_buf+1);     
	}
	else
	{
		tmp_buf[0] = len;
		tmp_len = SmsEncode8bit(p_src,len,tmp_buf+1);  
	}
	tmp_len = HexToAscii(tmp_buf,tmp_len+1,p_dst);

	return tmp_len;
}
uint8 SmsDecodePdu(uint8 *p_src,uint8 *p_dst)   
{    
	uint8 tmp_len,need_decode_flag,dcs_flag;
	uint16 tmp_len_16,txt_len,tmp_index=0;   
	
	/// SMSC地址信息段    
	AsciiToHex(p_src,2,&tmp_len);    /// 取长度    
	if(tmp_len > 12)  
	{   
		tmp_len = 0;
		goto RETURN_LAB;
	}
	
	if(tmp_len == 0)///无短信中心情况
	{
		MemCpy(sms_struct.sms_center_num,(uint8*)sys_const_para_struct.sms_city_center_num,StrLen((uint8*)sys_const_para_struct.sms_city_center_num,LEN_32));
		p_src += 2;
		tmp_index += 2;
	}
	else
	{
		tmp_len = (tmp_len - 1) << 1;						  /// SMSC号码串长度,去掉了91;    
		p_src += 4;              							  /// 指针后移,长度两个字节，91两个字节。共4个字节    
		tmp_index += 4; 
		SmsNumDecode(p_src,tmp_len,sms_struct.sms_center_num);    /// 转换SMSC号码到目标PDU串      
		p_src += tmp_len;        /// 指针后移,此时到了PDUType    
		tmp_index += tmp_len;   
	}
	/// TPDU段基本参数、回复地址等    
	///--PDUType     
	p_src += 2;        					/// 指针后移    
	tmp_index += 2;   
	///--OA-- 
	///包含回复地址，取回复地址信息    
	AsciiToHex(p_src,2,&tmp_len);    /// 取长度,OA的长度是指具体的号码长度
	if(tmp_len & 1) 
	{
		tmp_len += 1;         				/// 调整奇偶性
	}
	p_src += 4;          			  /// 指针后移，除去长度，和91,共4个字节    
	tmp_index += 4;   
	if((tmp_len < 2*2)||(tmp_len > 12*2))   
	{  
		tmp_len = 0;
		goto RETURN_LAB;
	}	
	SmsNumDecode(p_src,tmp_len,sms_struct.sms_tx_num);
	p_src += tmp_len;        		 /// 指针后移    
	tmp_index += tmp_len;   

	/// TPDU段协议标识、编码方式、用户信息等    
	AsciiToHex(p_src,2,&sms_struct.pid);    /// 取协议标识(TP-PID)    
	p_src += 2;   
	tmp_index += 2;        
	AsciiToHex(p_src,2,&sms_struct.dcs);    /// 取编码方式(TP-DCS),这个解码时和回复时用    
	p_src += 2;       
	tmp_index += 2;   

	p_src += 14;       				/// 指针后移,时间戳14字节
	tmp_index+=14;   
	AsciiToHex(p_src,2,&tmp_len);    /// 用户信息长度(TP-UDL)    
	p_src += 2;         
	tmp_index += 2;   
	tmp_len_16 = tmp_len;
	
	need_decode_flag = FALSE;
	if((sms_struct.dcs & 0x0c) == GSM_7BIT)       
	{   
		/// 7-bit解码    
		txt_len = (tmp_len_16 % 8)?(tmp_len_16 * 7 / 8 + 1):(tmp_len_16 * 7 / 8);   
		txt_len *= 2;   
		need_decode_flag = TRUE;
		dcs_flag = GSM_7BIT;   
	}   
	else if((sms_struct.dcs & 0x0c) == GSM_UCS2)   
	{   
		/// UCS2解码    
		txt_len = tmp_len_16 * 2;   
		need_decode_flag = TRUE;
		dcs_flag = GSM_UCS2;
	}   
	else   
	{   
		/// 8-bit解码 
		dcs_flag = GSM_8BIT; 
		txt_len=tmp_len_16 * 2;  
		need_decode_flag = TRUE;
	}
	
	tmp_index += txt_len;
	
	if(need_decode_flag)
	{
		tmp_len = SmsTextDecode(p_src,txt_len,p_dst,dcs_flag);
		if(p_dst[tmp_len-1] != '#')
		{
			tmp_len -= 1;
		}
	}
	else
	{
		tmp_len = 0;
	}
	p_dst[tmp_len] = '\0';

RETURN_LAB: 
	return tmp_index;   
}   
uint8 SmsEncodePdu(uint8 *p_src,uint8 len,uint8 *p_dst,uint16 *dst_len)  
{   
	uint8 *s_tmp,*p_tmp = p_dst,buf[256]; 
	uint8 dcs_flag;
	uint16 i,j;

	/// SMSC地址信息段  
	p_tmp[0] = 0x30;//SCA = 00
	p_tmp[1] = 0x30;
	p_tmp += 2;
	s_tmp = p_tmp;
	
	buf[0]=0x11;///PDUTYPE    
	buf[1]=0x00;///MR    
	HexToAscii(buf,2,p_tmp); 
	p_tmp += 4;
	
	///TPDU段基本参数、目标地址等 
	i = StrLen(sms_struct.sms_tx_num,LEN_32);	
	buf[0] = i;   	/// 目标地址数字个数(TP-DA地址字符串真实长度)  
	buf[1] = 0x91;	/// 固定: 用国际格式号码  
	HexToAscii(buf,2,p_tmp);  		  /// 转换4个字节到目标PDU串 
	p_tmp += 4;
	i = SmsNumEncode(sms_struct.sms_tx_num,i,p_tmp);
	p_tmp += i;  

	/// TPDU段协议标识、编码方式、用户信息等  
	buf[0] = sms_struct.pid;        /// 协议标识(TP-PID)    
	buf[1] = sms_struct.dcs & 0x0c; /// 用户信息编码方式(TP-DCS)    
	buf[2] = 0x8f;            		/// 有效期(TP-VP)为12小时    
	HexToAscii(buf,3,p_tmp);
	p_tmp += 6;

	if((sms_struct.dcs & 0x0c) == GSM_7BIT)       
	{   
		/// 7-bit编码方式 
		dcs_flag = GSM_7BIT;      
	}   
	else if((sms_struct.dcs & 0x0c)== GSM_UCS2)   
	{   
		/// UCS2编码方式 
		dcs_flag = GSM_UCS2;		    
	}   
	else   
	{   
		/// 8-bit编码方式  
		dcs_flag = GSM_8BIT;		    
	}   
	i = SmsTextEncode(p_src,len,p_tmp,dcs_flag);
	p_tmp += i;
	j = p_tmp - s_tmp;
	*dst_len = p_tmp - p_dst;
	
	return (j >> 1);
} 

void SmsProcessFun(uint8 data[],uint16 len)
{
	uint8 sms_len,sms_txt[256];
	uint8 sms_index,mat_data_1[2] = {0x0d,0x0a},mat_data_2[] = {"CMGL: "};
	uint16 i,mat_index;
	
	i = 0;
	while(len > 0)
	{
	
		sms_index = data[i] - 0x30;
		if(data[i+1] != ',')
		{
			sms_index = (sms_index << 8) + data[i+1] - 0x30;
		}
		
		mat_index = SubMatch(mat_data_1,2,data,len);
		if(mat_index == 0)
		{
			break;
		}
		data += mat_index;
		
		sms_txt[0] = '\0';
		sms_len = SmsDecodePdu(data,sms_txt);
		SmsAnalysis(sms_txt);
		GprsDelay(3*WAIT_1S,FALSE);
		data += sms_len;
		
		if(len >= sms_len)
		{
			len -= sms_len;
			mat_index = SubMatch(mat_data_2,StrLen(mat_data_2,0),data,len);
			if(mat_index == 0)
			{
				break;
			}
			data += mat_index;
		}
		else
		{
			break;
		}
	}
	gsm_misc_struct.gsm_state_array[AT_CMGD_INDEX] = AT_NOT_EXE;
	GprsSendAtCmd(AT_CMGD_INDEX,NULL,0,"OK",2);///删除所有短信
}
