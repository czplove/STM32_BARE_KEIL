/****************************************Copyright (c)**************************************************
**                                                                
**--------------File Info-------------------------------------------------------------------------------
** File Name: xg_protocol.c
** Last modified Date:
** Last Version:
** Descriptions: 
*******************************************************************************************************/
#define FTP_GLOBAL 
#define FTP_DEBUG
#include "tiza_include.h"

#define FTP_FLAG_FIN 	0X01
#define FTP_FLAG_SYN 	0X02
#define FTP_FLAG_RST 	0X04
#define FTP_FLAG_PSH 	0X08
#define FTP_FLAG_ACK 	0X10
#define FTP_FLAG_URG 	0X20
#define FTP_FLAG_MASK 	(FTP_FLAG_FIN|FTP_FLAG_SYN|FTP_FLAG_RST|FTP_FLAG_PSH|FTP_FLAG_ACK|FTP_FLAG_URG)

uint8 ftp_wait_flag = FALSE;

//===========================================================================================================


static uint8 FtpCheckSIM(void)
{
//	uint8 i,res=RES_WAIT;
//	uint8 tmp_str[100];
//	uint8 tmp_len;
//	
//		
//	L218SendAtCmd(AT_CPIN_INDEX,NULL,0,(uint8 *)OK_ACK,2);//-检查SIM
//		
//	if(g_at_cmd_struct[AT_CPIN_INDEX].exe_flag == EXE_OK)
//		res = RES_TRUE;
//	else
//		res = RES_FALSE;	
//	
//	return res;
	
	return RES_TRUE;
}

static uint8 FtpCheckNET(void)
{
	uint8 res=RES_WAIT;
	
	L218SendAtCmd(AT_CREG_INDEX,NULL,0,(uint8 *)OK_ACK,2);			
	
	if(g_at_cmd_struct[AT_CREG_INDEX].exe_flag == EXE_OK)
		res = RES_TRUE;
	else
		res = RES_FALSE;	
		
	return res;
}

static uint8 FtpCGATT(void)
{
	uint8 res=RES_WAIT;
	
	L218SendAtCmd(AT_CGATT_INDEX,NULL,0,(uint8 *)OK_ACK,2);
	
	if(g_at_cmd_struct[AT_CGATT_INDEX].exe_flag == EXE_OK)
		res = RES_TRUE;
	else
		res = RES_FALSE;	
		
	return res;
}

static uint8 FtpSAPBR(void)
{
	uint8 res=RES_FALSE;
	
	//-1.Set the type of internet connection for FTP
	L218SendAtCmd(AT_SAPBR_INDEX,(uint8*)FTP_TYPEOFINTERCONNECT,21,(uint8 *)OK_ACK,2);
	//L218SendAtCmd(AT_SAPBR_INDEX, NULL, 0,(uint8 *)OK_ACK,2);
	
	if(g_at_cmd_struct[AT_SAPBR_INDEX].exe_flag == EXE_OK)
		res = RES_TRUE;
	else
		return res;
	//-2.Set APN for FTP	
	L218SendAtCmd(AT_SAPBR_INDEX,(uint8*)FTP_SETAPN,18,(uint8 *)OK_ACK,2);
	
	if(g_at_cmd_struct[AT_SAPBR_INDEX].exe_flag == EXE_OK)
		res = RES_TRUE;
	else
		return res;	
	//-3.Open bearer	
	L218SendAtCmd(AT_SAPBR_INDEX,(uint8*)FTP_OPENBEARER,4,(uint8 *)OK_ACK,2);
	//-暂时不考虑应答?是否有应答待验证??
	if(g_at_cmd_struct[AT_SAPBR_INDEX].exe_flag == EXE_OK)
		res = RES_TRUE;
	else
		return res;		
		
	return res;
}

static uint8 FtpFTPTYPE(void)
{
	uint8 res=RES_WAIT;
	
	L218SendAtCmd(AT_FTPTYPE_INDEX,NULL,0,(uint8 *)OK_ACK,2);
	
	if(g_at_cmd_struct[AT_FTPTYPE_INDEX].exe_flag == EXE_OK)
		res = RES_TRUE;
	else
		res = RES_FALSE;	
		
	return res;
}

static uint8 FtpFTPSERV(void)
{
	uint8 res=RES_WAIT;
	uint8 temp_cmd[40];
	
	sprintf((char*)temp_cmd, (const char*)AT_FTPSERV, g_proupgread_struct.ip_domain[2],g_proupgread_struct.ip_domain[3],g_proupgread_struct.ip_domain[4],g_proupgread_struct.ip_domain[5]);
	g_at_cmd_struct[AT_FTPSERV_INDEX].cmd_text = (uint8 *)temp_cmd;
	
	L218SendAtCmd(AT_FTPSERV_INDEX,NULL,0,(uint8 *)OK_ACK,2);	
	if(g_at_cmd_struct[AT_FTPSERV_INDEX].exe_flag == EXE_OK)
		res = RES_TRUE;
	else
		res = RES_FALSE;	
		
	return res;
}

static uint8 FtpFTPPORT(void)
{
	uint8 res=RES_WAIT;
	uint8 temp_cmd[40];
	
	sprintf((char*)temp_cmd, (const char*)AT_FTPPORT, g_proupgread_struct.port);
	g_at_cmd_struct[AT_FTPPORT_INDEX].cmd_text = (uint8 *)temp_cmd;
	
	L218SendAtCmd(AT_FTPPORT_INDEX,NULL,0,(uint8 *)OK_ACK,2);	
	if(g_at_cmd_struct[AT_FTPPORT_INDEX].exe_flag == EXE_OK)
		res = RES_TRUE;
	else
		res = RES_FALSE;	
		
	return res;
}

static uint8 FtpFTPUN(void)
{
	uint8 res=RES_WAIT;
	uint8 temp_cmd[40];
	
	sprintf((char*)temp_cmd, (const char*)AT_FTPUN, g_proupgread_struct.user_name);
	g_at_cmd_struct[AT_FTPUN_INDEX].cmd_text = (uint8 *)temp_cmd;
	
	L218SendAtCmd(AT_FTPUN_INDEX,NULL,0,(uint8 *)OK_ACK,2);	
	if(g_at_cmd_struct[AT_FTPUN_INDEX].exe_flag == EXE_OK)
		res = RES_TRUE;
	else
		res = RES_FALSE;	
		
	return res;
}

static uint8 FtpFTPPW(void)
{
	uint8 res=RES_WAIT;
	uint8 temp_cmd[40];
	
	sprintf((char*)temp_cmd, (const char*)AT_FTPPW, g_proupgread_struct.pass_word);
	g_at_cmd_struct[AT_FTPPW_INDEX].cmd_text = (uint8 *)temp_cmd;
	
	L218SendAtCmd(AT_FTPPW_INDEX,NULL,0,(uint8 *)OK_ACK,2);	
	if(g_at_cmd_struct[AT_FTPPW_INDEX].exe_flag == EXE_OK)
		res = RES_TRUE;
	else
		res = RES_FALSE;	
		
	return res;
}

static uint8 FtpFTPSIZE(void)
{
	uint8 ack[] = {"+FTPSIZE: "};
	uint8 res=RES_WAIT;
	
	L218SendAtCmd(AT_FTPSIZE_INDEX,NULL,0,(uint8 *)ack,10);
	
	if(g_at_cmd_struct[AT_FTPSIZE_INDEX].exe_flag == EXE_OK)
		res = RES_TRUE;
	else
		res = RES_FALSE;	
		
	return res;
}

static uint8 FtpSETDLAFile(void)	//-简单跑流程,组织是有问题的???
{
	uint8 res=RES_FALSE;
	uint8 temp_cmd[70],temp_file[40];
//	uint8 TEMP_file_size[32] = "2,1024\x0d";	
  uint8  i = 0,cmp_data[20];// = {"+FTPGET:1,1"};
	
	
	cmp_data[i++] = '+';
	cmp_data[i++] = 'F';
	cmp_data[i++] = 'T';
	cmp_data[i++] = 'P';
	cmp_data[i++] = 'G';
	cmp_data[i++] = 'E';
	cmp_data[i++] = 'T';
	cmp_data[i++] = ':';
	cmp_data[i++] = '1';	//8
	cmp_data[i++] = ',';
	cmp_data[i++] = '1';
	cmp_data[i++] = 0;
	
	i = StrLen(g_proupgread_struct.file_path,0);
	while( i--){
		if(g_proupgread_struct.file_path[i] == '/'){
			break;
		}
	}
	i += 1;
	sprintf((char*)temp_cmd, (const char*)AT_FTPGETNAME, &g_proupgread_struct.file_path[i]);
	g_at_cmd_struct[AT_FTPGETNAME_INDEX].cmd_text = (uint8 *)temp_cmd;
	
	L218SendAtCmd(AT_FTPGETNAME_INDEX,NULL,0,(uint8 *)OK_ACK,2);
	if(g_at_cmd_struct[AT_FTPGETNAME_INDEX].exe_flag == EXE_OK)
		res = RES_TRUE;
	else
		return res;

	memcpy(temp_file, g_proupgread_struct.file_path, i);
	temp_file[i] = 0;
	sprintf((char*)temp_cmd, (const char*)AT_FTPGETPATH, temp_file);
	g_at_cmd_struct[AT_FTPGETPATH_INDEX].cmd_text = (uint8 *)temp_cmd;
	res=RES_FALSE;
	L218SendAtCmd(AT_FTPGETPATH_INDEX,NULL,0,(uint8 *)OK_ACK,2);
	
	if(g_at_cmd_struct[AT_FTPGETPATH_INDEX].exe_flag == EXE_OK)
		res = RES_TRUE;
	else
		return res;

	res = FtpFTPSIZE();
//	res=RES_FALSE;
//	L218SendAtCmd(AT_FTPSIZE_INDEX,NULL,0,(uint8 *)OK_ACK,2);
//	if(g_at_cmd_struct[AT_FTPSIZE_INDEX].exe_flag == EXE_OK)
//		res = RES_TRUE;
//	else
//		return res;

	res=RES_FALSE;
	L218SendAtCmd(AT_FTPGET1_INDEX,NULL,0, cmp_data, 11);
	
	if(g_at_cmd_struct[AT_FTPGET1_INDEX].exe_flag == EXE_OK)
		res = RES_TRUE;
	else
		return res;

	//-L218SendAtCmd(AT_FTPGET_INDEX,TEMP_file_size,7,(uint8 *)OK_ACK,2);
	//-
	//-if(g_at_cmd_struct[AT_FTPGET_INDEX].exe_flag == EXE_OK)
	//-	res = RES_TRUE;
	//-else
	//-	res = RES_FALSE;
	
		
	return res;
}




static uint8 FtpDLAFile(void)
{
	uint8 res=RES_WAIT;
	uint8 TEMP_file_size[10] = "2,1024\x0d";
  uint8  i = 0,TEMP_OK_ACK[20];// = {"+FTPGET:1,1"};
	
	TEMP_OK_ACK[i++] = '+';
	TEMP_OK_ACK[i++] = 'F';
	TEMP_OK_ACK[i++] = 'T';
	TEMP_OK_ACK[i++] = 'P';
	TEMP_OK_ACK[i++] = 'G';
	TEMP_OK_ACK[i++] = 'E';
	TEMP_OK_ACK[i++] = 'T';
	TEMP_OK_ACK[i++] = ':';
	TEMP_OK_ACK[i++] = '1';	//8
	TEMP_OK_ACK[i++] = ',';
	TEMP_OK_ACK[i++] = '1';
	TEMP_OK_ACK[i++] = 0;

	L218SendAtCmd(AT_FTPGET2_INDEX,TEMP_file_size,7,(uint8 *)TEMP_OK_ACK,11);
	
	if(g_at_cmd_struct[AT_FTPGET2_INDEX].exe_flag == EXE_OK)
		res = RES_TRUE;
	else if(g_at_cmd_struct[AT_FTPGET2_INDEX].exe_flag == EXE_FAIL)
		res = RES_FALSE;
		
	return res;
}

static uint8 FtpFTPQUIT(void)
{
	uint8 res=RES_WAIT;
	//-uint8 cmp_data[] = {"+FTPGET: 1,86"};
	
	L218SendAtCmd(AT_FTPQUIT_INDEX,NULL,0,(uint8 *)OK_ACK,2);
	
	if(g_at_cmd_struct[AT_FTPQUIT_INDEX].exe_flag == EXE_OK)
		res = RES_TRUE;
	else
		res = RES_FALSE;	
		
	return res;
}

static void FtpCBearer(void)
{
	uint8 temp_data[] = {"0,1\x0d"};
		
	//-1.Close bearer
	L218SendAtCmd(AT_SAPBR_INDEX,temp_data,StrLen(temp_data,0),(uint8 *)OK_ACK,2);	
}





///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//-接收到FTP升级命令之后进入升级流程
uint8 FtpMain(void)
{
	uint8 res = RES_WAIT;	
	
	if(ftp_struct.ftp_upgrade_flag == 0)
	{
		res = RES_NOFTPUP;
		goto RETURN_LAB;	
	}		
	
	//-FtpDelay();	//-内部有写flash
	
	switch(ftp_struct.ftp_txstep)
	{
		case e_ftpstart:		//升级前准备,进行必要的初始化
		{		
			//-ftp_txstep = e_efsdelfile;
			ftp_struct.ftp_txstep = e_CheckSIM;
			ftp_struct.ftp_upgrade_fail_flag=0;
			ftp_struct.ftp_delaywait_time 	= 0;
			ftp_struct.ftp_rx_len = 0;
			ftp_struct.ftp_rx_file_byte_counter = 0;
			ftp_struct.ftp_upgrade_success_flag = 0;
			boot_struct.program_total_size = 0;
		}
		break;
		
		case e_CheckSIM:
		{
			res = FtpCheckSIM();
			if(res == RES_TRUE)
				ftp_struct.ftp_txstep = e_CheckNET;
			else if(res == RES_FALSE) 	
				goto RETURN_LAB;	
		}
		break;
		
		case e_CheckNET:
		{
			res = FtpCheckNET();
			if(res == RES_TRUE)
				ftp_struct.ftp_txstep = e_CGATT;
			else if(res == RES_FALSE) 	
				goto RETURN_LAB;	
		}
		break;
		
		case e_CGATT:
		{
			res = FtpCGATT();
			if(res == RES_TRUE)
				ftp_struct.ftp_txstep = e_SAPBR;
			else if(res == RES_FALSE) 	
				goto RETURN_LAB;	
		}
		break;
		
		case e_SAPBR:
		{
			res = FtpSAPBR();
			if(res == RES_TRUE)
				ftp_struct.ftp_txstep = e_FTPTYPE;
			else if(res == RES_FALSE) 	
				goto RETURN_LAB;	
		}
		break;

		case e_FTPTYPE:
		{
			res = FtpFTPTYPE();
			if(res == RES_TRUE)
				ftp_struct.ftp_txstep = e_FTPSERV;
			else if(res == RES_FALSE) 	
				goto RETURN_LAB;	
		}
		break;

		case e_FTPSERV:
		{
			res = FtpFTPSERV();
			if(res == RES_TRUE)
				ftp_struct.ftp_txstep = e_FTPPORT;
			else if(res == RES_FALSE) 	
				goto RETURN_LAB;	
		}
		break;

		case e_FTPPORT:
		{
			res = FtpFTPPORT();
			if(res == RES_TRUE)
				ftp_struct.ftp_txstep = e_FTPUN;
			else if(res == RES_FALSE) 	
				goto RETURN_LAB;	
		}
		break;

		case e_FTPUN:
		{
			res = FtpFTPUN();
			if(res == RES_TRUE)
				ftp_struct.ftp_txstep = e_FTPPW;
			else if(res == RES_FALSE) 	
				goto RETURN_LAB;	
		}
		break;

		case e_FTPPW:
		{
			res = FtpFTPPW();
			if(res == RES_TRUE)
				ftp_struct.ftp_txstep = e_SETDLAFile;
			else if(res == RES_FALSE) 	
				goto RETURN_LAB;	
		}
		break;

		case e_SETDLAFile:
		{
			res = FtpSETDLAFile();
			if(res == RES_TRUE)
				ftp_struct.ftp_txstep = e_DLAFile;
			else if(res == RES_FALSE) 	
				goto RETURN_LAB;	
		}
		break;

		case e_FTPSIZE:	//-获取文件大小
		{
			res = FtpFTPSIZE();
			if(res == RES_TRUE)
				ftp_struct.ftp_txstep = e_DLAFile;
			else if(res == RES_FALSE) 	
				goto RETURN_LAB;	
		}
		break;

		case e_DLAFile:
		{
			res = FtpDLAFile();
			if(res == RES_TRUE)
				ftp_struct.ftp_txstep = e_FTPQUIT;
			else if(res == RES_WAIT)
				ftp_struct.ftp_txstep = e_DLAFile;
			else if(res == RES_FALSE) 	
				goto RETURN_LAB;	
		}
		break;

		case e_FTPQUIT:
		{
			res = FtpFTPQUIT();
			if(res == RES_TRUE)
			{
				ftp_struct.ftp_txstep = e_ftpend;
				ftp_struct.ftp_upgrade_success_flag = 1;
			}
			else if(res == RES_FALSE) 	
				goto RETURN_LAB;	
		}
		break;
		
		
		case e_ftpend:		//恢复初始状态
		{
			FtpCBearer();
			ftp_struct.ftp_txstep = e_ftpstart;
			ftp_struct.ftp_upgrade_flag = 0;
		}
		break;
		
		default:	break;		
	}
	
RETURN_LAB:	
	if(res == RES_FALSE) 
	{
		ftp_struct.ftp_txstep = e_ftpend;
		ftp_struct.ftp_upgrade_fail_flag=1;		
		#ifdef FTP_DEBUG
			LocalUartFixedLenSend((uint8*)"!!!warn ftp_upgrade_fail\r\n",StrLen((uint8*)"!!!warn ftp_upgrade_fail\r\n",0));
		#endif
	}
		
	return res;
}

