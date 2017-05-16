#ifndef _SMS_H 
#define _SMS_H

		#include "stm32f10x.h"
		
		#ifdef SMS_GLOBAL
			#define SMS_EXTERN
		#else
			#define SMS_EXTERN extern
		#endif

		#define GSM_7BIT 0x00
		#define GSM_UCS2 0x08
		#define GSM_8BIT 0x04
		
		typedef struct 
		{
			uint8 sms_center_num[LEN_32];
			uint8 sms_tx_num[LEN_32];///·¢ËÍ·½¶ÌÐÅºÅ
			uint8 pid;
			uint8 dcs;	
			uint8 change_sms_center_num_flag;
		}SMS_STRUCT;
		
		SMS_EXTERN SMS_STRUCT sms_struct;
		
		uint8 SmsParaLen(uint8 p_in[],uint8 max_count);
		void SmsAckPacket(uint8 *p_in,uint8 *p_out,uint8 ack_flag);
		uint8 SmsParaReInit(uint8 p_in[],uint8 p_out[]);
		void SmsParaQuery(uint8 p_out[]);
		void SmsParaSet(uint8 p_in[],uint8 p_out[]);
		uint8 SmsProgramUpgrade(uint8 p_in[],uint8 p_out[]);
		void SmsTx(uint8 p_in[]);
		void SmsTxMsg(uint8 msg[]);
		
		void SmsNumDecode(uint8 rev_num[],uint8 num_len,uint8 normal_num[]);
		uint8 SmsNumEncode(uint8 normal_num[],uint8 num_len,uint8 rev_num[]);
		uint8 SmsDecode7bit(uint8* p_src,uint8 src_len,uint8* p_dst);
		uint8 SmsEncode7bit(uint8* p_src,uint16 src_len,uint8* p_dst);
		uint8 SmsDecodeUcs2(uint8* p_src,uint8 src_len,uint8* p_dst);
		uint8 SmsEncodeUcs2(uint8 *p_src,uint8 src_len,uint8 *p_dst);
		uint8 SmsDecode8bit(uint8 *p_src,uint8 src_len,uint8 *p_dst);
		uint8 SmsEncode8bit(uint8 *p_src,uint16 src_len,uint8 *p_dst);
		
		uint8 SmsTextDecode(uint8 *p_src,uint16 len,uint8 *p_dst,uint8 dcs_flag);
		uint8 SmsTextEncode(uint8 *p_src,uint8 len,uint8 *p_dst,uint8 dcs_flag);
		uint8 SmsDecodePdu(uint8 *p_src,uint8 *p_dst);
		uint8 SmsEncodePdu(uint8 *p_src,uint8 len,uint8 *p_dst,uint16 *dst_len);
		void SmsProcessFun(uint8 data[],uint16 len);
		
#endif
