#define CAN_GLOBAL
#define CAN_DEBUG
#include "tiza_include.h"



/******************************************************
//读写VIN码
******************************************************/
static void ProWrite_VIN(uint8* str)
{
	uint8 data[20];
	//头2A + VIN + 校验
	// 初始化VIN
	memcpy(&data[1], str, 17);
	data[0]  = 0x2A;
	data[18] = U8SumCheck(data, 18);
	FlashErase(SYS_SAVEVIN_INDEXPAGE);
	FlashWrite(SYS_SAVEVIN_INDEXPAGE, data, 19);		
	
	memcpy(g_VIN, &data[1], 17);
}
void ProRead_VIN(void)
{
	uint8 data[20];
	//头2A + VIN + 校验
	FlashRead(SYS_SAVEVIN_INDEXPAGE, data, 19);

	if(data[0]==0x2A && data[18]==U8SumCheck(data, 18)){
		memcpy(g_VIN, &data[1], 17);
	}
	else {// 初始化VIN		
		memcpy(&data[1], "01234567890123456", 17);
		ProWrite_VIN(&data[1]);	
	}
}

/******************************************************
//CAN 初始化一系列函数
******************************************************/
//14个32位滤波器  
void CanFilterInit(uint8 filter_index,uint32 can_id_1,uint32 can_id_2)
{
	u32 tmp_u32_val;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	
	CAN_FilterInitStructure.CAN_FilterNumber = filter_index;		///指定过滤器0 ，该值范围是 0 - 13
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	///过滤器位宽 16 or 32; 16 bit 时 每组由2个16位过滤器组成，32时则只有一个过滤器

	tmp_u32_val = can_id_1;
	tmp_u32_val = tmp_u32_val << 3;
	tmp_u32_val |= CAN_Id_Extended;
	CAN_FilterInitStructure.CAN_FilterIdHigh = tmp_u32_val >> 16;			
	CAN_FilterInitStructure.CAN_FilterIdLow = tmp_u32_val;
	
	tmp_u32_val = can_id_2;
	tmp_u32_val = tmp_u32_val << 3;
	tmp_u32_val |= CAN_Id_Extended;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = tmp_u32_val >> 16;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = tmp_u32_val;

	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;		///过滤器关联到FIFO
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
}

 void CanDeInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);
	CAN_DeInit(CAN1);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void CanInit(void)
{
	GPIO_InitTypeDef 	   GPIO_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	///CanTxMsg TxMessage;
	RCC_ClocksTypeDef RCC_ClocksStatus;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	///Configure CAN pin: TX 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	RCC_GetClocksFreq(&RCC_ClocksStatus);
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	/// CAN cell init
	CAN_InitStructure.CAN_TTCM=DISABLE;
	CAN_InitStructure.CAN_ABOM=ENABLE;
	CAN_InitStructure.CAN_AWUM=DISABLE;
	CAN_InitStructure.CAN_NART=DISABLE;
	CAN_InitStructure.CAN_RFLM=DISABLE;
	CAN_InitStructure.CAN_TXFP=DISABLE;
	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1=CAN_BS1_13tq;
	CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;

	CAN_InitStructure.CAN_Prescaler=RCC_ClocksStatus.PCLK1_Frequency/(16*250000);      ///波特率计算，16M/（1+8+7）/4=250k
	if(!CAN_Init(CAN1, &CAN_InitStructure)){
		///CAN初始化失败
		return;
	}
	//0~13
	CanFilterInit(0,CAN_R_03,CAN_R_04);
	CanFilterInit(1,CAN_R_05,CAN_R_06);
	CanFilterInit(2,CAN_R_07,CAN_R_08);
	CanFilterInit(3,CAN_R_09,CAN_R_10);
	CanFilterInit(4,CAN_R_11,CAN_R_12);
	CanFilterInit(5,CAN_R_13,CAN_R_14);
	CanFilterInit(6,CAN_R_15,CAN_R_16);
	CanFilterInit(7,CAN_R_17,CAN_R_18);
	CanFilterInit(8,CAN_R_19,CAN_R_20);
	CanFilterInit(9,CAN_R_21,CAN_R_22);
	CanFilterInit(10,CAN_R_23,CAN_R_24);
	CanFilterInit(11,CAN_R_25,CAN_R_26);
	CanFilterInit(12,CAN_R_0101,CAN_R_0102);
	CanFilterInit(13,CAN_R_0201,CAN_R_0202);//CAN_R_0230
	 
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1 ;//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			 = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}
void CanDisableReceive(void)
{
//NVIC_InitTypeDef NVIC_InitStructure;	
//g_sysm_on_off_struct.canrx_switch = SYSM_OFF;
////			CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);
//NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1 ;//抢占优先级1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			 = 2;
//NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
//NVIC_Init(&NVIC_InitStructure);
}
void CanEnableReceive(void)
{
//	NVIC_InitTypeDef NVIC_InitStructure;	
//	g_sysm_on_off_struct.canrx_switch = SYSM_ON;
//	//			CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
//	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1 ;//抢占优先级1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			 = 2;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
}

void CanTxData(uint8 data[],uint32 id)
{
	CanTxMsg TxMessage;
	TxMessage.ExtId = id;
	TxMessage.IDE = CAN_ID_EXT;   
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	MemCpy(TxMessage.Data,data,8);
	CAN_Transmit(CAN1, &TxMessage);
}

/******************************************************
//应用层面CAN发送函数
******************************************************/
void CanTx(void)
{
	uint8 heart_data[CAN_LOCK_DATA_LEN] = {0xFD,0x20,0x00,0x00,0x00,0x00,0x00,0x00};
	
	CanTxData(heart_data,0X12345678);
}
/******************************************************
//应用层面CAN接收函数
******************************************************/
void CanRx(void)
{
	uint8 index = 0xff;
	CanRxMsg can_msg;
	uint32 tmp_u32;
	
//	CAN_FIFORelease(CAN1,CAN_FIFO0); //清中断标志
	CAN_Receive(CAN1,CAN_FIFO0,&can_msg);
	
	
//	printf("\r\n can id is %02X \r\n", can_msg.ExtId);
	if(0x08 == can_msg.DLC){
		tmp_u32 = can_msg.ExtId;
		if((tmp_u32&CAN_R1_VT) == CAN_R1_VT){													//是单体电压温度CANID
			if(tmp_u32>=CAN_R_0101 && tmp_u32<=CAN_R_0160){							//单体电压
				index = ((tmp_u32>>16)&0xFF) - 1;//[0~23]
			}
			else if(tmp_u32>=CAN_R_0201 && tmp_u32<=CAN_R_0230){	//单体温度
				index = ((tmp_u32>>16)&0xFF) - ((CAN_R_0201>>16)&0xFF) + ((CAN_R_0160>>16)&0xFF) ;//[24~26]
			}
		}
		else{		
			switch(can_msg.ExtId){
				case CAN_R_03:{//BMS单体电压极值数据帧
					index = ECAN_R_03;
					break;
				}
				case CAN_R_04:{//BMS温度模块极值数据帧
					index = ECAN_R_04;
					break;
				}
				case CAN_R_05:{//BMS报警1
					index = ECAN_R_05;
					break;
				}
				case CAN_R_06:{//BMS报警2
					index = ECAN_R_06;
					break;
				}
				case CAN_R_07:{//BMS系统信息1
					index = ECAN_R_07;
					break;
				}
				case CAN_R_08:{//BMS系统信息2
					index = ECAN_R_08;
					break;
				}
				case CAN_R_09:{//BMS系统信息3
					index = ECAN_R_09;
					break;
				}
				case CAN_R_10:{//VCU报文1
					index = ECAN_R_10;
					break;
				}
				case CAN_R_11:{//VCU报文2
					index = ECAN_R_11;
					break;
				}
				case CAN_R_12:{//VCU报文3
					index = ECAN_R_12;
					break;
				}
				case CAN_R_13:{//VCU报文4
					index = ECAN_R_13;
					break;
				}
				case CAN_R_14:{//VCU报文5
					index = ECAN_R_14;
					break;
				}
				case CAN_R_15:{//VCU报文6
					index = ECAN_R_15;
					break;
				}

				case CAN_R_26:{//VIN码 3 //131415//242526;
				if(can_msg.Data[0] == 1)
					index = ECAN_R_2601;
				else if(can_msg.Data[0] == 2)
					index = ECAN_R_2602;
				else if(can_msg.Data[0] == 3)
					index = ECAN_R_2603;
				
				break;
				}
				
				default:				break;
			}
		}
	}
	can_struct.rx_can_buf_flag[index] = index; //有数据
	
	if(can_struct.rx_data_flag==0xFF)
		can_struct.rx_data_flag = index;
	
	if(index == 0xFF){
		goto RETURN_LAB;
	}

//	can_struct.rx_can_buf[index].Item.id = can_msg.ExtId;
	MemCpy(&can_struct.rx_can_buf[index].Item.data[0].byte,can_msg.Data,8);

RETURN_LAB:
	return;
}



/******************************************************
//CANID可变滤波器程序（当一个CANID收到数据后通过该程序改变滤波器）
//单体电压占用滤波器12
//单体温度占用滤波器13
******************************************************/
static void CANR01xx_Filter(uint8 eindex)
{
//	uint8 eindexmax = ECAN_R_0160 - ECAN_R_0101 + 1;
	union32 tmpval,tmpu32;
	
//	tmpval.dword = CAN_R_0101;
//	tmpval.byte.HL += ((eindex+2) % eindexmax);
	if(eindex & 0x01){//奇数(ID偶数)  更改第三变量对应CANID，保留第二变量对应CANID		
		tmpval.dword = CAN1->sFilterRegister[12].FR2;
		tmpval.dword = (tmpval.dword >> 3);
		tmpval.byte.HL += 2;
		if(tmpval.byte.HL > CAN01MAX){
			tmpval.byte.HL = CAN01MINEVEN;
		}
		tmpu32.dword = CAN1->sFilterRegister[12].FR1 ;
		tmpu32.dword = (tmpu32.dword >> 3);
		CanFilterInit(12,tmpu32.dword,tmpval.dword);
	}
	else{							//偶数(ID奇数)  更改第二变量对应CANID，保留第三变量对应CANID
		tmpval.dword = CAN1->sFilterRegister[12].FR1;
		tmpval.dword = (tmpval.dword >> 3);
		tmpval.byte.HL += 2;
		if(tmpval.byte.HL > CAN01MAX){
			tmpval.byte.HL = CAN01MINODD;
		}
		tmpu32.dword = CAN1->sFilterRegister[12].FR2;
		tmpu32.dword = (tmpu32.dword >> 3);
		CanFilterInit(12,tmpval.dword,tmpu32.dword);
	}
	
}
static void CANR02xx_Filter(uint8 eindex)
{
//	uint8 eindexmax = ECAN_R_0230 - ECAN_R_0201 + 1;
	union32 tmpval,tmpu32;
	
//	tmpval.dword = CAN_R_0201;
//	tmpval.byte.HL += ((eindex+2) % eindexmax);
	if(eindex & 0x01){//奇数(ID偶数)  更改第二变量对应CANID，保留第三变量对应CANID
		tmpval.dword = CAN1->sFilterRegister[13].FR2;
		tmpval.dword = (tmpval.dword >> 3);
		tmpval.byte.HL += 2;
		if(tmpval.byte.HL > CAN02MAX){
			tmpval.byte.HL = CAN02MINEVEN;
		}
		tmpu32.dword = CAN1->sFilterRegister[13].FR1;
		tmpu32.dword = (tmpu32.dword >> 3);
		CanFilterInit(13,tmpu32.dword,tmpval.dword);
	}
	else{							//偶数(ID奇数)  更改第三变量对应CANID，保留第二变量对应CANID	
		tmpval.dword = CAN1->sFilterRegister[13].FR1;
		tmpval.dword = (tmpval.dword >> 3);
		tmpval.byte.HL += 2;
		if(tmpval.byte.HL > CAN02MAX){
			tmpval.byte.HL = CAN02MINEVEN;
		}
		if(tmpval.byte.HL > CAN02MAX){
			tmpval.byte.HL = CAN02MINODD;
		}
		tmpu32.dword = CAN1->sFilterRegister[13].FR2;
		tmpu32.dword = (tmpu32.dword >> 3);
		CanFilterInit(13,tmpval.dword,tmpu32.dword);
	}
	
}

/******************************************************
//CANID定时可变滤波器程序（当一个CANID长时间收不到数据就通过该程序改变滤波器）
//单体电压占用滤波器12
//单体温度占用滤波器13
******************************************************/
static void CANR_PeriodChargeFilter(void)
{
	static uint32 CAN12filter1,CAN12filter2,CAN13filter1,CAN13filter2;
	static uint8 CAN12count1,CAN12count2,CAN13count1,CAN13count2;
	union32 tmpu321,tmpu322;
	
	if(CAN12count1 > 10){	//超时更改滤波器
		CAN12count1 = 0;
		tmpu321.dword = CAN1->sFilterRegister[12].FR1 ;
		tmpu321.dword = (tmpu321.dword >> 3);		
		tmpu321.byte.HL += 2;
		if(tmpu321.byte.HL > CAN01MAX){
			tmpu321.byte.HL = CAN01MINODD;
		}
		tmpu322.dword = CAN1->sFilterRegister[12].FR2 ;
		tmpu322.dword = (tmpu322.dword >> 3);
		CanFilterInit(12,tmpu321.dword,tmpu322.dword);
	}
	else{									//更改滤波器计时
		if(CAN12filter1 == CAN1->sFilterRegister[12].FR1){
			CAN12count1++;			
		}
		else{
			CAN12count1 = 0;
			CAN12filter1 = CAN1->sFilterRegister[12].FR1 ;
		}
	}
	
	if(CAN12count2 > 10){	//超时更改滤波器
		CAN12count2 = 0;
		tmpu321.dword = CAN1->sFilterRegister[12].FR1 ;
		tmpu321.dword = (tmpu321.dword >> 3);		
		tmpu322.dword = CAN1->sFilterRegister[12].FR2 ;
		tmpu322.dword = (tmpu322.dword >> 3);
		tmpu322.byte.HL += 2;
		if(tmpu322.byte.HL > CAN01MAX){
			tmpu322.byte.HL = CAN01MINEVEN;
		}
		CanFilterInit(12,tmpu321.dword,tmpu322.dword);
	}
	else{									//更改滤波器计时
		if(CAN12filter2 == CAN1->sFilterRegister[12].FR2){
			CAN12count2++;			
		}
		else{
			CAN12count2 = 0;
			CAN12filter2 = CAN1->sFilterRegister[12].FR2 ;
		}
	}
	
	
	if(CAN13count1 > 10){	//超时更改滤波器
		CAN13count1 = 0;
		tmpu321.dword = CAN1->sFilterRegister[13].FR1 ;
		tmpu321.dword = (tmpu321.dword >> 3);		
		tmpu321.byte.HL +=2;
		if(tmpu321.byte.HL > CAN02MAX){
			tmpu321.byte.HL = CAN02MINODD;
		}
		tmpu322.dword = CAN1->sFilterRegister[13].FR2 ;
		tmpu322.dword = (tmpu322.dword >> 3);
		CanFilterInit(13,tmpu321.dword,tmpu322.dword);
	}
	else{									//更改滤波器计时
		if(CAN13filter1 == CAN1->sFilterRegister[13].FR1){
			CAN13count1++;			
		}
		else{
			CAN13count1 = 0;
			CAN13filter1 = CAN1->sFilterRegister[13].FR1 ;
		}
	}
	
	if(CAN13count2 > 10){	//超时更改滤波器
		CAN13count2 = 0;
		tmpu321.dword = CAN1->sFilterRegister[13].FR1 ;
		tmpu321.dword = (tmpu321.dword >> 3);		
		tmpu322.dword = CAN1->sFilterRegister[13].FR2 ;
		tmpu322.dword = (tmpu322.dword >> 3);
		tmpu322.byte.HL += 2;
		if(tmpu321.byte.HL > CAN02MAX){
			tmpu321.byte.HL = CAN02MINEVEN;
		}
		CanFilterInit(13,tmpu321.dword,tmpu322.dword);
	}
	else{									//更改滤波器计时
		if(CAN13filter2 == CAN1->sFilterRegister[12].FR2){
			CAN13count2++;			
		}
		else{
			CAN13count2 = 0;
			CAN13filter2 = CAN1->sFilterRegister[12].FR2 ;
		}
	}
}

/******************************************************
//CAN接收数据解析函数
//注意：有些项在两个CAN报文中有体现 取哪个 待定？？ qlj
******************************************************/
static void CanRx_Analyse(void)
{
//	uint32 CAN_R_01xx[] = {};
	uint8  data[20];
	uint8  index;
	union32 tmpval;
	
	for(index = ECAN_R_0101; index <= ECAN_R_27; index++){
		
		if(can_struct.rx_can_buf_flag[index] != 0xFF){
			index = can_struct.rx_can_buf_flag[index];
			can_struct.rx_can_buf_flag[index] = 0xFF;
			if(index>=ECAN_R_0101 && index<=ECAN_R_0160){//单体电压
				tmpval.byte.LL = can_struct.rx_can_buf[index].Item.data[0].byte;
				tmpval.byte.LH = can_struct.rx_can_buf[index].Item.data[1].byte;
				g_sigbatt_val[0][index*4] 	= tmpval.word.L;
				tmpval.byte.LL = can_struct.rx_can_buf[index].Item.data[2].byte;
				tmpval.byte.LH = can_struct.rx_can_buf[index].Item.data[3].byte;
				g_sigbatt_val[0][index*4+1] = tmpval.word.L;
				tmpval.byte.LL = can_struct.rx_can_buf[index].Item.data[4].byte;
				tmpval.byte.LH = can_struct.rx_can_buf[index].Item.data[5].byte;
				g_sigbatt_val[0][index*4+2] = tmpval.word.L;
				tmpval.byte.LL = can_struct.rx_can_buf[index].Item.data[6].byte;
				tmpval.byte.LH = can_struct.rx_can_buf[index].Item.data[7].byte;
				g_sigbatt_val[0][index*4+3] = tmpval.word.L;
				
				CANR01xx_Filter(index);//变更滤波ID index = 0~59
			}
			else if(index>=ECAN_R_0201 && index<=ECAN_R_0230){//单体温度		
				memcpy(&g_btprobe_val[0][(index-ECAN_R_0201)*8],can_struct.rx_can_buf[index].Item.data,8);
				CANR02xx_Filter(index-ECAN_R_0201);//变更滤波ID index-ECAN_R_0201 = 0~29
			}
			else if(index>=ECAN_R_2601 && index<=ECAN_R_2603){//VIN
				if(can_struct.rx_can_buf[ECAN_R_2601].Item.data[0].byte == 0x01 && \
					 can_struct.rx_can_buf[ECAN_R_2602].Item.data[0].byte == 0x02 && \
					 can_struct.rx_can_buf[ECAN_R_2603].Item.data[0].byte == 0x03	){
						 
					can_struct.rx_can_buf[ECAN_R_2601].Item.data[0].byte = 0xF1 ;
					can_struct.rx_can_buf[ECAN_R_2602].Item.data[0].byte = 0xF2 ;
					can_struct.rx_can_buf[ECAN_R_2603].Item.data[0].byte = 0xF3 ;
					memcpy(data,      &can_struct.rx_can_buf[ECAN_R_2601].Item.data[1].byte, 7);	
					memcpy(&data[7],  &can_struct.rx_can_buf[ECAN_R_2602].Item.data[1].byte, 7);	
					memcpy(&data[14], &can_struct.rx_can_buf[ECAN_R_2603].Item.data[1].byte, 3);	
					ProWrite_VIN(data);
				}
			}
			else{		
				switch(index){
					case ECAN_R_03:{//BMS单体电压极值数据帧
						g_proextreme_union.Item.maxvolsys_id = can_struct.rx_can_buf[ECAN_R_03].Item.data[0].byte;
						g_proextreme_union.Item.maxvolbat_id = can_struct.rx_can_buf[ECAN_R_03].Item.data[1].byte;
						tmpval.byte.LL = can_struct.rx_can_buf[ECAN_R_03].Item.data[2].byte;
						tmpval.byte.LH = can_struct.rx_can_buf[ECAN_R_03].Item.data[3].byte;
						g_proextreme_union.Item.maxvolbat_val = tmpval.word.L;
						g_proextreme_union.Item.minvolsys_id 	= can_struct.rx_can_buf[ECAN_R_03].Item.data[4].byte;
						g_proextreme_union.Item.minvolbat_id 	= can_struct.rx_can_buf[ECAN_R_03].Item.data[5].byte;
						tmpval.byte.LL = can_struct.rx_can_buf[ECAN_R_03].Item.data[6].byte;
						tmpval.byte.LH = can_struct.rx_can_buf[ECAN_R_03].Item.data[7].byte;
						g_proextreme_union.Item.minvolbat_val = tmpval.word.L;
						break;
					}
					case ECAN_R_04:{//BMS温度模块极值数据帧				
						g_proextreme_union.Item.maxtemsys_id 	= can_struct.rx_can_buf[ECAN_R_04].Item.data[0].byte;
						g_proextreme_union.Item.maxtempro_id 	= can_struct.rx_can_buf[ECAN_R_04].Item.data[1].byte;
						g_proextreme_union.Item.maxtempro_val = can_struct.rx_can_buf[ECAN_R_04].Item.data[2].byte;
						g_proextreme_union.Item.mintemsys_id 	= can_struct.rx_can_buf[ECAN_R_04].Item.data[3].byte;
						g_proextreme_union.Item.mintempro_id 	= can_struct.rx_can_buf[ECAN_R_04].Item.data[4].byte;
						g_proextreme_union.Item.mintempro_val = can_struct.rx_can_buf[ECAN_R_04].Item.data[5].byte;
						tmpval.byte.LL = can_struct.rx_can_buf[ECAN_R_04].Item.data[6].byte;	//绝缘电阻
						tmpval.byte.LH = can_struct.rx_can_buf[ECAN_R_04].Item.data[7].byte;
						g_provehice_union.Item.resistance = tmpval.word.L;		
						break;
					}
					case ECAN_R_05:{//BMS报警1
						g_proalarm_union.Item.rank 				= can_struct.rx_can_buf[ECAN_R_05].Item.data[0].byte;
						g_proalarm_union.Item.storagenum 	= can_struct.rx_can_buf[ECAN_R_05].Item.data[1].byte;
						if(g_proalarm_union.Item.rank == 0){
							g_proalarm_union.Item.flag.dword 	= 0;
							g_proalarm_union.Item.motornum 		= 0;
							g_proalarm_union.Item.enginenum 	= 0;
							g_proalarm_union.Item.othersnum 	= 0;
						}
						else if(g_proalarm_union.Item.rank == 1){
							g_storage_alr[0] = can_struct.rx_can_buf[ECAN_R_05].Item.data[6].byte;	//
							g_storage_alr[1] = can_struct.rx_can_buf[ECAN_R_05].Item.data[7].byte;
							g_storage_alr[2] = 0;
							g_storage_alr[3] = 0;
						}
						else if(g_proalarm_union.Item.rank == 2){
							g_storage_alr[0] = can_struct.rx_can_buf[ECAN_R_05].Item.data[4].byte;	//
							g_storage_alr[1] = can_struct.rx_can_buf[ECAN_R_05].Item.data[5].byte;
							g_storage_alr[2] = 0;
							g_storage_alr[3] = 0;
						}
						else if(g_proalarm_union.Item.rank == 3){
							g_storage_alr[0] = can_struct.rx_can_buf[ECAN_R_05].Item.data[2].byte;	//
							g_storage_alr[1] = can_struct.rx_can_buf[ECAN_R_05].Item.data[3].byte;
							g_storage_alr[2] = 0;
							g_storage_alr[3] = 0;		
						}	
						
						break;
					}
					case ECAN_R_06:{//BMS报警2
						/* tmpval.dword = 0;
						tmpval.byte.LL = can_struct.rx_can_buf[ECAN_R_06].Item.data[0].byte;	//通用报警标志
						tmpval.byte.LH = can_struct.rx_can_buf[ECAN_R_06].Item.data[1].byte;
						g_proalarm_union.Item.flag.dword 	= tmpval.dword;			
						*/
						g_provehice_union.Item.charge = can_struct.rx_can_buf[ECAN_R_06].Item.data[2].byte;
					/*	tmpval.byte.LL = can_struct.rx_can_buf[ECAN_R_06].Item.data[3].byte;	//单次充电量
						tmpval.byte.LH = can_struct.rx_can_buf[ECAN_R_06].Item.data[4].byte;
						
						tmpval.dword = 0;
						tmpval.byte.LL = can_struct.rx_can_buf[ECAN_R_06].Item.data[5].byte;	//累计充电量
						tmpval.byte.LH = can_struct.rx_can_buf[ECAN_R_06].Item.data[6].byte;					
						tmpval.byte.HL = can_struct.rx_can_buf[ECAN_R_06].Item.data[7].byte;	
				 */
						break;
					}
					case ECAN_R_07:{//BMS系统信息1										
						g_prologin_union.Item.storage_num = can_struct.rx_can_buf[ECAN_R_07].Item.data[0].byte;//可充电储能子系统个数
						g_provbattsys_union.Item.serial = can_struct.rx_can_buf[ECAN_R_07].Item.data[1].byte;//可充电储能子系统号 
						tmpval.byte.LL = can_struct.rx_can_buf[ECAN_R_07].Item.data[2].byte;	//
						tmpval.byte.LH = can_struct.rx_can_buf[ECAN_R_07].Item.data[3].byte;
						g_provbattsys_union.Item.voltage  = tmpval.word.L;		
						tmpval.byte.LL = can_struct.rx_can_buf[ECAN_R_07].Item.data[4].byte;	//
						tmpval.byte.LH = can_struct.rx_can_buf[ECAN_R_07].Item.data[5].byte;
						g_provbattsys_union.Item.current  = tmpval.word.L;		
						
						g_prologin_union.Item.storage_len = can_struct.rx_can_buf[ECAN_R_07].Item.data[6].byte;
						break;
					}
					case ECAN_R_08:{//BMS系统信息2
						g_protbattsys_union.Item.serial = can_struct.rx_can_buf[ECAN_R_08].Item.data[0].byte;//可充电储能子系统号 
						tmpval.byte.LL = can_struct.rx_can_buf[ECAN_R_08].Item.data[1].byte;	//
						tmpval.byte.LH = can_struct.rx_can_buf[ECAN_R_08].Item.data[2].byte;
						g_protbattsys_union.Item.btprobe_num = tmpval.word.L;		 
						tmpval.byte.LL = can_struct.rx_can_buf[ECAN_R_08].Item.data[3].byte;	//
						tmpval.byte.LH = can_struct.rx_can_buf[ECAN_R_08].Item.data[4].byte;
						g_provbattsys_union.Item.sigbatt_num = tmpval.word.L;		
						break;
					}
					case ECAN_R_09:{//BMS系统信息3
						g_provehice_union.Item.SOC = can_struct.rx_can_buf[ECAN_R_09].Item.data[2].byte;//
						tmpval.byte.LL = can_struct.rx_can_buf[ECAN_R_09].Item.data[3].byte;	//
						tmpval.byte.LH = can_struct.rx_can_buf[ECAN_R_09].Item.data[4].byte;
						g_provehice_union.Item.current = tmpval.word.L;		 
						tmpval.byte.LL = can_struct.rx_can_buf[ECAN_R_09].Item.data[5].byte;	//
						tmpval.byte.LH = can_struct.rx_can_buf[ECAN_R_09].Item.data[6].byte;
						g_provehice_union.Item.voltage = tmpval.word.L;		
					
						break;
					}
					case ECAN_R_10:{//VCU报文1
						tmpval.byte.LL = can_struct.rx_can_buf[ECAN_R_10].Item.data[0].byte & 0x0F;//  车辆状态
						if(tmpval.byte.LL>=0x01 && tmpval.byte.LL<=0x03){
							g_provehice_union.Item.vehicle = tmpval.byte.LL;
						}
						else if(tmpval.byte.LL==0x0E || tmpval.byte.LL==0x0F){
							g_provehice_union.Item.vehicle = tmpval.byte.LL | 0XF0;
						}
						tmpval.byte.LL = (can_struct.rx_can_buf[ECAN_R_10].Item.data[0].byte>>4) & 0x0F;//  运行状态
						if(tmpval.byte.LL>=0x01 && tmpval.byte.LL<=0x03){
							g_provehice_union.Item.mode = tmpval.byte.LL;
						}
						else if(tmpval.byte.LL==0x0E || tmpval.byte.LL==0x0F){
							g_provehice_union.Item.mode = tmpval.byte.LL | 0XF0;
						}
						tmpval.byte.LL = can_struct.rx_can_buf[ECAN_R_10].Item.data[1].byte;	//
						tmpval.byte.LH = can_struct.rx_can_buf[ECAN_R_10].Item.data[2].byte;
						g_provehice_union.Item.speed = tmpval.word.L;		 
						tmpval.byte.LL = can_struct.rx_can_buf[ECAN_R_10].Item.data[3].byte;	//
						tmpval.byte.LH = can_struct.rx_can_buf[ECAN_R_10].Item.data[4].byte;
						tmpval.byte.HL = can_struct.rx_can_buf[ECAN_R_10].Item.data[5].byte;	//
						tmpval.byte.HH = can_struct.rx_can_buf[ECAN_R_10].Item.data[6].byte;
						g_provehice_union.Item.mileage = tmpval.dword;		
						
						break;
					}
					case ECAN_R_11:{//VCU报文2				
						g_provehice_union.Item.DC_statu 	= can_struct.rx_can_buf[ECAN_R_11].Item.data[4].byte;//
						g_provehice_union.Item.stall.byte = can_struct.rx_can_buf[ECAN_R_11].Item.data[5].byte;//
						break;
					}
					case ECAN_R_12:{//VCU报文3
						g_promotor_union[0].Item.serial 	= can_struct.rx_can_buf[ECAN_R_12].Item.data[0].byte;//
						g_promotor_union[0].Item.status 	= can_struct.rx_can_buf[ECAN_R_12].Item.data[1].byte;//
						g_promotor_union[0].Item.ctr_tem 	= can_struct.rx_can_buf[ECAN_R_12].Item.data[2].byte;//
						tmpval.byte.LL = can_struct.rx_can_buf[ECAN_R_12].Item.data[3].byte;	//
						tmpval.byte.LH = can_struct.rx_can_buf[ECAN_R_12].Item.data[4].byte;
						g_promotor_union[0].Item.rpm = tmpval.word.L;		 
						tmpval.byte.LL = can_struct.rx_can_buf[ECAN_R_12].Item.data[5].byte;	//
						tmpval.byte.LH = can_struct.rx_can_buf[ECAN_R_12].Item.data[6].byte;
						g_promotor_union[0].Item.torque = tmpval.word.L;		 
						g_promotor_union[0].Item.motor_tem 	= can_struct.rx_can_buf[ECAN_R_12].Item.data[7].byte;//
					
						break;
					}
					case ECAN_R_13:{//VCU报文4
						g_promotor_union[0].Item.serial 	= can_struct.rx_can_buf[ECAN_R_13].Item.data[0].byte;//
						tmpval.byte.LL = can_struct.rx_can_buf[ECAN_R_13].Item.data[1].byte;	//
						tmpval.byte.LH = can_struct.rx_can_buf[ECAN_R_13].Item.data[2].byte;
						g_promotor_union[0].Item.ctr_in_vol = tmpval.word.L;		 
						tmpval.byte.LL = can_struct.rx_can_buf[ECAN_R_13].Item.data[3].byte;	//
						tmpval.byte.LH = can_struct.rx_can_buf[ECAN_R_13].Item.data[4].byte;
						g_promotor_union[0].Item.ctr_bus_cur = tmpval.word.L;		 
						g_provehice_union.Item.Accelerate 	= can_struct.rx_can_buf[ECAN_R_13].Item.data[5].byte;//
						g_provehice_union.Item.braking 	= can_struct.rx_can_buf[ECAN_R_13].Item.data[6].byte;//
						break;
					}
					case ECAN_R_14:{//VCU报文5
						g_proalarm_union.Item.rank 	= can_struct.rx_can_buf[ECAN_R_14].Item.data[0].byte;//
						tmpval.byte.LL = can_struct.rx_can_buf[ECAN_R_14].Item.data[1].byte;	//通用报警标志
						tmpval.byte.LH = can_struct.rx_can_buf[ECAN_R_14].Item.data[2].byte;
						tmpval.byte.HL = can_struct.rx_can_buf[ECAN_R_14].Item.data[3].byte;	
						tmpval.byte.HH = can_struct.rx_can_buf[ECAN_R_14].Item.data[4].byte;
						g_proalarm_union.Item.flag.dword 	= tmpval.dword;		
						g_proalarm_union.Item.storagenum 	= can_struct.rx_can_buf[ECAN_R_14].Item.data[5].byte;
						g_proalarm_union.Item.enginenum 	= can_struct.rx_can_buf[ECAN_R_14].Item.data[6].byte;
						g_proalarm_union.Item.othersnum 	= can_struct.rx_can_buf[ECAN_R_14].Item.data[7].byte;					
						
						if(g_proalarm_union.Item.rank == 0x03){
							AlarmHaveJudge(1);
						}
						else{
							AlarmHaveJudge(0);
						}
						break;
					}
					case ECAN_R_15:{//VCU报文6
						g_motro_alr[0] = can_struct.rx_can_buf[ECAN_R_15].Item.data[0].byte;//
						g_motro_alr[1] = can_struct.rx_can_buf[ECAN_R_15].Item.data[1].byte;//
						g_motro_alr[2] = can_struct.rx_can_buf[ECAN_R_15].Item.data[2].byte;//
						g_motro_alr[3] = can_struct.rx_can_buf[ECAN_R_15].Item.data[3].byte;//
						
						g_others_alr[0] = can_struct.rx_can_buf[ECAN_R_15].Item.data[4].byte;//
						g_others_alr[1] = can_struct.rx_can_buf[ECAN_R_15].Item.data[5].byte;//
						g_others_alr[2] = can_struct.rx_can_buf[ECAN_R_15].Item.data[6].byte;//
						g_others_alr[3] = can_struct.rx_can_buf[ECAN_R_15].Item.data[7].byte;//
						break;
					}
					
					default:				break;
				}
			}
		}
			
	}
}



/******************************************************
//CAN监测函数
//暂用1s调用
******************************************************/
void CanMonitor(uint16 past_sec)
{
//	if(1){//ACC开
		if(can_struct.can_open_flag == 0){//CAN没打开
			can_struct.can_open_flag = TRUE;
			ON_CAN_PWR();
			CanInit();
			g_sysm_timeout_struct.canrx_count = 0;
			return;
		}		
		
		CanTx();	
		
		if(g_sysm_on_off_struct.canrx_switch == SYSM_OFF){				///CAN接收中断关闭后下面内容不执行
			return;
		}
		
		if(can_struct.rx_data_flag != 0xFF){//解析CAN
			CanRx_Analyse();
			g_sysm_timeout_struct.canrx_count = 0;
			can_struct.rx_data_flag = 0xFF;
		}
		else{			
			g_sysm_timeout_struct.canrx_count += past_sec;
			if(g_sysm_timeout_struct.canrx_count >= SYSM_CANRX_TIMEOUT){///仪表长时间无数据,重启
				#ifdef CAN_DEBUG
				LocalUartFixedLenSend((uint8*)"\r\n meter break down alarm,wo will restart...\r\n",StrLen((uint8*)"\r\n meter break down alarm,wo will restart...\r\n",0));
				#endif				
				OFF_CAN_PWR();			
				CanDeInit();
				OSTimeDlyHMSM(0, 0, 2, 0);
				ON_CAN_PWR();
				CanInit();
				g_sysm_timeout_struct.canrx_count = 0;
			}
		}
		
		CANR_PeriodChargeFilter();
//	}

}


