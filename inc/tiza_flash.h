#ifndef _FLASH_H 
#define _FLASH_H

#include "stdint.h"

	#ifdef INTFLASH_STRUCT_GLOBAL
		#define INTFLSAH_EXTERN
	#else
		#define INTFLSAH_EXTERN extern
	#endif
		
	
	/***************** 内部FLASH区域划分 ********************/
	/********************************************************/
	/*0x00000												*/
	/*			 BOOT程序	   扇区0 (8K)					*/
	/*0x01FFF												*/
	/********************************************************/
	/*0x02000												*/
	/*			 APP程序   		扇区1 - 扇区4 (112K) 		*/
	/*0x1DFFF												*/
	/********************************************************/
	/*0x1E000												*/
	/*			 用户自定义1	 扇区5 - 扇区6 (8K)	 		*/
	/*0x1FFFF												*/
	/********************************************************/
	/*0x20000												*/
	/*			 升级程序存储  扇区7 - 扇区8 (112K) 		*/
	/*0x3BFFF												*/
	/********************************************************/
	/*0x3C000												*/
	/*			 用户自定义2   扇区9 - 扇区11 (16K)			*/
	/*0x40000												*/
	/********************************************************/

/**************************** 注意 *****************************/
/*															   */	
/*	因为内部FLASH的扇区比较大，RAM小，存储数据时不适合作缓存,  */
/*	所以存储数据时，选择合适大小的扇区存储数据，在写入数据时， */
/*	会将地址所在扇区擦除，然后写入新的数据					   */ 
/***************************************************************/
		   
			//FLASH起始地址
	#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH的起始地址
	#define STM32_FLASH_SIZE 0x80000 			//STM32 FLASH的大小512k	
	#define ST32_PAGE_SIZE	 0x800 				//2k
	
	#define FLASH_UASER1_SATART_ADD					0x01E000		
	#define FLASH_UASER1_END_ADD						0x01FFFF		//16K	
	#define FlASH_STORE_START_ADD						0x020000	
	#define FLASH_STORE_END_ADD							0x03BFFF		//112K
//	#define FLASH_UASER2_SATART_ADD					0x03C000		 
	#define FLASH_UASER2_SATART_ADD					0x03B800	//0x03C000		 // qlj 等长沙板子回来 在BOOT中添加升级  擦除参数保存  这里回复3c000
	#define FLASH_UASER2_END_ADD						0x03FFFF		//16K	
	#define FLASH_OTHER_SATART_ADD					0x040000		
	#define FLASH_OTHER_END_ADD							0x800000		//256K	
	
	
	#define FLASH_BOOT_PARA_START_ADDR				FLASH_UASER1_SATART_ADD		
	#define FLASH_BOOT_PARA_END_ADDR					0x01E7FF		//2k	
	#define FLASH_ALARM_START_ADDR						FlASH_STORE_START_ADD	//64PAGE
	#define FLASH_ALARM_END_ADDR							0x29FFF								//与SYS_ALARM_MAXINDEX对应
	#define FLASH_ALARM_INDEXPAGE							0x2A000								//2K
	#define FLASH_SAVEPARA_INDEXPAGE					FLASH_UASER2_SATART_ADD				
	#define FLASH_SAVEVAILDPOS_INDEXPAGE			0x03F000							//2K	
	#define FLASH_SAVEVIN_INDEXPAGE						0x03F800							//2K
	
	
	#define FTP_BLIND_FLASH_START 	 0x8020000
	#define FTP_BLIND_FLASH_END 		(0x8040000 - 0x2000)

	INTFLSAH_EXTERN uint8 FlashErase(uint32 page_addr);
	INTFLSAH_EXTERN uint8 FlashWrite(uint32 start_addr,uint8 wr_data[], uint16 len);
	INTFLSAH_EXTERN void  FlashRead(uint32 start_addr, uint8 rd_data[], uint16 len);
	INTFLSAH_EXTERN uint8 CpuFlashWrite(uint32 Addr, uint8 *Data, uint16 Len);	
		
		
#endif
