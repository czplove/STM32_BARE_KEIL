

#define INTFLASH_STRUCT_GLOBAL

#include "tiza_include.h"


uint8 FlashErase(uint32 page_addr)
{
	FLASH_Status status;
	uint8 res = FALSE;
	OS_CPU_SR  cpu_sr = 0u;
	
	OS_ENTER_CRITICAL(); 
	FeedWtd();
	FLASH_Unlock();	
	status = FLASH_ErasePage(0x8000000 + page_addr);
	FLASH_Lock();
	FeedWtd();
	OS_EXIT_CRITICAL();
	if(status == FLASH_COMPLETE)
	{
		res = TRUE;
	}
	
	return res;
}
uint8 FlashWrite(uint32 start_addr,uint8 wr_data[], uint16 len)
{
	uint8 res;
	uint32 i,dst,src;
	OS_CPU_SR  cpu_sr = 0u;
	
	res = TRUE;
	dst = 0x8000000 + start_addr;		
	src = (uint32)wr_data;
	
	
	OS_ENTER_CRITICAL(); 
	FeedWtd();
	FLASH_Unlock();
	for(i=0;i<len;i+=4)
	{
		FLASH_ProgramWord(dst, *(u32*)src);
		if(*(u32*)dst != *(u32*)src)
		{
			res = FALSE;
			break;
		}
		dst += 4;
		src += 4;
	}
	FLASH_Lock();
	FeedWtd();
	OS_EXIT_CRITICAL();
	
	return res;
}
void FlashRead(uint32 start_addr, uint8 rd_data[], uint16 len)
{	
	uint16 i=0;
	uint32 tmp_addr;
	
	tmp_addr = 0x8000000 + start_addr;
	
 	for(i=0;i<len;i++)
	{
		rd_data[i] = *(uint8 *)tmp_addr;
		tmp_addr++;
	}
}


///ftp

//读取指定地址的字(32位数据) 
//faddr:读地址 
//返回值:对应数据.
static uint32 STMFLASH_ReadWord(uint32 faddr)
{
	return *(uint32*)faddr; 
}  

//从指定地址开始写入指定长度的数据
//特别注意:因为STM32F4的扇区实在太大,没办法本地保存扇区数据,所以本函数
//         写地址如果非0XFF,那么会先擦除整个扇区且不保存扇区数据.所以
//         写非0XFF的地址,将导致整个扇区数据丢失.建议写之前确保扇区里
//         没有重要数据,最好是整个扇区先擦除了,然后慢慢往后写. 
//该函数对OTP区域也有效!可以用来写OTP区!
//OTP区域地址范围:0X1FFF7800~0X1FFF7A0F

// 0 : success    1 : failed
uint8 CpuFlashWrite(uint32 Addr, uint8 *Data, uint16 Len)
{ 
  	FLASH_Status status = FLASH_COMPLETE;
	uint32 addrx = 0;
	uint32 endaddr = 0;	
	uint32 *Data_pt;
	bool flag = TRUE;
	
	if(Addr < STM32_FLASH_BASE || (Addr + Len) >= (STM32_FLASH_BASE + STM32_FLASH_SIZE))
		return 1;	//非法地址
		
	FLASH_Unlock();									//解锁 
	//-FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
 		
	addrx = Addr;				//写入的起始地址
	endaddr = Addr + Len;	//写入的结束地址

	if(addrx < 0X1FFF0000)			//只有主存储区,才需要执行擦除操作!!
	{
		while(addrx < endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(STMFLASH_ReadWord(addrx) != 0XFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
			{   
				//-status = FLASH_EraseSector(STMFLASH_GetFlashSector(addrx), VoltageRange_3);//VCC=2.7~3.6V之间!!
				status = FLASH_ErasePage(addrx);
				if(status != FLASH_COMPLETE)
				{
					flag = FALSE;
					break;	//发生错误了
				}
			}
			else
			{
				addrx+=4;
			}
		} 
	}

	Data_pt = (uint32 *)Data;
	
	if(status == FLASH_COMPLETE)
	{
		while(Addr < endaddr)//写数据
		{
			//-if(FLASH_ProgramByte(Addr, *Data) != FLASH_COMPLETE)//写入数据
			if(FLASH_ProgramWord(Addr, *Data_pt) != FLASH_COMPLETE)//写入数据
			{ 
				break;	//写入异常
			}
			
			Addr += 4;
			Data_pt += 1;
		} 
	}
	
	//-FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁

	if(flag == TRUE)
	{
		return 0;
	}
	else
	{
		return 1;
	}
} 




