#ifndef __SELFDEF_H
#define __SELFDEF_H

#include <stdint.h>

	uint8 U8SumCheck(uint8 data[],uint16 len);
	uint8 XorCheck(uint8 data[],uint16 len);
	void RamClear(uint8 *ram_addr,uint16 len);
	void RamZero(uint8 *ram_addr,uint16 len);
	uint16 CrcCheck(uint8 *p_data,uint16 len);
	uint16 Crc16Verify(uint16 check, uint8 *str, uint16 len);
	uint8 StrLen(const uint8 *str,uint8 max_count);
	void LongTimeDly(uint32 dly);
	uint16 SubMatch(uint8 sub[],uint8 sub_len,uint8 tar[],uint16 tar_len);
	uint8 IsValidChar(uint8 data[],uint8 len);
	uint8 IsValidAscii(uint8 data[],uint8 len);
	uint8 IsValidNum(uint8 data[],uint8 len);
	uint8 IsValidCharFloatNum(uint8 data[],uint8 len);
	uint8 IsValidNumOrChar(uint8 data[],uint8 len);
	uint8 IsValidNumOrCharOrDot(uint8 data[],uint8 len);
	uint8 IsValidSimCardNum(uint8 data[]);
	uint32 LittleBigSwap(uint32 val);
	void MemCpy(uint8 dst[],uint8 src[],uint16 len);
	uint8 MemCmp(uint8 dst[],uint8 src[],uint16 len);
	uint8 AsciiToHexVal(uint8 h_b,uint8 l_b);
	uint16 AsciiToHex(uint8 *p_src,uint16 len,uint8 *p_dst);
	uint16 HexToAscii(uint8 *p_src,uint16 len,uint8 *p_dst);
	uint32 U8ToUint32(uint8 data[]);
	uint8 U16ToAscii(uint8 in_data[],uint8 in_len,uint8 out_data[]);
	void Uint32ToU8(uint8 data[],uint32 val);
	uint32 U8ToUint32Rev(uint8 data[]);
	void Uint32ToU8Rev(uint8 data[],uint32 val);
#endif
