#ifndef TIZA_BUSINESS_H
#define TIZA_BUSINESS_H

#include <stdint.h>
	
	#ifdef BUSINESS_GLOABL
		#define EXTERN_BUSINESS
	#else
		#define EXTERN_BUSINESS extern
	#endif
	
	#define _YES 	1
	#define _NO 	0
	
	EXTERN_BUSINESS uint8  GPRStestdata[600];
	
	
	typedef struct
	{
		uint8 usemodule;
		uint8 needlinkser;
		uint8 needopengps;
	}BUSINESS_STRUCT;
	
	
	EXTERN_BUSINESS BUSINESS_STRUCT g_business_struct;
	
	
	EXTERN_BUSINESS void BusiDisconSer(void);
	EXTERN_BUSINESS void BusiConSer(void);
	EXTERN_BUSINESS void BusiDisconGps(void);
	EXTERN_BUSINESS void BusiConGps(void);
	EXTERN_BUSINESS void BusiResetModule(void);
	
	EXTERN_BUSINESS void GPRStestfun(void);
#endif
