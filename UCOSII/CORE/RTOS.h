;/*********************** (C) COPYRIGHT 2013 Libraworks *************************
;* File Name	: RTOS_ASM.s
;* Author		: ¬���� 
;* Version		: V1.0
;* Date			: 01/26/2013
;* Description	: LXM-RTOS asm port
;*******************************************************************************/
#ifndef __RTOS_H
#define __RTOS_H

#include"sys.h"

typedef struct TaskCtrBlockHead		/*������ƿ����ݽṹ*/
{
	u32 OSTCBStkPtr;				/*��������ջ��*/
	u32 OSTCBDly;					/*������ʱʱ��*/
}TaskCtrBlock;

#define OS_TASKS 32				/*��������*/
#define IdelTask_Prio 31		/*�����������ȼ�*/

extern TaskCtrBlock TCB[OS_TASKS - 1];	/*������ƿ鶨��*/
extern TaskCtrBlock *p_OSTCBCur;		/*ָ��ǰ������ƿ��ָ��*/
extern TaskCtrBlock *p_OSTCBHighRdy;	/*ָ��������ȼ�����������ƿ��ָ��*/
extern u8 OSPrioCur;					/*��ǰִ������*/
extern u8 OSPrioHighRdy;				/*������ȼ�*/
extern u8 OSRunning;					/*���������б�־0:Ϊ���У�1:����*/
extern u32 OSInterruptSum;				/*�����жϴ���*/
extern u32 OSTime;						/*ϵͳʱ��(����ʱ���жϴ���)*/
extern u32 OSRdyTbl;					/*���������,0��ʾ����,1��ʾ����*/
extern u32 OSIntNesting;				/*����Ƕ����*/

/*�ھ������еǼ�����*/
__inline void OSSetPrioRdy(u8 prio)
{
	OSRdyTbl |= 1 << prio;
}

/*�ھ�������ɾ������*/
__inline void OSDelPrioRdy(u8 prio)
{
	OSRdyTbl &= ~(1<<prio);
}

/*�ھ������в��Ҹ��߼��ľ�������*/
__inline void OSGetHighRdy(void)
{
	u8 OSNextTaskPrio = 0;		/*�������ȼ�*/

	for (OSNextTaskPrio = 0; (OSNextTaskPrio < OS_TASKS) && (!(OSRdyTbl&(0x01<<OSNextTaskPrio))); OSNextTaskPrio++ );
	OSPrioHighRdy = OSNextTaskPrio;	
}

void OSTimeDly(u32 ticks);		/*����������ʱʱ��*/
void TicksInterrupt(void);		/*��ʱ���ж϶�������ʱ������*/
void IdleTask(void *pdata);		/*ϵͳ��������*/
void OSSched(void);				/*�����л�*/
void OSStart(void);				/*������ϵͳ��ʼ*/
void OSIntExit(void);			/*�ж��˳�����*/

void OSTaskCreate(void  (*Task)(void  *parg), void *parg, u32 *p_Stack, u8 TaskID);	/*����������*/
void OSTaskSuspend(u8 prio);	/*����ָ������*/
void OSTaskResume(u8 prio);		/*�ظ�ָ���Ĺ�������*/

void OSTaskSwHook(void);		/*�պ���*/
/*in asm function*/
void OS_EXIT_CRITICAL(void);	/*�˳��ٽ���*/
void OS_ENTER_CRITICAL(void);	/*�����ٽ���*/
void OSStartHighRdy(void);		/*���ȵ�һ������*/
void OSCtxSw(void);				/*�����������л�*/
void OSIntCtxSw(void);			/*�жϼ������л�*/

#endif
