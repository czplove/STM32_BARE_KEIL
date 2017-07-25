/*********************** (C) COPYRIGHT 2013 Libraworks *************************
* File Name	    : RTOS.c
* Author		: ¬���� 
* Version		: V1.0
* Date			: 01/26/2013
* Description	: LXM-RTOS �������
*******************************************************************************/

#include"RTOS.h"

TaskCtrBlock TCB[OS_TASKS - 1];	/*������ƿ鶨��*/
TaskCtrBlock *p_OSTCBCur;		/*ָ��ǰ������ƿ��ָ��*/
TaskCtrBlock *p_OSTCBHighRdy;	/*ָ��������ȼ�����������ƿ��ָ��*/
u8 OSPrioCur;					/*��ǰִ������*/
u8 OSPrioHighRdy;				/*������ȼ�*/
u8 OSRunning;					/*���������б�־0:Ϊ���У�1:����*/

u32 OSInterruptSum;				/*�����жϴ���*/

u32 OSTime;						/*ϵͳʱ��(����ʱ���жϴ���)*/

u32 OSRdyTbl;					/*���������,0��ʾ����,1��ʾ����*/

u32 OSIntNesting;				/*����Ƕ����*/

/*����������ʱʱ��*/
void OSTimeDly(u32 ticks)
{
	if(ticks > 0)
	{
		OS_ENTER_CRITICAL();				//�����ٽ�����������Ҫ�ر��жϣ����ظ����ֲ����ͺ�
		OSDelPrioRdy(OSPrioCur);			//���������
		TCB[OSPrioCur].OSTCBDly = ticks;	//����TCB��������ʱ������
		OS_EXIT_CRITICAL();					//�˳��ٽ���
		OSSched();
	}
}

/*��ʱ���ж϶�������ʱ������*/
void TicksInterrupt(void)
{
	static u8 i;

	OSTime++;
	for(i=0;i<OS_TASKS;i++)
	{
		if(TCB[i].OSTCBDly)
		{
			TCB[i].OSTCBDly--;
			if(TCB[i].OSTCBDly==0)	//��ʱʱ�ӵ���
			{
				OSSetPrioRdy(i);	//�������¾���
			}	
		}
	}
}

/*�����л�*/
void OSSched(void)
{
	OSGetHighRdy();					//�ҳ���������������ȼ���ߵ�����
	if(OSPrioHighRdy!=OSPrioCur)	//������ǵ�ǰ�������񣬽����������
	{
		p_OSTCBCur = &TCB[OSPrioCur];				//���������			
		p_OSTCBHighRdy = &TCB[OSPrioHighRdy];		//���������
		OSPrioCur = OSPrioHighRdy;	//����OSPrio
		OSCtxSw();					//��������
	}
}
/*���񴴽�*/
void OSTaskCreate(void  (*Task)(void  *parg), void *parg, u32 *p_Stack, u8 TaskID)	//-���Ǳ����ջ��Ϣ���Ա�CPU���в�ͬ�ĳ���
{
	if(TaskID <= OS_TASKS)	//-�ж��������ȼ��Ƿ����
	{
        *(p_Stack) = (u32)0x01000000L;					/*  xPSR  ����״̬�Ĵ���         */ 
	    *(--p_Stack) = (u32)Task;								/*  Entry Point of the task  ������ڵ�ַ   */
	    *(--p_Stack) = (u32)0xFFFFFFFEL;				/*  R14 (LR)  (init value will  */
	                                                                           
	    *(--p_Stack) = (u32)0x12121212L;				/*  R12                         */
	    *(--p_Stack) = (u32)0x03030303L;				/*  R3                          */
	    *(--p_Stack) = (u32)0x02020202L;				/*  R2                          */
	    *(--p_Stack) = (u32)0x01010101L;				/*  R1                          */
			*(--p_Stack) = (u32)parg;								/*  R0 : argument  �������     */

			*(--p_Stack) = (u32)0x11111111L;				/*  R11                         */
	    *(--p_Stack) = (u32)0x10101010L;				/*  R10                         */
	    *(--p_Stack) = (u32)0x09090909L;				/*  R9                          */
	    *(--p_Stack) = (u32)0x08080808L;				/*  R8                          */
	    *(--p_Stack) = (u32)0x07070707L;				/*  R7                          */
	    *(--p_Stack) = (u32)0x06060606L;				/*  R6                          */
	    *(--p_Stack) = (u32)0x05050505L;				/*  R5                          */
	    *(--p_Stack) = (u32)0x04040404L;				/*  R4                          */

		TCB[TaskID].OSTCBStkPtr = (u32)p_Stack;		/*�����ջ��ַ*/
		TCB[TaskID].OSTCBDly = 0;									/*��ʼ��������ʱ*/
		OSSetPrioRdy(TaskID);											/*������������еǼ�*/
	}
}

void OSTaskSuspend(u8 prio)
{
	OS_ENTER_CRITICAL();		/*�����ٽ���*/
	TCB[prio].OSTCBDly = 0;
	OSDelPrioRdy(prio);			/*��������*/
	OS_EXIT_CRITICAL();			/*�˳��ٽ���*/

	if(OSPrioCur == prio)		/*���������Ϊ��ǰ���е�����*/
	{
		OSSched();				/*���µ���*/
	}
}

void OSTaskResume(u8 prio)
{
	OS_ENTER_CRITICAL();
	TCB[prio].OSTCBDly = 0;		/*����������ʱʱ��Ϊ0*/
	OSSetPrioRdy(prio);			/*��������*/
	OS_EXIT_CRITICAL();

	if(OSPrioCur > prio)		/*��ǰ�������ȼ�С�ڻָ����������ȼ�*/
	{
		OSSched();
	}
}

u32 IDELTASK_STK[32];

void OSStart(void)
{
	if(OSRunning == 0)
	{
		OSRunning = 1;
		
		OSTaskCreate(IdleTask, (void *)0, (u32 *)&IDELTASK_STK[31], IdelTask_Prio);	//������������

		OSGetHighRdy();					/*�����߼��ľ�������*/
		OSPrioCur = OSPrioHighRdy;		/*���������ȼ���������ID*/
		p_OSTCBCur = &TCB[OSPrioCur];
		p_OSTCBHighRdy = &TCB[OSPrioHighRdy];
		OSStartHighRdy();
	}
}

void OSIntExit(void)
{
	OS_ENTER_CRITICAL();
	
	if(OSIntNesting > 0)
		OSIntNesting--;
	if(OSIntNesting == 0)	//-û���ж�Ƕ��ʱ���ſ��Խ����������
	{
		OSGetHighRdy();					/*�ҳ��������ȼ���ߵľ�������*/
		if(OSPrioHighRdy!=OSPrioCur)	/*��ǰ���񲢷����ȼ���ߵľ�������*/
		{
			p_OSTCBCur = &TCB[OSPrioCur];
			p_OSTCBHighRdy = &TCB[OSPrioHighRdy];
			OSPrioCur = OSPrioHighRdy;
			OSIntCtxSw();				/*�жϼ��������*/	//-ע�������OSCtxSw��һ��������������һ����
		}
	}

	OS_EXIT_CRITICAL();
}

/*ϵͳ��������*/
void IdleTask(void *pdata)
{
	u32 IdleCount = 0;
	while(1)
	{
		IdleCount++;
	}
}

void OSTaskSwHook(void)
{
}
