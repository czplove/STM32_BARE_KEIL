/*********************** (C) COPYRIGHT 2013 Libraworks *************************
* File Name	    : RTOS.c
* Author		: 卢晓铭 
* Version		: V1.0
* Date			: 01/26/2013
* Description	: LXM-RTOS 任务管理
*******************************************************************************/

#include"RTOS.h"

TaskCtrBlock TCB[OS_TASKS - 1];	/*任务控制块定义*/
TaskCtrBlock *p_OSTCBCur;		/*指向当前任务控制块的指针*/
TaskCtrBlock *p_OSTCBHighRdy;	/*指向最高优先级就绪任务控制块的指针*/
u8 OSPrioCur;					/*当前执行任务*/
u8 OSPrioHighRdy;				/*最高优先级*/
u8 OSRunning;					/*多任务运行标志0:为运行，1:运行*/

u32 OSInterruptSum;				/*进入中断次数*/

u32 OSTime;						/*系统时间(进入时钟中断次数)*/

u32 OSRdyTbl;					/*任务就绪表,0表示挂起,1表示就绪*/

u32 OSIntNesting;				/*任务嵌套数*/

/*设置任务延时时间*/
void OSTimeDly(u32 ticks)
{
	if(ticks > 0)
	{
		OS_ENTER_CRITICAL();				//进入临界区，仅仅需要关闭中断，并回复部分参数就好
		OSDelPrioRdy(OSPrioCur);			//将任务挂起
		TCB[OSPrioCur].OSTCBDly = ticks;	//设置TCB中任务延时节拍数
		OS_EXIT_CRITICAL();					//退出临界区
		OSSched();
	}
}

/*定时器中断对任务延时处理函数*/
void TicksInterrupt(void)
{
	static u8 i;

	OSTime++;
	for(i=0;i<OS_TASKS;i++)
	{
		if(TCB[i].OSTCBDly)
		{
			TCB[i].OSTCBDly--;
			if(TCB[i].OSTCBDly==0)	//延时时钟到达
			{
				OSSetPrioRdy(i);	//任务重新就绪
			}	
		}
	}
}

/*任务切换*/
void OSSched(void)
{
	OSGetHighRdy();					//找出任务就绪表中优先级最高的任务
	if(OSPrioHighRdy!=OSPrioCur)	//如果不是当前运行任务，进行任务调度
	{
		p_OSTCBCur = &TCB[OSPrioCur];				//汇编中引用			
		p_OSTCBHighRdy = &TCB[OSPrioHighRdy];		//汇编中引用
		OSPrioCur = OSPrioHighRdy;	//更新OSPrio
		OSCtxSw();					//调度任务
	}
}
/*任务创建*/
void OSTaskCreate(void  (*Task)(void  *parg), void *parg, u32 *p_Stack, u8 TaskID)	//-就是保存堆栈信息，以便CPU运行不同的程序
{
	if(TaskID <= OS_TASKS)	//-判断任务优先级是否合理
	{
        *(p_Stack) = (u32)0x01000000L;					/*  xPSR  程序状态寄存器         */ 
	    *(--p_Stack) = (u32)Task;								/*  Entry Point of the task  任务入口地址   */
	    *(--p_Stack) = (u32)0xFFFFFFFEL;				/*  R14 (LR)  (init value will  */
	                                                                           
	    *(--p_Stack) = (u32)0x12121212L;				/*  R12                         */
	    *(--p_Stack) = (u32)0x03030303L;				/*  R3                          */
	    *(--p_Stack) = (u32)0x02020202L;				/*  R2                          */
	    *(--p_Stack) = (u32)0x01010101L;				/*  R1                          */
			*(--p_Stack) = (u32)parg;								/*  R0 : argument  输入参数     */

			*(--p_Stack) = (u32)0x11111111L;				/*  R11                         */
	    *(--p_Stack) = (u32)0x10101010L;				/*  R10                         */
	    *(--p_Stack) = (u32)0x09090909L;				/*  R9                          */
	    *(--p_Stack) = (u32)0x08080808L;				/*  R8                          */
	    *(--p_Stack) = (u32)0x07070707L;				/*  R7                          */
	    *(--p_Stack) = (u32)0x06060606L;				/*  R6                          */
	    *(--p_Stack) = (u32)0x05050505L;				/*  R5                          */
	    *(--p_Stack) = (u32)0x04040404L;				/*  R4                          */

		TCB[TaskID].OSTCBStkPtr = (u32)p_Stack;		/*保存堆栈地址*/
		TCB[TaskID].OSTCBDly = 0;									/*初始化任务延时*/
		OSSetPrioRdy(TaskID);											/*在任务就绪表中登记*/
	}
}

void OSTaskSuspend(u8 prio)
{
	OS_ENTER_CRITICAL();		/*进入临界区*/
	TCB[prio].OSTCBDly = 0;
	OSDelPrioRdy(prio);			/*挂起任务*/
	OS_EXIT_CRITICAL();			/*退出临界区*/

	if(OSPrioCur == prio)		/*挂起的任务为当前运行的任务*/
	{
		OSSched();				/*重新调度*/
	}
}

void OSTaskResume(u8 prio)
{
	OS_ENTER_CRITICAL();
	TCB[prio].OSTCBDly = 0;		/*设置任务延时时间为0*/
	OSSetPrioRdy(prio);			/*就绪任务*/
	OS_EXIT_CRITICAL();

	if(OSPrioCur > prio)		/*当前任务优先级小于恢复的任务优先级*/
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
		
		OSTaskCreate(IdleTask, (void *)0, (u32 *)&IDELTASK_STK[31], IdelTask_Prio);	//创建空闲任务

		OSGetHighRdy();					/*获得最高级的就绪任务*/
		OSPrioCur = OSPrioHighRdy;		/*获得最高优先级就绪任务ID*/
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
	if(OSIntNesting == 0)	//-没有中断嵌套时，才可以进行任务调度
	{
		OSGetHighRdy();					/*找出任务优先级最高的就绪任务*/
		if(OSPrioHighRdy!=OSPrioCur)	/*当前任务并非优先级最高的就绪任务*/
		{
			p_OSTCBCur = &TCB[OSPrioCur];
			p_OSTCBHighRdy = &TCB[OSPrioHighRdy];
			OSPrioCur = OSPrioHighRdy;
			OSIntCtxSw();				/*中断级任务调度*/	//-注意这里和OSCtxSw不一样，但是作用是一样的
		}
	}

	OS_EXIT_CRITICAL();
}

/*系统空闲任务*/
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
