;/*********************** (C) COPYRIGHT 2013 Libraworks *************************
;* File Name	: RTOS_ASM.s
;* Author		: 卢晓铭 
;* Version		: V1.0
;* Date			: 01/26/2013
;* Description	: LXM-RTOS asm port
;*******************************************************************************/
	IMPORT OSInterruptSum
	IMPORT OSRunning
	IMPORT p_OSTCBCur
	IMPORT p_OSTCBHighRdy
	IMPORT OSTaskSwHook
	IMPORT OSPrioCur
	IMPORT OSPrioHighRdy

	EXPORT OS_ENTER_CRITICAL
	EXPORT OS_EXIT_CRITICAL
	EXPORT OSStartHighRdy
	EXPORT OSCtxSw
	EXPORT OSIntCtxSw
	EXPORT PendSV_Handler

NVIC_INT_CTRL   	EQU     0xE000ED04  ; 中断控制寄存器
NVIC_SYSPRI2    	EQU     0xE000ED20  ; 系统优先级寄存器(2)
NVIC_PENDSV_PRI 	EQU     0xFFFF0000  ; 软件中断和系统节拍中断
                                        ; (都为最低，0xff).
NVIC_PENDSVSET  	EQU     0x10000000  ; 触发软件中断的值.

		PRESERVE8 
		AREA    |.text|, CODE, READONLY
        THUMB 
;/***************************************************************************************
;* 函数名称: OS_ENTER_CRITICAL
;*
;* 功能描述: 进入临界区：禁止中断，并记录中断次数 
;*            
;* 参    数: None
;*
;* 返 回 值: None
;*****************************************************************************************/ 
;PRIMASK这个寄存器只有一个位，置1后，将关闭所有可屏蔽中断的异常，只剩NMI和硬fault，默认值为0；
OS_ENTER_CRITICAL
 
		CPSID   I                       ; Disable all the interrupts;PRIMASK=0
                                                                        
		PUSH 	{R1,R2}      

		LDR 	R1, =OSInterruptSum	    ; OSInterrputSum++，实现一个变量加1
        LDRB 	R2, [R1]
        ADD   	R2, R2, #1
        STRB 	R2, [R1]
		POP     {R1,R2}
  		BX LR

;/***************************************************************************************
;* 函数名称: OS_EXIT_CRITICAL
;*
;* 功能描述: 退出临界区 ，根据中断嵌套的情况决定是否使能中断
;*            
;* 参    数: None
;*
;* 返 回 值: None
;*****************************************************************************************/

OS_EXIT_CRITICAL
		PUSH    {R1, R2}
		LDR     R1, =OSInterruptSum     ; OSInterrputSum--,这个变量的目的是控制中断嵌套，一旦发现嵌套就不使能中断
        LDRB    R2, [R1]
        SUB     R2, R2, #1
        STRB    R2, [R1]
		MOV     R1,  #0	      
		CMP     R2,  #0			        ; if OSInterrputSum=0,enable 
                                        ; interrupts如果OSInterrputSum=0，
		MSREQ  PRIMASK, R1   			;是两条指令MSR和EQ，上面比较相等就执行MSR指令;PRIMASK=R1
	    POP   	{R1, R2}
		BX LR
;/**************************************************************************************
;* 函数名称: OSStartHighRdy
;*
;* 功能描述: 使用调度器运行第一个任务
;* 
;* 参    数: None
;*
;* 返 回 值: None
;**************************************************************************************/  

OSStartHighRdy
        LDR     R4, =NVIC_SYSPRI2      ; set the PendSV exception priority，设置PendSV异常的优先级
        LDR     R5, =NVIC_PENDSV_PRI
        STR     R5, [R4]

        MOV     R4, #0                 ; set the PSP to 0 for initial context switch call
        MSR     PSP, R4				   ;设置为 0，是告诉具体的任务切换程序（ OS_CPU_PendSVHandler()），这是第一次任务切换。做过切换后 PSP 就不会为 0 了

        LDR     R4, =OSRunning         ; OSRunning = TRUE
        MOV     R5, #1
        STRB    R5, [R4]

                                       ;切换到最高优先级的任务
        LDR     R4, =NVIC_INT_CTRL     ;rigger the PendSV exception (causes context switch)
        LDR     R5, =NVIC_PENDSVSET
        STR     R5, [R4]				;*(uint32_t *)NVIC_INT_CTRL = NVIC_PENDSVSET,触发 PendSV 中断

        CPSIE   I                      ;enable interrupts at processor level,PRIMASK=1
OSStartHang
        B       OSStartHang            ;should never get here

;/**************************************************************************************
;* 函数名称: OSCtxSw
;*
;* 功能描述: 函数级任务切换，触发特定的中断进行任务切换         
;*
;* 参    数: None
;*
;* 返 回 值: None
;***************************************************************************************/
;OSCtxSw和OSIntCtxSw目的是不同的，但是由于CM3核的原因，这里实现代码一样，再别的平台上需要考虑具体实现方法 
OSCtxSw
		PUSH    {R4, R5}				;由于需要使用这两个寄存器，所以先保存起来，最后再恢复
        LDR     R4, =NVIC_INT_CTRL  	;触发PendSV异常 (causes context switch)
        LDR     R5, =NVIC_PENDSVSET
        STR     R5, [R4]				;激活中断开始切换任务
		POP     {R4, R5}
        BX      LR
;/**************************************************************************************
;* 函数名称: OSIntCtxSw
;*
;* 功能描述: 中断级任务切换
;*
;* 参    数: None
;*
;* 返 回 值: None
;***************************************************************************************/

OSIntCtxSw
		PUSH    {R4, R5}
        LDR     R4, =NVIC_INT_CTRL      ;触发PendSV异常 (causes context switch)
        LDR     R5, =NVIC_PENDSVSET
        STR     R5, [R4]
		POP     {R4, R5}
        BX      LR
        NOP

;/**************************************************************************************
;* 函数名称: OSPendSV
;*
;* 功能描述: OSPendSV is used to cause a context switch.通过这个中断获取优先级最高的任务堆栈内容
;*			然后开始任务的运行。
;* 参    数: None
;*
;* 返 回 值: None
;***************************************************************************************/
;CM3核所有任务切换都被放到 PendSV 的中断处理函数中去做了
PendSV_Handler													;跳转到这里时xPSR, PC, LR, R12, R0-R3 已自动保存
    CPSID   I                                                   ; Prevent interruption during context switch
    MRS     R0, PSP                                             ; PSP is process stack pointer 如果在用PSP堆栈,则可以忽略保存寄存器,参考CM3权威中的双堆栈-白菜注
    CBZ     R0, PendSV_Handler_Nosave		                    ; Skip register save the first time，如果 PSP == 0，跳转到PendSV_Handler_Nosave
																;PSP == 0说明 OSStartHighRdy()启动后第一次做任务切换，而任务刚创建时 R4-R11 已经保存在堆栈中了，所以不需要再保存一次了。
    SUBS    R0, R0, #0x20                                       ; Save remaining regs r4-11 on process stack，R0 -= 0x20并影响标志位
    STM     R0, {R4-R11}										;R0现在指向了栈顶部，除去自动保存的8个寄存器偏移后保存这里的寄存器值

    LDR     R1, =p_OSTCBCur                                     ; OSTCBCur->OSTCBStkPtr = SP;
    LDR     R1, [R1]											;获得了栈顶指针
    STR     R0, [R1]                                            ; R0 is SP of process being switched out,任务控制块，需要记录新的参数，这里就是记录了当前任务的堆栈指针

                                                                ; At this point, entire context of process has been saved
PendSV_Handler_Nosave
    PUSH    {R14}                                               ; Save LR exc_return value，保存 R14，因为后面要调用函数
    LDR     R0, =OSTaskSwHook                                   ; OSTaskSwHook();
    BLX     R0													;调用 OSTaskSwHook()
    POP     {R14}												;恢复 R14

    LDR     R0, =OSPrioCur                                      ; OSPrioCur = OSPrioHighRdy;
    LDR     R1, =OSPrioHighRdy
    LDRB    R2, [R1]
    STRB    R2, [R0]											;OSPrioCur = OSPrioHighRdy对一个变量进行赋值

    LDR     R0, =p_OSTCBCur                                     ; OSTCBCur  = OSTCBHighRdy;
    LDR     R1, =p_OSTCBHighRdy
    LDR     R2, [R1]
    STR     R2, [R0]											;又一次对变量进行赋值，这里是任务控制块
																;下面开始获取新的任务的运行环境
    LDR     R0, [R2]                                            ; R0 is new process SP; SP = OSTCBHighRdy->OSTCBStkPtr;R0是准备运行的任务的SP
    LDM     R0, {R4-R11}                                        ; Restore r4-11 from new process stack
    ADDS    R0, R0, #0x20										;中间跳过的8个寄存器已经自动保存了
    MSR     PSP, R0                                             ; Load PSP with new process SP
    ORR     LR, LR, #0x04                                       ; Ensure exception return uses process stack，确保 LR 位 2 为 1，返回后使用进程堆栈
    CPSIE   I
    BX      LR                                                  ; Exception return will restore remaining context


;************************************************************
		ALIGN

		END
