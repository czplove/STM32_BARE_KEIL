;/*********************** (C) COPYRIGHT 2013 Libraworks *************************
;* File Name	: RTOS_ASM.s
;* Author		: ¬���� 
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

NVIC_INT_CTRL   	EQU     0xE000ED04  ; �жϿ��ƼĴ���
NVIC_SYSPRI2    	EQU     0xE000ED20  ; ϵͳ���ȼ��Ĵ���(2)
NVIC_PENDSV_PRI 	EQU     0xFFFF0000  ; ����жϺ�ϵͳ�����ж�
                                        ; (��Ϊ��ͣ�0xff).
NVIC_PENDSVSET  	EQU     0x10000000  ; ��������жϵ�ֵ.

		PRESERVE8 
		AREA    |.text|, CODE, READONLY
        THUMB 
;/***************************************************************************************
;* ��������: OS_ENTER_CRITICAL
;*
;* ��������: �����ٽ�������ֹ�жϣ�����¼�жϴ��� 
;*            
;* ��    ��: None
;*
;* �� �� ֵ: None
;*****************************************************************************************/ 
;PRIMASK����Ĵ���ֻ��һ��λ����1�󣬽��ر����п������жϵ��쳣��ֻʣNMI��Ӳfault��Ĭ��ֵΪ0��
OS_ENTER_CRITICAL
 
		CPSID   I                       ; Disable all the interrupts;PRIMASK=0
                                                                        
		PUSH 	{R1,R2}      

		LDR 	R1, =OSInterruptSum	    ; OSInterrputSum++��ʵ��һ��������1
        LDRB 	R2, [R1]
        ADD   	R2, R2, #1
        STRB 	R2, [R1]
		POP     {R1,R2}
  		BX LR

;/***************************************************************************************
;* ��������: OS_EXIT_CRITICAL
;*
;* ��������: �˳��ٽ��� �������ж�Ƕ�׵���������Ƿ�ʹ���ж�
;*            
;* ��    ��: None
;*
;* �� �� ֵ: None
;*****************************************************************************************/

OS_EXIT_CRITICAL
		PUSH    {R1, R2}
		LDR     R1, =OSInterruptSum     ; OSInterrputSum--,���������Ŀ���ǿ����ж�Ƕ�ף�һ������Ƕ�׾Ͳ�ʹ���ж�
        LDRB    R2, [R1]
        SUB     R2, R2, #1
        STRB    R2, [R1]
		MOV     R1,  #0	      
		CMP     R2,  #0			        ; if OSInterrputSum=0,enable 
                                        ; interrupts���OSInterrputSum=0��
		MSREQ  PRIMASK, R1   			;������ָ��MSR��EQ������Ƚ���Ⱦ�ִ��MSRָ��;PRIMASK=R1
	    POP   	{R1, R2}
		BX LR
;/**************************************************************************************
;* ��������: OSStartHighRdy
;*
;* ��������: ʹ�õ��������е�һ������
;* 
;* ��    ��: None
;*
;* �� �� ֵ: None
;**************************************************************************************/  

OSStartHighRdy
        LDR     R4, =NVIC_SYSPRI2      ; set the PendSV exception priority������PendSV�쳣�����ȼ�
        LDR     R5, =NVIC_PENDSV_PRI
        STR     R5, [R4]

        MOV     R4, #0                 ; set the PSP to 0 for initial context switch call
        MSR     PSP, R4				   ;����Ϊ 0���Ǹ��߾���������л����� OS_CPU_PendSVHandler()�������ǵ�һ�������л��������л��� PSP �Ͳ���Ϊ 0 ��

        LDR     R4, =OSRunning         ; OSRunning = TRUE
        MOV     R5, #1
        STRB    R5, [R4]

                                       ;�л���������ȼ�������
        LDR     R4, =NVIC_INT_CTRL     ;rigger the PendSV exception (causes context switch)
        LDR     R5, =NVIC_PENDSVSET
        STR     R5, [R4]				;*(uint32_t *)NVIC_INT_CTRL = NVIC_PENDSVSET,���� PendSV �ж�

        CPSIE   I                      ;enable interrupts at processor level,PRIMASK=1
OSStartHang
        B       OSStartHang            ;should never get here

;/**************************************************************************************
;* ��������: OSCtxSw
;*
;* ��������: �����������л��������ض����жϽ��������л�         
;*
;* ��    ��: None
;*
;* �� �� ֵ: None
;***************************************************************************************/
;OSCtxSw��OSIntCtxSwĿ���ǲ�ͬ�ģ���������CM3�˵�ԭ������ʵ�ִ���һ�����ٱ��ƽ̨����Ҫ���Ǿ���ʵ�ַ��� 
OSCtxSw
		PUSH    {R4, R5}				;������Ҫʹ���������Ĵ����������ȱ�������������ٻָ�
        LDR     R4, =NVIC_INT_CTRL  	;����PendSV�쳣 (causes context switch)
        LDR     R5, =NVIC_PENDSVSET
        STR     R5, [R4]				;�����жϿ�ʼ�л�����
		POP     {R4, R5}
        BX      LR
;/**************************************************************************************
;* ��������: OSIntCtxSw
;*
;* ��������: �жϼ������л�
;*
;* ��    ��: None
;*
;* �� �� ֵ: None
;***************************************************************************************/

OSIntCtxSw
		PUSH    {R4, R5}
        LDR     R4, =NVIC_INT_CTRL      ;����PendSV�쳣 (causes context switch)
        LDR     R5, =NVIC_PENDSVSET
        STR     R5, [R4]
		POP     {R4, R5}
        BX      LR
        NOP

;/**************************************************************************************
;* ��������: OSPendSV
;*
;* ��������: OSPendSV is used to cause a context switch.ͨ������жϻ�ȡ���ȼ���ߵ������ջ����
;*			Ȼ��ʼ��������С�
;* ��    ��: None
;*
;* �� �� ֵ: None
;***************************************************************************************/
;CM3�����������л������ŵ� PendSV ���жϴ�������ȥ����
PendSV_Handler													;��ת������ʱxPSR, PC, LR, R12, R0-R3 ���Զ�����
    CPSID   I                                                   ; Prevent interruption during context switch
    MRS     R0, PSP                                             ; PSP is process stack pointer �������PSP��ջ,����Ժ��Ա���Ĵ���,�ο�CM3Ȩ���е�˫��ջ-�ײ�ע
    CBZ     R0, PendSV_Handler_Nosave		                    ; Skip register save the first time����� PSP == 0����ת��PendSV_Handler_Nosave
																;PSP == 0˵�� OSStartHighRdy()�������һ���������л���������մ���ʱ R4-R11 �Ѿ������ڶ�ջ���ˣ����Բ���Ҫ�ٱ���һ���ˡ�
    SUBS    R0, R0, #0x20                                       ; Save remaining regs r4-11 on process stack��R0 -= 0x20��Ӱ���־λ
    STM     R0, {R4-R11}										;R0����ָ����ջ��������ȥ�Զ������8���Ĵ���ƫ�ƺ󱣴�����ļĴ���ֵ

    LDR     R1, =p_OSTCBCur                                     ; OSTCBCur->OSTCBStkPtr = SP;
    LDR     R1, [R1]											;�����ջ��ָ��
    STR     R0, [R1]                                            ; R0 is SP of process being switched out,������ƿ飬��Ҫ��¼�µĲ�����������Ǽ�¼�˵�ǰ����Ķ�ջָ��

                                                                ; At this point, entire context of process has been saved
PendSV_Handler_Nosave
    PUSH    {R14}                                               ; Save LR exc_return value������ R14����Ϊ����Ҫ���ú���
    LDR     R0, =OSTaskSwHook                                   ; OSTaskSwHook();
    BLX     R0													;���� OSTaskSwHook()
    POP     {R14}												;�ָ� R14

    LDR     R0, =OSPrioCur                                      ; OSPrioCur = OSPrioHighRdy;
    LDR     R1, =OSPrioHighRdy
    LDRB    R2, [R1]
    STRB    R2, [R0]											;OSPrioCur = OSPrioHighRdy��һ���������и�ֵ

    LDR     R0, =p_OSTCBCur                                     ; OSTCBCur  = OSTCBHighRdy;
    LDR     R1, =p_OSTCBHighRdy
    LDR     R2, [R1]
    STR     R2, [R0]											;��һ�ζԱ������и�ֵ��������������ƿ�
																;���濪ʼ��ȡ�µ���������л���
    LDR     R0, [R2]                                            ; R0 is new process SP; SP = OSTCBHighRdy->OSTCBStkPtr;R0��׼�����е������SP
    LDM     R0, {R4-R11}                                        ; Restore r4-11 from new process stack
    ADDS    R0, R0, #0x20										;�м�������8���Ĵ����Ѿ��Զ�������
    MSR     PSP, R0                                             ; Load PSP with new process SP
    ORR     LR, LR, #0x04                                       ; Ensure exception return uses process stack��ȷ�� LR λ 2 Ϊ 1�����غ�ʹ�ý��̶�ջ
    CPSIE   I
    BX      LR                                                  ; Exception return will restore remaining context


;************************************************************
		ALIGN

		END
