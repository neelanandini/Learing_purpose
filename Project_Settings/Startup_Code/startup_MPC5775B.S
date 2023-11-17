/* ---------------------------------------------------------------------------------------*/
/*  @file:    startup_MPC5775B.S                                                          */
/*  @purpose: GNU Compiler Collection Startup File                                        */
/*            MPC5775B                                                                    */
/*  @version: 1.0                                                                         */
/*  @date:    2017-2-9                                                                    */
/*  @build:   #                                                                           */
/* ---------------------------------------------------------------------------------------*/
/*                                                                                        */
/* Copyright 2018-2019 NXP                                                                     */
/* All rights reserved.                                                                   */
/*                                                                                        */
/* THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR                          */
/* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES              */
/* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.                */
/* IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,                    */
/* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES                     */
/* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR                     */
/* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)                     */
/* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,                    */
/* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING                  */
/* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF                         */
/* THE POSSIBILITY OF SUCH DAMAGE.                                                        */
/*****************************************************************************/
/* Version: GNU Compiler Collection                                          */

/* ----------------------------------------------------------------------------
   -- Boot header configuration
   ---------------------------------------------------------------------------- */
#define MPC57xx_ID    0x005A0000
#define VLE_ENABLE    0x01000000
#define RCHW_VAL      (MPC57xx_ID | VLE_ENABLE)

#ifndef I_CACHE
#define I_CACHE
#endif
#ifndef ICACHE_ENABLE
#define ICACHE_ENABLE 0
#endif

#ifndef D_CACHE
#define D_CACHE
#endif
#ifndef DCACHE_ENABLE
#define DCACHE_ENABLE 0
#endif

#ifndef DISABLE_WDOG
#define DISABLE_WDOG  1
#endif
#ifndef SWT_COUNT
#define SWT_COUNT     2
#endif
#ifndef SWT_BASE_ADDR
#define SWT_BASE_ADDR   0xFFF34000
#endif

#if defined(TURN_ON_CPU0)
/* RAppID Boot Data */
    .section .RAppID_AppKey, "a"
    #if defined(__RAPPID_APPKEY__)
    .long __RAPPID_APPKEY__
    #else
    .long 0x55AA55AA
    #endif
    .section .RAppID_AppDelay, "a"
    #if defined(__RAPPID_APPDELAY__)
    .long __RAPPID_APPDELAY__
    #else
    .long 0x00000000
    #endif
    .section .RAppID_AppKeyAddr, "a"
    .long __RAPPID_APP_KEY_ADDR
#endif

/* Init table */
    .section .init_table, "a"
    .long 5
    .long __VECTOR_TABLE
    .long __VECTOR_RAM
    .long __VECTOR_TABLE_COPY_END
    .long __CUSTOM_ROM
    .long __CUSTOM_RAM
    .long __CUSTOM_END
    .long __DATA_ROM
    .long __DATA_RAM
    .long __DATA_END
    .long __SDATA_ROM
    .long __SDATA_RAM
    .long __SDATA_END
    .long __CODE_ROM
    .long __CODE_RAM
    .long __CODE_END
/* Zero table */
    .section .zero_table, "a"
    .long 2
    .long __SBSS_START
    .long __SBSS_END
    .long __BSS_START
    .long __BSS_END

#ifdef TURN_ON_CPU0
    .section .cpu0_reset_vector, "a"
    .long       __start

    .section .rchw, "a"
        .long RCHW_VAL
#endif

    .section .startup, "ax"

    .globl    _start
    .globl    Reset_Handler

Reset_Handler:
_start:
    wrteei 0    ;# Disable interrupts
#if defined(MMU_CONFIG)
;#****************************** MMU configuration ********************************
    e_lis r3, 0x1003
    mtspr 624, r3

    e_lis r4, 0xC000
    e_or2i r4, 0x0480
    mtspr 625, r4

    e_lis r5, 0x4000
    e_or2i r5, 0x0020
    mtspr 626, r5

    e_lis r6, 0x4000
    e_or2i r6, 0x003f
    mtspr 627, r6

    tlbwe

    e_lis r3, 0x1001
    mtspr 624, r3

    e_lis r4, 0xC000
    e_or2i r4, 0x0700
    mtspr 625, r4

    e_lis r5, 0x0000
    e_or2i r5, 0x0020
    mtspr 626, r5

    e_lis r6,0x0000
    e_or2i r6, 0x003f
    mtspr 627, r6

    tlbwe

    e_lis r3, 0x1000
    mtspr 624, r3

    e_lis r4, 0xC000
    e_or2i r4, 0x0580
    mtspr 625, r4

    e_lis r5, 0xFFE0
    e_or2i r5, 0x002A
    mtspr 626, r5

    e_lis r6, 0xFFE0
    e_or2i r6, 0x003f
    mtspr 627, r6

    tlbwe

    e_lis r3, 0x1002
    mtspr 624, r3

    e_lis r4, 0xC000
    e_or2i r4, 0x0700
    mtspr 625, r4

    e_lis r5, 0x2000
    e_or2i r5, 0x0020
    mtspr 626, r5

    e_lis r6,0x0000
    e_or2i r6, 0x003f
    mtspr 627, r6

    tlbwe

    e_lis r3, 0x1004
    mtspr 624, r3

    e_lis r4, 0xC000
    e_or2i r4, 0x0580
    mtspr 625, r4

    e_lis r5, 0xC3E0
    e_or2i r5, 0x002A
    mtspr 626, r5

    e_lis r6, 0xC3E0
    e_or2i r6, 0x003f
    mtspr 627, r6

    tlbwe
#endif

;#*********************** Turn off Core Watchdog Timer*************************
        e_li    r3, 0x0000
        mtspr    340, r3

;#********************************* Enable BTB ********************************
;# Flush & Enable BTB - Set BBFI bit in BUCSR
    e_li      r3, 0x201
    mtspr     1013, r3
    se_isync

;#**************************** Init Core Registers ****************************
;# The E200Z7 core needs its registers initialising before they are used
;# otherwise in Lock Step mode the two cores will contain different random data.
;# If this is stored to memory (e.g. stacked) it will cause a Lock Step error.

;# GPRs 0-31
    e_li    r0, 0
    e_li    r1, 0
    e_li    r2, 0
    e_li    r3, 0
    e_li    r4, 0
    e_li    r5, 0
    e_li    r6, 0
    e_li    r7, 0
    e_li    r8, 0
    e_li    r9, 0
    e_li    r10, 0
    e_li    r11, 0
    e_li    r12, 0
    e_li    r13, 0
    e_li    r14, 0
    e_li    r15, 0
    e_li    r16, 0
    e_li    r17, 0
    e_li    r18, 0
    e_li    r19, 0
    e_li    r20, 0
    e_li    r21, 0
    e_li    r22, 0
    e_li    r23, 0
    e_li    r24, 0
    e_li    r25, 0
    e_li    r26, 0
    e_li    r27, 0
    e_li    r28, 0
    e_li    r29, 0
    e_li    r30, 0
    e_li    r31, 0

;# Init any other CPU register which might be stacked (before being used).

    mtspr    1, r1          ;#XER
    mtcrf    0xFF, r1
    mtspr    CTR, r1
    mtspr    272, r1        ;#SPRG0
    mtspr    273, r1        ;#SPRG1
    mtspr    274, r1        ;#SPRG2
    mtspr    275, r1        ;#SPRG3
    mtspr    58, r1         ;#CSRR0
    mtspr    59, r1         ;#CSRR1
    mtspr    570, r1        ;#MCSRR0
    mtspr    571, r1        ;#MCSRR1
    mtspr    61, r1         ;#DEAR
    mtspr    63, r1         ;#IVPR
    mtspr    256, r1        ;#USPRG0
    mtspr    62, r1         ;#ESR
    mtspr    8, r31         ;#LR

;#***************************** DISABLE WATCHDOG *****************************
#if DISABLE_WDOG
    e_lis   r4, SWT_BASE_ADDR@h   ;# Initialize the base address of SWT_0
    e_or2i  r4, SWT_BASE_ADDR@l

    e_li    r5, SWT_COUNT@l
    mtctr   r5                    ;# Move to counter number of SWT instances

disable_swt:
    e_li    r3, 0xC520
    e_stw   r3, 0x10(r4)          ;# Write the watchdog unlock value 0xc520

    e_li    r3, 0xD928
    e_stw   r3, 0x10(r4)          ;# Write the watchdog unlock value 0xD928

    e_lis   r3, 0xFF00
    e_or2i  r3, 0x010A
    e_stw   r3, 0(r4)             ;# Write reset value with SWT disabled

    e_addi  r4,r4, 0x4000         ;# Increase the pointer to the next instance of SWT
    e_bdnz  disable_swt           ;# Loop for all instance of SWT
#endif

;#***************************** Initialise SRAM ECC **************************
;# Store number of 128Byte (32GPRs) segments in Counter
    e_lis       r5, __SRAM_SIZE@h  # Initialize r5 to size of SRAM (Bytes)
    e_or2i      r5, __SRAM_SIZE@l
    e_srwi      r5, r5, 0x7         ;# Divide SRAM size by 128
    mtctr       r5                  ;# Move to counter for use with "bdnz"

;# Base Address of the internal SRAM
    e_lis       r5, __SRAM_BASE_ADDR@h
    e_or2i      r5, __SRAM_BASE_ADDR@l

;# Fill SRAM with writes of 32GPRs
sram_loop:
    e_stmw      r0, 0(r5)           ;# Write all 32 registers to SRAM
    e_addi      r5, r5, 128         ;# Increment the RAM pointer to next 128bytes
    e_bdnz      sram_loop           ;# Loop for all of SRAM


;#******************** Clear reservations on external interrupt *****************
;# Set ICR in HID0
    e_lis     r3, 0x2
    mtspr     1008, r3
    se_isync

;#*************************** Enable ME Bit in MSR *****************************
    mfmsr    r6
    e_or2i   r6, 0x1000
    mtmsr    r6

#if defined(I_CACHE) && (ICACHE_ENABLE == 1)
;#****************** Invalidate and Enable the Instruction cache **************
__icache_cfg:
    e_li     r5, 0x2
    mtspr    1011, r5

    e_li     r7, 0x4
    e_li     r8, 0x2
    e_lis    r11, 0xFFFF
    e_or2i   r11, 0xFFFB

__icache_inv:
    mfspr    r9, 1011
    and.     r10, r7, r9
    e_beq    __icache_no_abort
    and.     r10, r11, r9
    mtspr    1011, r10
    e_b      __icache_cfg

__icache_no_abort:
    and.     r10, r8, r9
    e_bne    __icache_inv

    mfspr    r5, 1011
    e_ori    r5, r5, 0x0001
    se_isync
    mtspr    1011, r5
#endif

#if defined(D_CACHE) && (DCACHE_ENABLE == 1)
;#****************** Invalidate and Enable the Data cache **************
__dcache_cfg:
    e_li r5, 0x2
    mtspr 1010, r5

    e_li r7, 0x4
    e_li r8, 0x2
    e_lis    r11, 0xFFFF
    e_or2i   r11, 0xFFFB

__dcache_inv:
    mfspr r9, 1010
    and.  r10, r7, r9
    e_beq   __dcache_no_abort
    and.  r10, r11, r9
    mtspr 1010, r10
    e_b __dcache_cfg

__dcache_no_abort:
    and.  r10, r8, r9
    e_bne __dcache_inv

    mfspr r5, 1010
    e_ori   r5, r5, 0x0001
    se_isync
    msync
    mtspr 1010, r5
#endif

;#****************************** Initialize IVORs *****************************
;# Support for which cores do not set the IVORs automatically e.g. e200z759n3.
    e_lis       r0, VTABLE@ha
    e_add16i    r0, r0, VTABLE@l
    e_rlwinm    r0, r0, 0, 16, 27
    mtspr       400, r0                 ;# IVOR0 400 Critical input
    e_add16i    r3, r0, 0x010
    mtspr       401, r3                 ;# IVOR1 401 Machine check
    e_add16i    r3, r0, 0x020
    mtspr       402, r3                 ;# IVOR2 402 Data storage
    e_add16i    r3, r0, 0x030
    mtspr       403, r3                 ;# IVOR3 403 Instruction storage
    e_add16i    r3, r0, 0x040
    mtspr       404, r3                 ;# IVOR4 404 External input
    e_add16i    r3, r0, 0x050
    mtspr       405, r3                 ;# IVOR5 405 Alignment
    e_add16i    r3, r0, 0x060
    mtspr       406, r3                 ;# IVOR7 406 Program
    e_add16i    r3, r0, 0x070
    mtspr       407, r3                 ;# IVOR7 407 Floating-point unavailable
    e_add16i    r3, r0, 0x080
    mtspr       408, r3                 ;# IVOR8 408 System call
    e_add16i    r3, r0, 0x090
    mtspr       409, r3                 ;# IVOR9 409 Auxiliary processor unavailable. Not used by the e200z6.
    e_add16i    r3, r0, 0x0A0
    mtspr       410, r3                 ;# IVOR10 410 Decrementer
    e_add16i    r3, r0, 0x0B0
    mtspr       411, r3                 ;# IVOR11 411 Fixed-interval timer interrupt
    e_add16i    r3, r0, 0x0C0
    mtspr       412, r3                 ;# IVOR12 412 Watchdog timer interrupt
    e_add16i    r3, r0, 0x0D0
    mtspr       413, r3                 ;# IVOR13 413 Data TLB error
    e_add16i    r3, r0, 0x0E0
    mtspr       414, r3                 ;# IVOR14 414 Instruction TLB error
    e_add16i    r3, r0, 0x0F0
    mtspr       415, r3                 ;# IVOR15 415 Debug
;# IVOR16-IVOR31 - Reserved for future architectural use
    e_add16i    r3, r0, 0x100
    mtspr       528, r3                 ;# IVOR32 528 SPE APU unavailable
    e_add16i    r3, r0, 0x110
    mtspr       529, r3                 ;# IVOR33 529 SPE floating-point data exception
    e_add16i    r3, r0, 0x120
    mtspr       530, r3                 ;# IVOR34 530 SPE floating-point round exception
    e_add16i    r3, r0, 0x130
    mtspr       531, r3                 ;# IVOR35 531 Performance Monitor Interrupt

;#****************************** Configure Stack *****************************
    e_lis       r1, __SP_INIT@h         ;# Initialize stack pointer r1 to
    e_or2i      r1, __SP_INIT@l         ;# value in linker command file.

    e_lis       r13, _SDA_BASE_@h       ;# Initialize r13 to sdata base
    e_or2i      r13, _SDA_BASE_@l       ;# (provided by linker).

    e_lis       r2, _SDA2_BASE_@h       ;# Initialize r2 to sdata2 base
    e_or2i      r2, _SDA2_BASE_@l       ;# (provided by linker).

    e_stwu      r0, -64(r1)             ;# Terminate stack.

#ifndef __NO_SYSTEM_INIT
;# Call the system init routine
    e_bl   SystemInit
#endif
;# Init .data and .bss sections
    e_bl   init_data_bss
    wrteei 1    ;# Enable interrupts
;# Call custom init function
;# This symbol must be defined in the ASM preprocessor e.g -DCUSTOM_INIT=my_func, 
;# where my_func is a void function that does not take any parameters and it is defined in the application code
#ifdef CUSTOM_INIT
    e_bl CUSTOM_INIT
#endif

;# Jump to Main
    e_bl        main

    .section .intc_vector_table, "a"
    .align 2
    .globl __isr_vector
__isr_vector:
    .long  SS0_IRQHandler                                  /**< Software setable flag 0 SSCIR0[CLR0] */
    .long  SS1_IRQHandler                                  /**< Software setable flag 1 SSCIR0[CLR1] */
    .long  SS2_IRQHandler                                  /**< Software setable flag 2 SSCIR0[CLR2] */
    .long  SS3_IRQHandler                                  /**< Software setable flag 3 SSCIR0[CLR3] */
    .long  SS4_IRQHandler                                  /**< Software setable flag 4 SSCIR0[CLR4] */
    .long  SS5_IRQHandler                                  /**< Software setable flag 5 SSCIR0[CLR5] */
    .long  SS6_IRQHandler                                  /**< Software setable flag 6 SSCIR0[CLR6] */
    .long  SS7_IRQHandler                                  /**< Software setable flag 7 SSCIR0[CLR7] */
    .long  SWT0_IRQHandler                                 /**< Software Watchdog 0 Interrupt flag */
    .long  FCCU_MISC_IRQHandler                            /**< FCCU ALARM state entry | FCCU CONFIG state watchdog timeout */
    .long  DMA0_Ch0_Ch31_Error_IRQHandler                  /**< eDMA0 channel Error flags 0-31 */
    .long  DMA0_Ch0_IRQHandler                             /**< eDMA0 channel Interrupt 0 */
    .long  DMA0_Ch1_IRQHandler                             /**< eDMA0 channel Interrupt 1 */
    .long  DMA0_Ch2_IRQHandler                             /**< eDMA0 channel Interrupt 2 */
    .long  DMA0_Ch3_IRQHandler                             /**< eDMA0 channel Interrupt 3 */
    .long  DMA0_Ch4_IRQHandler                             /**< eDMA0 channel Interrupt 4 */
    .long  DMA0_Ch5_IRQHandler                             /**< eDMA0 channel Interrupt 5 */
    .long  DMA0_Ch6_IRQHandler                             /**< eDMA0 channel Interrupt 6 */
    .long  DMA0_Ch7_IRQHandler                             /**< eDMA0 channel Interrupt 7 */
    .long  DMA0_Ch8_IRQHandler                             /**< eDMA0 channel Interrupt 8 */
    .long  DMA0_Ch9_IRQHandler                             /**< eDMA0 channel Interrupt 9 */
    .long  DMA0_Ch10_IRQHandler                            /**< eDMA0 channel Interrupt 10 */
    .long  DMA0_Ch11_IRQHandler                            /**< eDMA0 channel Interrupt 11 */
    .long  DMA0_Ch12_IRQHandler                            /**< eDMA0 channel Interrupt 12 */
    .long  DMA0_Ch13_IRQHandler                            /**< eDMA0 channel Interrupt 13 */
    .long  DMA0_Ch14_IRQHandler                            /**< eDMA0 channel Interrupt 14 */
    .long  DMA0_Ch15_IRQHandler                            /**< eDMA0 channel Interrupt 15 */
    .long  DMA0_Ch16_IRQHandler                            /**< eDMA0 channel Interrupt 16 */
    .long  DMA0_Ch17_IRQHandler                            /**< eDMA0 channel Interrupt 17 */
    .long  DMA0_Ch18_IRQHandler                            /**< eDMA0 channel Interrupt 18 */
    .long  DMA0_Ch19_IRQHandler                            /**< eDMA0 channel Interrupt 19 */
    .long  DMA0_Ch20_IRQHandler                            /**< eDMA0 channel Interrupt 20 */
    .long  DMA0_Ch21_IRQHandler                            /**< eDMA0 channel Interrupt 21 */
    .long  DMA0_Ch22_IRQHandler                            /**< eDMA0 channel Interrupt 22 */
    .long  DMA0_Ch23_IRQHandler                            /**< eDMA0 channel Interrupt 23 */
    .long  DMA0_Ch24_IRQHandler                            /**< eDMA0 channel Interrupt 24 */
    .long  DMA0_Ch25_IRQHandler                            /**< eDMA0 channel Interrupt 25 */
    .long  DMA0_Ch26_IRQHandler                            /**< eDMA0 channel Interrupt 26 */
    .long  DMA0_Ch27_IRQHandler                            /**< eDMA0 channel Interrupt 27 */
    .long  DMA0_Ch28_IRQHandler                            /**< eDMA0 channel Interrupt 28 */
    .long  DMA0_Ch29_IRQHandler                            /**< eDMA0 channel Interrupt 29 */
    .long  DMA0_Ch30_IRQHandler                            /**< eDMA0 channel Interrupt 30 */
    .long  DMA0_Ch31_IRQHandler                            /**< eDMA0 channel Interrupt 31 */
    .long  PCS_IRQHandler                                  /**< Progressive Clock Switch Interrupt SIU_PCSIFR[PCSI] */
    .long  PLL_IRQHandler                                  /**< PLL Loss of Lock Flags PLL0_SR[LOLF] | PLL1_SR[LOLF] */
    .long  SIU_OVF_IRQHandler                              /**< SIU combined overrun interrupt requests of the external interrupt Overrun Flags */
    .long  SIU_0_IRQHandler                                /**< SIU External Interrupt Flag 0 SIU_EIISR[EIF0] */
    .long  SIU_1_IRQHandler                                /**< SIU External Interrupt Flag 1 SIU_EIISR[EIF1] */
    .long  SIU_2_IRQHandler                                /**< SIU External Interrupt Flag 2 SIU_EIISR[EIF2] */
    .long  SIU_3_IRQHandler                                /**< SIU External Interrupt Flag 3 SIU_EIISR[EIF3] */
    .long  SIU_4_15_IRQHandler                             /**< SIU External Interrupt Flag 15-4 SIU_EIISR[EIF15:EIF4] */
    .long  EMIOS0_F0_IRQHandler                            /**< eMIOS_0 channel 0 Flag */
    .long  EMIOS0_F1_IRQHandler                            /**< eMIOS_0 channel 1 Flag */
    .long  EMIOS0_F2_IRQHandler                            /**< eMIOS_0 channel 2 Flag */
    .long  EMIOS0_F3_IRQHandler                            /**< eMIOS_0 channel 3 Flag */
    .long  EMIOS0_F4_IRQHandler                            /**< eMIOS_0 channel 4 Flag */
    .long  EMIOS0_F5_IRQHandler                            /**< eMIOS_0 channel 5 Flag */
    .long  EMIOS0_F6_IRQHandler                            /**< eMIOS_0 channel 6 Flag */
    .long  EMIOS0_F7_IRQHandler                            /**< eMIOS_0 channel 7 Flag */
    .long  EMIOS1_F0_IRQHandler                            /**< eMIOS_1 channel 0 Flag */
    .long  EMIOS1_F1_IRQHandler                            /**< eMIOS_1 channel 1 Flag */
    .long  EMIOS1_F2_IRQHandler                            /**< eMIOS_1 channel 2 Flag */
    .long  EMIOS1_F3_IRQHandler                            /**< eMIOS_1 channel 3 Flag */
    .long  EMIOS1_F4_IRQHandler                            /**< eMIOS_1 channel 4 Flag */
    .long  EMIOS1_F5_IRQHandler                            /**< eMIOS_1 channel 5 Flag */
    .long  EMIOS1_F6_IRQHandler                            /**< eMIOS_1 channel 6 Flag */
    .long  EMIOS1_F7_IRQHandler                            /**< eMIOS_1 channel 7 Flag */
    .long  ETPU01_GE_IRQHandler                            /**< eTPU Engine 0 and 1 Global Exception */
    .long  ETPU0_CIS0_IRQHandler                           /**< eTPU Engine 0 Channel 0 Interrupt Status */
    .long  ETPU0_CIS1_IRQHandler                           /**< eTPU Engine 0 Channel 1 Interrupt Status */
    .long  ETPU0_CIS2_IRQHandler                           /**< eTPU Engine 0 Channel 2 Interrupt Status */
    .long  ETPU0_CIS3_IRQHandler                           /**< eTPU Engine 0 Channel 3 Interrupt Status */
    .long  ETPU0_CIS4_IRQHandler                           /**< eTPU Engine 0 Channel 4 Interrupt Status */
    .long  ETPU0_CIS5_IRQHandler                           /**< eTPU Engine 0 Channel 5 Interrupt Status */
    .long  ETPU0_CIS6_IRQHandler                           /**< eTPU Engine 0 Channel 6 Interrupt Status */
    .long  ETPU0_CIS7_IRQHandler                           /**< eTPU Engine 0 Channel 7 Interrupt Status */
    .long  ETPU0_CIS8_IRQHandler                           /**< eTPU Engine 0 Channel 8 Interrupt Status */
    .long  ETPU0_CIS9_IRQHandler                           /**< eTPU Engine 0 Channel 9 Interrupt Status */
    .long  ETPU0_CIS10_IRQHandler                          /**< eTPU Engine 0 Channel 10 Interrupt Status */
    .long  ETPU0_CIS11_IRQHandler                          /**< eTPU Engine 0 Channel 11 Interrupt Status */
    .long  ETPU0_CIS12_IRQHandler                          /**< eTPU Engine 0 Channel 12 Interrupt Status */
    .long  ETPU0_CIS13_IRQHandler                          /**< eTPU Engine 0 Channel 13 Interrupt Status */
    .long  ETPU0_CIS14_IRQHandler                          /**< eTPU Engine 0 Channel 14 Interrupt Status */
    .long  ETPU0_CIS15_IRQHandler                          /**< eTPU Engine 0 Channel 15 Interrupt Status */
    .long  ETPU0_CIS16_IRQHandler                          /**< eTPU Engine 0 Channel 16 Interrupt Status */
    .long  ETPU0_CIS17_IRQHandler                          /**< eTPU Engine 0 Channel 17 Interrupt Status */
    .long  ETPU0_CIS18_IRQHandler                          /**< eTPU Engine 0 Channel 18 Interrupt Status */
    .long  ETPU0_CIS19_IRQHandler                          /**< eTPU Engine 0 Channel 19 Interrupt Status */
    .long  ETPU0_CIS20_IRQHandler                          /**< eTPU Engine 0 Channel 20 Interrupt Status */
    .long  ETPU0_CIS21_IRQHandler                          /**< eTPU Engine 0 Channel 21 Interrupt Status */
    .long  ETPU0_CIS22_IRQHandler                          /**< eTPU Engine 0 Channel 22 Interrupt Status */
    .long  ETPU0_CIS23_IRQHandler                          /**< eTPU Engine 0 Channel 23 Interrupt Status */
    .long  ETPU0_CIS24_IRQHandler                          /**< eTPU Engine 0 Channel 24 Interrupt Status */
    .long  ETPU0_CIS25_IRQHandler                          /**< eTPU Engine 0 Channel 25 Interrupt Status */
    .long  ETPU0_CIS26_IRQHandler                          /**< eTPU Engine 0 Channel 26 Interrupt Status */
    .long  ETPU0_CIS27_IRQHandler                          /**< eTPU Engine 0 Channel 27 Interrupt Status */
    .long  ETPU0_CIS28_IRQHandler                          /**< eTPU Engine 0 Channel 28 Interrupt Status */
    .long  ETPU0_CIS29_IRQHandler                          /**< eTPU Engine 0 Channel 29 Interrupt Status */
    .long  ETPU0_CIS30_IRQHandler                          /**< eTPU Engine 0 Channel 30 Interrupt Status */
    .long  ETPU0_CIS31_IRQHandler                          /**< eTPU Engine 0 Channel 31 Interrupt Status */
    .long  EQADC0_OVRx_IRQHandler                          /**< eQADC combined overrun interrupt requests from all of the FIFOs: Trigger Overrun, Receive FIFO Overflow and command FIFO Underflow */
    .long  EQADC0_FIFO0_NCF_IRQHandler                     /**< eQADC command FIFO 0 Non-Coherency Flag */
    .long  EQADC0_FIFO0_PF_IRQHandler                      /**< eQADC command FIFO 0 Pause Flag */
    .long  EQADC0_FIFO0_EOQF_IRQHandler                    /**< eQADC command FIFO 0 command queue End of Queue Flag */
    .long  EQADC0_FIFO0_CFFF_IRQHandler                    /**< eQADC Command FIFO 0 Fill Flag */
    .long  EQADC0_FIFO0_RFDF_IRQHandler                    /**< eQADC Receive FIFO 0 Drain Flag */
    .long  EQADC0_FIFO1_NCF_IRQHandler                     /**< eQADC command FIFO 1 Non-Coherency Flag */
    .long  EQADC0_FIFO1_PF_IRQHandler                      /**< eQADC command FIFO 1 Pause Flag */
    .long  EQADC0_FIFO1_EOQF_IRQHandler                    /**< eQADC command FIFO 1 command queue End of Queue Flag */
    .long  EQADC0_FIFO1_CFFF_IRQHandler                    /**< eQADC Command FIFO 1 Fill Flag */
    .long  EQADC0_FIFO1_RFDF_IRQHandler                    /**< eQADC Receive FIFO 1 Drain Flag */
    .long  EQADC0_FIFO2_NCF_IRQHandler                     /**< eQADC command FIFO 2 Non-Coherency Flag */
    .long  EQADC0_FIFO2_PF_IRQHandler                      /**< eQADC command FIFO 2 Pause Flag */
    .long  EQADC0_FIFO2_EOQF_IRQHandler                    /**< eQADC command FIFO 2 command queue End of Queue Flag */
    .long  EQADC0_FIFO2_CFFF_IRQHandler                    /**< eQADC Command FIFO 2 Fill Flag */
    .long  EQADC0_FIFO2_RFDF_IRQHandler                    /**< eQADC Receive FIFO 2 Drain Flag */
    .long  EQADC0_FIFO3_NCF_IRQHandler                     /**< eQADC command FIFO 3 Non-Coherency Flag */
    .long  EQADC0_FIFO3_PF_IRQHandler                      /**< eQADC command FIFO 3 Pause Flag */
    .long  EQADC0_FIFO3_EOQF_IRQHandler                    /**< eQADC command FIFO 3 command queue End of Queue Flag */
    .long  EQADC0_FIFO3_CFFF_IRQHandler                    /**< eQADC Command FIFO 3 Fill Flag */
    .long  EQADC0_FIFO3_RFDF_IRQHandler                    /**< eQADC Receive FIFO 3 Drain Flag */
    .long  EQADC0_FIFO4_NCF_IRQHandler                     /**< eQADC command FIFO 4 Non-Coherency Flag */
    .long  EQADC0_FIFO4_PF_IRQHandler                      /**< eQADC command FIFO 4 Pause Flag */
    .long  EQADC0_FIFO4_EOQF_IRQHandler                    /**< eQADC command FIFO 4 command queue End of Queue Flag */
    .long  EQADC0_FIFO4_CFFF_IRQHandler                    /**< eQADC Command FIFO 4 Fill Flag */
    .long  EQADC0_FIFO4_RFDF_IRQHandler                    /**< eQADC Receive FIFO 4 Drain Flag */
    .long  EQADC0_FIFO5_NCF_IRQHandler                     /**< eQADC command FIFO 5 Non-Coherency Flag */
    .long  EQADC0_FIFO5_PF_IRQHandler                      /**< eQADC command FIFO 5 Pause Flag */
    .long  EQADC0_FIFO5_EOQF_IRQHandler                    /**< eQADC command FIFO 5 command queue End of Queue Flag */
    .long  EQADC0_FIFO5_CFFF_IRQHandler                    /**< eQADC Command FIFO 5 Fill Flag */
    .long  EQADC0_FIFO5_RFDF_IRQHandler                    /**< eQADC Receive FIFO 5 Drain Flag */
    .long  DSPI1_FIFO_Error_IRQHandler                     /**< DSPI_1 combined overrun and parity error interrupt requests: Transmit FIFO Underflow/Receive FIFO Overflow SPI Parity Error/DSI Parity Error */
    .long  DSPI1_TXFIFO_EOQF_IRQHandler                    /**< DSPI_1 transmit FIFO End of Queue Flag */
    .long  DSPI1_Send_IRQHandler                           /**< DSPI_1 Transmit FIFO Fill Flag */
    .long  DSPI1_TCF_IRQHandler                            /**< DSPI_1 Transfer Complete/DSI Data Match Flag */
    .long  DSPI1_Receive_IRQHandler                        /**< DSPI_1 Receive FIFO Drain Flag */
    .long  DSPI2_FIFO_Error_IRQHandler                     /**< DSPI_2 combined overrun and parity error interrupt requests: Transmit FIFO Underflow/Receive FIFO Overflow SPI Parity Error/DSI Parity Error */
    .long  DSPI2_TXFIFO_EOQF_IRQHandler                    /**< DSPI_2 transmit FIFO End of Queue Flag */
    .long  DSPI2_Send_IRQHandler                           /**< DSPI_2 Transmit FIFO Fill Flag */
    .long  DSPI2_TCF_IRQHandler                            /**< DSPI_2 Transfer Complete/DSI Data Match Flag */
    .long  DSPI2_Receive_IRQHandler                        /**< DSPI_2 Receive FIFO Drain Flag */
    .long  DSPI3_FIFO_Error_IRQHandler                     /**< DSPI_3 combined overrun and parity error interrupt requests: Transmit FIFO Underflow/Receive FIFO Overflow SPI Parity Error/DSI Parity Error */
    .long  DSPI3_TXFIFO_EOQF_IRQHandler                    /**< DSPI_3 transmit FIFO End of Queue Flag */
    .long  DSPI3_Send_IRQHandler                           /**< DSPI_3 Transmit FIFO Fill Flag */
    .long  DSPI3_TCF_IRQHandler                            /**< DSPI_3 Transfer Complete/DSI Data Match Flag */
    .long  DSPI3_Receive_IRQHandler                        /**< DSPI_3 Receive FIFO Drain Flag */
    .long  ESCI0_CIR_IRQHandler                            /**< Combined Interrupt Requests of ESCI Module 0 */
    .long  PCU_MASTER0_IRQHandler                          /**< PCU_IR0[OIF] | PCU_IR0[EIF] */
    .long  PCU_MASTER1_IRQHandler                          /**< PCU_IR1[OIF] | PCU_IR1[EIF] */
    .long  ESCI1_CIR_IRQHandler                            /**< Combined Interrupt Requests of ESCI Module 1 */
    .long  PSI50_SDOE_IRQHandler                           /**< PSI5_0 DMA Status, New data, OverWrite, Error interrupts */
    .long  PSI51_SDOE_IRQHandler                           /**< PSI5_1 DMA Status, New data, OverWrite, Error interrupts */
    .long  CAN0_ESR1_IRQHandler                            /**< FlexCAN_0 Bus Off, Transmit Warning, Receive Warning */
    .long  CAN0_ESR2_IRQHandler                            /**< FlexCAN_0 Error, FlexCAN_0 ECC Correctable Error, FlexCAN_0 ECC Host Access Non-Correctable Error, FlexCAN_0 ECC CAN Access Non-Correctable Error */
    .long  DefaultISR                         
    .long  CAN0_BUF0_IRQHandler                            /**< FlexCAN_0 Buffer 0 Interrupt */
    .long  CAN0_BUF1_IRQHandler                            /**< FlexCAN_0 Buffer 1 Interrupt */
    .long  CAN0_BUF2_IRQHandler                            /**< FlexCAN_0 Buffer 2 Interrupt */
    .long  CAN0_BUF3_IRQHandler                            /**< FlexCAN_0 Buffer 3 Interrupt */
    .long  CAN0_BUF4_IRQHandler                            /**< FlexCAN_0 Buffer 4 Interrupt */
    .long  CAN0_BUF5_IRQHandler                            /**< FlexCAN_0 Buffer 5 Interrupt */
    .long  CAN0_BUF6_IRQHandler                            /**< FlexCAN_0 Buffer 6 Interrupt */
    .long  CAN0_BUF7_IRQHandler                            /**< FlexCAN_0 Buffer 7 Interrupt */
    .long  CAN0_BUF8_IRQHandler                            /**< FlexCAN_0 Buffer 8 Interrupt */
    .long  CAN0_BUF9_IRQHandler                            /**< FlexCAN_0 Buffer 9 Interrupt */
    .long  CAN0_BUF10_IRQHandler                           /**< FlexCAN_0 Buffer 10 Interrupt */
    .long  CAN0_BUF11_IRQHandler                           /**< FlexCAN_0 Buffer 11 Interrupt */
    .long  CAN0_BUF12_IRQHandler                           /**< FlexCAN_0 Buffer 12 Interrupt */
    .long  CAN0_BUF13_IRQHandler                           /**< FlexCAN_0 Buffer 13 Interrupt */
    .long  CAN0_BUF14_IRQHandler                           /**< FlexCAN_0 Buffer 14 Interrupt */
    .long  CAN0_BUF15_IRQHandler                           /**< FlexCAN_0 Buffer 15 Interrupt */
    .long  CAN0_BUF16_31_IRQHandler                        /**< FlexCAN_0 Buffers 31-16 Interrupts */
    .long  CAN0_BUF32_63_IRQHandler                        /**< FlexCAN_0 Buffers 63-32 Interrupts */
    .long  CAN2_ESR1_IRQHandler                            /**< FlexCAN_2 Bus Off, Transmit Warning, Receive Warning */
    .long  CAN2_ESR2_IRQHandler                            /**< FlexCAN_2 Error, FlexCAN_2 ECC Correctable Error, FlexCAN_2 ECC Host Access Non-Correctable Error, FlexCAN_2 ECC CAN Access Non-Correctable Error */
    .long  DefaultISR                         
    .long  CAN2_BUF0_IRQHandler                            /**< FlexCAN_2 Buffer 0 Interrupt */
    .long  CAN2_BUF1_IRQHandler                            /**< FlexCAN_2 Buffer 1 Interrupt */
    .long  CAN2_BUF2_IRQHandler                            /**< FlexCAN_2 Buffer 2 Interrupt */
    .long  CAN2_BUF3_IRQHandler                            /**< FlexCAN_2 Buffer 3 Interrupt */
    .long  CAN2_BUF4_IRQHandler                            /**< FlexCAN_2 Buffer 4 Interrupt */
    .long  CAN2_BUF5_IRQHandler                            /**< FlexCAN_2 Buffer 5 Interrupt */
    .long  CAN2_BUF6_IRQHandler                            /**< FlexCAN_2 Buffer 6 Interrupt */
    .long  CAN2_BUF7_IRQHandler                            /**< FlexCAN_2 Buffer 7 Interrupt */
    .long  CAN2_BUF8_IRQHandler                            /**< FlexCAN_2 Buffer 8 Interrupt */
    .long  CAN2_BUF9_IRQHandler                            /**< FlexCAN_2 Buffer 9 Interrupt */
    .long  CAN2_BUF10_IRQHandler                           /**< FlexCAN_2 Buffer 10 Interrupt */
    .long  CAN2_BUF11_IRQHandler                           /**< FlexCAN_2 Buffer 11 Interrupt */
    .long  CAN2_BUF12_IRQHandler                           /**< FlexCAN_2 Buffer 12 Interrupt */
    .long  CAN2_BUF13_IRQHandler                           /**< FlexCAN_2 Buffer 13 Interrupt */
    .long  CAN2_BUF14_IRQHandler                           /**< FlexCAN_2 Buffer 14 Interrupt */
    .long  CAN2_BUF15_IRQHandler                           /**< FlexCAN_2 Buffer 15 Interrupt */
    .long  CAN2_BUF16_31_IRQHandler                        /**< FlexCAN_2 Buffers 31-16 Interrupts */
    .long  CAN2_BUF32_63_IRQHandler                        /**< FlexCAN_2 Buffers 63-32 Interrupts */
    .long  FEC_TXF_IRQHandler                              /**< FEC Transmit Frame flag */
    .long  FEC_RXF_IRQHandler                              /**< FEC Receive Frame flag */
    .long  FEC_ERR_IRQHandler                              /**< Combined Interrupt Requests of the FEC Ethernet Interrupt Event Register */
    .long  DEC0_IDF_IRQHandler                             /**< Decimation 0 Input (Fill) */
    .long  DEC0_OD_SD_IRQHandler                           /**< Decimation 0 Output/Integ (Drain/Integ) */
    .long  DEC0_ERR_IRQHandler                             /**< Decimation 0 Error */
    .long  STM_CIR0_IRQHandler                             /**< System Timer Module Interrupt 0 */
    .long  STM_CIR123_IRQHandler                           /**< System Timer Module Interrupts 1, 2, 3 */
    .long  EMIOS0_CH16_IRQHandler                          /**< eMIOS_0 channel 16 Flag */
    .long  EMIOS0_CH17_IRQHandler                          /**< eMIOS_0 channel 17 Flag */
    .long  EMIOS0_CH18_IRQHandler                          /**< eMIOS_0 channel 18 Flag */
    .long  EMIOS0_CH19_IRQHandler                          /**< eMIOS_0 channel 19 Flag */
    .long  EMIOS0_CH20_IRQHandler                          /**< eMIOS_0 channel 20 Flag */
    .long  EMIOS0_CH21_IRQHandler                          /**< eMIOS_0 channel 21 Flag */
    .long  EMIOS0_CH22_IRQHandler                          /**< eMIOS_0 channel 22 Flag */
    .long  EMIOS0_CH23_IRQHandler                          /**< eMIOS_0 channel 23 Flag */
    .long  DMA0_Ch32_Ch63_Error_IRQHandler                 /**< eDMA0 channel Error flags 32-63 */
    .long  DMA0_Ch32_IRQHandler                            /**< eDMA0 channel Interrupt 32 */
    .long  DMA0_Ch33_IRQHandler                            /**< eDMA0 channel Interrupt 33 */
    .long  DMA0_Ch34_IRQHandler                            /**< eDMA0 channel Interrupt 34 */
    .long  DMA0_Ch35_IRQHandler                            /**< eDMA0 channel Interrupt 35 */
    .long  DMA0_Ch36_IRQHandler                            /**< eDMA0 channel Interrupt 36 */
    .long  DMA0_Ch37_IRQHandler                            /**< eDMA0 channel Interrupt 37 */
    .long  DMA0_Ch38_IRQHandler                            /**< eDMA0 channel Interrupt 38 */
    .long  DMA0_Ch39_IRQHandler                            /**< eDMA0 channel Interrupt 39 */
    .long  DMA0_Ch40_IRQHandler                            /**< eDMA0 channel Interrupt 40 */
    .long  DMA0_Ch41_IRQHandler                            /**< eDMA0 channel Interrupt 41 */
    .long  DMA0_Ch42_IRQHandler                            /**< eDMA0 channel Interrupt 42 */
    .long  DMA0_Ch43_IRQHandler                            /**< eDMA0 channel Interrupt 43 */
    .long  DMA0_Ch44_IRQHandler                            /**< eDMA0 channel Interrupt 44 */
    .long  DMA0_Ch45_IRQHandler                            /**< eDMA0 channel Interrupt 45 */
    .long  DMA0_Ch46_IRQHandler                            /**< eDMA0 channel Interrupt 46 */
    .long  DMA0_Ch47_IRQHandler                            /**< eDMA0 channel Interrupt 47 */
    .long  DMA0_Ch48_IRQHandler                            /**< eDMA0 channel Interrupt 48 */
    .long  DMA0_Ch49_IRQHandler                            /**< eDMA0 channel Interrupt 49 */
    .long  DMA0_Ch50_IRQHandler                            /**< eDMA0 channel Interrupt 50 */
    .long  DMA0_Ch51_IRQHandler                            /**< eDMA0 channel Interrupt 51 */
    .long  DMA0_Ch52_IRQHandler                            /**< eDMA0 channel Interrupt 52 */
    .long  DMA0_Ch53_IRQHandler                            /**< eDMA0 channel Interrupt 53 */
    .long  DMA0_Ch54_IRQHandler                            /**< eDMA0 channel Interrupt 54 */
    .long  DMA0_Ch55_IRQHandler                            /**< eDMA0 channel Interrupt 55 */
    .long  DMA0_Ch56_IRQHandler                            /**< eDMA0 channel Interrupt 56 */
    .long  DMA0_Ch57_IRQHandler                            /**< eDMA0 channel Interrupt 57 */
    .long  DMA0_Ch58_IRQHandler                            /**< eDMA0 channel Interrupt 58 */
    .long  DMA0_Ch59_IRQHandler                            /**< eDMA0 channel Interrupt 59 */
    .long  DMA0_Ch60_IRQHandler                            /**< eDMA0 channel Interrupt 60 */
    .long  DMA0_Ch61_IRQHandler                            /**< eDMA0 channel Interrupt 61 */
    .long  DMA0_Ch62_IRQHandler                            /**< eDMA0 channel Interrupt 62 */
    .long  DMA0_Ch63_IRQHandler                            /**< eDMA0 channel Interrupt 63 */
    .long  ETPU1_CIS0_IRQHandler                           /**< eTPU Engine 1 Channel 0 Interrupt Status */
    .long  ETPU1_CIS1_IRQHandler                           /**< eTPU Engine 1 Channel 1 Interrupt Status */
    .long  ETPU1_CIS2_IRQHandler                           /**< eTPU Engine 1 Channel 2 Interrupt Status */
    .long  ETPU1_CIS3_IRQHandler                           /**< eTPU Engine 1 Channel 3 Interrupt Status */
    .long  ETPU1_CIS4_IRQHandler                           /**< eTPU Engine 1 Channel 4 Interrupt Status */
    .long  ETPU1_CIS5_IRQHandler                           /**< eTPU Engine 1 Channel 5 Interrupt Status */
    .long  ETPU1_CIS6_IRQHandler                           /**< eTPU Engine 1 Channel 6 Interrupt Status */
    .long  ETPU1_CIS7_IRQHandler                           /**< eTPU Engine 1 Channel 7 Interrupt Status */
    .long  ETPU1_CIS8_IRQHandler                           /**< eTPU Engine 1 Channel 8 Interrupt Status */
    .long  ETPU1_CIS9_IRQHandler                           /**< eTPU Engine 1 Channel 9 Interrupt Status */
    .long  ETPU1_CIS10_IRQHandler                          /**< eTPU Engine 1 Channel 10 Interrupt Status */
    .long  ETPU1_CIS11_IRQHandler                          /**< eTPU Engine 1 Channel 11 Interrupt Status */
    .long  ETPU1_CIS12_IRQHandler                          /**< eTPU Engine 1 Channel 12 Interrupt Status */
    .long  ETPU1_CIS13_IRQHandler                          /**< eTPU Engine 1 Channel 13 Interrupt Status */
    .long  ETPU1_CIS14_IRQHandler                          /**< eTPU Engine 1 Channel 14 Interrupt Status */
    .long  ETPU1_CIS15_IRQHandler                          /**< eTPU Engine 1 Channel 15 Interrupt Status */
    .long  ETPU1_CIS16_IRQHandler                          /**< eTPU Engine 1 Channel 16 Interrupt Status */
    .long  ETPU1_CIS17_IRQHandler                          /**< eTPU Engine 1 Channel 17 Interrupt Status */
    .long  ETPU1_CIS18_IRQHandler                          /**< eTPU Engine 1 Channel 18 Interrupt Status */
    .long  ETPU1_CIS19_IRQHandler                          /**< eTPU Engine 1 Channel 19 Interrupt Status */
    .long  ETPU1_CIS20_IRQHandler                          /**< eTPU Engine 1 Channel 20 Interrupt Status */
    .long  ETPU1_CIS21_IRQHandler                          /**< eTPU Engine 1 Channel 21 Interrupt Status */
    .long  ETPU1_CIS22_IRQHandler                          /**< eTPU Engine 1 Channel 22 Interrupt Status */
    .long  ETPU1_CIS23_IRQHandler                          /**< eTPU Engine 1 Channel 23 Interrupt Status */
    .long  ETPU1_CIS24_IRQHandler                          /**< eTPU Engine 1 Channel 24 Interrupt Status */
    .long  ETPU1_CIS25_IRQHandler                          /**< eTPU Engine 1 Channel 25 Interrupt Status */
    .long  ETPU1_CIS26_IRQHandler                          /**< eTPU Engine 1 Channel 26 Interrupt Status */
    .long  ETPU1_CIS27_IRQHandler                          /**< eTPU Engine 1 Channel 27 Interrupt Status */
    .long  ETPU1_CIS28_IRQHandler                          /**< eTPU Engine 1 Channel 28 Interrupt Status */
    .long  ETPU1_CIS29_IRQHandler                          /**< eTPU Engine 1 Channel 29 Interrupt Status */
    .long  ETPU1_CIS30_IRQHandler                          /**< eTPU Engine 1 Channel 30 Interrupt Status */
    .long  ETPU1_CIS31_IRQHandler                          /**< eTPU Engine 1 Channel 31 Interrupt Status */
    .long  DSPI0_FIFO_Error_IRQHandler                     /**< DSPI_0 combined overrun and parity error interrupt requests: Transmit FIFO Underflow/Receive FIFO Overflow SPI Parity Error/DSI Parity Error */
    .long  DSPI0_TXFIFO_EOQF_IRQHandler                    /**< DSPI_0 transmit FIFO End of Queue Flag */
    .long  DSPI0_Send_IRQHandler                           /**< DSPI_0 Transmit FIFO Fill Flag */
    .long  DSPI0_TCF_IRQHandler                            /**< DSPI_0 Transfer Complete/DSI Data Match Flag */
    .long  DSPI0_Receive_IRQHandler                        /**< DSPI_0 Receive FIFO Drain Flag */
    .long  CAN1_ESR1_IRQHandler                            /**< FlexCAN_1 Bus Off, Transmit Warning, Receive Warning */
    .long  CAN1_ESR2_IRQHandler                            /**< FlexCAN_1 Error, FlexCAN_1 ECC Correctable Error, FlexCAN_1 ECC Host Access Non-Correctable Error, FlexCAN_1 ECC CAN Access Non-Correctable Error */
    .long  DefaultISR                         
    .long  CAN1_BUF0_IRQHandler                            /**< FlexCAN_1 Buffer 0 Interrupt */
    .long  CAN1_BUF1_IRQHandler                            /**< FlexCAN_1 Buffer 1 Interrupt */
    .long  CAN1_BUF2_IRQHandler                            /**< FlexCAN_1 Buffer 2 Interrupt */
    .long  CAN1_BUF3_IRQHandler                            /**< FlexCAN_1 Buffer 3 Interrupt */
    .long  CAN1_BUF4_IRQHandler                            /**< FlexCAN_1 Buffer 4 Interrupt */
    .long  CAN1_BUF5_IRQHandler                            /**< FlexCAN_1 Buffer 5 Interrupt */
    .long  CAN1_BUF6_IRQHandler                            /**< FlexCAN_1 Buffer 6 Interrupt */
    .long  CAN1_BUF7_IRQHandler                            /**< FlexCAN_1 Buffer 7 Interrupt */
    .long  CAN1_BUF8_IRQHandler                            /**< FlexCAN_1 Buffer 8 Interrupt */
    .long  CAN1_BUF9_IRQHandler                            /**< FlexCAN_1 Buffer 9 Interrupt */
    .long  CAN1_BUF10_IRQHandler                           /**< FlexCAN_1 Buffer 10 Interrupt */
    .long  CAN1_BUF11_IRQHandler                           /**< FlexCAN_1 Buffer 11 Interrupt */
    .long  CAN1_BUF12_IRQHandler                           /**< FlexCAN_1 Buffer 12 Interrupt */
    .long  CAN1_BUF13_IRQHandler                           /**< FlexCAN_1 Buffer 13 Interrupt */
    .long  CAN1_BUF14_IRQHandler                           /**< FlexCAN_1 Buffer 14 Interrupt */
    .long  CAN1_BUF15_IRQHandler                           /**< FlexCAN_1 Buffer 15 Interrupt */
    .long  CAN1_BUF16_31_IRQHandler                        /**< FlexCAN_1 Buffers 31-16 Interrupts */
    .long  CAN1_BUF32_63_IRQHandler                        /**< FlexCAN_1 Buffers 63-32 Interrupts */
    .long  PIT_RTI0_IRQHandler                             /**< Periodic Interrupt Timer Interrupt 0 */
    .long  PIT_RTI1_IRQHandler                             /**< Periodic Interrupt Timer Interrupt 1 */
    .long  PIT_RTI2_IRQHandler                             /**< Periodic Interrupt Timer Interrupt 2 */
    .long  PIT_RTI3_IRQHandler                             /**< Periodic Interrupt Timer Interrupt 3 */
    .long  PIT_RTIINT_IRQHandler                           /**< Real Time Interrupt Interrupt */
    .long  DefaultISR                         
    .long  FMC_Done_IRQHandler                             /**< Flash memory program/erase complete */
    .long  CAN3_ESR1_IRQHandler                            /**< FlexCAN_3 Bus Off, Transmit Warning, Receive Warning */
    .long  CAN3_ESR2_IRQHandler                            /**< FlexCAN_3 Error, FlexCAN_3 ECC Correctable Error, FlexCAN_3 ECC Host Access Non-Correctable Error, FlexCAN_3 ECC CAN Access Non-Correctable Error */
    .long  DefaultISR                         
    .long  CAN3_BUF0_IRQHandler                            /**< FlexCAN_3 Buffer 0 Interrupt */
    .long  CAN3_BUF1_IRQHandler                            /**< FlexCAN_3 Buffer 1 Interrupt */
    .long  CAN3_BUF2_IRQHandler                            /**< FlexCAN_3 Buffer 2 Interrupt */
    .long  CAN3_BUF3_IRQHandler                            /**< FlexCAN_3 Buffer 3 Interrupt */
    .long  CAN3_BUF4_IRQHandler                            /**< FlexCAN_3 Buffer 4 Interrupt */
    .long  CAN3_BUF5_IRQHandler                            /**< FlexCAN_3 Buffer 5 Interrupt */
    .long  CAN3_BUF6_IRQHandler                            /**< FlexCAN_3 Buffer 6 Interrupt */
    .long  CAN3_BUF7_IRQHandler                            /**< FlexCAN_3 Buffer 7 Interrupt */
    .long  CAN3_BUF8_IRQHandler                            /**< FlexCAN_3 Buffer 8 Interrupt */
    .long  CAN3_BUF9_IRQHandler                            /**< FlexCAN_3 Buffer 9 Interrupt */
    .long  CAN3_BUF10_IRQHandler                           /**< FlexCAN_3 Buffer 10 Interrupt */
    .long  CAN3_BUF11_IRQHandler                           /**< FlexCAN_3 Buffer 11 Interrupt */
    .long  CAN3_BUF12_IRQHandler                           /**< FlexCAN_3 Buffer 12 Interrupt */
    .long  CAN3_BUF13_IRQHandler                           /**< FlexCAN_3 Buffer 13 Interrupt */
    .long  CAN3_BUF14_IRQHandler                           /**< FlexCAN_3 Buffer 14 Interrupt */
    .long  CAN3_BUF15_IRQHandler                           /**< FlexCAN_3 Buffer 15 Interrupt */
    .long  CAN3_BUF16_31_IRQHandler                        /**< FlexCAN_3 Buffers 31-16 Interrupts */
    .long  CAN3_BUF32_63_IRQHandler                        /**< FlexCAN_3 Buffers 63-32 Interrupts */
    .long  SRX0_GBL_STATUS_IRQHandler                      /**< SENT_0 Module Interrupts */
    .long  SRX0_CH0_IRQHandler                             /**< SENT_0_CH0 Interrupts */
    .long  SRX0_CH1_IRQHandler                             /**< SENT_0_CH1 Interrupts */
    .long  SRX0_CH2_IRQHandler                             /**< SENT_0_CH2 Interrupts */
    .long  SRX0_CH3_IRQHandler                             /**< SENT_0_CH3 Interrupts */
    .long  SRX0_CH4_IRQHandler                             /**< SENT_0_CH4 Interrupts */
    .long  SRX0_CH5_IRQHandler                             /**< SENT_0_CH5 Interrupts */
    .long  SRX1_GBL_STATUS_IRQHandler                      /**< SENT_1 Module Interrupts */
    .long  SRX1_CH0_IRQHandler                             /**< SENT_1_CH0 Interrupts */
    .long  SRX1_CH1_IRQHandler                             /**< SENT_1_CH1 Interrupts */
    .long  SRX1_CH2_IRQHandler                             /**< SENT_1_CH2 Interrupts */
    .long  SRX1_CH3_IRQHandler                             /**< SENT_1_CH3 Interrupts */
    .long  SRX1_CH4_IRQHandler                             /**< SENT_1_CH4 Interrupts */
    .long  SRX1_CH5_IRQHandler                             /**< SENT_1_CH5 Interrupts */
    .long  PMC_IRQHandler                                  /**< Power Management Controller Interrupts */
    .long  PMC_TEMP_IRQHandler                             /**< Temperature Sensor Interrupts: TEMP0_0, TEMP0_2, TEMP0_3, TEMP1_0, TEMP1_2, TEMP1_3 of PMC_ESR_TD */
    .long  JDC_IRQHandler                                  /**< JDC Interrupts: JDC_MSR[JIN_INT] | JDC_MSR[JOUT_INT] */
    .long  SIPI0_IRQHandler                                /**< SIPI Combined Interrupts: SIPI_ERR | SIPI_SR | SIPI_CSR0 */
    .long  LFAST_IRQHandler                                /**< LFAST Combined Interrupts */
    .long  MCAN_IRQHandler                                 /**< M_CAN0_0, M_CAN0_1, M_CAN1_0, M_CAN1_1 Combined Interrupts */
    .long  ERM_IRQHandler                                  /**< ERM Combined Interrupts: Single bit Correction | Multi bit Detection */
    .long  CMU01_IRQHandler                                /**< CMU_0, CMU_1 Clock Error Interrupts */
    .long  CMU23_IRQHandler                                /**< CMU_2, CMU_3 Clock Error Interrupts */
    .long  CMU45_IRQHandler                                /**< CMU_4, CMU_5 Clock Error Interrupts */
    .long  CMU67_IRQHandler                                /**< CMU_6, CMU_7 Clock Error Interrupts */
    .long  CMU8_IRQHandler                                 /**< CMU_8 Clock Error Interrupts */
    .long  CMU_RSV0_IRQHandler                             /**< CMU_RSV0 */
    .long  CMU_RSV1_IRQHandler                             /**< CMU_RSV1 */
    .long  CMU_RSV2_IRQHandler                             /**< CMU_RSV2 */
    .long  REACM_GBL_IRQHandler                            /**< Reaction Module Global Interrupt: REACM_GEFR[OVR|EF7:0] */
    .long  REACM_CH01_IRQHandler                           /**< Reaction Channel 0 and Reaction Channel 1 Combined Interrupts */
    .long  REACM_CH23_IRQHandler                           /**< Reaction Channel 2 and Reaction Channel 3 Combined Interrupts */
    .long  REACM_CH45_IRQHandler                           /**< Reaction Channel 4 and Reaction Channel 5 Combined Interrupts */
    .long  REACM_CH67_IRQHandler                           /**< Reaction Channel 6 and Reaction Channel 7 Combined Interrupts */
    .long  REACM_CH89_IRQHandler                           /**< Reaction Channel 8 and Reaction Channel 9 Combined Interrupts */
    .long  REACM_RSV0_IRQHandler                           /**< REACM_RSV0 */
    .long  REACM_RSV1_IRQHandler                           /**< REACM_RSV1 */
    .long  DEC1_IDF_IRQHandler                             /**< Decimation 1 Input (Fill) */
    .long  DEC1_OD_SD_IRQHandler                           /**< Decimation 1 Output/Integ (Drain/Integ) */
    .long  DEC1_ERR_IRQHandler                             /**< Decimation 1 Error */
    .long  ETPU2_GE_IRQHandler                             /**< eTPU Engine 2 and 1 Global Exception */
    .long  ETPU2_CIS0_IRQHandler                           /**< eTPU Engine 2 Channel 0 Interrupt Status */
    .long  ETPU2_CIS1_IRQHandler                           /**< eTPU Engine 2 Channel 1 Interrupt Status */
    .long  ETPU2_CIS2_IRQHandler                           /**< eTPU Engine 2 Channel 2 Interrupt Status */
    .long  ETPU2_CIS3_IRQHandler                           /**< eTPU Engine 2 Channel 3 Interrupt Status */
    .long  ETPU2_CIS4_IRQHandler                           /**< eTPU Engine 2 Channel 4 Interrupt Status */
    .long  ETPU2_CIS5_IRQHandler                           /**< eTPU Engine 2 Channel 5 Interrupt Status */
    .long  ETPU2_CIS6_IRQHandler                           /**< eTPU Engine 2 Channel 6 Interrupt Status */
    .long  ETPU2_CIS7_IRQHandler                           /**< eTPU Engine 2 Channel 7 Interrupt Status */
    .long  ETPU2_CIS8_IRQHandler                           /**< eTPU Engine 2 Channel 8 Interrupt Status */
    .long  ETPU2_CIS9_IRQHandler                           /**< eTPU Engine 2 Channel 9 Interrupt Status */
    .long  ETPU2_CIS10_IRQHandler                          /**< eTPU Engine 2 Channel 10 Interrupt Status */
    .long  ETPU2_CIS11_IRQHandler                          /**< eTPU Engine 2 Channel 11 Interrupt Status */
    .long  ETPU2_CIS12_IRQHandler                          /**< eTPU Engine 2 Channel 12 Interrupt Status */
    .long  ETPU2_CIS13_IRQHandler                          /**< eTPU Engine 2 Channel 13 Interrupt Status */
    .long  ETPU2_CIS14_IRQHandler                          /**< eTPU Engine 2 Channel 14 Interrupt Status */
    .long  ETPU2_CIS15_IRQHandler                          /**< eTPU Engine 2 Channel 15 Interrupt Status */
    .long  ETPU2_CIS16_IRQHandler                          /**< eTPU Engine 2 Channel 16 Interrupt Status */
    .long  ETPU2_CIS17_IRQHandler                          /**< eTPU Engine 2 Channel 17 Interrupt Status */
    .long  ETPU2_CIS18_IRQHandler                          /**< eTPU Engine 2 Channel 18 Interrupt Status */
    .long  ETPU2_CIS19_IRQHandler                          /**< eTPU Engine 2 Channel 19 Interrupt Status */
    .long  ETPU2_CIS20_IRQHandler                          /**< eTPU Engine 2 Channel 20 Interrupt Status */
    .long  ETPU2_CIS21_IRQHandler                          /**< eTPU Engine 2 Channel 21 Interrupt Status */
    .long  ETPU2_CIS22_IRQHandler                          /**< eTPU Engine 2 Channel 22 Interrupt Status */
    .long  ETPU2_CIS23_IRQHandler                          /**< eTPU Engine 2 Channel 23 Interrupt Status */
    .long  EQADC1_OVRx_IRQHandler                          /**< eQADC combined overrun interrupt requests from all of the FIFOs: Trigger Overrun, Receive FIFO Overflow and command FIFO Underflow */
    .long  EQADC1_FIFO0_NCF_IRQHandler                     /**< eQADC command FIFO 0 Non-Coherency Flag */
    .long  EQADC1_FIFO0_PF_IRQHandler                      /**< eQADC command FIFO 0 Pause Flag */
    .long  EQADC1_FIFO0_EOQF_IRQHandler                    /**< eQADC command FIFO 0 command queue End of Queue Flag */
    .long  EQADC1_FIFO0_CFFF_IRQHandler                    /**< eQADC Command FIFO 0 Fill Flag */
    .long  EQADC1_FIFO0_RFDF_IRQHandler                    /**< eQADC Receive FIFO 0 Drain Flag */
    .long  EQADC1_FIFO1_NCF_IRQHandler                     /**< eQADC command FIFO 1 Non-Coherency Flag */
    .long  EQADC1_FIFO1_PF_IRQHandler                      /**< eQADC command FIFO 1 Pause Flag */
    .long  EQADC1_FIFO1_EOQF_IRQHandler                    /**< eQADC command FIFO 1 command queue End of Queue Flag */
    .long  EQADC1_FIFO1_CFFF_IRQHandler                    /**< eQADC Command FIFO 1 Fill Flag */
    .long  EQADC1_FIFO1_RFDF_IRQHandler                    /**< eQADC Receive FIFO 1 Drain Flag */
    .long  EQADC1_FIFO2_NCF_IRQHandler                     /**< eQADC command FIFO 2 Non-Coherency Flag */
    .long  EQADC1_FIFO2_PF_IRQHandler                      /**< eQADC command FIFO 2 Pause Flag */
    .long  EQADC1_FIFO2_EOQF_IRQHandler                    /**< eQADC command FIFO 2 command queue End of Queue Flag */
    .long  EQADC1_FIFO2_CFFF_IRQHandler                    /**< eQADC Command FIFO 2 Fill Flag */
    .long  EQADC1_FIFO2_RFDF_IRQHandler                    /**< eQADC Receive FIFO 2 Drain Flag */
    .long  EQADC1_FIFO3_NCF_IRQHandler                     /**< eQADC command FIFO 3 Non-Coherency Flag */
    .long  EQADC1_FIFO3_PF_IRQHandler                      /**< eQADC command FIFO 3 Pause Flag */
    .long  EQADC1_FIFO3_EOQF_IRQHandler                    /**< eQADC command FIFO 3 command queue End of Queue Flag */
    .long  EQADC1_FIFO3_CFFF_IRQHandler                    /**< eQADC Command FIFO 3 Fill Flag */
    .long  EQADC1_FIFO3_RFDF_IRQHandler                    /**< eQADC Receive FIFO 3 Drain Flag */
    .long  EQADC1_FIFO4_NCF_IRQHandler                     /**< eQADC command FIFO 4 Non-Coherency Flag */
    .long  EQADC1_FIFO4_PF_IRQHandler                      /**< eQADC command FIFO 4 Pause Flag */
    .long  EQADC1_FIFO4_EOQF_IRQHandler                    /**< eQADC command FIFO 4 command queue End of Queue Flag */
    .long  EQADC1_FIFO4_CFFF_IRQHandler                    /**< eQADC Command FIFO 4 Fill Flag */
    .long  EQADC1_FIFO4_RFDF_IRQHandler                    /**< eQADC Receive FIFO 4 Drain Flag */
    .long  EQADC1_FIFO5_NCF_IRQHandler                     /**< eQADC command FIFO 5 Non-Coherency Flag */
    .long  EQADC1_FIFO5_PF_IRQHandler                      /**< eQADC command FIFO 5 Pause Flag */
    .long  EQADC1_FIFO5_EOQF_IRQHandler                    /**< eQADC command FIFO 5 command queue End of Queue Flag */
    .long  EQADC1_FIFO5_CFFF_IRQHandler                    /**< eQADC Command FIFO 5 Fill Flag */
    .long  EQADC1_FIFO5_RFDF_IRQHandler                    /**< eQADC Receive FIFO 5 Drain Flag */
    .long  DMA1_Ch0_Ch31_Error_IRQHandler                  /**< eDMA1 channel Error flags 0-31 */
    .long  DMA1_Ch0_IRQHandler                             /**< eDMA1 channel Interrupt 0 */
    .long  DMA1_Ch1_IRQHandler                             /**< eDMA1 channel Interrupt 1 */
    .long  DMA1_Ch2_IRQHandler                             /**< eDMA1 channel Interrupt 2 */
    .long  DMA1_Ch3_IRQHandler                             /**< eDMA1 channel Interrupt 3 */
    .long  DMA1_Ch4_IRQHandler                             /**< eDMA1 channel Interrupt 4 */
    .long  DMA1_Ch5_IRQHandler                             /**< eDMA1 channel Interrupt 5 */
    .long  DMA1_Ch6_IRQHandler                             /**< eDMA1 channel Interrupt 6 */
    .long  DMA1_Ch7_IRQHandler                             /**< eDMA1 channel Interrupt 7 */
    .long  DMA1_Ch8_IRQHandler                             /**< eDMA1 channel Interrupt 8 */
    .long  DMA1_Ch9_IRQHandler                             /**< eDMA1 channel Interrupt 9 */
    .long  DMA1_Ch10_IRQHandler                            /**< eDMA1 channel Interrupt 10 */
    .long  DMA1_Ch11_IRQHandler                            /**< eDMA1 channel Interrupt 11 */
    .long  DMA1_Ch12_IRQHandler                            /**< eDMA1 channel Interrupt 12 */
    .long  DMA1_Ch13_IRQHandler                            /**< eDMA1 channel Interrupt 13 */
    .long  DMA1_Ch14_IRQHandler                            /**< eDMA1 channel Interrupt 14 */
    .long  DMA1_Ch15_IRQHandler                            /**< eDMA1 channel Interrupt 15 */
    .long  DMA1_Ch16_IRQHandler                            /**< eDMA1 channel Interrupt 16 */
    .long  DMA1_Ch17_IRQHandler                            /**< eDMA1 channel Interrupt 17 */
    .long  DMA1_Ch18_IRQHandler                            /**< eDMA1 channel Interrupt 18 */
    .long  DMA1_Ch19_IRQHandler                            /**< eDMA1 channel Interrupt 19 */
    .long  DMA1_Ch20_IRQHandler                            /**< eDMA1 channel Interrupt 20 */
    .long  DMA1_Ch21_IRQHandler                            /**< eDMA1 channel Interrupt 21 */
    .long  DMA1_Ch22_IRQHandler                            /**< eDMA1 channel Interrupt 22 */
    .long  DMA1_Ch23_IRQHandler                            /**< eDMA1 channel Interrupt 23 */
    .long  DMA1_Ch24_IRQHandler                            /**< eDMA1 channel Interrupt 24 */
    .long  DMA1_Ch25_IRQHandler                            /**< eDMA1 channel Interrupt 25 */
    .long  DMA1_Ch26_IRQHandler                            /**< eDMA1 channel Interrupt 26 */
    .long  DMA1_Ch27_IRQHandler                            /**< eDMA1 channel Interrupt 27 */
    .long  DMA1_Ch28_IRQHandler                            /**< eDMA1 channel Interrupt 28 */
    .long  DMA1_Ch29_IRQHandler                            /**< eDMA1 channel Interrupt 29 */
    .long  DMA1_Ch30_IRQHandler                            /**< eDMA1 channel Interrupt 30 */
    .long  DMA1_Ch31_IRQHandler                            /**< eDMA1 channel Interrupt 31 */
    .long  SDADC1234_IRQHandler                            /**< SDADC1 to SDADC4 Interrupts */
    .long  EMIOS1_CH16_IRQHandler                          /**< eMIOS_1 channel 16 Flag */
    .long  EMIOS1_CH17_IRQHandler                          /**< eMIOS_1 channel 17 Flag */
    .long  EMIOS1_CH18_IRQHandler                          /**< eMIOS_1 channel 18 Flag */
    .long  EMIOS1_CH19_IRQHandler                          /**< eMIOS_1 channel 19 Flag */
    .long  EMIOS1_CH20_IRQHandler                          /**< eMIOS_1 channel 20 Flag */
    .long  EMIOS1_CH21_IRQHandler                          /**< eMIOS_1 channel 21 Flag */
    .long  EMIOS1_CH22_IRQHandler                          /**< eMIOS_1 channel 22 Flag */
    .long  EMIOS1_CH23_IRQHandler                          /**< eMIOS_1 channel 23 Flag */
    .long  DEC2_IDF_IRQHandler                             /**< Decimation 2 Input (Fill) */
    .long  DEC2_OD_SD_IRQHandler                           /**< Decimation 2 Output/Integ (Drain/Integ) */
    .long  DEC2_ERR_IRQHandler                             /**< Decimation 2 Error */
    .long  DEC3_IDF_IRQHandler                             /**< Decimation 3 Input (Fill) */
    .long  DEC3_OD_SD_IRQHandler                           /**< Decimation 3 Output/Integ (Drain/Integ) */
    .long  DEC3_ERR_IRQHandler                             /**< Decimation 3 Error */
    .long  ESCI2_CIR_IRQHandler                            /**< Combined Interrupt Requests of ESCI Module 2 */
    .long  ESCI3_CIR_IRQHandler                            /**< Combined Interrupt Requests of ESCI Module 3 */
    .long  ESCI4_CIR_IRQHandler                            /**< Combined Interrupt Requests of ESCI Module 4 */
    .long  DECFILTER4_IRQHandler                           /**< Decimation Filter 4 Input/Output/Err/Intg (Fill/Drain/Error/Intg) */
    .long  DECFILTER5_IRQHandler                           /**< Decimation Filter 5 Input/Output/Err/Intg (Fill/Drain/Error/Intg) */
    .long  DECFILTER6_IRQHandler                           /**< Decimation Filter 6 Input/Output/Err/Intg (Fill/Drain/Error/Intg) */
    .long  DECFILTER7_IRQHandler                           /**< Decimation Filter 7 Input/Output/Err/Intg (Fill/Drain/Error/Intg) */
    .long  DECFILTER8_IRQHandler                           /**< Decimation Filter 8 Input/Output/Err/Intg (Fill/Drain/Error/Intg) */
    .long  DECFILTER9_IRQHandler                           /**< Decimation Filter 9 Input/Output/Err/Intg (Fill/Drain/Error/Intg) */
    .long  DECFILTER10_IRQHandler                          /**< Decimation Filter 10 Input/Output/Err/Intg (Fill/Drain/Error/Intg) */
    .long  DECFILTER11_IRQHandler                          /**< Decimation Filter 11 Input/Output/Err/Intg (Fill/Drain/Error/Intg) */
    .long  DefaultISR                         
    .long  DefaultISR                         
    .long  DefaultISR                         
    .long  DefaultISR                         
    .long  DMA1_Ch32_Ch63_Error_IRQHandler                 /**< eDMA1 channel Error flags 32-63 */
    .long  DMA1_Ch32_Ch39_IRQHandler                         /**< eDMA1 channel Interrupts 32-39 */
    .long  DMA1_Ch40_Ch47_IRQHandler                         /**< eDMA1 channel Interrupts 40-47 */
    .long  DMA1_Ch48_Ch55_IRQHandler                         /**< eDMA1 channel Interrupts 48-55 */
    .long  DMA1_Ch56_Ch63_IRQHandler                         /**< eDMA1 channel Interrupts 56-63 */
    .long  ETPU2_CIS24_IRQHandler                          /**< eTPU Engine 2 Channel 24 Interrupt Status */
    .long  ETPU2_CIS25_IRQHandler                          /**< eTPU Engine 2 Channel 25 Interrupt Status */
    .long  ETPU2_CIS26_IRQHandler                          /**< eTPU Engine 2 Channel 26 Interrupt Status */
    .long  ETPU2_CIS27_IRQHandler                          /**< eTPU Engine 2 Channel 27 Interrupt Status */
    .long  ETPU2_CIS28_IRQHandler                          /**< eTPU Engine 2 Channel 28 Interrupt Status */
    .long  ETPU2_CIS29_IRQHandler                          /**< eTPU Engine 2 Channel 29 Interrupt Status */
    .long  ETPU2_CIS30_IRQHandler                          /**< eTPU Engine 2 Channel 30 Interrupt Status */
    .long  ETPU2_CIS31_IRQHandler                          /**< eTPU Engine 2 Channel 31 Interrupt Status */
    .long  SWT1_IRQHandler                                 /**< Software Watchdog B Interrupt flag */
    .long  SEMA4_CORE0_IRQHandler                          /**< Core 0 requested semaphore has unlocked */
    .long  SEMA4_CORE1_IRQHandler                          /**< Core 1 requested semaphore has unlocked */
    .long  CSE_IRQ_IRQHandler                              /**< CSE Interrupt */
    .long  ESCI5_CIR_IRQHandler                            /**< Combined Interrupt Requests of ESCI Module 5 */
    .long  DSPI4_FIFO_Error_IRQHandler                     /**< DSPI_4 combined overrun and parity error interrupt requests: Transmit FIFO Underflow/Receive FIFO Overflow SPI Parity Error/DSI Parity Error */
    .long  DSPI4_TXFIFO_EOQF_IRQHandler                    /**< DSPI_4 transmit FIFO End of Queue Flag */
    .long  DSPI4_Send_IRQHandler                           /**< DSPI_4 Transmit FIFO Fill Flag */
    .long  DSPI4_TCF_IRQHandler                            /**< DSPI_4 Transfer Complete/DSI Data Match Flag */
    .long  DSPI4_Receive_IRQHandler                        /**< DSPI_4 Receive FIFO Drain Flag */
    .long  STCU_IRQHandler                                 /**< MBIST interrupt */

    .align  2
    .weak DefaultISR
    .type DefaultISR, %function
DefaultISR:
    e_b       DefaultISR
    .size DefaultISR, . - DefaultISR

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro def_irq_handler    handler_name
    .weak \handler_name
    .set  \handler_name, DefaultISR
    .endm

/* Exception Handlers */
    def_irq_handler  IVOR0_Handler
    def_irq_handler  IVOR1_Handler
    def_irq_handler  IVOR2_Handler
    def_irq_handler  IVOR3_Handler
    def_irq_handler  IVOR4_Handler
    def_irq_handler  IVOR5_Handler
    def_irq_handler  IVOR6_Handler
    def_irq_handler  IVOR7_Handler
    def_irq_handler  IVOR8_Handler
    def_irq_handler  IVOR9_Handler
    def_irq_handler  IVOR10_Handler
    def_irq_handler  IVOR11_Handler
    def_irq_handler  IVOR12_Handler
    def_irq_handler  IVOR13_Handler
    def_irq_handler  IVOR14_Handler
    def_irq_handler  IVOR15_Handler
    def_irq_handler  IVOR32_Handler
    def_irq_handler  IVOR33_Handler
    def_irq_handler  IVOR34_Handler
    def_irq_handler  IVOR35_Handler
    def_irq_handler  IVOR0_Exception_Handler
    def_irq_handler  IVOR1_Exception_Handler
    def_irq_handler  IVOR2_Exception_Handler
    def_irq_handler  IVOR3_Exception_Handler
    def_irq_handler  IVOR5_Exception_Handler
    def_irq_handler  IVOR6_Exception_Handler
    def_irq_handler  IVOR7_Exception_Handler
    def_irq_handler  IVOR8_Exception_Handler
    def_irq_handler  IVOR10_Exception_Handler
    def_irq_handler  IVOR11_Exception_Handler
    def_irq_handler  IVOR12_Exception_Handler
    def_irq_handler  IVOR13_Exception_Handler
    def_irq_handler  IVOR14_Exception_Handler
    def_irq_handler  IVOR15_Exception_Handler
    def_irq_handler  IVOR32_Exception_Handler
    def_irq_handler  IVOR33_Exception_Handler
    def_irq_handler  IVOR34_Exception_Handler
    def_irq_handler  IVOR35_Exception_Handler
    def_irq_handler  SS0_IRQHandler
    def_irq_handler  SS1_IRQHandler
    def_irq_handler  SS2_IRQHandler
    def_irq_handler  SS3_IRQHandler
    def_irq_handler  SS4_IRQHandler
    def_irq_handler  SS5_IRQHandler
    def_irq_handler  SS6_IRQHandler
    def_irq_handler  SS7_IRQHandler
    def_irq_handler  SWT0_IRQHandler
    def_irq_handler  FCCU_MISC_IRQHandler
    def_irq_handler  DMA0_Ch0_Ch31_Error_IRQHandler
    def_irq_handler  DMA0_Ch0_IRQHandler
    def_irq_handler  DMA0_Ch1_IRQHandler
    def_irq_handler  DMA0_Ch2_IRQHandler
    def_irq_handler  DMA0_Ch3_IRQHandler
    def_irq_handler  DMA0_Ch4_IRQHandler
    def_irq_handler  DMA0_Ch5_IRQHandler
    def_irq_handler  DMA0_Ch6_IRQHandler
    def_irq_handler  DMA0_Ch7_IRQHandler
    def_irq_handler  DMA0_Ch8_IRQHandler
    def_irq_handler  DMA0_Ch9_IRQHandler
    def_irq_handler  DMA0_Ch10_IRQHandler
    def_irq_handler  DMA0_Ch11_IRQHandler
    def_irq_handler  DMA0_Ch12_IRQHandler
    def_irq_handler  DMA0_Ch13_IRQHandler
    def_irq_handler  DMA0_Ch14_IRQHandler
    def_irq_handler  DMA0_Ch15_IRQHandler
    def_irq_handler  DMA0_Ch16_IRQHandler
    def_irq_handler  DMA0_Ch17_IRQHandler
    def_irq_handler  DMA0_Ch18_IRQHandler
    def_irq_handler  DMA0_Ch19_IRQHandler
    def_irq_handler  DMA0_Ch20_IRQHandler
    def_irq_handler  DMA0_Ch21_IRQHandler
    def_irq_handler  DMA0_Ch22_IRQHandler
    def_irq_handler  DMA0_Ch23_IRQHandler
    def_irq_handler  DMA0_Ch24_IRQHandler
    def_irq_handler  DMA0_Ch25_IRQHandler
    def_irq_handler  DMA0_Ch26_IRQHandler
    def_irq_handler  DMA0_Ch27_IRQHandler
    def_irq_handler  DMA0_Ch28_IRQHandler
    def_irq_handler  DMA0_Ch29_IRQHandler
    def_irq_handler  DMA0_Ch30_IRQHandler
    def_irq_handler  DMA0_Ch31_IRQHandler
    def_irq_handler  PCS_IRQHandler
    def_irq_handler  PLL_IRQHandler
    def_irq_handler  SIU_OVF_IRQHandler
    def_irq_handler  SIU_0_IRQHandler
    def_irq_handler  SIU_1_IRQHandler
    def_irq_handler  SIU_2_IRQHandler
    def_irq_handler  SIU_3_IRQHandler
    def_irq_handler  SIU_4_15_IRQHandler
    def_irq_handler  EMIOS0_F0_IRQHandler
    def_irq_handler  EMIOS0_F1_IRQHandler
    def_irq_handler  EMIOS0_F2_IRQHandler
    def_irq_handler  EMIOS0_F3_IRQHandler
    def_irq_handler  EMIOS0_F4_IRQHandler
    def_irq_handler  EMIOS0_F5_IRQHandler
    def_irq_handler  EMIOS0_F6_IRQHandler
    def_irq_handler  EMIOS0_F7_IRQHandler
    def_irq_handler  EMIOS1_F0_IRQHandler
    def_irq_handler  EMIOS1_F1_IRQHandler
    def_irq_handler  EMIOS1_F2_IRQHandler
    def_irq_handler  EMIOS1_F3_IRQHandler
    def_irq_handler  EMIOS1_F4_IRQHandler
    def_irq_handler  EMIOS1_F5_IRQHandler
    def_irq_handler  EMIOS1_F6_IRQHandler
    def_irq_handler  EMIOS1_F7_IRQHandler
    def_irq_handler  ETPU01_GE_IRQHandler
    def_irq_handler  ETPU0_CIS0_IRQHandler
    def_irq_handler  ETPU0_CIS1_IRQHandler
    def_irq_handler  ETPU0_CIS2_IRQHandler
    def_irq_handler  ETPU0_CIS3_IRQHandler
    def_irq_handler  ETPU0_CIS4_IRQHandler
    def_irq_handler  ETPU0_CIS5_IRQHandler
    def_irq_handler  ETPU0_CIS6_IRQHandler
    def_irq_handler  ETPU0_CIS7_IRQHandler
    def_irq_handler  ETPU0_CIS8_IRQHandler
    def_irq_handler  ETPU0_CIS9_IRQHandler
    def_irq_handler  ETPU0_CIS10_IRQHandler
    def_irq_handler  ETPU0_CIS11_IRQHandler
    def_irq_handler  ETPU0_CIS12_IRQHandler
    def_irq_handler  ETPU0_CIS13_IRQHandler
    def_irq_handler  ETPU0_CIS14_IRQHandler
    def_irq_handler  ETPU0_CIS15_IRQHandler
    def_irq_handler  ETPU0_CIS16_IRQHandler
    def_irq_handler  ETPU0_CIS17_IRQHandler
    def_irq_handler  ETPU0_CIS18_IRQHandler
    def_irq_handler  ETPU0_CIS19_IRQHandler
    def_irq_handler  ETPU0_CIS20_IRQHandler
    def_irq_handler  ETPU0_CIS21_IRQHandler
    def_irq_handler  ETPU0_CIS22_IRQHandler
    def_irq_handler  ETPU0_CIS23_IRQHandler
    def_irq_handler  ETPU0_CIS24_IRQHandler
    def_irq_handler  ETPU0_CIS25_IRQHandler
    def_irq_handler  ETPU0_CIS26_IRQHandler
    def_irq_handler  ETPU0_CIS27_IRQHandler
    def_irq_handler  ETPU0_CIS28_IRQHandler
    def_irq_handler  ETPU0_CIS29_IRQHandler
    def_irq_handler  ETPU0_CIS30_IRQHandler
    def_irq_handler  ETPU0_CIS31_IRQHandler
    def_irq_handler  EQADC0_OVRx_IRQHandler
    def_irq_handler  EQADC0_FIFO0_NCF_IRQHandler
    def_irq_handler  EQADC0_FIFO0_PF_IRQHandler
    def_irq_handler  EQADC0_FIFO0_EOQF_IRQHandler
    def_irq_handler  EQADC0_FIFO0_CFFF_IRQHandler
    def_irq_handler  EQADC0_FIFO0_RFDF_IRQHandler
    def_irq_handler  EQADC0_FIFO1_NCF_IRQHandler
    def_irq_handler  EQADC0_FIFO1_PF_IRQHandler
    def_irq_handler  EQADC0_FIFO1_EOQF_IRQHandler
    def_irq_handler  EQADC0_FIFO1_CFFF_IRQHandler
    def_irq_handler  EQADC0_FIFO1_RFDF_IRQHandler
    def_irq_handler  EQADC0_FIFO2_NCF_IRQHandler
    def_irq_handler  EQADC0_FIFO2_PF_IRQHandler
    def_irq_handler  EQADC0_FIFO2_EOQF_IRQHandler
    def_irq_handler  EQADC0_FIFO2_CFFF_IRQHandler
    def_irq_handler  EQADC0_FIFO2_RFDF_IRQHandler
    def_irq_handler  EQADC0_FIFO3_NCF_IRQHandler
    def_irq_handler  EQADC0_FIFO3_PF_IRQHandler
    def_irq_handler  EQADC0_FIFO3_EOQF_IRQHandler
    def_irq_handler  EQADC0_FIFO3_CFFF_IRQHandler
    def_irq_handler  EQADC0_FIFO3_RFDF_IRQHandler
    def_irq_handler  EQADC0_FIFO4_NCF_IRQHandler
    def_irq_handler  EQADC0_FIFO4_PF_IRQHandler
    def_irq_handler  EQADC0_FIFO4_EOQF_IRQHandler
    def_irq_handler  EQADC0_FIFO4_CFFF_IRQHandler
    def_irq_handler  EQADC0_FIFO4_RFDF_IRQHandler
    def_irq_handler  EQADC0_FIFO5_NCF_IRQHandler
    def_irq_handler  EQADC0_FIFO5_PF_IRQHandler
    def_irq_handler  EQADC0_FIFO5_EOQF_IRQHandler
    def_irq_handler  EQADC0_FIFO5_CFFF_IRQHandler
    def_irq_handler  EQADC0_FIFO5_RFDF_IRQHandler
    def_irq_handler  DSPI1_FIFO_Error_IRQHandler
    def_irq_handler  DSPI1_TXFIFO_EOQF_IRQHandler
    def_irq_handler  DSPI1_Send_IRQHandler
    def_irq_handler  DSPI1_TCF_IRQHandler
    def_irq_handler  DSPI1_Receive_IRQHandler
    def_irq_handler  DSPI2_FIFO_Error_IRQHandler
    def_irq_handler  DSPI2_TXFIFO_EOQF_IRQHandler
    def_irq_handler  DSPI2_Send_IRQHandler
    def_irq_handler  DSPI2_TCF_IRQHandler
    def_irq_handler  DSPI2_Receive_IRQHandler
    def_irq_handler  DSPI3_FIFO_Error_IRQHandler
    def_irq_handler  DSPI3_TXFIFO_EOQF_IRQHandler
    def_irq_handler  DSPI3_Send_IRQHandler
    def_irq_handler  DSPI3_TCF_IRQHandler
    def_irq_handler  DSPI3_Receive_IRQHandler
    def_irq_handler  ESCI0_CIR_IRQHandler
    def_irq_handler  PCU_MASTER0_IRQHandler
    def_irq_handler  PCU_MASTER1_IRQHandler
    def_irq_handler  ESCI1_CIR_IRQHandler
    def_irq_handler  PSI50_SDOE_IRQHandler
    def_irq_handler  PSI51_SDOE_IRQHandler
    def_irq_handler  CAN0_ESR1_IRQHandler
    def_irq_handler  CAN0_ESR2_IRQHandler
    def_irq_handler  CAN0_BUF0_IRQHandler
    def_irq_handler  CAN0_BUF1_IRQHandler
    def_irq_handler  CAN0_BUF2_IRQHandler
    def_irq_handler  CAN0_BUF3_IRQHandler
    def_irq_handler  CAN0_BUF4_IRQHandler
    def_irq_handler  CAN0_BUF5_IRQHandler
    def_irq_handler  CAN0_BUF6_IRQHandler
    def_irq_handler  CAN0_BUF7_IRQHandler
    def_irq_handler  CAN0_BUF8_IRQHandler
    def_irq_handler  CAN0_BUF9_IRQHandler
    def_irq_handler  CAN0_BUF10_IRQHandler
    def_irq_handler  CAN0_BUF11_IRQHandler
    def_irq_handler  CAN0_BUF12_IRQHandler
    def_irq_handler  CAN0_BUF13_IRQHandler
    def_irq_handler  CAN0_BUF14_IRQHandler
    def_irq_handler  CAN0_BUF15_IRQHandler
    def_irq_handler  CAN0_BUF16_31_IRQHandler
    def_irq_handler  CAN0_BUF32_63_IRQHandler
    def_irq_handler  CAN2_ESR1_IRQHandler
    def_irq_handler  CAN2_ESR2_IRQHandler
    def_irq_handler  CAN2_BUF0_IRQHandler
    def_irq_handler  CAN2_BUF1_IRQHandler
    def_irq_handler  CAN2_BUF2_IRQHandler
    def_irq_handler  CAN2_BUF3_IRQHandler
    def_irq_handler  CAN2_BUF4_IRQHandler
    def_irq_handler  CAN2_BUF5_IRQHandler
    def_irq_handler  CAN2_BUF6_IRQHandler
    def_irq_handler  CAN2_BUF7_IRQHandler
    def_irq_handler  CAN2_BUF8_IRQHandler
    def_irq_handler  CAN2_BUF9_IRQHandler
    def_irq_handler  CAN2_BUF10_IRQHandler
    def_irq_handler  CAN2_BUF11_IRQHandler
    def_irq_handler  CAN2_BUF12_IRQHandler
    def_irq_handler  CAN2_BUF13_IRQHandler
    def_irq_handler  CAN2_BUF14_IRQHandler
    def_irq_handler  CAN2_BUF15_IRQHandler
    def_irq_handler  CAN2_BUF16_31_IRQHandler
    def_irq_handler  CAN2_BUF32_63_IRQHandler
    def_irq_handler  FEC_TXF_IRQHandler
    def_irq_handler  FEC_RXF_IRQHandler
    def_irq_handler  FEC_ERR_IRQHandler
    def_irq_handler  DEC0_IDF_IRQHandler
    def_irq_handler  DEC0_OD_SD_IRQHandler
    def_irq_handler  DEC0_ERR_IRQHandler
    def_irq_handler  STM_CIR0_IRQHandler
    def_irq_handler  STM_CIR123_IRQHandler
    def_irq_handler  EMIOS0_CH16_IRQHandler
    def_irq_handler  EMIOS0_CH17_IRQHandler
    def_irq_handler  EMIOS0_CH18_IRQHandler
    def_irq_handler  EMIOS0_CH19_IRQHandler
    def_irq_handler  EMIOS0_CH20_IRQHandler
    def_irq_handler  EMIOS0_CH21_IRQHandler
    def_irq_handler  EMIOS0_CH22_IRQHandler
    def_irq_handler  EMIOS0_CH23_IRQHandler
    def_irq_handler  DMA0_Ch32_Ch63_Error_IRQHandler
    def_irq_handler  DMA0_Ch32_IRQHandler
    def_irq_handler  DMA0_Ch33_IRQHandler
    def_irq_handler  DMA0_Ch34_IRQHandler
    def_irq_handler  DMA0_Ch35_IRQHandler
    def_irq_handler  DMA0_Ch36_IRQHandler
    def_irq_handler  DMA0_Ch37_IRQHandler
    def_irq_handler  DMA0_Ch38_IRQHandler
    def_irq_handler  DMA0_Ch39_IRQHandler
    def_irq_handler  DMA0_Ch40_IRQHandler
    def_irq_handler  DMA0_Ch41_IRQHandler
    def_irq_handler  DMA0_Ch42_IRQHandler
    def_irq_handler  DMA0_Ch43_IRQHandler
    def_irq_handler  DMA0_Ch44_IRQHandler
    def_irq_handler  DMA0_Ch45_IRQHandler
    def_irq_handler  DMA0_Ch46_IRQHandler
    def_irq_handler  DMA0_Ch47_IRQHandler
    def_irq_handler  DMA0_Ch48_IRQHandler
    def_irq_handler  DMA0_Ch49_IRQHandler
    def_irq_handler  DMA0_Ch50_IRQHandler
    def_irq_handler  DMA0_Ch51_IRQHandler
    def_irq_handler  DMA0_Ch52_IRQHandler
    def_irq_handler  DMA0_Ch53_IRQHandler
    def_irq_handler  DMA0_Ch54_IRQHandler
    def_irq_handler  DMA0_Ch55_IRQHandler
    def_irq_handler  DMA0_Ch56_IRQHandler
    def_irq_handler  DMA0_Ch57_IRQHandler
    def_irq_handler  DMA0_Ch58_IRQHandler
    def_irq_handler  DMA0_Ch59_IRQHandler
    def_irq_handler  DMA0_Ch60_IRQHandler
    def_irq_handler  DMA0_Ch61_IRQHandler
    def_irq_handler  DMA0_Ch62_IRQHandler
    def_irq_handler  DMA0_Ch63_IRQHandler
    def_irq_handler  ETPU1_CIS0_IRQHandler
    def_irq_handler  ETPU1_CIS1_IRQHandler
    def_irq_handler  ETPU1_CIS2_IRQHandler
    def_irq_handler  ETPU1_CIS3_IRQHandler
    def_irq_handler  ETPU1_CIS4_IRQHandler
    def_irq_handler  ETPU1_CIS5_IRQHandler
    def_irq_handler  ETPU1_CIS6_IRQHandler
    def_irq_handler  ETPU1_CIS7_IRQHandler
    def_irq_handler  ETPU1_CIS8_IRQHandler
    def_irq_handler  ETPU1_CIS9_IRQHandler
    def_irq_handler  ETPU1_CIS10_IRQHandler
    def_irq_handler  ETPU1_CIS11_IRQHandler
    def_irq_handler  ETPU1_CIS12_IRQHandler
    def_irq_handler  ETPU1_CIS13_IRQHandler
    def_irq_handler  ETPU1_CIS14_IRQHandler
    def_irq_handler  ETPU1_CIS15_IRQHandler
    def_irq_handler  ETPU1_CIS16_IRQHandler
    def_irq_handler  ETPU1_CIS17_IRQHandler
    def_irq_handler  ETPU1_CIS18_IRQHandler
    def_irq_handler  ETPU1_CIS19_IRQHandler
    def_irq_handler  ETPU1_CIS20_IRQHandler
    def_irq_handler  ETPU1_CIS21_IRQHandler
    def_irq_handler  ETPU1_CIS22_IRQHandler
    def_irq_handler  ETPU1_CIS23_IRQHandler
    def_irq_handler  ETPU1_CIS24_IRQHandler
    def_irq_handler  ETPU1_CIS25_IRQHandler
    def_irq_handler  ETPU1_CIS26_IRQHandler
    def_irq_handler  ETPU1_CIS27_IRQHandler
    def_irq_handler  ETPU1_CIS28_IRQHandler
    def_irq_handler  ETPU1_CIS29_IRQHandler
    def_irq_handler  ETPU1_CIS30_IRQHandler
    def_irq_handler  ETPU1_CIS31_IRQHandler
    def_irq_handler  DSPI0_FIFO_Error_IRQHandler
    def_irq_handler  DSPI0_TXFIFO_EOQF_IRQHandler
    def_irq_handler  DSPI0_Send_IRQHandler
    def_irq_handler  DSPI0_TCF_IRQHandler
    def_irq_handler  DSPI0_Receive_IRQHandler
    def_irq_handler  CAN1_ESR1_IRQHandler
    def_irq_handler  CAN1_ESR2_IRQHandler
    def_irq_handler  CAN1_BUF0_IRQHandler
    def_irq_handler  CAN1_BUF1_IRQHandler
    def_irq_handler  CAN1_BUF2_IRQHandler
    def_irq_handler  CAN1_BUF3_IRQHandler
    def_irq_handler  CAN1_BUF4_IRQHandler
    def_irq_handler  CAN1_BUF5_IRQHandler
    def_irq_handler  CAN1_BUF6_IRQHandler
    def_irq_handler  CAN1_BUF7_IRQHandler
    def_irq_handler  CAN1_BUF8_IRQHandler
    def_irq_handler  CAN1_BUF9_IRQHandler
    def_irq_handler  CAN1_BUF10_IRQHandler
    def_irq_handler  CAN1_BUF11_IRQHandler
    def_irq_handler  CAN1_BUF12_IRQHandler
    def_irq_handler  CAN1_BUF13_IRQHandler
    def_irq_handler  CAN1_BUF14_IRQHandler
    def_irq_handler  CAN1_BUF15_IRQHandler
    def_irq_handler  CAN1_BUF16_31_IRQHandler
    def_irq_handler  CAN1_BUF32_63_IRQHandler
    def_irq_handler  PIT_RTI0_IRQHandler
    def_irq_handler  PIT_RTI1_IRQHandler
    def_irq_handler  PIT_RTI2_IRQHandler
    def_irq_handler  PIT_RTI3_IRQHandler
    def_irq_handler  PIT_RTIINT_IRQHandler
    def_irq_handler  FMC_Done_IRQHandler
    def_irq_handler  CAN3_ESR1_IRQHandler
    def_irq_handler  CAN3_ESR2_IRQHandler
    def_irq_handler  CAN3_BUF0_IRQHandler
    def_irq_handler  CAN3_BUF1_IRQHandler
    def_irq_handler  CAN3_BUF2_IRQHandler
    def_irq_handler  CAN3_BUF3_IRQHandler
    def_irq_handler  CAN3_BUF4_IRQHandler
    def_irq_handler  CAN3_BUF5_IRQHandler
    def_irq_handler  CAN3_BUF6_IRQHandler
    def_irq_handler  CAN3_BUF7_IRQHandler
    def_irq_handler  CAN3_BUF8_IRQHandler
    def_irq_handler  CAN3_BUF9_IRQHandler
    def_irq_handler  CAN3_BUF10_IRQHandler
    def_irq_handler  CAN3_BUF11_IRQHandler
    def_irq_handler  CAN3_BUF12_IRQHandler
    def_irq_handler  CAN3_BUF13_IRQHandler
    def_irq_handler  CAN3_BUF14_IRQHandler
    def_irq_handler  CAN3_BUF15_IRQHandler
    def_irq_handler  CAN3_BUF16_31_IRQHandler
    def_irq_handler  CAN3_BUF32_63_IRQHandler
    def_irq_handler  SRX0_GBL_STATUS_IRQHandler
    def_irq_handler  SRX0_CH0_IRQHandler
    def_irq_handler  SRX0_CH1_IRQHandler
    def_irq_handler  SRX0_CH2_IRQHandler
    def_irq_handler  SRX0_CH3_IRQHandler
    def_irq_handler  SRX0_CH4_IRQHandler
    def_irq_handler  SRX0_CH5_IRQHandler
    def_irq_handler  SRX1_GBL_STATUS_IRQHandler
    def_irq_handler  SRX1_CH0_IRQHandler
    def_irq_handler  SRX1_CH1_IRQHandler
    def_irq_handler  SRX1_CH2_IRQHandler
    def_irq_handler  SRX1_CH3_IRQHandler
    def_irq_handler  SRX1_CH4_IRQHandler
    def_irq_handler  SRX1_CH5_IRQHandler
    def_irq_handler  PMC_IRQHandler
    def_irq_handler  PMC_TEMP_IRQHandler
    def_irq_handler  JDC_IRQHandler
    def_irq_handler  SIPI0_IRQHandler
    def_irq_handler  LFAST_IRQHandler
    def_irq_handler  MCAN_IRQHandler
    def_irq_handler  ERM_IRQHandler
    def_irq_handler  CMU01_IRQHandler
    def_irq_handler  CMU23_IRQHandler
    def_irq_handler  CMU45_IRQHandler
    def_irq_handler  CMU67_IRQHandler
    def_irq_handler  CMU8_IRQHandler
    def_irq_handler  CMU_RSV0_IRQHandler
    def_irq_handler  CMU_RSV1_IRQHandler
    def_irq_handler  CMU_RSV2_IRQHandler
    def_irq_handler  REACM_GBL_IRQHandler
    def_irq_handler  REACM_CH01_IRQHandler
    def_irq_handler  REACM_CH23_IRQHandler
    def_irq_handler  REACM_CH45_IRQHandler
    def_irq_handler  REACM_CH67_IRQHandler
    def_irq_handler  REACM_CH89_IRQHandler
    def_irq_handler  REACM_RSV0_IRQHandler
    def_irq_handler  REACM_RSV1_IRQHandler
    def_irq_handler  DEC1_IDF_IRQHandler
    def_irq_handler  DEC1_OD_SD_IRQHandler
    def_irq_handler  DEC1_ERR_IRQHandler
    def_irq_handler  ETPU2_GE_IRQHandler
    def_irq_handler  ETPU2_CIS0_IRQHandler
    def_irq_handler  ETPU2_CIS1_IRQHandler
    def_irq_handler  ETPU2_CIS2_IRQHandler
    def_irq_handler  ETPU2_CIS3_IRQHandler
    def_irq_handler  ETPU2_CIS4_IRQHandler
    def_irq_handler  ETPU2_CIS5_IRQHandler
    def_irq_handler  ETPU2_CIS6_IRQHandler
    def_irq_handler  ETPU2_CIS7_IRQHandler
    def_irq_handler  ETPU2_CIS8_IRQHandler
    def_irq_handler  ETPU2_CIS9_IRQHandler
    def_irq_handler  ETPU2_CIS10_IRQHandler
    def_irq_handler  ETPU2_CIS11_IRQHandler
    def_irq_handler  ETPU2_CIS12_IRQHandler
    def_irq_handler  ETPU2_CIS13_IRQHandler
    def_irq_handler  ETPU2_CIS14_IRQHandler
    def_irq_handler  ETPU2_CIS15_IRQHandler
    def_irq_handler  ETPU2_CIS16_IRQHandler
    def_irq_handler  ETPU2_CIS17_IRQHandler
    def_irq_handler  ETPU2_CIS18_IRQHandler
    def_irq_handler  ETPU2_CIS19_IRQHandler
    def_irq_handler  ETPU2_CIS20_IRQHandler
    def_irq_handler  ETPU2_CIS21_IRQHandler
    def_irq_handler  ETPU2_CIS22_IRQHandler
    def_irq_handler  ETPU2_CIS23_IRQHandler
    def_irq_handler  EQADC1_OVRx_IRQHandler
    def_irq_handler  EQADC1_FIFO0_NCF_IRQHandler
    def_irq_handler  EQADC1_FIFO0_PF_IRQHandler
    def_irq_handler  EQADC1_FIFO0_EOQF_IRQHandler
    def_irq_handler  EQADC1_FIFO0_CFFF_IRQHandler
    def_irq_handler  EQADC1_FIFO0_RFDF_IRQHandler
    def_irq_handler  EQADC1_FIFO1_NCF_IRQHandler
    def_irq_handler  EQADC1_FIFO1_PF_IRQHandler
    def_irq_handler  EQADC1_FIFO1_EOQF_IRQHandler
    def_irq_handler  EQADC1_FIFO1_CFFF_IRQHandler
    def_irq_handler  EQADC1_FIFO1_RFDF_IRQHandler
    def_irq_handler  EQADC1_FIFO2_NCF_IRQHandler
    def_irq_handler  EQADC1_FIFO2_PF_IRQHandler
    def_irq_handler  EQADC1_FIFO2_EOQF_IRQHandler
    def_irq_handler  EQADC1_FIFO2_CFFF_IRQHandler
    def_irq_handler  EQADC1_FIFO2_RFDF_IRQHandler
    def_irq_handler  EQADC1_FIFO3_NCF_IRQHandler
    def_irq_handler  EQADC1_FIFO3_PF_IRQHandler
    def_irq_handler  EQADC1_FIFO3_EOQF_IRQHandler
    def_irq_handler  EQADC1_FIFO3_CFFF_IRQHandler
    def_irq_handler  EQADC1_FIFO3_RFDF_IRQHandler
    def_irq_handler  EQADC1_FIFO4_NCF_IRQHandler
    def_irq_handler  EQADC1_FIFO4_PF_IRQHandler
    def_irq_handler  EQADC1_FIFO4_EOQF_IRQHandler
    def_irq_handler  EQADC1_FIFO4_CFFF_IRQHandler
    def_irq_handler  EQADC1_FIFO4_RFDF_IRQHandler
    def_irq_handler  EQADC1_FIFO5_NCF_IRQHandler
    def_irq_handler  EQADC1_FIFO5_PF_IRQHandler
    def_irq_handler  EQADC1_FIFO5_EOQF_IRQHandler
    def_irq_handler  EQADC1_FIFO5_CFFF_IRQHandler
    def_irq_handler  EQADC1_FIFO5_RFDF_IRQHandler
    def_irq_handler  DMA1_Ch0_Ch31_Error_IRQHandler
    def_irq_handler  DMA1_Ch0_IRQHandler
    def_irq_handler  DMA1_Ch1_IRQHandler
    def_irq_handler  DMA1_Ch2_IRQHandler
    def_irq_handler  DMA1_Ch3_IRQHandler
    def_irq_handler  DMA1_Ch4_IRQHandler
    def_irq_handler  DMA1_Ch5_IRQHandler
    def_irq_handler  DMA1_Ch6_IRQHandler
    def_irq_handler  DMA1_Ch7_IRQHandler
    def_irq_handler  DMA1_Ch8_IRQHandler
    def_irq_handler  DMA1_Ch9_IRQHandler
    def_irq_handler  DMA1_Ch10_IRQHandler
    def_irq_handler  DMA1_Ch11_IRQHandler
    def_irq_handler  DMA1_Ch12_IRQHandler
    def_irq_handler  DMA1_Ch13_IRQHandler
    def_irq_handler  DMA1_Ch14_IRQHandler
    def_irq_handler  DMA1_Ch15_IRQHandler
    def_irq_handler  DMA1_Ch16_IRQHandler
    def_irq_handler  DMA1_Ch17_IRQHandler
    def_irq_handler  DMA1_Ch18_IRQHandler
    def_irq_handler  DMA1_Ch19_IRQHandler
    def_irq_handler  DMA1_Ch20_IRQHandler
    def_irq_handler  DMA1_Ch21_IRQHandler
    def_irq_handler  DMA1_Ch22_IRQHandler
    def_irq_handler  DMA1_Ch23_IRQHandler
    def_irq_handler  DMA1_Ch24_IRQHandler
    def_irq_handler  DMA1_Ch25_IRQHandler
    def_irq_handler  DMA1_Ch26_IRQHandler
    def_irq_handler  DMA1_Ch27_IRQHandler
    def_irq_handler  DMA1_Ch28_IRQHandler
    def_irq_handler  DMA1_Ch29_IRQHandler
    def_irq_handler  DMA1_Ch30_IRQHandler
    def_irq_handler  DMA1_Ch31_IRQHandler
    def_irq_handler  SDADC1234_IRQHandler
    def_irq_handler  EMIOS1_CH16_IRQHandler
    def_irq_handler  EMIOS1_CH17_IRQHandler
    def_irq_handler  EMIOS1_CH18_IRQHandler
    def_irq_handler  EMIOS1_CH19_IRQHandler
    def_irq_handler  EMIOS1_CH20_IRQHandler
    def_irq_handler  EMIOS1_CH21_IRQHandler
    def_irq_handler  EMIOS1_CH22_IRQHandler
    def_irq_handler  EMIOS1_CH23_IRQHandler
    def_irq_handler  DEC2_IDF_IRQHandler
    def_irq_handler  DEC2_OD_SD_IRQHandler
    def_irq_handler  DEC2_ERR_IRQHandler
    def_irq_handler  DEC3_IDF_IRQHandler
    def_irq_handler  DEC3_OD_SD_IRQHandler
    def_irq_handler  DEC3_ERR_IRQHandler
    def_irq_handler  ESCI2_CIR_IRQHandler
    def_irq_handler  ESCI3_CIR_IRQHandler
    def_irq_handler  ESCI4_CIR_IRQHandler
    def_irq_handler  DECFILTER4_IRQHandler
    def_irq_handler  DECFILTER5_IRQHandler
    def_irq_handler  DECFILTER6_IRQHandler
    def_irq_handler  DECFILTER7_IRQHandler
    def_irq_handler  DECFILTER8_IRQHandler
    def_irq_handler  DECFILTER9_IRQHandler
    def_irq_handler  DECFILTER10_IRQHandler
    def_irq_handler  DECFILTER11_IRQHandler
    def_irq_handler  DMA1_Ch32_Ch63_Error_IRQHandler
    def_irq_handler  DMA1_Ch32_Ch39_IRQHandler
    def_irq_handler  DMA1_Ch40_Ch47_IRQHandler
    def_irq_handler  DMA1_Ch48_Ch55_IRQHandler
    def_irq_handler  DMA1_Ch56_Ch63_IRQHandler
    def_irq_handler  ETPU2_CIS24_IRQHandler
    def_irq_handler  ETPU2_CIS25_IRQHandler
    def_irq_handler  ETPU2_CIS26_IRQHandler
    def_irq_handler  ETPU2_CIS27_IRQHandler
    def_irq_handler  ETPU2_CIS28_IRQHandler
    def_irq_handler  ETPU2_CIS29_IRQHandler
    def_irq_handler  ETPU2_CIS30_IRQHandler
    def_irq_handler  ETPU2_CIS31_IRQHandler
    def_irq_handler  SWT1_IRQHandler
    def_irq_handler  SEMA4_CORE0_IRQHandler
    def_irq_handler  SEMA4_CORE1_IRQHandler
    def_irq_handler  CSE_IRQ_IRQHandler
    def_irq_handler  ESCI5_CIR_IRQHandler
    def_irq_handler  DSPI4_FIFO_Error_IRQHandler
    def_irq_handler  DSPI4_TXFIFO_EOQF_IRQHandler
    def_irq_handler  DSPI4_Send_IRQHandler
    def_irq_handler  DSPI4_TCF_IRQHandler
    def_irq_handler  DSPI4_Receive_IRQHandler
    def_irq_handler  STCU_IRQHandler
