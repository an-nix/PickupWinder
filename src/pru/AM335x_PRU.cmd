/****************************************************************************/
/*  AM335x_PRU.cmd — Linker command file for PRU firmware on AM335x         */
/*  Used for both PRU0 (orchestrator) and PRU1 (motor control).             */
/*                                                                          */
/*  Key point: .resource_table MUST be in local DMEM (not DDR) so that     */
/*  the remoteproc driver can map it.                                       */
/*                                                                          */
/*  Constant table notes:                                                    */
/*    CREGISTER=24 → local DMEM     (fixed, always correct)                 */
/*    CREGISTER=25 → peer DMEM      (fixed, always correct)                 */
/*    CREGISTER=26 → IEP timer      (CT_IEP.TMR_CNT, used by pru_stepper.h) */
/*    CREGISTER=28 → Shared RAM     (PROGRAMMABLE — reset to 0 at boot)     */
/*                                                                          */
/*  WARNING: CT28 resets to 0 at PRU boot. Using CREGISTER=28 in the       */
/*  MEMORY definition causes clpru to emit CT28-relative LBBO/SBBO for     */
/*  all accesses to 0x00010000, which is WRONG until CTPPR0 is programmed.  */
/*  Solution: omit CREGISTER=28 so clpru generates direct absolute         */
/*  addressing (LBBO/SBBO with a full 32-bit address register).             */
/*  PRU local address 0x00010000 is always the shared RAM — no CT needed.  */
/****************************************************************************/

-cr                     /* link using C runtime conventions */

MEMORY
{
    PAGE 0:
        PRU_IMEM        : org = 0x00000000  len = 0x00002000  /* 8 kB instruction RAM */

    PAGE 1:
        PRU_DMEM_LOCAL  : org = 0x00000000  len = 0x00002000  CREGISTER=24  /* local 8 kB data RAM */
        PRU_DMEM_PEER   : org = 0x00002000  len = 0x00002000  CREGISTER=25  /* peer  8 kB data RAM */

    PAGE 2:
        /* PRU_SHAREDMEM: NO CREGISTER here. Direct absolute addressing is used
         * instead of CT28 to avoid the CT28-resets-to-0 pitfall. clpru will
         * load 0x00010000 into a register and use LBBO/SBBO with that register,
         * which correctly resolves to the PRUSS shared RAM regardless of CT28. */
        PRU_SHAREDMEM   : org = 0x00010000  len = 0x00003000  /* 12 kB shared RAM */
        DDR             : org = 0x80000000  len = 0x00100000  CREGISTER=31

        /* Peripherals accessed via constant table */
        PRU_IEP         : org = 0x0002E000  len = 0x0000031C  CREGISTER=26
        PRU_INTC        : org = 0x00020000  len = 0x00001504  CREGISTER=0
        PRU_CFG         : org = 0x00026000  len = 0x00000044  CREGISTER=4
}

SECTIONS
{
    /* Force _c_int00 to address 0 in instruction RAM */
    .text:_c_int00*     >  0x0,             PAGE 0

    .text               >  PRU_IMEM,        PAGE 0

    /* resource_table MUST be in local DMEM — remoteproc maps it by physical addr */
    .resource_table     >  PRU_DMEM_LOCAL,  PAGE 1

    .stack              >  PRU_DMEM_LOCAL,  PAGE 1
    .bss                >  PRU_DMEM_LOCAL,  PAGE 1
    .data               >  PRU_DMEM_LOCAL,  PAGE 1
    .rodata             >  PRU_DMEM_LOCAL,  PAGE 1
    .cinit              >  PRU_DMEM_LOCAL,  PAGE 1
    .sysmem             >  PRU_DMEM_LOCAL,  PAGE 1
    .switch             >  PRU_DMEM_LOCAL,  PAGE 1
    .cio                >  PRU_DMEM_LOCAL,  PAGE 1
}
