/* pru_rsc_table.h — Minimal empty resource table for remoteproc.
 *
 * The Linux remoteproc driver requires a ".resource_table" ELF section
 * to be present in PRU firmware.  An empty table (version=1, num=0)
 * satisfies the driver when no RPMsg, carveout, or interrupt resources
 * are needed.
 *
 * Usage: include exactly ONCE in the top-level main.c of each firmware
 * binary.  Do NOT include from shared headers.
 *
 * Requires clpru (TI compiler) — uses #pragma DATA_SECTION and RETAIN.
 * The pru-elf-gcc equivalent would use __attribute__((section(...))).
 */

#ifndef PRU_RSC_TABLE_H
#define PRU_RSC_TABLE_H

#ifdef __TI_COMPILER_VERSION__

#include <rsc_types.h>

#pragma DATA_SECTION(pru_remoteproc_ResourceTable, ".resource_table")
#pragma RETAIN(pru_remoteproc_ResourceTable)
const struct resource_table pru_remoteproc_ResourceTable = {
    1,       /* version   */
    0,       /* num entries — no resources declared */
    { 0, 0 } /* reserved  */
};

#else /* pru-elf-gcc */

/* Minimal struct without rsc_types.h dependency */
typedef struct {
    unsigned int ver;
    unsigned int num;
    unsigned int reserved[2];
} __attribute__((packed)) pru_rsc_table_t;

__attribute__((section(".resource_table")))
__attribute__((used))
const pru_rsc_table_t pru_remoteproc_ResourceTable = { 1, 0, { 0, 0 } };

#endif /* __TI_COMPILER_VERSION__ */

#endif /* PRU_RSC_TABLE_H */
