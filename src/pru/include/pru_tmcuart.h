/* pru_tmcuart.h — PRU-side TMC UART constants & API.
 *
 * Minimal header extracted to mirror Klipper-style separation between
 * public declarations and implementation. The implementation remains
 * in `pru1_lateral/main.c` to comply with PRU size/inline constraints.
 */

#ifndef PRU_TMCUART_H
#define PRU_TMCUART_H

#include <stdint.h>
#include "pru_ipc.h"

/* Default PRU pins used for TMC UART (tweak for your board) */
#define TMC_TX_BIT   (1u << 5)
#define TMC_RX_BIT   (1u << 6)

enum {
    TU_LINE_HIGH = 1<<0,
    TU_ACTIVE    = 1<<1,
    TU_READ_SYNC = 1<<2,
    TU_REPORT    = 1<<3,
    TU_PULLUP    = 1<<4,
    TU_SINGLE_WIRE = 1<<5
};

#define TMC_MODE_IDLE        0u
#define TMC_MODE_READ_EVENT  1u
#define TMC_MODE_READ_SYNC   2u
#define TMC_MODE_SEND_EVENT  3u
#define TMC_MODE_SEND_FINISH 4u
/* Enhanced send-sync mode: used to send a 4-bit sync nibble and
 * measure scheduling offsets to refine actual bit time on-PRU. */
#define TMC_MODE_SEND_SYNC   5u

/* API (implemented in pru1_lateral/main.c) */
static inline void tmcuart_reset_line(void);
static inline void tmcuart_finalize(void);
static inline void tmcuart_send_sync_event(void);
static inline void tmcuart_poll(void);

/* Board hooks: by default these are no-ops suitable for many boards where
 * the PRU pins are already configured as input+output. Boards that need
 * explicit pad reconfiguration for single-wire operation can override
 * these macros in a board-specific header included before building PRU.
 *
 * - TMC_SET_RX_INPUT_PULLUP(pull): configure RX pin as input; if pull!=0
 *   enable internal pull-up if supported.
 * - TMC_SET_TX_OUTPUT(val):   force TX output to `val` (0/1).
 * - TMC_SET_TX_TRISTATE():   release TX line (tri-state / input); default
 *   implementation sets TX high to emulate release when HW has pull-ups.
 */
#ifndef TMC_SET_RX_INPUT_PULLUP
#define TMC_SET_RX_INPUT_PULLUP(pull) do { (void)(pull); } while(0)
#endif

#ifndef TMC_SET_TX_OUTPUT
#define TMC_SET_TX_OUTPUT(val) do { if (val) __R30 |= TMC_TX_BIT; else __R30 &= ~TMC_TX_BIT; } while(0)
#endif

#ifndef TMC_SET_TX_TRISTATE
#define TMC_SET_TX_TRISTATE() do { __R30 |= TMC_TX_BIT; } while(0)
#endif

#endif /* PRU_TMCUART_H */
