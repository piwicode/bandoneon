#ifndef BOOT_DEBUG_H
#define BOOT_DEBUG_H

/* Minimal TX-only debug console, USART1 at the same 921600 baud as the
 * application's console (../main-g474/Core/Src/main.c's MX_USART1_UART_Init())
 * — so the existing `just console` / `tio` setup reads it with no new tooling.
 *
 * The FN LEDs are the only other diagnostic here and are not reliably
 * connected on every board, so an update that fails needs somewhere to say
 * why. printf() works after debug_init(): _write() in debug.c routes it here
 * directly, the same pattern ../common/console/console.c uses for the
 * application, minus microrl/RX/echo — this is output-only. */
void debug_init(void);

#endif /* BOOT_DEBUG_H */
