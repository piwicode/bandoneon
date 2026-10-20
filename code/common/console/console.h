// UART-backed interactive console with printf routing.
// Call console_init() once after UART is ready.
// Implement console_execute() in your application to handle commands.
#ifndef CONSOLE_H
#define CONSOLE_H

#include "microrl.h"
#include "stm32g4xx_hal.h"

extern microrl_t console_rl;

// Set up microrl, route printf to huart, print banner, arm UART RX interrupt.
void console_init(UART_HandleTypeDef *huart, IRQn_Type irqn);

// Feed one received byte into microrl. Called from thread context by
// console_task(); may also be called directly for testing.
void console_rx_callback(uint8_t ch);

// Drain any byte buffered by the UART RX ISR, feed it into microrl, and
// redraw the prompt if any async output landed since the last call.
// Call once per main-loop iteration.
void console_task(void);

// Reprint the prompt and current input line, restoring the cursor. Assumes the
// cursor is at column 0 of a fresh line (async prints end with \r\n).
void console_redraw_prompt(void);

// ===== Live-metrics dashboard ============================================== //
// A fixed block of R rows pinned to the TOP of the screen, refreshed in place
// each frame, while logs and the prompt scroll in a DECSTBM region below it
// (rows R+1..bottom). Anchoring at the top means the bottom row never has to be
// known, so no terminal-height query is needed: the region is set as
// "\033[{R+1};r" (omitted bottom param => the terminal's last line on xterm and
// most others). The dashboard is overdrawn with absolute cursor moves + clear-
// to-end-of-line, so there is no full-screen clear and thus no flicker.
//
// Per frame: console_dash_begin(), one console_dash_println() per row (in the
// order rows should stack), then console_dash_end(). The frame is self-contained
// (saves/restores the cursor), so logs printed between frames scroll normally in
// the region below and never collide with the dashboard.

// Start a dashboard frame: save the cursor and reset the row counter to the top.
void console_dash_begin(void);

// Render exactly ONE dashboard row (no embedded newlines): position to the next
// row, print the formatted text, clear to end of line, advance the row counter.
void console_dash_println(const char *fmt, ...);

// End a dashboard frame: draw the closing rule (a separator carrying the refresh
// period in ms, which marks the boundary with the scrolling log section below),
// clear any rows the previous (taller) frame left behind, re-set the scroll
// region if the row count changed, and restore the cursor into the scrolling
// region.
void console_dash_end(unsigned period_ms);

// Tear the dashboard down: release the scroll region and clear the band, so the
// console behaves exactly as before any report was shown. No-op if no dashboard
// is up, so it is safe (and intended) to call every iteration that no report is
// active.
void console_dash_hide(void);

#endif
