#include "config.h"
#include "console.h"
#include "ansi.h"
#include <stdarg.h>
#include <stdio.h>

microrl_t console_rl;

static UART_HandleTypeDef *console_uart;
static uint8_t console_rx_char;

/* Small FIFO between the UART RX ISR and the main-loop processing.
 * The ISR pushes bytes here; console_poll() drains them from thread context
 * so that TX output (echo, history recall) never blocks inside an ISR.
 * Size must be a power of 2 and large enough for the longest escape sequence
 * the terminal can burst (arrow = 3 bytes, F1-F4 = 5 bytes → 8 is sufficient). */
#define RX_FIFO_SIZE 8
#define RX_FIFO_MASK (RX_FIFO_SIZE - 1)
static volatile uint8_t  rx_fifo[RX_FIFO_SIZE];
static volatile uint32_t rx_head; /* written by ISR  */
static volatile uint32_t rx_tail; /* read  by poll() */

/* >0 while microrl (or our own redraw) is rendering the line; output produced
 * in that window is the prompt/echo itself and must not mark the line dirty. */
static volatile int console_internal_depth;
/* Set when output is written outside microrl rendering (i.e. async app output). */
static volatile uint8_t console_dirty;

/* Routes printf to console_uart. Callers use explicit \r\n. */
int _write(int fd, char *buf, int len)
{
  (void)fd;
  if (!console_uart) return len;
  /* First async write since the prompt was drawn: wipe the prompt line so the
   * message starts at column 0 instead of right after "IRin > ". The prompt is
   * reprinted by console_redraw_prompt() at the bottom of the main loop. */
  if (console_internal_depth == 0 && !console_dirty)
  {
    HAL_UART_Transmit(console_uart, (uint8_t *)("\r" ANSI_CLEAR_LINE_END), 4, HAL_MAX_DELAY);
  }
  HAL_UART_Transmit(console_uart, (uint8_t *)buf, len, HAL_MAX_DELAY);
  if (console_internal_depth == 0) console_dirty = 1;
  return len;
}

static void console_print(const char *str)
{
  printf("%s", str);
  fflush(stdout);
}

extern int console_execute(int argc, const char * const *argv);

/* Optional tab-completion provider. If the application defines it, microrl uses
 * it; if not, the weak symbol resolves to NULL and completion is simply off. */
extern char ** console_complete(int argc, const char * const *argv) __attribute__((weak));

void console_rx_callback(uint8_t ch)
{
  /* Ctrl-L brings the prompt back for a terminal attached after it was drawn
   * (a UART has no connect event to trigger this). The typed line is kept, and
   * the key is not passed to microrl, which would insert it as text. */
  if (ch == 0x0C)
  {
    console_internal_depth++;
    printf("\r\n");
    console_internal_depth--;
    console_redraw_prompt();
    return;
  }
  /* microrl echoes the char and may redraw the line; that output is not async. */
  console_internal_depth++;
  microrl_insert_char(&console_rl, ch);
  console_internal_depth--;
}

void console_task(void)
{
  while (rx_tail != rx_head)
  {
    uint8_t ch = rx_fifo[rx_tail & RX_FIFO_MASK];
    rx_tail++;
    console_rx_callback(ch);
  }
  __disable_irq();
  int dirty = console_dirty;
  console_dirty = 0;
  __enable_irq();
  if (dirty) console_redraw_prompt();
}

void console_redraw_prompt(void)
{
  /* Cursor is at column 0 of a fresh line (async prints end with \r\n). */
  console_internal_depth++;
  printf(ANSI_CLEAR_LINE_END);           /* clear to end of line */
  console_print(console_rl.prompt_str);  /* colored prompt */
  for (int i = 0; i < console_rl.cmdlen; i++) {
    char c = console_rl.cmdline[i] ? console_rl.cmdline[i] : ' ';
    putchar(c);
  }
  int back = console_rl.cmdlen - console_rl.cursor;
  if (back > 0) printf(ANSI_CURSOR_LEFT_FMT, back);  /* restore cursor position */
  fflush(stdout);
  console_internal_depth--;
}

/* ===== Live-metrics dashboard ============================================= *
 * The dashboard is the top dash_rows rows of the screen; logs and the prompt
 * live in the DECSTBM scroll region below it. dash_row is the 1-based row the
 * next console_dash_println() will write during a frame; dash_rows is the height
 * the last completed frame settled on (and thus the current scroll-region top).
 */
static int dash_row;   /* rows used so far in the frame in progress (reset by begin) */
static int dash_rows;  /* height of the dashboard / reserved band; 0 = no dashboard */

/* Re-set the scroll region to leave the top R rows fixed (R==0 releases it back
 * to full screen). DECSTBM homes the cursor and a resized band may now sit over
 * stale text or the prompt, so clear the band and re-establish the prompt at the
 * bottom of the scroll region. \033[999;1H clamps to the last line, so the screen
 * height never has to be known. Only called when R changes (a show_* toggled). */
static void console_dash_set_region(int R)
{
  console_internal_depth++;
  if (R > 0) printf(ANSI_SCROLL_TOP_FMT, R + 1);   /* region = rows R+1..bottom */
  else       printf(ANSI_SCROLL_FULL);             /* full-screen scrolling */
  for (int i = 1; i <= R; i++) printf(ANSI_CURSOR_ROW_FMT ANSI_CLEAR_LINE_END, i);  /* clear the band */
  printf(ANSI_CURSOR_BOTTOM);              /* park at the bottom row */
  console_internal_depth--;
  console_redraw_prompt();                 /* prompt back in the scroll region */
}

void console_dash_begin(void)
{
  console_internal_depth++;
  /* Hide the cursor and save its prompt position: the frame moves the cursor all
   * over the dashboard while drawing, which would otherwise be visible as a blink
   * up in the report band. It is revealed again at the prompt by console_dash_end. */
  printf(ANSI_CURSOR_HIDE ANSI_CURSOR_SAVE);
  fflush(stdout);
  dash_row = 1;
}

void console_dash_println(const char *fmt, ...)
{
  printf(ANSI_CURSOR_ROW_FMT, dash_row);   /* absolute: ignores the scroll margins */
  va_list ap;
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
  printf(ANSI_CLEAR_LINE_END);      /* clear to end of line, no full clear */
  dash_row++;
}

void console_dash_end(unsigned period_ms)
{
  console_dash_println("---- live every %u ms "
                       "------------------------------------------", period_ms);
  int used = dash_row - 1;
  /* Wipe rows a previous, taller frame left behind. */
  for (int i = dash_row; i <= dash_rows; i++) printf(ANSI_CURSOR_ROW_FMT ANSI_CLEAR_LINE_END, i);
  printf(ANSI_CURSOR_RESTORE);        /* restore cursor to the prompt */
  if (used != dash_rows)
  {
    dash_rows = used;
    console_dash_set_region(used);   /* self-guarded; also leaves cursor at the prompt */
  }
  printf(ANSI_CURSOR_SHOW);     /* reveal the cursor, now back at the prompt */
  fflush(stdout);
  console_internal_depth--;
}

void console_dash_hide(void)
{
  if (dash_rows == 0) return;
  dash_rows = 0;
  dash_row = 1;
  console_dash_set_region(0);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == console_uart) {
    rx_fifo[rx_head & RX_FIFO_MASK] = console_rx_char;
    rx_head++;
    HAL_UART_Receive_IT(console_uart, &console_rx_char, 1);
  }
}

void console_init(UART_HandleTypeDef *huart, IRQn_Type irqn)
{
  console_uart = huart;

  printf("\r\n=== Bandolibre Console ===\r\n");
  fflush(stdout);

  if (!NVIC_GetEnableIRQ(irqn)) {
    printf("WARNING: IRQ %d not enabled in NVIC, console RX will not work.\r\n"
           "Enable the UART global interrupt in the .ioc NVIC settings and regenerate code.\r\n",
           (int)irqn);
    fflush(stdout);
  }

  microrl_init(&console_rl, console_print);
  microrl_set_execute_callback(&console_rl, console_execute);
  microrl_set_complete_callback(&console_rl, console_complete);

  HAL_UART_Receive_IT(console_uart, &console_rx_char, 1);
}
