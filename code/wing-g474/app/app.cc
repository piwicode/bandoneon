#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern "C" {
#include "main.h"
#include "console.h"
#include "hall_report.h"
#include "app/app.h"
#include "spi_link.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;
extern ADC_HandleTypeDef hadc5;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
}

static uint8_t read_wing_id(void);

#define HALL_NUM_SEL   4
#define HALL_NUM_ADC   5
#define HALL_NUM_RANK  2
// Slots per ADC across a full sweep: one (sel, rank) pair each.
#define HALL_SLOTS_PER_ADC (HALL_NUM_SEL * HALL_NUM_RANK)
#define HALL_DEAD_ZONE 64
// Settling delay after switching the SEL mux, in spin-loop iterations.
#define MUX_LATENCY_CYCLES 160
// Settling delay after the Hall supply FET turns on, in spin-loop iterations.
// Untuned starting value: shorten until readings change, then add margin.
#define HALL_POWER_SETTLE_CYCLES 2000

// Number of hall sensor slots == number of physical keys on this wing.
#define KEYBOARD_NUM_KEYS (HALL_NUM_ADC * HALL_SLOTS_PER_ADC)

typedef struct {
  uint16_t wing_id;
  uint16_t sensors[HALL_NUM_ADC][HALL_SLOTS_PER_ADC];
} Frame;

// Layout matches the per-ADC DMA buffer: [adc][sel*HALL_NUM_RANK + rank]. This
// is also exactly the flattened key order the SPI frame and hall_report use
// (index = adc * HALL_SLOTS_PER_ADC + slot), so Frame.sensors can be passed to
// the shared report as a flat uint16_t[KEYBOARD_NUM_KEYS].
_Static_assert(KEYBOARD_NUM_KEYS == HALL_REPORT_NUM_KEYS,
               "wing key count must match the shared hall report layout");
_Static_assert(KEYBOARD_NUM_KEYS == SPI_LINK_NUM_KEYS,
               "wing key count must match SPI link protocol");
_Static_assert(sizeof(Frame) == SPI_LINK_FRAME_WORDS * sizeof(uint16_t),
               "Frame struct size must match SPI link protocol frame size");

// Running per-key statistics, fed by hall_stats_update() every sweep.
static hall_stats_t g_hall_stats;

// Reusable frame for SPI transmission.
static Frame g_frame;

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
  int remaining;
  uint32_t last_tick;
  int led_on;
} blink_state_t;

static volatile blink_state_t g_blink;

#define BLINK_HALF_PERIOD_MS  200

// Live-report period in ms; written by the "rate" command, read by reporting_task.
// Default 4 Hz (250 ms). Clamped to [33, 10000] ms (0.1–30 Hz) on write.
static volatile uint32_t g_report_period_ms = 250;
static volatile int g_show_keys = 0;

// Toggled by the "report" command; off stops the periodic dashboard (and its
// UART stall) entirely.
static volatile int g_report_enabled = 0;
static uint32_t poll_count = 0;       // scans since the last report
static uint32_t last_poll_tick = 0;   // tick of the last report

// Toggled by the "show_stats" console command; adds min/max/stddev tables
// below the live readings while g_show_keys is also set.
static volatile int g_show_stats = 0;

// Per-window CPU time, in DWT cycles, spent in each part of main_task. "other"
// in the report is the window total minus these (blink, buttons, console,
// loop overhead). The report's own cost is only known after it ran, so it is
// charged to the following window.
static struct
{
  uint64_t scan, tx, report;
  uint32_t loops;
  uint32_t window_start;
} g_prof;

static inline uint32_t cycles(void) { return DWT->CYCCNT; }

static uint32_t cycles_to_us(uint64_t c)
{
  return (uint32_t)(c * 1000000u / SystemCoreClock);
}


// The .ioc only sets the board layer (pins -> analog, DMA1_Ch1..5 -> ADC1..5,
// circular periph->mem halfword, MINC, NVIC). It leaves each ADC in single-
// conversion mode (ScanConvMode disabled, NbrOfConversion=1, DMAContinuousReq
// disabled). We switch each ADC to a 2-rank scan once at startup.

// Switch one ADC to a 2-rank regular scan feeding the (circular) DMA.
static void adc_set_scan2(ADC_HandleTypeDef *hadc, uint32_t chA, uint32_t chB)
{
  HAL_ADC_Stop(hadc);
  hadc->Init.ScanConvMode          = ADC_SCAN_ENABLE;
  hadc->Init.NbrOfConversion       = 2;
  hadc->Init.DMAContinuousRequests = ENABLE;   // match DMA_CIRCULAR from the MSP
  HAL_ADC_Init(hadc);

  ADC_ChannelConfTypeDef c = {0};
  c.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  c.SingleDiff   = ADC_SINGLE_ENDED;
  c.OffsetNumber = ADC_OFFSET_NONE;
  c.Rank = ADC_REGULAR_RANK_1; c.Channel = chA; HAL_ADC_ConfigChannel(hadc, &c);
  c.Rank = ADC_REGULAR_RANK_2; c.Channel = chB; HAL_ADC_ConfigChannel(hadc, &c);
}

// Restarts the ADC's DMA channel at the start of buf. The channel is circular,
// so it must be disabled while CNDTR/CMAR are rewritten.
static void hall_dma_rearm(ADC_HandleTypeDef *adc, uint16_t *buf)
{
  DMA_Channel_TypeDef *ch = adc->DMA_Handle->Instance;
  CLEAR_BIT(ch->CCR, DMA_CCR_EN);
  ch->CNDTR = HALL_SLOTS_PER_ADC;
  ch->CMAR = (uint32_t)buf;
  SET_BIT(ch->CCR, DMA_CCR_EN);
}

// Performs one full sweep into hall_data. The Hall supply is only powered for
// the duration of the sweep (HALL_NEN is active low, high at boot = off).
static void hall_keyboard_scan(uint16_t hall_data[HALL_NUM_ADC][HALL_SLOTS_PER_ADC])
{
  static ADC_HandleTypeDef *const adcs[5] = { &hadc1, &hadc2, &hadc3, &hadc4, &hadc5 };

  // Re-point each DMA channel at the start of its buffer for the whole sweep
  // (8 = 4 sel x 2 ranks, straight into hall_data, whose layout already matches
  // the per-ADC buffer). Only the channel is rearmed, not the ADC, which keeps
  // the sweep aligned even after a stray conversion.
  for (int a = 0; a < HALL_NUM_ADC; a++)
    hall_dma_rearm(adcs[a], hall_data[a]);

  HAL_GPIO_WritePin(HALL_NEN_GPIO_Port, HALL_NEN_Pin, GPIO_PIN_RESET);
  for (volatile int i = 0; i < HALL_POWER_SETTLE_CYCLES; i++) __NOP();

  for (int sel = 0; sel < HALL_NUM_SEL; sel++)
  {
    HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, (sel & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, (sel & 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    for (volatile int i = 0; i < MUX_LATENCY_CYCLES; i++) __NOP();

    // ADCs stay enabled; ContinuousConvMode is disabled, so each ADC halts
    // after its 2-rank sequence and ADSTART is a clean per-sel trigger.
    for (auto adc : adcs)
      SET_BIT(adc->Instance->CR, ADC_CR_ADSTART);

    // Wait for all 5 sequences (2 conversions each) to finish.
    for (auto adc : adcs)
      while (READ_BIT(adc->Instance->CR, ADC_CR_ADSTART)) { }
  }

  HAL_GPIO_WritePin(HALL_NEN_GPIO_Port, HALL_NEN_Pin, GPIO_PIN_SET);

  // DMA wrote straight into hall_data in its [adc][sel*rank + rank] layout,
  // so no de-interleave step is needed.
}


// Builds and sends one full keyboard frame over the SPI link: word 0 is this
// wing's id, words 1..KEYBOARD_NUM_KEYS are the raw hall measurement for each
// key in flattened order (adc * HALL_SLOTS_PER_ADC + slot). The frame carries
// the full absolute keyboard state every cycle, so a dropped (CRC-failed)
// frame just leaves the receiver at its previous state until the next arrives.
static void transmit_keyboard_frame(Frame *frame)
{
  frame->wing_id = read_wing_id();

  uint32_t sr_before = hspi1.Instance->SR;
  uint32_t cr1_before = hspi1.Instance->CR1;
  HAL_StatusTypeDef spi_status = HAL_SPI_Transmit(&hspi1, (uint8_t *)frame, SPI_LINK_FRAME_WORDS, 500);
  if (spi_status != HAL_OK)
    printf("SPI transmit failed: size=%u status=%d error=0x%lx SR_before=0x%lx CR1_before=0x%lx SR=0x%lx CR1=0x%lx\r\n",
           SPI_LINK_FRAME_WORDS, (int)spi_status, (unsigned long)hspi1.ErrorCode,
           (unsigned long)sr_before, (unsigned long)cr1_before,
           (unsigned long)hspi1.Instance->SR, (unsigned long)hspi1.Instance->CR1);

  // HAL_SPI_Transmit leaves SPE set after a successful transfer, but with CRC
  // enabled the next call's SPI_RESET_CRC (toggling CRCEN) is only valid while
  // SPE=0 (RM0440). Disable here so each frame starts from a clean state.
  __HAL_SPI_DISABLE(&hspi1);

  // CRCNEXT is left set (HAL never clears it once the data write races ahead
  // of the shift register for single-word transfers), so the next transfer
  // starts with the peripheral expecting to send a CRC instead of data. Clear
  // it explicitly so each frame starts from a known state.
  CLEAR_BIT(hspi1.Instance->CR1, SPI_CR1_CRCNEXT);
}

// The wing id is strapped on fixed ID0..ID2 pins, so read it once on first
// use and cache it forever.
static uint8_t read_wing_id(void)
{
  static int cached = 0;
  static uint8_t id = 0;
  if (!cached)
  {
    if (HAL_GPIO_ReadPin(ID0_GPIO_Port, ID0_Pin) == GPIO_PIN_SET) id |= 1;
    if (HAL_GPIO_ReadPin(ID1_GPIO_Port, ID1_Pin) == GPIO_PIN_SET) id |= 2;
    if (HAL_GPIO_ReadPin(ID2_GPIO_Port, ID2_Pin) == GPIO_PIN_SET) id |= 4;
    cached = 1;
  }
  return id;
}

// Emits one shared-report line as a dashboard row.
static void hall_report_emit(void *ctx, const char *line)
{
  (void)ctx;
  console_dash_println("%s", line);
}


static const char *const g_cmds[] = { "blink", "report", "key", "stat", "rate", "reset", "help", NULL };
static const char *const g_blink_leds[] = { "fn0", "ready", NULL };

extern "C" char **console_complete(int argc, const char * const *argv)
{
  static const char *result[8];
  const char *const *pool;
  const char *prefix;

  if (argc <= 1)       { pool = g_cmds;       prefix = argc == 1 ? argv[0] : ""; }
  else if (strcmp(argv[0], "blink") == 0 && argc == 2)
                       { pool = g_blink_leds;  prefix = argv[1]; }
  else return NULL;

  int n = 0;
  size_t plen = strlen(prefix);
  for (int i = 0; pool[i] && n < 7; i++)
    if (strncmp(pool[i], prefix, plen) == 0)
      result[n++] = pool[i];
  result[n] = NULL;
  return n ? (char **)result : NULL;
}

extern "C" int console_execute(int argc, const char * const *argv)
{
  if (argc == 0)
    return 0;
  if (strcmp(argv[0], "blink") == 0 && argc >= 2)
  {
    GPIO_TypeDef *port = NULL;
    uint16_t pin = 0;
    if (strcmp(argv[1], "fn0") == 0)       { port = LED_FN0_GPIO_Port; pin = LED_FN0_Pin; }
    else if (strcmp(argv[1], "ready") == 0) { port = READY_GPIO_Port;   pin = READY_Pin;   }
    else printf("Unknown LED: %s (fn0, ready)\r\n", argv[1]);
    if (port)
    {
      g_blink.port = port;
      g_blink.pin = pin;
      g_blink.remaining = 5;
      g_blink.last_tick = HAL_GetTick();
      g_blink.led_on = 0;
    }
  }
  else if (strcmp(argv[0], "report") == 0)
  {
    g_report_enabled = !g_report_enabled;
    if (g_report_enabled)
    {
      memset(&g_prof, 0, sizeof g_prof);
      g_prof.window_start = cycles();
      poll_count = 0;
      last_poll_tick = HAL_GetTick();
    }
    else
      console_dash_hide();
    printf("report: %s\r\n", g_report_enabled ? "on" : "off");
  }
  else if (strcmp(argv[0], "key") == 0)
  {
    g_show_keys = !g_show_keys;
    printf("show_keys: %s\r\n", g_show_keys ? "on" : "off");
  }
  else if (strcmp(argv[0], "stat") == 0)
  {
    g_show_stats = !g_show_stats;
    printf("show_stats: %s\r\n", g_show_stats ? "on" : "off");
  }
  else if (strcmp(argv[0], "reset") == 0)
  {
    hall_stats_init(&g_hall_stats);
    printf("stats reset\r\n");
  }
  else if (strcmp(argv[0], "rate") == 0 && argc >= 2)
  {
    char *end;
    float hz = strtof(argv[1], &end);
    if (end == argv[1] || hz <= 0.0f)
    {
      printf("usage: rate <hz>  (e.g. rate 4, rate 0.5)\r\n");
    }
    else
    {
      uint32_t period = (uint32_t)(1000.0f / hz + 0.5f);
      if (period <    33) period =    33;   /* 30 Hz max */
      if (period > 10000) period = 10000;   /* 0.1 Hz min */
      g_report_period_ms = period;
      uint32_t hz_int  = 1000000u / period;
      uint32_t hz_frac = hz_int % 1000u;
      hz_int /= 1000u;
      printf("rate: %lu.%03lu Hz (period %lu ms)\r\n",
             (unsigned long)hz_int, (unsigned long)hz_frac, (unsigned long)period);
    }
  }
  else if (strcmp(argv[0], "help") == 0)
  {
    printf("blink fn0     blink FN0 LED 5 times\r\n");
    printf("blink ready   blink READY LED 5 times\r\n");
    printf("key           toggle hall sensor table\r\n");
    printf("stat          toggle min/max/stddev stats (requires key)\r\n");
    printf("report        toggle the periodic report\r\n");
    printf("rate <hz>     set live report rate (e.g. rate 4, rate 0.5)\r\n");
    printf("reset         reset hall stats (min/max/stddev)\r\n");
    printf("help          this message\r\n");
  }
  else
    printf("Unknown command: %s (try help)\r\n", argv[0]);
  return 0;
}

void main_init(void)
{
  console_init(&huart1, USART1_IRQn);

  // Free-running DWT cycle counter, used for the per-loop timing in the report.
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  g_prof.window_start = cycles();

  adc_set_scan2(&hadc1, ADC_CHANNEL_1, ADC_CHANNEL_2);
  adc_set_scan2(&hadc2, ADC_CHANNEL_3, ADC_CHANNEL_4);
  adc_set_scan2(&hadc3, ADC_CHANNEL_12, ADC_CHANNEL_1);
  adc_set_scan2(&hadc4, ADC_CHANNEL_4, ADC_CHANNEL_5);
  adc_set_scan2(&hadc5, ADC_CHANNEL_1, ADC_CHANNEL_2);

  // Enable each ADC and its DMA request once; the scan only rearms the DMA
  // channel afterwards. Start_DMA also fires one conversion, so let it finish
  // (it lands in g_frame and is overwritten by the first sweep). The per-half
  // and per-transfer DMA interrupts are not needed and would only add ISR
  // load to every sweep, so they are turned off; the error interrupt stays.
  static ADC_HandleTypeDef *const adcs[5] = { &hadc1, &hadc2, &hadc3, &hadc4, &hadc5 };
  for (int a = 0; a < HALL_NUM_ADC; a++)
  {
    HAL_ADC_Start_DMA(adcs[a], (uint32_t *)g_frame.sensors[a], HALL_SLOTS_PER_ADC);
    CLEAR_BIT(adcs[a]->DMA_Handle->Instance->CCR, DMA_CCR_TCIE | DMA_CCR_HTIE);
  }
  for (auto adc : adcs)
    while (READ_BIT(adc->Instance->CR, ADC_CR_ADSTART)) { }
}

static void blink_task(void)
{
  if (!g_blink.remaining || !g_blink.port)
    return;
  uint32_t now = HAL_GetTick();
  if (now - g_blink.last_tick < BLINK_HALF_PERIOD_MS)
    return;
  g_blink.last_tick = now;
  g_blink.led_on = !g_blink.led_on;
  HAL_GPIO_WritePin(g_blink.port, g_blink.pin,
                    g_blink.led_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
  if (!g_blink.led_on)
    g_blink.remaining--;
}

static void fn_button_task(void)
{
  static uint8_t fn0_prev = 0;
  uint8_t fn0 = HAL_GPIO_ReadPin(SW_FN0_GPIO_Port, SW_FN0_Pin) == GPIO_PIN_RESET ? 1 : 0;
  if (fn0 != fn0_prev)
  {
    fn0_prev = fn0;
    printf("FN0: %c\r\n", fn0 ? '1' : '0');
  }
}

static void reporting_task(Frame *frame)
{
  hall_stats_update(&g_hall_stats, (const uint16_t *)frame->sensors);

  if (!g_report_enabled)
    return;

  poll_count++;
  uint32_t now = HAL_GetTick();
  uint32_t elapsed = now - last_poll_tick;

  uint32_t period = g_report_period_ms;
  if (elapsed < period)
    return;

  uint32_t scan_freq_hz = poll_count * 1000u / elapsed;
  poll_count = 0;
  last_poll_tick = now;

  uint32_t now_cyc = cycles();
  uint64_t total = (uint32_t)(now_cyc - g_prof.window_start);
  uint64_t other = total - g_prof.scan - g_prof.tx - g_prof.report;
  uint32_t loops = g_prof.loops ? g_prof.loops : 1;
  uint32_t scan_us = cycles_to_us(g_prof.scan / loops);
  uint32_t tx_us = cycles_to_us(g_prof.tx / loops);
  uint32_t other_us = cycles_to_us(other / loops);
  uint32_t report_us = cycles_to_us(g_prof.report);
  uint32_t scan_pc = (uint32_t)(g_prof.scan * 100u / total);
  uint32_t tx_pc = (uint32_t)(g_prof.tx * 100u / total);
  uint32_t report_pc = (uint32_t)(g_prof.report * 100u / total);
  uint32_t other_pc = (uint32_t)(other * 100u / total);
  memset(&g_prof, 0, sizeof g_prof);
  g_prof.window_start = now_cyc;

  console_dash_begin();
  console_dash_println("wing_id=%u   scan rate %6lu Hz", (unsigned)frame->wing_id, (unsigned long)scan_freq_hz);
  console_dash_println("per loop: scan %5lu us %3lu%%  tx %5lu us %3lu%%  other %5lu us %3lu%%  | last report %lu us %lu%%",
                       (unsigned long)scan_us, (unsigned long)scan_pc,
                       (unsigned long)tx_us, (unsigned long)tx_pc,
                       (unsigned long)other_us, (unsigned long)other_pc,
                       (unsigned long)report_us, (unsigned long)report_pc);
  if (g_show_keys)
  {
    hall_report_keys((const uint16_t *)frame->sensors, hall_report_emit, NULL);
    if (g_show_stats)
      hall_report_stats(&g_hall_stats, NULL, hall_report_emit, NULL);
  }
  hall_stats_reset_window(&g_hall_stats);
  console_dash_end(1000u / period);
}

void main_task(void)
{
  blink_task();
  fn_button_task();

  uint32_t t0 = cycles();
  hall_keyboard_scan(g_frame.sensors);
  uint32_t t1 = cycles();
  transmit_keyboard_frame(&g_frame);
  uint32_t t2 = cycles();
  g_prof.scan += t1 - t0;
  g_prof.tx += t2 - t1;
  g_prof.loops++;
  reporting_task(&g_frame);
  g_prof.report += cycles() - t2;

  console_task();
}
