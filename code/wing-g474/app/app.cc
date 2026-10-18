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
// Minimum time between the starts of two scans, in us; 0 = scan as fast as
// possible. Written by the "scan_period" command.
static volatile uint32_t g_scan_min_period_us = 500;

// Settling delay after switching the SEL mux, in ns. Written by the "mux"
// command. The first mux position also counts towards the rail ramp-up, so
// settle + mux is the time from the FET turning on to the first conversion.
// Defaults come from the "tune" command: the rail needs ~14 us, the mux far
// less; 20 us + 1 us gives ~1.5x margin on the rail.
static volatile uint32_t g_mux_settle_ns = 600;

// Settling delay after the Hall supply FET turns on, in us. Written by the
// "settle" command. See the mux delay above for how the default was chosen.
static volatile uint32_t g_hall_settle_us = 10;

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
  uint32_t settle_start = cycles();
  uint32_t settle_cycles = g_hall_settle_us * (SystemCoreClock / 1000000u);
  while (cycles() - settle_start < settle_cycles) { }

  for (int sel = 0; sel < HALL_NUM_SEL; sel++)
  {
    HAL_GPIO_WritePin(SEL0_GPIO_Port, SEL0_Pin, (sel & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, (sel & 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    uint32_t mux_start = cycles();
    uint32_t mux_cycles = g_mux_settle_ns * (SystemCoreClock / 1000000u) / 1000u;
    while (cycles() - mux_start < mux_cycles) { }

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


// ---- tune: explore the settle / mux delays and recommend values ----------
//
// Scans the keyboard for a grid of (settle, mux) delays and compares each point
// with a reference taken at the slowest, safest delays:
//  - noise: per-key standard deviation of the samples;
//  - bias: shift of each key's mean from its reference mean. A delay that is too
//    short usually settles to the wrong value rather than a noisy one.
// A point passes when it is no worse than the test's own floor (see below).
// Keys that read ~0 (not populated) or pinned at full scale have zero standard
// deviation whatever the delays are, so they are left out of every average.
// Values are fixed point, in tenths of an ADC count (x10) unless noted.
//
// The grid is dense where the answer usually is (rail settles in ~15 us; the
// mux needs ~0 to 1 us on the right wing and ~5 us on the left one) and sparse
// above. The settle range goes up to 300 us, below the 400 us of the reference,
// to catch keys that settle slowly on the power rail. Keys may be held down while it
// runs, as long as they stay still: a held key gives the mux something to
// settle, which a keyboard at rest barely does.

#define TUNE_KEYS        (HALL_NUM_ADC * HALL_SLOTS_PER_ADC)
#define TUNE_REF_SETTLE_US 400
#define TUNE_REF_MUX_NS    16000
#define TUNE_MUX_PER_SWEEP HALL_NUM_SEL
// A key counts as connected when its reference mean is in [3.2, 4090] counts.
#define TUNE_CONNECTED_MIN_X10 32u
#define TUNE_CONNECTED_MAX_X10 40900u
// The reference is measured twice. The second run compared with the first is
// the floor of the test itself (drift, a noisy key, interference), and a point
// passes when it is no worse than that floor by more than:
//  - noise: 25% + 0.5 count of the reference noise;
//  - average shift: the floor's average shift + 0.2 count, or half the
//    reference noise, whichever is larger;
//  - failing keys: the floor's failing keys + 10% of the keys (at least 3); it
//    scatters by a few keys from run to run, hence the allowance.
// A key fails when its shift exceeds 0.5 count plus 3 standard errors of the
// difference of the two means, or half its own standard deviation if larger,
// so a noisy key is not blamed for its noise.
#define TUNE_NOISE_SLACK_X10   5u
#define TUNE_KEY_FLOOR_X10     5u
#define TUNE_BIAS_SLACK_X10    2u
// A shift smaller than this share of the noise (standard deviation) is
// ignored: it hides inside the scatter of every single reading. Used as a
// lower bound for both the average-shift limit and each key's threshold.
#define TUNE_SHIFT_SD_PERCENT  50u
// Recommended values are the best passing point times this safety margin (%).
// The mux is never recommended below TUNE_MUX_MIN_NS: a keyboard at rest hides
// most of the mux settling error.
#define TUNE_MARGIN_PERCENT   150u
#define TUNE_MUX_MIN_NS       1000u

static const uint16_t k_tune_settle_us[] = { 0, 4, 8, 10, 12, 15, 20, 30, 50, 75, 100, 150, 200, 300 };
static const uint16_t k_tune_mux_ns[]    = { 0, 250, 500, 1000, 2000, 3000, 4000, 6000, 8000, 12000 };
#define TUNE_NSETTLE ((unsigned)(sizeof k_tune_settle_us / sizeof *k_tune_settle_us))
#define TUNE_NMUX    ((unsigned)(sizeof k_tune_mux_ns / sizeof *k_tune_mux_ns))

typedef struct
{
  uint32_t mean_x10[TUNE_KEYS];
  uint32_t sd_x10[TUNE_KEYS];
  uint32_t scan_us;
} tune_stat_t;

static uint32_t isqrt_u64(uint64_t v)
{
  uint64_t r = 0, bit = 1ull << 62;
  while (bit > v) bit >>= 2;
  while (bit)
  {
    if (v >= r + bit) { v -= r + bit; r = (r >> 1) + bit; }
    else r >>= 1;
    bit >>= 2;
  }
  return (uint32_t)r;
}

static void print_x10(uint32_t v)
{
  printf("%lu.%lu", (unsigned long)(v / 10), (unsigned long)(v % 10));
}

// Same, right-aligned in a field of the given width (for table columns).
static void print_x10w(uint32_t v, int width)
{
  char buf[16];
  snprintf(buf, sizeof buf, "%lu.%lu", (unsigned long)(v / 10), (unsigned long)(v % 10));
  printf("%*s", width, buf);
}

// Takes n scans at the given delays and fills out with per-key mean and
// standard deviation. Leaves g_hall_settle_us / g_mux_settle_ns set to them.
static void tune_measure(uint32_t settle_us, uint32_t mux_ns, uint32_t n, tune_stat_t *out)
{
  static uint16_t buf[HALL_NUM_ADC][HALL_SLOTS_PER_ADC];
  uint32_t sum[TUNE_KEYS] = {0};
  uint64_t sumsq[TUNE_KEYS] = {0};

  g_hall_settle_us = settle_us;
  g_mux_settle_ns = mux_ns;

  hall_keyboard_scan(buf);   // warm-up: first sweep after a delay change
  uint32_t t0 = cycles();
  for (uint32_t i = 0; i < n; i++)
  {
    hall_keyboard_scan(buf);
    const uint16_t *flat = &buf[0][0];
    for (int k = 0; k < TUNE_KEYS; k++)
    {
      sum[k] += flat[k];
      sumsq[k] += (uint32_t)flat[k] * flat[k];
    }
  }
  uint32_t elapsed = cycles() - t0;
  out->scan_us = cycles_to_us((uint64_t)elapsed / n);

  for (int k = 0; k < TUNE_KEYS; k++)
  {
    out->mean_x10[k] = (uint32_t)(((uint64_t)sum[k] * 10 + n / 2) / n);
    uint64_t var_x100 = (sumsq[k] * n - (uint64_t)sum[k] * sum[k]) * 100u / ((uint64_t)n * n);
    out->sd_x10[k] = isqrt_u64(var_x100);
  }
}

// Compares a measurement with the reference over the connected keys: average
// noise, average shift of the means, and the number of keys whose shift is
// beyond 0.5 count plus 3 standard errors of the difference of the two means.
// Optionally also gives each key's shift (x10, clipped to 255) and a bit mask
// of the failing keys.
static void tune_compare(const tune_stat_t *cur, const tune_stat_t *ref, const bool *connected,
                         uint32_t n, uint32_t nconn,
                         uint32_t *noise_avg, uint32_t *bias_avg, uint32_t *failing,
                         uint8_t *key_shift = nullptr, uint64_t *key_fail = nullptr)
{
  if (key_fail) *key_fail = 0;
  uint32_t noise_sum = 0, bias_sum = 0, bad = 0;
  for (int k = 0; k < TUNE_KEYS; k++)
  {
    if (!connected[k]) continue;
    uint32_t b = cur->mean_x10[k] > ref->mean_x10[k] ? cur->mean_x10[k] - ref->mean_x10[k]
                                                       : ref->mean_x10[k] - cur->mean_x10[k];
    // Standard error of the difference, x100, then the threshold in x10.
    uint64_t var_sum = (uint64_t)cur->sd_x10[k] * cur->sd_x10[k] +
                       (uint64_t)ref->sd_x10[k] * ref->sd_x10[k];
    uint32_t se_x100 = isqrt_u64(var_sum * 100u / n);
    uint32_t tol = TUNE_KEY_FLOOR_X10 + (3u * se_x100 + 5u) / 10u;
    uint32_t sd_tol = ref->sd_x10[k] * TUNE_SHIFT_SD_PERCENT / 100u;
    if (tol < sd_tol) tol = sd_tol;
    noise_sum += cur->sd_x10[k];
    bias_sum += b;
    if (key_shift) key_shift[k] = b > 255 ? 255 : (uint8_t)b;
    if (b > tol)
    {
      bad++;
      if (key_fail) *key_fail |= 1ull << k;
    }
  }
  *noise_avg = noise_sum / nconn;
  *bias_avg = bias_sum / nconn;
  *failing = bad;
}

static void tune_run(uint32_t n)
{
  const uint32_t saved_settle = g_hall_settle_us;
  const uint32_t saved_mux = g_mux_settle_ns;
  tune_stat_t ref, ref2, ref_row, cur;
  static bool pass[TUNE_NSETTLE][TUNE_NMUX];
  static uint8_t key_shift[TUNE_NSETTLE][TUNE_NMUX][TUNE_KEYS];
  static uint64_t key_fail[TUNE_NSETTLE][TUNE_NMUX];

  printf("tune: keep the keyboard still. %lu scans per point, %u points.\r\n",
         (unsigned long)n, TUNE_NSETTLE * TUNE_NMUX);
  tune_measure(TUNE_REF_SETTLE_US, TUNE_REF_MUX_NS, n, &ref);

  bool connected[TUNE_KEYS];
  uint32_t nconn = 0, ref_noise_sum = 0, noisiest = 0;
  printf("excluded keys (flat index, mean):");
  for (int k = 0; k < TUNE_KEYS; k++)
  {
    connected[k] = ref.mean_x10[k] >= TUNE_CONNECTED_MIN_X10 &&
                   ref.mean_x10[k] <= TUNE_CONNECTED_MAX_X10;
    if (connected[k])
    {
      nconn++;
      ref_noise_sum += ref.sd_x10[k];
      if (ref.sd_x10[k] > ref.sd_x10[noisiest] || !connected[noisiest]) noisiest = k;
    }
    else { printf(" %d(", k); print_x10(ref.mean_x10[k]); printf(")"); }
  }
  printf("%s\r\n", nconn == TUNE_KEYS ? " none" : "");
  if (nconn == 0)
  {
    printf("tune: no connected key, aborting\r\n");
    g_hall_settle_us = saved_settle; g_mux_settle_ns = saved_mux;
    return;
  }
  // Second reference run: its difference from the first is the test's floor.
  tune_measure(TUNE_REF_SETTLE_US, TUNE_REF_MUX_NS, n, &ref2);
  uint32_t ref2_noise, base_bias, base_failing;
  tune_compare(&ref2, &ref, connected, n, nconn, &ref2_noise, &base_bias, &base_failing);
  uint32_t ref_noise = (ref_noise_sum / nconn + ref2_noise) / 2;
  uint32_t noise_limit = ref_noise + ref_noise / 4 + TUNE_NOISE_SLACK_X10;
  uint32_t bias_limit = base_bias + TUNE_BIAS_SLACK_X10;
  if (bias_limit < ref_noise * TUNE_SHIFT_SD_PERCENT / 100u)
    bias_limit = ref_noise * TUNE_SHIFT_SD_PERCENT / 100u;
  uint32_t bad_keys_allowed = base_failing + (nconn / 10 > 3 ? nconn / 10 : 3);

  printf("%lu keys used. reference (settle %u us, mux %u ns): scan %lu us, noise ",
         (unsigned long)nconn, TUNE_REF_SETTLE_US, TUNE_REF_MUX_NS, (unsigned long)ref.scan_us);
  print_x10(ref_noise);
  printf(" counts\r\nnoisiest key: %lu (noise ", (unsigned long)noisiest);
  print_x10(ref.sd_x10[noisiest]);
  printf(" counts)\r\ntest floor (2nd reference vs 1st): shift ");
  print_x10(base_bias);
  printf(" counts, %lu failing keys\r\nlimits: noise ", (unsigned long)base_failing);
  print_x10(noise_limit);
  printf(", shift ");
  print_x10(bias_limit);
  printf(", failing keys %lu\r\n", (unsigned long)bad_keys_allowed);
  if (base_failing > nconn / 4)
    printf("warning: the two reference runs disagree, the keyboard probably moved\r\n");
  printf("settle_us mux_ns | rail_us scan_us | noise shift keys | result\r\n");

  for (unsigned si = 0; si < TUNE_NSETTLE; si++)
  {
    // The readings drift slowly during the run (warming, supply), so every row
    // is compared with a reference measured just before it, not the first one.
    // The drift of that reference since the start is printed to make it visible.
    tune_measure(TUNE_REF_SETTLE_US, TUNE_REF_MUX_NS, n, &ref_row);
    {
      uint32_t d_noise, d_bias, d_failing;
      tune_compare(&ref_row, &ref, connected, n, nconn, &d_noise, &d_bias, &d_failing);
      printf("-- reference re-measured: drift since the start ");
      print_x10(d_bias);
      printf(" counts, %lu failing keys\r\n", (unsigned long)d_failing);
    }
    for (unsigned mi = 0; mi < TUNE_NMUX; mi++)
    {
      uint32_t settle = k_tune_settle_us[si], mux = k_tune_mux_ns[mi];
      tune_measure(settle, mux, n, &cur);

      uint32_t noise_avg, bias_avg, failing;
      tune_compare(&cur, &ref_row, connected, n, nconn, &noise_avg, &bias_avg, &failing,
                   key_shift[si][mi], &key_fail[si][mi]);
      bool noise_ok = noise_avg <= noise_limit;
      bool shift_ok = bias_avg <= bias_limit;
      bool keys_ok = failing <= bad_keys_allowed;
      bool ok = noise_ok && shift_ok && keys_ok;
      pass[si][mi] = ok;

      // rail_us: time from the FET turning on to the first conversion.
      printf("%9lu %6lu | ", (unsigned long)settle, (unsigned long)mux);
      print_x10w(settle * 10u + mux / 100u, 7);
      printf(" %7lu | ", (unsigned long)cur.scan_us);
      print_x10w(noise_avg, 5); printf(" ");
      print_x10w(bias_avg, 5);
      printf(" %4lu | ", (unsigned long)failing);
      if (ok)
      {
        printf("ok");
      }
      else
      {
        printf("BAD");
        if (!shift_ok)
        {
          printf(": shift "); print_x10(bias_avg); printf(" > "); print_x10(bias_limit);
        }
        if (!keys_ok)
          printf(": keys %lu > %lu", (unsigned long)failing, (unsigned long)bad_keys_allowed);
        if (!noise_ok)
        {
          printf(": noise "); print_x10(noise_avg); printf(" > "); print_x10(noise_limit);
        }
      }
      printf("\r\n");
    }
  }

  g_hall_settle_us = saved_settle;
  g_mux_settle_ns = saved_mux;

  // Best point = cheapest sweep among passing points where at least 2 of the 3
  // larger neighbours (next settle, next mux, both) also pass, so a single lucky
  // run is not enough but one unlucky neighbour does not discard the point.
  uint32_t best_cost = 0xFFFFFFFFu, best_settle = 0, best_mux = 0;
  unsigned best_mi = TUNE_NMUX - 1;   // column used for the per-key report
  for (unsigned si = 0; si < TUNE_NSETTLE; si++)
    for (unsigned mi = 0; mi < TUNE_NMUX; mi++)
    {
      unsigned good = (si + 1 >= TUNE_NSETTLE || pass[si + 1][mi]) +
                      (mi + 1 >= TUNE_NMUX || pass[si][mi + 1]) +
                      (si + 1 >= TUNE_NSETTLE || mi + 1 >= TUNE_NMUX || pass[si + 1][mi + 1]);
      bool solid = pass[si][mi] && good >= 2;
      uint32_t cost = k_tune_settle_us[si] * 1000u + TUNE_MUX_PER_SWEEP * k_tune_mux_ns[mi];
      if (solid && cost < best_cost)
      {
        best_cost = cost; best_settle = k_tune_settle_us[si]; best_mux = k_tune_mux_ns[mi];
        best_mi = mi;
      }
    }

  // Per-key sensitivity to the settle delay, at the mux of the best point (or
  // the largest mux when there is none). A key "needs" the smallest settle from
  // which it matches its reference at that settle and the next two.
  {
    unsigned need[TUNE_KEYS];
    unsigned order[TUNE_KEYS], nsens = 0;
    for (int k = 0; k < TUNE_KEYS; k++)
    {
      need[k] = 0;
      if (!connected[k]) continue;
      unsigned si = 0;
      for (; si < TUNE_NSETTLE; si++)
      {
        bool good = true;
        for (unsigned j = si; j < TUNE_NSETTLE && j <= si + 2; j++)
          if ((key_fail[j][best_mi] >> k) & 1u) good = false;
        if (good) break;
      }
      need[k] = si;   // TUNE_NSETTLE = never matched inside the grid
      if (si > 0) order[nsens++] = k;
    }
    for (unsigned a = 0; a + 1 < nsens; a++)   // most sensitive first
      for (unsigned b = a + 1; b < nsens; b++)
        if (need[order[b]] > need[order[a]]) { unsigned t = order[a]; order[a] = order[b]; order[b] = t; }

    // A key on mux position sel converts settle + (sel + 1) * mux after the FET
    // turns on (the mux delay is waited before every position, the first too),
    // so later positions get more rail time than earlier ones.
    unsigned mux_ns = k_tune_mux_ns[best_mi];
    printf("\r\nkeys sensitive to a low hall power settle (mux fixed at %u ns): ", mux_ns);
    if (nsens == 0)
      printf("none, every key matches its reference from settle %u us\r\n", k_tune_settle_us[0]);
    else
    {
      printf("%u of %lu keys do not match at settle %u us\r\n", nsens, (unsigned long)nconn,
             k_tune_settle_us[0]);
      printf("  key  adc slot sel | power settle needed | rail time then | shift at settle %u us\r\n",
             k_tune_settle_us[0]);
      for (unsigned i = 0; i < nsens && i < 15; i++)
      {
        unsigned k = order[i];
        unsigned slot = k % HALL_SLOTS_PER_ADC, sel = slot / HALL_NUM_RANK;
        printf("  %3u  %3u %4u %3u | ", k, k / HALL_SLOTS_PER_ADC, slot, sel);
        if (need[k] >= TUNE_NSETTLE)
          printf("%16s    | %11s", "> grid", "-");
        else
        {
          printf("%16u us | ", k_tune_settle_us[need[k]]);
          print_x10w(k_tune_settle_us[need[k]] * 10u + (sel + 1u) * mux_ns / 100u, 9);
          printf(" us");
        }
        printf(" | ");
        print_x10w(key_shift[0][best_mi][k], 8);
        printf("\r\n");
      }
      if (nsens > 15)
        printf("  ... and %u more\r\n", nsens - 15);
    }
    printf("\r\n");
  }

  if (best_cost == 0xFFFFFFFFu)
  {
    printf("tune: no solid passing point. If the shift keeps falling towards the largest mux\r\n"
           "in the table, the mux needs longer than the grid allows (widen k_tune_mux_ns).\r\n"
           "Otherwise keys moved or the rail is too noisy.\r\n");
    return;
  }
  uint32_t rec_settle = (best_settle * TUNE_MARGIN_PERCENT + 99) / 100;
  uint32_t rec_mux = (best_mux * TUNE_MARGIN_PERCENT + 99) / 100;
  if (rec_mux < TUNE_MUX_MIN_NS) rec_mux = TUNE_MUX_MIN_NS;
  printf("fastest solid point: settle %lu us, mux %lu ns (rail time ",
         (unsigned long)best_settle, (unsigned long)best_mux);
  print_x10(best_settle * 10u + best_mux / 100u);
  printf(" us)\r\n");
  printf("recommended (x%lu%% margin, mux at least %lu ns): settle %lu, mux %lu\r\n",
         (unsigned long)TUNE_MARGIN_PERCENT, (unsigned long)TUNE_MUX_MIN_NS,
         (unsigned long)rec_settle, (unsigned long)rec_mux);
  printf("rail time at the recommendation: ");
  print_x10(rec_settle * 10u + rec_mux / 100u);
  printf(" us\r\n");
  printf("try:  settle %lu  then  mux %lu   (values restored to %lu us / %lu ns)\r\n",
         (unsigned long)rec_settle, (unsigned long)rec_mux,
         (unsigned long)saved_settle, (unsigned long)saved_mux);
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


static const char *const g_cmds[] = { "blink", "report", "scan_period", "settle", "mux", "tune", "key", "stat", "rate", "reset", "help", NULL };
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
  else if (strcmp(argv[0], "scan_period") == 0)
  {
    if (argc >= 2)
    {
      char *end;
      unsigned long us = strtoul(argv[1], &end, 10);
      if (end == argv[1] || *end || us > 1000000ul)
      {
        printf("usage: scan_period <us>  (0-1000000, 0 = no limit)\r\n");
        return 0;
      }
      g_scan_min_period_us = us;
    }
    printf("scan_period: %lu us\r\n", (unsigned long)g_scan_min_period_us);
  }
  else if (strcmp(argv[0], "settle") == 0)
  {
    if (argc >= 2)
    {
      char *end;
      unsigned long us = strtoul(argv[1], &end, 10);
      if (end == argv[1] || *end || us > 10000ul)
      {
        printf("usage: settle <us>  (0-10000)\r\n");
        return 0;
      }
      g_hall_settle_us = us;
    }
    printf("settle: %lu us\r\n", (unsigned long)g_hall_settle_us);
  }
  else if (strcmp(argv[0], "mux") == 0)
  {
    if (argc >= 2)
    {
      char *end;
      unsigned long ns = strtoul(argv[1], &end, 10);
      if (end == argv[1] || *end || ns > 100000ul)
      {
        printf("usage: mux <ns>  (0-100000)\r\n");
        return 0;
      }
      g_mux_settle_ns = ns;
    }
    printf("mux: %lu ns\r\n", (unsigned long)g_mux_settle_ns);
  }
  else if (strcmp(argv[0], "tune") == 0)
  {
    unsigned long n = 500;
    if (argc >= 2)
    {
      char *end;
      n = strtoul(argv[1], &end, 10);
      if (end == argv[1] || *end || n < 10 || n > 2000)
      {
        printf("usage: tune [scans per point]  (10-2000, default 500)\r\n");
        return 0;
      }
    }
    tune_run(n);
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
    printf("scan_period <us>  minimum time between scans, 0 = no limit\r\n");
    printf("settle <us>   delay after the hall supply turns on\r\n");
    printf("mux <ns>      delay after switching the mux select\r\n");
    printf("tune [n]      scan with many settle/mux values and recommend some\r\n");
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

  // Throttle: wait until the minimum period since the previous scan started.
  // The wait is spent in the loop (console etc. keep running) and shows up as
  // "other" in the report.
  static uint32_t last_scan_start = 0;
  uint32_t t0 = cycles();
  uint32_t min_cycles = g_scan_min_period_us * (SystemCoreClock / 1000000u);
  if (t0 - last_scan_start >= min_cycles)
  {
    last_scan_start = t0;
    hall_keyboard_scan(g_frame.sensors);
    uint32_t t1 = cycles();
    transmit_keyboard_frame(&g_frame);
    uint32_t t2 = cycles();
    g_prof.scan += t1 - t0;
    g_prof.tx += t2 - t1;
    g_prof.loops++;
    reporting_task(&g_frame);
    g_prof.report += cycles() - t2;
  }

  console_task();
}
