#include "bellow.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "bellow_classify.h"
#include "bellow_curve.h"
#include "buttons.h"
#include "console.h"
#include "fixed_format.h"
#include "keyboard.h"   /* L_MIDI_CH / R_MIDI_CH */
#include "main.h"
#include "midi.h"
#include "one_euro_filter.h"
#include "properties.h"
#include "report.h"
#include "usb_app.h"


/* ADC handles for the two hall sensors, defined by the CubeMX-generated main.c. */
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;

typedef struct {
  bellows_t direction;
  float     intensity;  /* 0.0..1.0, fraction of full travel (0 in BELLOWS_NEUTRAL) */
} bellow_output_t;

static bellow_output_t g_bellow_out = {.direction = BELLOWS_NEUTRAL, .intensity = 0};

/* Latest reading and intermediate values for monitoring */
static uint16_t g_bellow_last_hall0; // raw hall reading.
static uint16_t g_bellow_last_hall1; // raw hall reading.
static float    g_bellow_1e_out;   /* filtered hall-total, raw hall-total units */
static uint16_t g_bellow_cc_out;   /* CC value sent, 0..16383 */
/* CC#11 messages sent, and how many of them moved the opposite way from the
 * previous one, since the last report frame. A steady value that dribbles out
 * as +1/-1 steps shows up as many reversals. Reset by bellow_report(). */
static uint32_t g_bellow_cc_sent;
static uint32_t g_bellow_cc_reversals;

/* Combined-hall calibration: center is the at-rest reading, push/pull travel
 * how far the reading moves from it at full push/pull. The deadzone sets how far from center the
 * bellows must move to leave BELLOWS_NEUTRAL (no air moves there, so
 * note_table maps every key to NOTE_NONE). The hysteresis margin then has to
 * be given back before returning to NEUTRAL, so a bellows resting right at
 * the deadzone edge doesn't chatter between NEUTRAL and PUSH/PULL. Deadzone,
 * travel and the response curves come from the active program
 * (bellow_program, cycled by FN1); the center and the hysteresis are shared by
 * all programs. */

/* One program's bellow_p<n>_* properties. property_table.def generates them as
 * separate flat fields, so current_program() gathers the selected set. */
typedef struct {
  uint16_t dead, push_travel, pull_travel;
  uint16_t push_x1, push_y1, push_x2, push_y2;
  uint16_t pull_x1, pull_y1, pull_x2, pull_y2;
} bellow_program_t;

#define BELLOW_PROGRAM_FIELDS(n) (bellow_program_t){                                       \
  g_properties->bellow_p##n##_dead,                                                         \
  g_properties->bellow_p##n##_push_travel, g_properties->bellow_p##n##_pull_travel,         \
  g_properties->bellow_p##n##_push_curve_x1, g_properties->bellow_p##n##_push_curve_y1,     \
  g_properties->bellow_p##n##_push_curve_x2, g_properties->bellow_p##n##_push_curve_y2,     \
  g_properties->bellow_p##n##_pull_curve_x1, g_properties->bellow_p##n##_pull_curve_y1,     \
  g_properties->bellow_p##n##_pull_curve_x2, g_properties->bellow_p##n##_pull_curve_y2 }

_Static_assert(BELLOW_PROGRAM_COUNT == 3, "current_program() lists every program");

static bellow_program_t current_program(void)
{
  switch (g_properties->bellow_program)
  {
    case 1:  return BELLOW_PROGRAM_FIELDS(1);
    case 2:  return BELLOW_PROGRAM_FIELDS(2);
    default: return BELLOW_PROGRAM_FIELDS(0);
  }
}

bellows_t bellow_direction(void)
{
  return g_bellow_out.direction;
}

float bellow_intensity(void)
{
  return g_bellow_out.intensity;
}

void bellow_get_raw(uint16_t *hall0, uint16_t *hall1)
{
  *hall0 = g_bellow_last_hall0;
  *hall1 = g_bellow_last_hall1;
}

/* Emits CC#11 (Expression, paired with CC#43 as the 14-bit LSB) from the
 * effective intensity.
 *
 * The rate limit itself is bypassed -- sending immediately -- on any
 * transition into or out of rest (value or last_out == 0).
 
 * Table mode is the exception: the bellows rests there, so it must not drive
 * expression at all (see the branch below). */
static void bellow_send_cc(void)
{
  static uint16_t last_out;
  static uint32_t last_emit_ms;
  static bool table_prev;
  static int8_t last_dir;   /* direction of the previous CC sent: +1, -1 or 0 */

  /* Table mode plays with the bellows at rest, so the bellows drives nothing
   * here: its intensity is 0 and the CC=0 that follows would silence every note.
   * Expression is constant instead, at the same tablemode_velocity the notes play
   * at. Send it once on entry, then stay quiet while table mode is engaged. */
  if (buttons_table_mode())
  {
    if (!table_prev)
    {
      table_prev = true;
      uint16_t cc14 = (uint16_t)g_properties->tablemode_velocity << 7;
      usb_app_midi_control_change_14bit(L_MIDI_CH, 11, cc14);
      usb_app_midi_control_change_14bit(R_MIDI_CH, 11, cc14);
    }
    g_bellow_cc_out = (uint16_t)g_properties->tablemode_velocity << 7;
    return;
  }
  table_prev = false;

  uint16_t value = (uint16_t)lroundf(bellow_intensity() * 16383.0f);
  g_bellow_cc_out = value;

  uint32_t now_ms = HAL_GetTick();
  bool changed = value != last_out;
  bool period_ok = value == 0 || last_out == 0
                 || (now_ms - last_emit_ms) >= g_properties->bellow_cc_period_ms;
  if (!(changed && period_ok)) return;

  int8_t dir = value > last_out ? 1 : -1;
  if (last_dir != 0 && dir != last_dir) g_bellow_cc_reversals++;
  last_dir = dir;
  g_bellow_cc_sent++;

  last_out = value;
  last_emit_ms = now_ms;

  /* The single bellows drives both keyboards, which play on separate MIDI
   * channels (L_MIDI_CH / R_MIDI_CH), so send the expression CC on both. */
  usb_app_midi_control_change_14bit(L_MIDI_CH, 11, value);
  usb_app_midi_control_change_14bit(R_MIDI_CH, 11, value);
}

static void delay_us(uint32_t us)
{
  uint32_t cycles = us * (SystemCoreClock / 1000000U);
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < cycles);
}

/* Caps the hall-sensor sampling rate at bellow_sample_period_us (default 4kHz):
 * true once that many microseconds have elapsed since the last due sample, so
 * bellow_poll() can skip the ADC work (and everything downstream of it) the
 * rest of the time. HAL_GetTick() is only millisecond-resolution, too coarse
 * for a sub-millisecond gate, so this uses the same DWT cycle counter
 * delay_us() above already relies on. */
static bool bellow_sample_due(void)
{
  static uint32_t last_cycles;
  static bool have_last;

  uint32_t now = DWT->CYCCNT;
  uint32_t period_cycles = g_properties->bellow_sample_period_us * (SystemCoreClock / 1000000U);
  if (have_last && (now - last_cycles) < period_cycles) return false;

  last_cycles = now;
  have_last = true;
  return true;
}

/* One acquisition of both hall sensors and their combined reading. */
typedef struct {
  uint16_t hall0;
  uint16_t hall1;
  uint32_t conv_cycles;  /* CPU cycles spent in the two ADC conversions */
} bellow_sample_t;

/* Reads both hall sensors and returns their values and combined total. */
static bellow_sample_t bellow_sample(void)
{
  /* The two hall sensors share a gate-switched supply (HALL_NEN). Enable it,
   * let the gate and sensor settle, sample both, then disable to save power. */
  HAL_GPIO_WritePin(HALL_NEN_GPIO_Port, HALL_NEN_Pin, GPIO_PIN_RESET);
  /* Settling budget: gate RC (R5||R6 * Ciss_Q1 = 909R * 130pF, 5t~600ns) +
   * VDDH caps (RDS_on_Q1 * (CP1+CP2) = ~120mO * 200nF, 5t~120ns) +
   * SC4015SO power-on start <1us (datasheet) => worst case <3us; the default
   * bellow_settle_us of 5us is ~1.7x margin. */
  delay_us(g_properties->bellow_settle_us);

  uint32_t conv_start = DWT->CYCCNT;
  HAL_ADC_Start(&hadc3);
  HAL_ADC_Start(&hadc4);
  HAL_ADC_PollForConversion(&hadc3, 10);
  uint32_t hall0 = HAL_ADC_GetValue(&hadc3);
  HAL_ADC_PollForConversion(&hadc4, 10);
  uint32_t hall1 = HAL_ADC_GetValue(&hadc4);
  uint32_t conv_cycles = DWT->CYCCNT - conv_start;

  HAL_GPIO_WritePin(HALL_NEN_GPIO_Port, HALL_NEN_Pin, GPIO_PIN_SET);

  return (bellow_sample_t){
    .hall0 = (uint16_t)hall0,
    .hall1 = (uint16_t)hall1,
    .conv_cycles = conv_cycles,
  };
}

/* Human-readable name for a bellows direction. */
static const char *bellows_dir_str(bellows_t dir)
{
  switch (dir)
  {
    case BELLOWS_PULL:    return "PULL";
    case BELLOWS_PUSH:    return "PUSH";
    case BELLOWS_NEUTRAL: return "NEUTRAL";
    default:              return "?";
  }
}

/* Emits the optional console log and live-report dashboard. Called every
 * bellow_poll() tick regardless of whether this tick actually sampled the hall
 * sensors, so the report dashboard's own frame (report_begin()/report_end(),
 * driven by its independent report_period_ms timer) always gets its rows even on a
 * tick where the two rates don't line up; a skipped sampling tick just adds
 * nothing new to accumulate. */
static void bellow_report(bool sampled, const bellow_sample_t *s, const bellow_output_t *state)
{

  /* Per-frame stats, accumulated every sample and reset after each emitted report
   * frame (and on (re-)enable so the first frame never shows stale data), so each
   * line reflects only the samples since the previous one. Extremes are of the
   * combined reading; the std (E[x^2]-E[x]^2) is per sensor, accumulated in
   * integers to keep the sums exact, with the count shown alongside. */
  static uint16_t hall_min, hall_max;
  static uint32_t n, sum0, sum1, sumT;
  static uint64_t sq0, sq1, sqT;
  static double sum1e, sq1e;   /* g_bellow_1e_out is a float, so accumulated in double */
  static uint32_t push_entries, pull_entries;
  /* Variation of the CC#11 value over the window (14-bit units). */
  static uint16_t cc_min, cc_max;
  static uint32_t sum_cc;
  static uint64_t sq_cc;
  static bool show_was_on;
  if (g_properties->show_bellow && !show_was_on)
  {
    hall_min = 0xFFFF; hall_max = 0;
    n = 0; sum0 = sum1 = sumT = 0; sq0 = sq1 = sqT = 0;
    sum1e = sq1e = 0.0;
    push_entries = pull_entries = 0;
    cc_min = 0xFFFF; cc_max = 0; sum_cc = 0; sq_cc = 0;
    g_bellow_cc_sent = g_bellow_cc_reversals = 0;
  }
  show_was_on = g_properties->show_bellow;
  if (sampled)
  {
    uint16_t total = s->hall0 + s->hall1;
    if (total < hall_min) hall_min = total;
    if (total > hall_max) hall_max = total;
    n++;
    sum0 += s->hall0; sq0 += (uint32_t)s->hall0 * s->hall0;
    sum1 += s->hall1; sq1 += (uint32_t)s->hall1 * s->hall1;
    sumT += total;     sqT += (uint32_t)total * total;
    sum1e += (double)g_bellow_1e_out; sq1e += (double)g_bellow_1e_out * g_bellow_1e_out;
    if (g_bellow_cc_out < cc_min) cc_min = g_bellow_cc_out;
    if (g_bellow_cc_out > cc_max) cc_max = g_bellow_cc_out;
    sum_cc += g_bellow_cc_out; sq_cc += (uint32_t)g_bellow_cc_out * g_bellow_cc_out;

    /* Zone-entry counters: how many times the direction transitioned into
     * PUSH/PULL since the last report frame (tracked independently of
     * show_bellow so a mid-window enable doesn't miscount the first sample
     * as an entry). */
    static bellows_t last_dir = BELLOWS_NEUTRAL;
    if (state->direction != last_dir)
    {
      if (state->direction == BELLOWS_PUSH) push_entries++;
      else if (state->direction == BELLOWS_PULL) pull_entries++;
      last_dir = state->direction;
    }
  }

  if (g_report_due && g_properties->show_bellow && n > 0)
  {
    /* Double precision for the E[x^2]-E[x]^2 subtraction: hall readings run in
     * the thousands, so their squares are large relative to a small variance,
     * and float32's ~7 significant digits aren't enough to keep the difference
     * from occasionally going slightly negative (sqrtf of that is NaN). The
     * result is clamped to 0 as a floor against any residual rounding. */
    double mean0 = (double)sum0 / n;
    double mean1 = (double)sum1 / n;
    double meanT = (double)sumT / n;
    double var0 = (double)sq0 / n - mean0 * mean0;
    double var1 = (double)sq1 / n - mean1 * mean1;
    double varT = (double)sqT / n - meanT * meanT;
    double mean1e = sum1e / n;
    double var1e = sq1e / n - mean1e * mean1e;
    float std0 = sqrtf((float)(var0 > 0.0 ? var0 : 0.0));
    float std1 = sqrtf((float)(var1 > 0.0 ? var1 : 0.0));
    float stdT = sqrtf((float)(varT > 0.0 ? varT : 0.0));
    float std1e = sqrtf((float)(var1e > 0.0 ? var1e : 0.0));
    float conv_us = (float)s->conv_cycles / (SystemCoreClock / 1000000.0f);
    float force = (state->direction == BELLOWS_PUSH) ? -state->intensity
                : (state->direction == BELLOWS_PULL) ?  state->intensity : 0.0f;
    /* Floats go through format_fixed() and print as %s (fixed_format.h). */
    char f[4][FORMAT_FIXED_BUF_SIZE];
    console_dash_println("BELLOW  dir=%-7s int=%s  force=%s keys=%u",
                         bellows_dir_str(state->direction),
                         format_fixed(f[0], sizeof f[0], state->intensity, 3, false),
                         format_fixed(f[1], sizeof f[1], force, 3, true), keyboard_keys_pressed());
    /* std0/std1/stdT line's "std0 =" / "std1 =" / "stdT =" labels are each
     * padded to the same width as "hall0=" / "hall1=" / "total=" above (and
     * the value fields use matching widths), so the two lines' columns
     * line up in the monospace console. */
    console_dash_println("        hall0=%5u hall1=%5u total=%5u  (min %u max %u)  sample_count=%lu  conv=%sus",
                         (unsigned)s->hall0, (unsigned)s->hall1, (unsigned)s->hall0 + s->hall1,
                         hall_min, hall_max, (unsigned long)n,
                         format_fixed(f[0], sizeof f[0], conv_us, 1, false));
    console_dash_println("        std0 =%5s std1 =%5s stdT =%5s",
                         format_fixed(f[0], sizeof f[0], std0, 2, false),
                         format_fixed(f[1], sizeof f[1], std1, 2, false),
                         format_fixed(f[2], sizeof f[2], stdT, 2, false));
    console_dash_println("        1euro=%8s std=%5s  (mincutoff=%sHz beta=%s)",
                         format_fixed(f[0], sizeof f[0], g_bellow_1e_out, 1, false),
                         format_fixed(f[1], sizeof f[1], std1e, 2, false),
                         format_fixed(f[2], sizeof f[2], g_properties->bellow_cc_1e_mincutoff / 256.0f, 2, false),
                         format_fixed(f[3], sizeof f[3], g_properties->bellow_cc_1e_beta / 65536.0f, 4, false));
    /* cc11's second form is the 14-bit value as MSB.LSB (CC#11.CC#43, each
     * 7-bit) rendered as one decimal -- value/128 puts the MSB in the integer
     * part and the LSB as a fraction of it, the same conversion midi.html's
     * live CC#11 readout uses (updateLiveDot() there). */
    console_dash_println("        zone entries: push=%6lu pull=%6lu   cc11=%5u (%6s)",
                         (unsigned long)push_entries, (unsigned long)pull_entries,
                         (unsigned)g_bellow_cc_out,
                         format_fixed(f[0], sizeof f[0], g_bellow_cc_out / 128.0f, 2, false));
    /* How much the CC#11 value moves over the window: its std and range (14-bit
     * units), the messages actually sent, and how many of those reversed
     * direction (jitter around a steady value shows as many reversals). */
    double mean_cc = (double)sum_cc / n;
    double var_cc = (double)sq_cc / n - mean_cc * mean_cc;
    float std_cc = sqrtf((float)(var_cc > 0.0 ? var_cc : 0.0));
    console_dash_println("        cc11 std=%6s  range=%5u..%5u  sent=%5lu  reversals=%4lu",
                         format_fixed(f[0], sizeof f[0], std_cc, 2, false),
                         (unsigned)cc_min, (unsigned)cc_max,
                         (unsigned long)g_bellow_cc_sent, (unsigned long)g_bellow_cc_reversals);

    hall_min = 0xFFFF; hall_max = 0;
    n = 0; sum0 = sum1 = sumT = 0; sq0 = sq1 = sqT = 0;
    sum1e = sq1e = 0.0;
    push_entries = pull_entries = 0;
    cc_min = 0xFFFF; cc_max = 0; sum_cc = 0; sq_cc = 0;
    g_bellow_cc_sent = g_bellow_cc_reversals = 0;
  }
}

/* Center calibration in progress (bellow_calibrate_center()). The raw total
 * is averaged, like the configuration tool's Calibrate button does: the
 * 1-euro filter lags and would bias a short window. 64-bit sum: at the
 * fastest bellow_sample_period_us a second of totals can overflow 32 bits. */
static struct {
  bool     active;
  uint32_t start_ms;
  uint64_t sum;
  uint32_t n;
} g_calib;

void bellow_calibrate_center(void)
{
  if (g_calib.active) return;
  g_calib.active = true;
  g_calib.start_ms = HAL_GetTick();
  g_calib.sum = 0;
  g_calib.n = 0;
  printf("bellow: calibrating the center, leave the bellows at rest\r\n");
}

bool bellow_calibrating(void)
{
  return g_calib.active;
}

/* Ends a calibration once BELLOW_CALIBRATE_MS have passed: the mean becomes
 * bellow_center (clamped to its range), saved to flash and pushed to the
 * configuration tool. The flash write runs here in the main loop, like a
 * save from the tool (~85 us, ~22 ms when it compacts). */
static void bellow_calibrate_poll(const bellow_sample_t *s, bool sampled)
{
  if (!g_calib.active) return;
  if (sampled)
  {
    g_calib.sum += (uint32_t)s->hall0 + s->hall1;
    g_calib.n++;
  }
  if (HAL_GetTick() - g_calib.start_ms < BELLOW_CALIBRATE_MS) return;
  g_calib.active = false;

  size_t idx;
  if (g_calib.n == 0 || !property_by_name("bellow_center", &idx))
  {
    printf("bellow: center calibration failed, no sample\r\n");
    return;
  }
  uint64_t mean = (g_calib.sum + g_calib.n / 2) / g_calib.n;
  property_set_u16(idx, mean > UINT16_MAX ? UINT16_MAX : (uint16_t)mean);
  property_save_result_t r = property_set_saved(idx, g_properties->bellow_center);
  printf("bellow: center %u from %lu samples, %s\r\n", g_properties->bellow_center,
         (unsigned long)g_calib.n, r == PROPERTY_SAVE_OK ? "saved" : "save FAILED");
  midi_send_property_changed(idx);
}

/* Samples both hall sensors, filters the combined reading through a 1-euro
 * filter, updates the bellows direction/intensity from the filtered value, emits the
 * expression CC, and reports. Call once per main loop iteration; sampling is
 * internally rate-limited to bellow_sample_period_us, so most calls skip the
 * ADC work and the direction/intensity/CC update (the previous values carry
 * over). bellow_report() still runs every call regardless: it has its own
 * report_period_ms cadence (g_report_due, set by report_begin() in report.c) that
 * is independent of the sample rate, and it must run on every tick so it
 * doesn't miss a report_due tick that happens to fall between samples -
 * which would leave that dashboard frame without its bellow rows. */
void bellow_poll(void)
{
  static bellow_sample_t s;
  static one_euro_state_t one_euro_st;

  bool sampled = bellow_sample_due();
  if (sampled)
  {
    // Measure sensor value.
    s = bellow_sample();
    g_bellow_last_hall0 = s.hall0;
    g_bellow_last_hall1 = s.hall1;

    // Filter total before classification.
    one_euro_config_t oe_cfg = {
      .mincutoff = g_properties->bellow_cc_1e_mincutoff / 256.0f,
      .beta = g_properties->bellow_cc_1e_beta / 65536.0f,
      .dcutoff = 1.0f,
    };
    float hall_total = one_euro_update(&one_euro_st, &oe_cfg, (float)(s.hall0 + s.hall1), HAL_GetTick());
    g_bellow_1e_out = hall_total;

    /* hall_total is the filtered combined reading, not the raw sample --
     * classification runs on a smoothed signal so sensor jitter near the
     * deadzone boundary doesn't flicker the direction. */
    bellow_program_t p = current_program();
    bellow_classify_result_t r = bellow_classify(g_bellow_out.direction, hall_total, g_properties->bellow_center,
                                                 p.dead, g_properties->bellow_hyst,
                                                 p.push_travel, p.pull_travel);
    g_bellow_out.direction = r.direction;
    if (r.direction == BELLOWS_PUSH)
      g_bellow_out.intensity = bellow_curve_apply(r.intensity, p.push_x1, p.push_y1, p.push_x2, p.push_y2);
    else
      g_bellow_out.intensity = bellow_curve_apply(r.intensity, p.pull_x1, p.pull_y1, p.pull_x2, p.pull_y2);

    bellow_send_cc();
  }
  bellow_calibrate_poll(&s, sampled);
  bellow_report(sampled, &s, &g_bellow_out);
}

/* Diagnostic sweep: for each bellow_settle_us value in a fixed range, take
 * BELLOW_TUNE_SAMPLES samples (each spaced BELLOW_TUNE_PAUSE_MS apart) and print
 * the per-sensor mean and standard deviation. Helps pick the smallest settling
 * delay that still yields stable hall readings. Blocks the main loop for the
 * duration; temporarily overrides bellow_settle_us and restores it on return. */
void bellow_tune(void)
{
  enum {
    BELLOW_TUNE_LO        = 0,
    BELLOW_TUNE_HI        = 20,
    BELLOW_TUNE_STEP      = 2,
    BELLOW_TUNE_SAMPLES   = 100,
    BELLOW_TUNE_PAUSE_MS  = 20,
  };

  size_t idx;
  if (!property_by_name("bellow_settle_us", &idx))
  {
    printf("bellow_settle_us property not found\r\n");
    return;
  }
  uint16_t saved;
  property_get_u16(idx, &saved);

  /* stdout is block-buffered and the main loop (which would flush it) is stalled
   * for the whole sweep, so flush each row explicitly or nothing appears until
   * the command returns. */
  printf("settle_us   h1_avg  h1_std   h2_avg  h2_std\r\n");
  fflush(stdout);
  for (uint16_t us = BELLOW_TUNE_LO; us <= BELLOW_TUNE_HI; us += BELLOW_TUNE_STEP)
  {
    property_set_u16(idx, us);

    uint32_t sum0 = 0, sum1 = 0;
    uint64_t sq0 = 0, sq1 = 0;
    for (int i = 0; i < BELLOW_TUNE_SAMPLES; i++)
    {
      bellow_sample_t s = bellow_sample();
      sum0 += s.hall0; sq0 += (uint32_t)s.hall0 * s.hall0;
      sum1 += s.hall1; sq1 += (uint32_t)s.hall1 * s.hall1;
      /* This runs in the USART RX ISR (microrl execute callback), where SysTick
       * cannot preempt, so HAL_Delay would hang on a frozen tick. Busy-wait on
       * the DWT cycle counter instead. */
      delay_us(BELLOW_TUNE_PAUSE_MS * 1000U);
    }

    /* Mean and variance (E[x^2] - E[x]^2) in float; the FPU does the divides and
     * sqrt. Samples are accumulated in integers to keep the sums exact. */
    float mean0 = (float)sum0 / BELLOW_TUNE_SAMPLES;
    float mean1 = (float)sum1 / BELLOW_TUNE_SAMPLES;
    float std0 = sqrtf((float)sq0 / BELLOW_TUNE_SAMPLES - mean0 * mean0);
    float std1 = sqrtf((float)sq1 / BELLOW_TUNE_SAMPLES - mean1 * mean1);

    char f[4][FORMAT_FIXED_BUF_SIZE];
    printf("%9u   %6s  %6s   %6s  %6s\r\n", us,
           format_fixed(f[0], sizeof f[0], mean0, 1, false),
           format_fixed(f[1], sizeof f[1], std0, 2, false),
           format_fixed(f[2], sizeof f[2], mean1, 1, false),
           format_fixed(f[3], sizeof f[3], std1, 2, false));
    fflush(stdout);
  }

  property_set_u16(idx, saved);
}
