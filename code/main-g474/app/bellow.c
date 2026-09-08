#include "bellow.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "bellow_classify.h"
#include "bellow_phys.h"
#include "buttons.h"
#include "console.h"
#include "swo.h"
#include "hysteresis.h"
#include "keyboard.h"   /* L_MIDI_CH / R_MIDI_CH */
#include "main.h"
#include "properties.h"
#include "report.h"
#include "usb_app.h"


/* ADC handles for the two hall sensors, defined by the CubeMX-generated main.c. */
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;

/* Naive bellows model state. */
typedef struct {
  bellows_t direction;
  uint16_t intensity;
} bellow_naive_state_t;

/* Physical simulation model state (HAL-side wrapper adds last_tick). */
typedef struct {
  bellow_phys_state_t core;
  uint32_t            last_tick;
} bellow_physical_simulation_state_t;

typedef struct {
  bellows_t direction;
  uint16_t  intensity;
} bellow_output_t;

static bellow_output_t g_bellow_out = {.direction = BELLOWS_NEUTRAL, .intensity = 0};

/* Combined-hall calibration: center is the at-rest reading, hard push/pull
 * the readings at full travel. The deadzone sets how far from center the
 * bellows must move to leave BELLOWS_NEUTRAL (no air moves there, so
 * note_table maps every key to NOTE_NONE). The hysteresis margin then has to
 * be given back before returning to NEUTRAL, so a bellows resting right at
 * the deadzone edge doesn't chatter between NEUTRAL and PUSH/PULL. These are
 * the bellow_* properties in g_properties (properties.h). */

bellows_t bellow_direction(void)
{
  return g_bellow_out.direction;
}

uint16_t bellow_intensity(void)
{
  return g_bellow_out.intensity;
}

/* Bellows sensitivity multiplier (Q8, 256 = x1.0) for the level FN1 currently
 * selects: level 0 is unity, levels 1 and 2 use the bellow_scale_mid/high
 * properties. Applied to the intensity, so it scales both note velocity and
 * CC#11; also reused for the table-mode velocity. */
uint16_t bellow_sens_scale_q8(void)
{
  switch (buttons_bellow_sens_level())
  {
    case 0:  return g_properties->bellow_scale_low;
    case 1:  return g_properties->bellow_scale_mid;
    case 2:  return g_properties->bellow_scale_high;
    default: return 256;
  }
}

/* Naive bellows model: classifies hall reading into direction and intensity. */
static void bellow_naive(uint32_t hall_total, bellow_naive_state_t *state)
{
  uint32_t center = g_properties->bellow_center;

  bellow_classify_result_t r = bellow_classify(state->direction, (int32_t)hall_total, (int32_t)center,
                                               g_properties->bellow_dead, g_properties->bellow_hyst,
                                               g_properties->bellow_full_push, g_properties->bellow_full_pull);
  state->direction = r.direction;
  state->intensity = r.intensity;
}

/* Physical simulation model: HAL wrapper that derives F from the hall reading,
 * computes dt from the system tick, and delegates the pure-math integration to
 * bellow_phys_step (bellow_phys.h). See documentation/bellow_simulation.md. */
static void bellow_physical_simulation(uint32_t hall_total, uint16_t pressed_key_count,
                                       bellows_t naive_dir,
                                       bellow_physical_simulation_state_t *state)
{
  uint32_t center = g_properties->bellow_center;

  /* Derive a continuous signed force from the raw sensor offset.
   * Inside the deadzone (±bellow_dead from center), F = 0.
   * Outside the deadzone, F increases linearly from 0 at the boundary
   * to ±1024 at the full travel limits (full_push/full_pull). */
  int32_t d = (int32_t)hall_total - (int32_t)center;
  int32_t dead = (int32_t)g_properties->bellow_dead /2;
  float F;

  if (d < -dead)
  {
    /* Push: interpolate from -dead to full_push, with F ranging from 0 to -1024 */
    int32_t span = center - g_properties->bellow_full_push - dead;
    int32_t effective_d = d + dead;  /* offset from deadzone boundary */
    F = (float)effective_d * 1024.0f / (float)span;
  }
  else if(d > dead)
  {
    /* Pull: interpolate from +dead to full_pull, with F ranging from 0 to +1024 */
    int32_t span = g_properties->bellow_full_pull - center - dead;
    int32_t effective_d = d - dead;  /* offset from deadzone boundary */
    F = (float)effective_d * 1024.0f / (float)span;
  }
  else
  {
    /* Inside deadzone */
    F = 0.0f;
  }

  uint32_t now = HAL_GetTick();
  float dt_s = (float)(now - state->last_tick) / 1000.0f;
  state->last_tick = now;
  if (dt_s > 0.02f) dt_s = 0.02f;   /* clamp so a stalled loop can't blow up the integrator */

  bellow_phys_params_t params = {
    .track_ms      = g_properties->bellow_inertia_track_ms,
    .damping       = g_properties->bellow_inertia_damping,
    .impulse_gain  = g_properties->bellow_inertia_impulse_gain,
    .leak_quiet    = g_properties->bellow_inertia_leak_quiet,
    .leak_per_key  = g_properties->bellow_inertia_leak_per_key,
    .dir_dead      = g_properties->bellow_inertia_dir_dead,
    .dir_hyst      = g_properties->bellow_inertia_dir_hyst,
  };
  bellow_phys_step(&state->core, F, dt_s, pressed_key_count, &params);
}

static void bellow_swo_trace(const bellow_naive_state_t *naive, const bellow_physical_simulation_state_t *phys,
                             int32_t hall_total_centred, uint16_t keys)
{
  static uint32_t last_tick;
  static uint32_t last_header_sent;
  uint32_t now = HAL_GetTick();
  if (now - last_tick < 10) return;   /* 100 Hz */
  last_tick = now;
  if (last_header_sent == 0)
  {
    swo_print("timestamp,use_inertia,"
              "naive.direction,naive.intensity,"
              "phys.v,phys.p,phys.eff_intensity,phys.eff_dir,phys.f_prev,"
              "hall_total_centred,keys\n");
    last_header_sent = 100;
  }
  last_header_sent--;
  swo_printf("%lu,%u,"
             "%d,%u,"
             "%.3f,%.3f,%u,%d,%.3f,"
             "%ld,%u\n",
             (unsigned long)now, (unsigned)g_properties->bellow_inertia_enable,
             (int)naive->direction, (unsigned)naive->intensity,
             (double)phys->core.v, (double)phys->core.p, (unsigned)phys->core.eff_intensity,
             (int)phys->core.eff_dir, (double)phys->core.f_prev,
             (long)hall_total_centred, (unsigned)keys);
}

/* Emits CC#11 (Expression) from the effective intensity through the shared
 * directional-hysteresis + rate-limit pipeline (hysteresis.h, as the pedals use).
 * bellow_cchyst sets the play required before the CC moves (suppresses sensor
 * jitter) and bellow_cc_period_ms caps the send rate; the rate limit coalesces
 * rather than drops, so the latest value is always eventually sent. Fed by
 * bellow_intensity(), so the naive and inertia modes share one CC pipeline.
 *
 * When intensity drops to 0 (bellow at rest), CC=0 is forced and the
 * hysteresis anchor is reset. Without this, the anchor stays at bellow_cchyst
 * and the fwd_thresh condition (x + fwd_thresh < anchor -> 0+16 < 16 -> false)
 * never advances it down, leaving CC=1 stuck indefinitely. */
static void bellow_send_cc(void)
{
  static hyst_state_t st;

  uint16_t intensity = bellow_intensity();
  if (intensity == 0)
  {
    if (st.have_out && st.last_out != 0)
    {
      usb_app_midi_control_change(L_MIDI_CH, 11, 0);
      usb_app_midi_control_change(R_MIDI_CH, 11, 0);
    }
    st = (hyst_state_t){0};
    return;
  }

  hyst_config_t cfg = {
    .in_min = 0, .in_max = 1024, .out_max = 127,
    .fwd_thresh = g_properties->bellow_cchyst, .rev_thresh = g_properties->bellow_cchyst,
    .ema_alpha = HYST_EMA_UNITY, .min_period_ms = g_properties->bellow_cc_period_ms,
  };

  uint8_t value;
  if (!hyst_update(&st, &cfg, intensity, HAL_GetTick(), &value)) return;

  /* The single bellows drives both keyboards, which play on separate MIDI
   * channels (L_MIDI_CH / R_MIDI_CH), so send the expression CC on both. */
  usb_app_midi_control_change(L_MIDI_CH, 11, value);
  usb_app_midi_control_change(R_MIDI_CH, 11, value);
}

static void delay_us(uint32_t us)
{
  uint32_t cycles = us * (SystemCoreClock / 1000000U);
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < cycles);
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

  bellow_sample_t s;
  s.hall0 = (uint16_t)hall0;
  s.hall1 = (uint16_t)hall1;
  s.conv_cycles = conv_cycles;
  return s;
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

/* Emits the optional console log and live-report dashboard for one sample. */
static void bellow_report(const bellow_sample_t *s, const bellow_naive_state_t *naive_state,
                          const bellow_physical_simulation_state_t *phys)
{

  /* Per-frame stats, accumulated every poll and reset after each emitted report
   * frame (and on (re-)enable so the first frame never shows stale data), so each
   * line reflects only the samples since the previous one. Extremes are of the
   * combined reading; the std (E[x^2]-E[x]^2) is per sensor, accumulated in
   * integers to keep the sums exact, with the count shown alongside. */
  static uint16_t hall_min, hall_max;
  static uint32_t n, sum0, sum1;
  static uint64_t sq0, sq1;
  static bool show_was_on;
  if (g_properties->show_bellow && !show_was_on)
  {
    hall_min = 0xFFFF; hall_max = 0;
    n = 0; sum0 = sum1 = 0; sq0 = sq1 = 0;
  }
  show_was_on = g_properties->show_bellow;
  uint16_t total = s->hall0 + s->hall1;
  if (total < hall_min) hall_min = total;
  if (total > hall_max) hall_max = total;
  n++;
  sum0 += s->hall0; sq0 += (uint32_t)s->hall0 * s->hall0;
  sum1 += s->hall1; sq1 += (uint32_t)s->hall1 * s->hall1;


  if (g_report_due && g_properties->show_bellow)
  {
    float mean0 = (float)sum0 / n;
    float mean1 = (float)sum1 / n;
    float std0 = sqrtf((float)sq0 / n - mean0 * mean0);
    float std1 = sqrtf((float)sq1 / n - mean1 * mean1);
    float conv_us = (float)s->conv_cycles / (SystemCoreClock / 1000000.0f);
    float force = (naive_state->direction == BELLOWS_PUSH) ? -(float)naive_state->intensity
                : (naive_state->direction == BELLOWS_PULL) ?  (float)naive_state->intensity : 0.0f;
    console_dash_println("BELLOW  dir=%-7s int=%4u  force=%+5d v=%+8.1f P=%+7.1f  eff=%4u(%d) keys=%u",
                         bellows_dir_str(naive_state->direction), naive_state->intensity,
                         (int)force, (double)phys->core.v, (double)phys->core.p,
                         phys->core.eff_intensity, (int)phys->core.eff_dir,
                         keyboard_keys_pressed());
    console_dash_println("        hall0=%4u hall1=%4u total=%5u  (min %u max %u)  sample_count=%lu  conv=%.1fus",
                         (unsigned)s->hall0, (unsigned)s->hall1, (unsigned)s->hall0 + s->hall1,
                         hall_min, hall_max, (unsigned long)n, (double)conv_us);
    console_dash_println("        std0=%7.2f std1=%7.2f", (double)std0, (double)std1);

    hall_min = 0xFFFF; hall_max = 0;
    n = 0; sum0 = sum1 = 0; sq0 = sq1 = 0;
  }
}

/* Samples both hall sensors, updates the bellows direction/intensity, emits the
 * expression CC, and reports. Call once per main loop iteration. Consumers read
 * the result via bellow_direction() and track changes themselves. */
void bellow_poll(void)
{
  static bellow_naive_state_t naive = {.direction = BELLOWS_NEUTRAL, .intensity = 0};
  static bellow_physical_simulation_state_t phys = {0};

  bellow_sample_t s = bellow_sample();
  uint32_t hall_total = s.hall0 + s.hall1;
  bellow_naive(hall_total, &naive);
  bellow_physical_simulation(hall_total, keyboard_keys_pressed(), naive.direction, &phys);

  if (g_properties->bellow_inertia_enable) {
    g_bellow_out.direction = phys.core.eff_dir;
    g_bellow_out.intensity = phys.core.eff_intensity;
  } else {
    g_bellow_out.direction = naive.direction;
    g_bellow_out.intensity = naive.intensity;
  }
  uint32_t scaled = ((uint32_t)g_bellow_out.intensity * bellow_sens_scale_q8()) >> 8;
  g_bellow_out.intensity = (uint16_t)(scaled > 1024 ? 1024 : scaled);

  int32_t hall_total_centred = (int32_t)hall_total - (int32_t)g_properties->bellow_center;
  bellow_swo_trace(&naive, &phys, hall_total_centred, keyboard_keys_pressed());
  bellow_send_cc();
  bellow_report(&s, &naive, &phys);
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

    printf("%9u   %6.1f  %6.2f   %6.1f  %6.2f\r\n",
           us, (double)mean0, (double)std0, (double)mean1, (double)std1);
    fflush(stdout);
  }

  property_set_u16(idx, saved);
}
