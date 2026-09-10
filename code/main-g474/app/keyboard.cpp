#include <stdio.h>
#include <string.h>

extern "C" {
#include "main.h"
#include "spi_link.h"
#include "hall_report.h"
#include "keyboard_layout.h"
#include "bellow.h"
#include "buttons.h"
#include "console.h"
#include "properties.h"
#include "report.h"
#include "usb_app.h"
#include "keyboard.h"

/* SPI handles for the two wing links, defined by the CubeMX-generated main.c. */
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

/* The frame's per-key payload is exactly what the shared hall report renders. */
_Static_assert(SPI_LINK_NUM_KEYS == HALL_REPORT_NUM_KEYS,
               "SPI frame key count must match the shared hall report layout");

/* Press/release hysteresis: a hall reading falls as a key is pressed and rises
 * back as it is released. key_release > key_press gives a dead band so a key
 * resting near the trip point doesn't chatter. See g_properties (properties.h). */





// Lock free producer consumer queue where only the latest
// produced buffer is made available to the consumer.
class TripleBuffer {

  struct Buffer {
    bool to_be_consumed = 0;
    uint16_t data[SPI_LINK_FRAME_WORDS] {};
  };

  Buffer buffers[3] {};

  // Invarient: At any time those 3 variables contains 0, 1 and 2 in any order.
  uint8_t producer_idx = 0;
  uint8_t shared_idx = 1;
  uint8_t consumer_idx = 2;

  public:
    // Make the current producer available to the consumer.
    void release_producer_buffer() {
      // The producer buffer is assumed to be valid.
      buffers[producer_idx].to_be_consumed = true;
      __atomic_exchange(&shared_idx, &producer_idx, &producer_idx, __ATOMIC_RELEASE);
    };

    bool acquire_consumer_buffer() {
      // This also releases a consumed buffer.
      buffers[consumer_idx].to_be_consumed = false;
      __atomic_exchange(&shared_idx, &consumer_idx, &consumer_idx, __ATOMIC_ACQUIRE);
      return buffers[consumer_idx].to_be_consumed;
    };

    uint16_t* producer_buffer() {
      return buffers[producer_idx].data;
    }

    uint16_t* consumer_buffer() {
      return buffers[consumer_idx].data;
    }
};

struct SPIBus
{
  SPI_HandleTypeDef *hspi;
  const char *name;
  uint8_t midi_ch;          /* 0-based MIDI channel this side's notes play on */
  GPIO_TypeDef *nss_port;   /* NSS line, read to find the inter-frame gap */
  uint16_t nss_pin;

  TripleBuffer buffers;

  volatile uint8_t needs_resync;

  /* Per-reception outcome counters (updated in interrupt context). Every
   * reception is classified as exactly one of these, so
   * good + misaligned + crc_err + bus_err = total receptions:
   *   good       - completed and word 0 is a known wing id -> decoded
   *   misaligned - completed but word 0 is NOT a wing id (the 16-bit word
   *                boundaries don't line up with the wing's frame; the
   *                hardware CRC did not reject it) -> dropped
   *   crc_err    - HAL reported a CRC mismatch
   *   bus_err    - HAL reported overrun / mode / frame error */
  volatile uint32_t rx_good;
  volatile uint32_t rx_misaligned;
  volatile uint32_t crc_err;
  volatile uint32_t bus_err;
  volatile uint8_t  last_good_wing;  /* word 0 of the last good reception */
  volatile uint16_t last_bad_word0;  /* word 0 of the last misaligned reception */

  /* Per-key decode state (main-loop only). key_pressed tracks the physical
   * hall state; sounding_note is NOTE_NONE unless a NOTE ON has been sent for
   * that key without a matching NOTE OFF yet (key_pressed alone isn't enough
   * since a key can be held through a bellows direction change). */
  uint16_t key_min[SPI_LINK_NUM_KEYS];
  uint8_t  key_pressed[SPI_LINK_NUM_KEYS];
  uint8_t  sounding_note[SPI_LINK_NUM_KEYS];
  uint16_t mapped_keys_pressed;

  /* Last good frame's raw readings and their running statistics, for the
   * show_keyboard live report (main-loop only). */
  uint16_t    last_keys[SPI_LINK_NUM_KEYS];
  hall_stats_t hall;

  /* Snapshots for the 1 Hz rate report and the throttled error log. */
  uint32_t last_good, last_misaligned, last_crc, last_bus;
  uint32_t logged_crc, logged_bus, logged_misaligned, last_err_log_tick;

  /* Separate good-count snapshot for the live (show_spi) dashboard, so its rate
   * is computed over the report interval without disturbing the 1 Hz log above. */
  uint32_t rep_last_good, rep_last_tick;

  /* Silence-on-timeout: last tick a good frame was decoded in the main loop, and
   * a flag so the silence action fires exactly once per gap (re-armed on recovery). */
  uint32_t last_good_frame_tick;
  bool     timed_out;
};

static SPIBus g_bus[2];
// Increated in situation that should never happen.
static uint16_t g_spi_bus_state_error = 0;

/* Number of mapped keys currently held down across both wing keyboards,
 * regardless of bellows direction. Main-loop-only state, so no locking. The
 * bellows inertia model reads this to bleed chamber pressure: air escapes through
 * an open pallet whether or not its reed is sounding. */
unsigned keyboard_keys_pressed(void)
{
  return g_bus[0].mapped_keys_pressed + g_bus[1].mapped_keys_pressed;
}

/* Effective bellows direction used to map and gate notes. Table mode pins it to
 * PULL so keys sound while the instrument rests on a table (bellows neutral),
 * playing each key's pull note; otherwise it follows the real bellows. */
static bellows_t kbd_bellows(void)
{
  return buttons_table_mode() ? BELLOWS_PULL : bellow_direction();
}

static SPIBus *bus_from_hspi(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi1) return &g_bus[0];
  if (hspi == &hspi2) return &g_bus[1];
  return NULL;
}


/* RxCplt and ErrorCallback re-arm immediately: by the time the callback fires,
 * the frame (including CRC) is fully received, NSS is already high, and the wing
 * has started its next hall scan. Safe to reset SPI state and re-arm DMA now.
 * If re-arm fails, set needs_resync to retry from the main loop.
 *
 * RxCplt classifies the frame (good/misaligned) before re-arming, so every
 * reception is counted even if the loop can't drain all of them. Only good frames
 * (word 0 is a known wing id) are handed to the consumer buffer for decode. */

static void spi_drain_and_reset(SPI_HandleTypeDef *hspi)
{
  // Drain any leftover data from the RX FIFO and clear error flags to ensure
  // a clean state before starting a new DMA transfer. Without this, stale bytes
  // or a stuck overrun condition would corrupt the next frame.
  __HAL_SPI_DISABLE(hspi);
  while (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE))
    (void)hspi->Instance->DR;
  __HAL_SPI_CLEAR_OVRFLAG(hspi);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{

  if(__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_BSY)) {
    // This should never happen.
    // code/wing-g474/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c
    // SPI_DMAReceiveCplt is supposed to wait for SPI_EndRxTransaction.
    g_spi_bus_state_error++;
  }

  SPIBus *bus = bus_from_hspi(hspi);
  if (!bus) {
    g_spi_bus_state_error++;
    return;
  }

  uint16_t wing_id = bus->buffers.producer_buffer()[0];
  if (wing_id <= UINT8_MAX && wing_name((uint8_t)wing_id) != NULL)
  {
    bus->rx_good++;
    bus->last_good_wing = (uint8_t)wing_id;
    bus->buffers.release_producer_buffer();
  }
  else
  {
    // The wing identifier is corrupted.
    bus->rx_misaligned++;
    bus->last_bad_word0 = wing_id;

    // Do not release a corrupted the buffer.
    // That would replace the current shared buffer that
    // may contain valid data.
  }

  spi_drain_and_reset(bus->hspi);

  if (HAL_SPI_Receive_DMA(bus->hspi, (uint8_t *)bus->buffers.producer_buffer(), SPI_LINK_FRAME_WORDS) != HAL_OK)
    bus->needs_resync = 1;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  SPIBus *b = bus_from_hspi(hspi);
  if (!b) return;
  uint32_t ec = hspi->ErrorCode;
  hspi->ErrorCode = HAL_SPI_ERROR_NONE;
  if (ec == HAL_SPI_ERROR_CRC) b->crc_err++;
  else                         b->bus_err++;

  spi_drain_and_reset(b->hspi);

  if (HAL_SPI_Receive_DMA(b->hspi, (uint8_t *)b->buffers.producer_buffer(), SPI_LINK_FRAME_WORDS) != HAL_OK)
    b->needs_resync = 1;
}

/* The active tuning's notes for one wing and bellows direction, or NULL if
 * wing_id is not a wing — which happens on a bus that has not yet received a
 * good frame, since last_good_wing starts at 0. Read fresh on every use so a
 * console edit takes effect live, like the other properties. property_set_u16
 * clamps to [min,max], but a future flash blob could carry anything, so fall
 * back to Rheinische rather than index out of bounds.
 *
 * Changing tuning while keys are held is deliberately not special-cased:
 * sounding_note[] holds the note that was actually sent and bus_note_off()
 * replays it, so a held note keeps its old pitch until released and cannot get
 * stranded. The next press uses the new tuning. */
static inline const uint8_t *notes_for(uint8_t wing_id, bellows_t dir)
{
  int side = wing_side(wing_id);
  if (side < 0) return NULL;
  uint16_t tuning = g_properties->keyboard_tuning;
  if (tuning >= NUM_TUNINGS) tuning = TUNING_RHEINISCHE;
  return note_table[tuning][side][dir];
}

/* Sends NOTE ON for key k on wing_id using the current bellows mapping, if
 * it isn't already sounding and that mapping has a note (it doesn't in
 * BELLOWS_NEUTRAL, where no air moves). No-op otherwise. */
static void bus_note_on(SPIBus *b, uint8_t wing_id, int k)
{
  if (b->sounding_note[k] != NOTE_NONE) return;
  bellows_t dir = kbd_bellows();
  if (dir == BELLOWS_NEUTRAL) return;  /* no air moves, no note; table has no neutral slice */
  const uint8_t *notes = notes_for(wing_id, dir);
  if (notes == NULL) return;           /* no good frame yet, so no keyboard to map */
  uint8_t note = notes[k];
  if (note == NOTE_NONE) return;
  b->sounding_note[k] = note;
  /* In table mode the bellows isn't moving, so every note plays at the same
   * constant velocity, tablemode_velocity (which CC#11 is also pinned to there);
   * otherwise derive velocity from how hard the bellows is moving (0..1024 ->
   * 1..127), floored at 1 so a note triggered just past the neutral deadzone is
   * still audible (0 would be a NOTE OFF). */
  uint8_t velocity;
  if (buttons_table_mode())
  {
    velocity = (uint8_t)g_properties->tablemode_velocity;
  }
  else
  {
    velocity = (uint8_t)(1 + (uint32_t)bellow_intensity() * 126u / 1024u);
  }
  printf("NOTE ON  %s wing=%u key=%2d note=%3u vel=%3u\r\n", b->name, wing_id, k, note, velocity);
  usb_app_midi_note_on(b->midi_ch, note, velocity);
}

/* Sends NOTE OFF for key k on wing_id if it is currently sounding. No-op
 * otherwise (e.g. the key was pressed/released while in BELLOWS_NEUTRAL and
 * never sounded). */
static void bus_note_off(SPIBus *b, uint8_t wing_id, int k)
{
  if (b->sounding_note[k] == NOTE_NONE) return;
  printf("NOTE OFF %s wing=%u key=%2d note=%3u\r\n", b->name, wing_id, k, b->sounding_note[k]);
  usb_app_midi_note_off(b->midi_ch, b->sounding_note[k]);
  b->sounding_note[k] = NOTE_NONE;
}

/* Called once from the main loop whenever the bellows direction changes. Entering
 * PUSH/PULL from NEUTRAL fires NOTE ON for keys already held down; entering
 * NEUTRAL fires NOTE OFF for keys currently sounding, even if still held,
 * since no air moves at rest. PUSH<->PULL never happens directly (NEUTRAL
 * sits between them), so this never has to swap one sounding note for
 * another. */
static void bus_bellows_changed(SPIBus *b)
{
  uint8_t wing_id = b->last_good_wing;
  for (int k = 0; k < SPI_LINK_NUM_KEYS; k++)
  {
    if (kbd_bellows() == BELLOWS_NEUTRAL) bus_note_off(b, wing_id, k);
    else if (b->key_pressed[k])       bus_note_on(b, wing_id, k);
  }
}

/* Decodes one received frame into key press/release events, driving NOTE
 * ON/OFF via bus_note_on/bus_note_off. Channels whose since-boot minimum is
 * still 0 are treated as unpopulated and skipped, matching the wing's
 * present-key detection. */
static void bus_process_frame(SPIBus *b, const uint16_t *frame)
{
  uint8_t wing_id = (uint8_t)frame[0];
  const uint16_t *meas = &frame[1];

  /* Hoisted out of the per-key loop below: one property read and one table
   * lookup per frame instead of forty. NULL means an unknown wing id. */
  const uint8_t *push_notes = notes_for(wing_id, BELLOWS_PUSH);
  if (push_notes == NULL) return; /* unknown wing -> nothing to decode */

  /* Capture the raw readings only while this keyboard's live table is shown,
   * and fold them into the stats only while its stat tables are shown, so a
   * disabled (or partly disabled) dashboard adds no per-frame work. The bus's
   * selector bit is its index in g_bus (bit 0 = left, bit 1 = right). */
  uint16_t bit = 1u << (unsigned)(b - g_bus);
  if (g_properties->show_keyboard & bit)
    memcpy(b->last_keys, meas, sizeof(b->last_keys));
  if (g_properties->show_keyboard & g_properties->show_keyboard_stats & bit)
    hall_stats_update(&b->hall, meas);

  b->mapped_keys_pressed = 0;
  for (int k = 0; k < SPI_LINK_NUM_KEYS; k++)
  {
    uint16_t v = meas[k];
    if (v < b->key_min[k]) b->key_min[k] = v;
    if (b->key_min[k] == 0) continue;          /* unpopulated channel */

    const bool is_mapped = push_notes[k] != NOTE_NONE;
    if (!b->key_pressed[k] && v <= g_properties->key_press)
    {
      b->key_pressed[k] = 1;
      if(is_mapped) {
        printf("PRESS %u %d\r\n", wing_id, k);
      }
      bus_note_on(b, wing_id, k);
    }
    else if (b->key_pressed[k] && v >= g_properties->key_release)
    {
      b->key_pressed[k] = 0;
      bus_note_off(b, wing_id, k);
    }
    b->mapped_keys_pressed += is_mapped && b->key_pressed[k];
  }
}

#define KEYBOARD_TIMEOUT_MS 500u

/* Silences all notes on a bus: NOTE OFF for every sounding key, then resets
 * pressed state (the wing's physical state is unknown while unresponsive). */
static void bus_silence(SPIBus *b)
{
  for (int k = 0; k < SPI_LINK_NUM_KEYS; k++) {
    if (b->sounding_note[k] != NOTE_NONE) {
      usb_app_midi_note_off(b->midi_ch, b->sounding_note[k]);
      b->sounding_note[k] = NOTE_NONE;
    }
    b->key_pressed[k] = 0;
  }
  b->mapped_keys_pressed = 0;
}

/* Decodes the latest good frame the ISR handed over, and fires a silence event
 * if no good frame has arrived for KEYBOARD_TIMEOUT_MS. */
static void bus_poll(SPIBus *b)
{
  if (b->buffers.acquire_consumer_buffer()) {
    b->last_good_frame_tick = HAL_GetTick();
    if (b->timed_out) {
      b->timed_out = false;
      printf("KEYBOARD RECOVERED %s\r\n", b->name);
    }
    bus_process_frame(b, b->buffers.consumer_buffer());
  } else if (!b->timed_out && HAL_GetTick() - b->last_good_frame_tick >= KEYBOARD_TIMEOUT_MS) {
    b->timed_out = true;
    printf("KEYBOARD TIMEOUT %s: no frame for %u ms, silencing all notes\r\n",
           b->name, KEYBOARD_TIMEOUT_MS);
    bus_silence(b);
  }
}

/* Retries reception when HAL_SPI_Receive_DMA failed in the callback (rare).
 * Polls NSS to avoid re-arming mid-frame, then attempts re-arm once per loop.
 * If successful, clears needs_resync; remains set otherwise for next iteration. */
static void bus_service_resync(SPIBus *b)
{
  if (!b->needs_resync) return;
  if (HAL_GPIO_ReadPin(b->nss_port, b->nss_pin) == GPIO_PIN_RESET) return;

  spi_drain_and_reset(b->hspi);

  if (HAL_GPIO_ReadPin(b->nss_port, b->nss_pin) == GPIO_PIN_RESET) return;
  if (HAL_SPI_Receive_DMA(b->hspi, (uint8_t *)b->buffers.producer_buffer(), SPI_LINK_FRAME_WORDS) == HAL_OK)
    b->needs_resync = 0;
}

/* Logs newly accumulated faults at most once per window, so a storm on a
 * misaligned/overrunning bus can't flood the console. "misaligned" means a
 * frame arrived whose word 0 wasn't a known wing id (1/2). */
#define SPI_ERR_LOG_THROTTLE_MS 500
static void bus_log_errors(SPIBus *b)
{
  if (!g_properties->log_spi_stat) return;
  uint32_t mis = b->rx_misaligned, crc = b->crc_err, bus = b->bus_err;
  uint32_t new_mis = mis - b->logged_misaligned;
  uint32_t new_crc = crc - b->logged_crc;
  uint32_t new_bus = bus - b->logged_bus;
  if (new_mis == 0 && new_crc == 0 && new_bus == 0) return;
  uint32_t now = HAL_GetTick();
  if (now - b->last_err_log_tick < SPI_ERR_LOG_THROTTLE_MS) return;
  b->last_err_log_tick = now;
  b->logged_misaligned = mis;
  b->logged_crc = crc;
  b->logged_bus = bus;
  printf("SPI %s: +%lu misaligned (last word0=%u, want wing id 1/2), +%lu crc-err, +%lu bus-err\r\n",
         b->name, (unsigned long)new_mis, (unsigned)b->last_bad_word0,
         (unsigned long)new_crc, (unsigned long)new_bus);
}

/* 1 Hz per-bus reception rates. good + misaligned + crc-err + bus-err = total
 * receptions; "good" are the only ones decoded into notes. */
static void bus_print_rates(SPIBus *b, uint32_t dt_ms)
{
  if (!g_properties->log_spi_stat) return;
  uint32_t good = b->rx_good, mis = b->rx_misaligned, crc = b->crc_err, bus = b->bus_err;
  uint32_t d_good = good - b->last_good;
  uint32_t d_mis  = mis  - b->last_misaligned;
  uint32_t d_crc  = crc  - b->last_crc;
  uint32_t d_bus  = bus  - b->last_bus;
  b->last_good = good;
  b->last_misaligned = mis;
  b->last_crc = crc;
  b->last_bus = bus;
  printf("%s: good: %lu/s, misaligned: %lu/s, crc-err: %lu/s, bus-err: %lu/s "
         "(last good wing=%u, last bad word0=%u)\r\n",
         b->name,
         (unsigned long)(d_good * 1000u / dt_ms),
         (unsigned long)(d_mis  * 1000u / dt_ms),
         (unsigned long)(d_crc  * 1000u / dt_ms),
         (unsigned long)(d_bus  * 1000u / dt_ms),
         b->last_good_wing, (unsigned)b->last_bad_word0);
}

/* One dashboard row per bus: running totals plus a good/s rate over the interval
 * since this bus' last report frame. */
static void bus_report(SPIBus *b)
{
  uint32_t good = b->rx_good;
  uint32_t now = HAL_GetTick();
  uint32_t dt = now - b->rep_last_tick;
  uint32_t rate = dt ? (good - b->rep_last_good) * 1000u / dt : 0;
  b->rep_last_good = good;
  b->rep_last_tick = now;
  console_dash_println("%-6s good=%8lu bad_id=%6lu bad_crc=%6lu bad_bus=%6lu  %5lu/s  (wing=%u bad0=%u)",
                       b->name, (unsigned long)good, (unsigned long)b->rx_misaligned,
                       (unsigned long)b->crc_err, (unsigned long)b->bus_err,
                       (unsigned long)rate, b->last_good_wing, (unsigned)b->last_bad_word0);
}

/* Renders one shared hall-report line as a dashboard row. The line already
 * carries its own ANSI color escapes; "%s" keeps any stray '%' literal. */
static void kbd_dash_emit(void *ctx, const char *line)
{
  (void)ctx;
  console_dash_println("%s", line);
}

/* One keyboard's live hall table (and, with show_keyboard_stats, its
 * min/max/diff/stddev tables), laid out exactly as the wing reports it: one row
 * per ADC, columns 0..n left-to-right in SPI-frame order. */
static void bus_report_keyboard(SPIBus *b, bool show_stats)
{
  const char *wname = wing_name(b->last_good_wing);
  const char *tname = tuning_name((uint8_t)g_properties->keyboard_tuning);
  console_dash_println("%-6s keyboard (wing_id=%u %s, tuning=%s)", b->name, b->last_good_wing,
                       wname ? wname : "?", tname ? tname : "?");
  hall_report_keys(b->last_keys, kbd_dash_emit, NULL);
  if (show_stats)
  {
    hall_report_stats(&b->hall, b->name, kbd_dash_emit, NULL);
    /* Start a fresh stddev window so the next report's stddev covers only the
     * samples gathered since this one; min/max are kept since the report was
     * enabled. */
    hall_stats_reset_window(&b->hall);
  }
}

void keyboard_init(void)
{
  g_bus[0].hspi = &hspi1; g_bus[0].name = "SPI1/L"; g_bus[0].midi_ch = L_MIDI_CH;
  g_bus[0].nss_port = L_SPI_NSS_GPIO_Port; g_bus[0].nss_pin = L_SPI_NSS_Pin;
  g_bus[1].hspi = &hspi2; g_bus[1].name = "SPI2/R"; g_bus[1].midi_ch = R_MIDI_CH;
  g_bus[1].nss_port = R_SPI_NSS_GPIO_Port; g_bus[1].nss_pin = R_SPI_NSS_Pin;
  uint32_t now = HAL_GetTick();
  for (int i = 0; i < 2; i++)
  {
    spi_drain_and_reset(g_bus[i].hspi);
    g_bus[i].needs_resync = 1;
    for (int k = 0; k < SPI_LINK_NUM_KEYS; k++)
      g_bus[i].key_min[k] = UINT16_MAX;
    hall_stats_init(&g_bus[i].hall);
    g_bus[i].last_good_frame_tick = now;
    g_bus[i].timed_out = false;
  }
}

/* Re-evaluates sounding notes for both buses when the bellows direction has
 * changed since the last poll (see bus_bellows_changed). */
static void keyboard_bellows_changed(void)
{
  for (int i = 0; i < 2; i++)
    bus_bellows_changed(&g_bus[i]);
}

void keyboard_poll(void)
{
  static bellows_t last_bellows = BELLOWS_NEUTRAL;
  bellows_t bellows = kbd_bellows();
  if (bellows != last_bellows)
  {
    last_bellows = bellows;
    keyboard_bellows_changed();
  }

  /* Rebase the dashboard rate snapshots when the report is (re-)enabled so the
   * first frame measures a real interval, not the whole uptime. */
  static bool show_was_on;
  if (g_properties->show_spi && !show_was_on)
  {
    uint32_t now = HAL_GetTick();
    for (int i = 0; i < 2; i++) { g_bus[i].rep_last_good = g_bus[i].rx_good; g_bus[i].rep_last_tick = now; }
  }
  show_was_on = g_properties->show_spi;

  /* A keyboard accumulates stats only while both its show_keyboard and
   * show_keyboard_stats bits are set (see bus_process_frame), so all the stats
   * bookkeeping below is keyed off the same mask and is skipped entirely when
   * no keyboard's stats are on. */
  uint16_t stats_mask = g_properties->show_keyboard & g_properties->show_keyboard_stats;

  /* Seed a keyboard's stats when its stat report is (re-)enabled, so it starts
   * a fresh min/max/window rather than showing data left from a previous run. */
  static uint16_t stats_was_on;
  for (int i = 0; i < 2; i++)
    if ((stats_mask & (1u << i)) && !(stats_was_on & (1u << i)))
      hall_stats_init(&g_bus[i].hall);
  stats_was_on = stats_mask;

  for (int i = 0; i < 2; i++)
  {
    bus_poll(&g_bus[i]);
    bus_service_resync(&g_bus[i]);
    bus_log_errors(&g_bus[i]);
    if (g_report_due && g_properties->show_spi) bus_report(&g_bus[i]);
  }

  /* show_keyboard and show_keyboard_stats are both bus selectors: bit 0 = left
   * (SPI1), bit 1 = right (SPI2), so 1=left, 2=right, 3=both. Selecting one
   * keyboard at a time (and stats only where wanted) keeps the tall report from
   * overflowing the terminal. Stats are shown only for a keyboard that is also
   * being reported. */
  if (g_report_due)
    for (int i = 0; i < 2; i++)
      if (g_properties->show_keyboard & (1u << i))
        bus_report_keyboard(&g_bus[i], g_properties->show_keyboard_stats & (1u << i));
}

void keyboard_print_rates(uint32_t dt_ms)
{
  for (int i = 0; i < 2; i++)
    bus_print_rates(&g_bus[i], dt_ms);
}
}  /* extern "C" */
