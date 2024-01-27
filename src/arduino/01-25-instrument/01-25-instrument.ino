#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <MIDI.h>

#include <array>
#include <cassert>
#include <vector>

// USB MIDI object
Adafruit_USBD_MIDI usb_midi;

// Create a new instance of the Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

#define digitalPinToBitShift(P) (g_APinDescription[P].ulPin)

// This is assuing the clear register is contiguous with the set.
// Static assertion complains when testing the pointers.
#define SET_PORT_BIT(P, mask, value)       \
  do {                                     \
    *(&REG_PORT_OUTCLR##P + value) = mask; \
  } while (0)

#define PROFILE(x)                    \
  {                                   \
    DWT->CYCCNT = 0;                  \
    x;                                \
    const uint32_t end = DWT->CYCCNT; \
    Serial.print(#x);                 \
    Serial.print(" took ");           \
    Serial.print(end);                \
    Serial.println(" cycles");        \
  }

#undef assert
#define assert(x)                      \
  do {                                 \
    if (!(x)) {                        \
      Serial.println();                \
      Serial.print("Assert failed: "); \
      Serial.println(#x);              \
      Serial.print(__FILE__);          \
      Serial.print(":");               \
      Serial.println(__LINE__);        \
    }                                  \
  } while (0);

void delayCycles(uint32_t count) {
  // This value takes into account the time spent in the function
  // itself. It has been determined experimentally by comparing the
  // delayCycles() function with the micro().
  constexpr uint32_t experimental_bias = 16;
  const uint32_t start = DWT->CYCCNT - experimental_bias;
  // The DWT->CYCCNT register is a 32 bits counter that counts the
  // number of cycles since the last reset. It is incremented every
  // cycle. It wraps around every 2^32 cycles (~37 secs at 120MHz).
  while (DWT->CYCCNT - start < count) {
  }
}

constexpr RwReg &port_to_clr(RwReg &port) { return *(&port + 1); }
template <typename... A>
struct TT {};
template <int N>
class SPIArray {
 private:
  enum Mode { IDLE_LOW, IDEL_HIGH };
  using PadValue = uint32_t;

  static_assert(
      sizeof(*portOutputRegister(digitalPinToPort(0))) == sizeof(PadValue),
      "the pad size looks incorrect. Please fix to match your architecture.");

  // User provided configuration.
  uint8_t clk_;
  uint8_t mosi_;
  std::array<uint8_t, N> miso_;
  uint8_t cs_;

  // Hardware mapping.
  PadValue cs_mask_, clk_mask_, mosi_mask_;
  std::array<uint32_t, N> miso_shifts_;

 public:
  uint32_t half_period_cyc_;
  SPIArray(uint8_t clk, uint8_t mosi, std::array<uint8_t, N> miso, uint8_t cs)
      : clk_(clk), mosi_(mosi), miso_(std::move(miso)), cs_(cs) {
    // Asserts that all specified pins are in the same port.
    clk_mask_ = digitalPinToBitMask(clk_);
    mosi_mask_ = digitalPinToBitMask(mosi_);
    for (int i = 0; i < N; i++) {
      miso_shifts_[i] = digitalPinToBitShift(miso[i]);
    }
    cs_mask_ = digitalPinToBitMask(cs_);
  }

  void begin() const {
    pinMode(cs_, OUTPUT);
    digitalWrite(cs_, HIGH);

    pinMode(clk_, OUTPUT);
    // Only Spi mode 0 is supported: idle low on mode 0 and 1
    digitalWrite(clk_, LOW);

    pinMode(mosi_, OUTPUT);
    digitalWrite(mosi_, HIGH);

    for (auto pin : miso_) {
      pinMode(pin, INPUT);
    }
  }

  int size() const { return miso_.size(); }
  inline void select(bool value) const { SET_PORT_BIT(0, cs_mask_, !value); }

  inline void transfer(const uint32_t wbuf, const uint32_t lsb_bits_to_transmit,
                       uint32_t *rbuf) const {  // Use std::span<uint32_t, N>
                                                // when available.
    for (int8_t shift = lsb_bits_to_transmit - 1; shift >= 0; shift--) {
      const bool mosi_value = (wbuf >> shift) & 0x1;
      SET_PORT_BIT(0, mosi_mask_, mosi_value);

      REG_PORT_OUTSET0 = clk_mask_;

      __NOP();
      __NOP();
      __NOP();
      __NOP();
      __NOP();
      const uint32_t in = REG_PORT_IN0;
      int i = 0;
      for (; i < N / 2; i++) {
        const bool bit = (in >> miso_shifts_[i]) & 0x1;
        rbuf[i] = (rbuf[i] << 1) | bit;
      }

      REG_PORT_OUTCLR0 = clk_mask_;

      for (; i < N; i++) {
        const bool bit = (in >> miso_shifts_[i]) & 0x1;
        rbuf[i] = (rbuf[i] << 1) | bit;
      }
    }
  }
};

template <unsigned int N>
class MCP3008Array {
  SPIArray<N> spi_array_;

 public:
  MCP3008Array(uint8_t clk, uint8_t mosi, std::array<uint8_t, N> miso,
               uint8_t cs)
      : spi_array_(clk, mosi, miso, cs) {}

  void begin() const { spi_array_.begin(); }

  void read(std::array<uint32_t, N * 8> &values) const {
    for (uint8_t channel = 0; channel < 8; channel++) {
      spi_array_.select(true);
      const uint32_t write = 0x18000 | (channel << 12);
      spi_array_.transfer(write, 17, &values[channel * N]);
      spi_array_.select(false);
    }
    for (auto &v : values) {
      v &= 0x3ff;
    }
  }
};

MCP3008Array<5> adc_array(10, 12,
                          {
                              21,
                              22,
                              5,
                              6,
                              9,
                          },
                          13);
uint32_t count = 0;

void setup() {
  // usb_midi.setStringDescriptor("BandoNeon MIDI");

  // Initialize MIDI, and listen to all MIDI channels
  // This will also call usb_midi's begin()
  MIDI.begin(MIDI_CHANNEL_OMNI);

  // Enable logging through the serial port.
  Serial.begin(115200);
  adc_array.begin();

  // wait until device mounted
  while (!TinyUSBDevice.mounted()) delay(1);
}

class Heartbit {
  uint32_t last_display_ms_;
  uint32_t tick_count_;
  static constexpr uint32_t kHeartbitPeriodMs = 10000;

 public:
  Heartbit() : last_display_ms_(millis()), tick_count_(0) {}
  void tick() {
    tick_count_++;
    uint32_t delay = millis() - last_display_ms_;
    if (delay > kHeartbitPeriodMs) {
      last_display_ms_ += kHeartbitPeriodMs;
      Serial.print("Heartbit ");
      Serial.print(tick_count_ * 1000 / delay);
      Serial.println(" Hz");
      tick_count_ = 0;
    }
  }
};

// Low pass filter on the derivate of the signal.
// The current derivate is merged in the previous one with a factor.
// 1 ignore the past, and 0 ignore the present.
constexpr float kDecayFactor = 0.25;
// Velocity profile.
constexpr float kMinVelocity = -0.0065;
constexpr float kMaxVelocity = -0.0570;

// Measure below which one a key is considered pressed.
constexpr int32_t kThreshold = 370;
// Percentage of the value used for hysteresis, to avoid bouncing.
constexpr float kHysteresisFactor = 0.06;

std::array<uint32_t, 5 * 8> measures[2] = {{}, {}};
std::array<float, measures[0].size()> derivatives{};
std::array<bool, measures[0].size()> isPressed{};

void loop() {
  Heartbit heartbit;
  // Capture time and measures.
  uint32_t last_measure_time = micros();
  adc_array.read(measures[1]);

  for (uint32_t measure_id = 0;; measure_id = 1 - measure_id) {
    auto &current_measures = measures[measure_id];
    const auto &last_measures = measures[1 - measure_id];

    // Capture time and measures.
    uint32_t current_measure_time = micros();
    adc_array.read(current_measures);

    // Compute derivatives.
    float inv_delta_t_us =
        1. / static_cast<float>(current_measure_time - last_measure_time);
    last_measure_time = current_measure_time;

    for (int i = 0; i < current_measures.size(); i++) {
      const float instant_derivative =
          (static_cast<float>(current_measures[i]) - last_measures[i]) *
          inv_delta_t_us;
      derivatives[i] = instant_derivative * kDecayFactor +
                       derivatives[i] * (1. - kDecayFactor);

      constexpr int32_t kThresholdLow = kThreshold * (1. - kHysteresisFactor);
      constexpr int32_t kThresholdHigh = kThreshold * (1. + kHysteresisFactor);

      if (current_measures[i] < kThresholdLow && !isPressed[i]) {
        isPressed[i] = true;

        // Compute the velocity.
        float derivate =
            min(max(derivatives[i], min(kMinVelocity, kMaxVelocity)),
                max(kMinVelocity, kMaxVelocity));
        int32_t velocity = 1. + (derivate - kMinVelocity) /
                                    (kMaxVelocity - kMinVelocity) * 126.;
        // Send Note On for current position at full velocity (127) on
        // channel 1.
        MIDI.sendNoteOn(74 + i, velocity, 1);

        Serial.print(">>> Key ");
        Serial.print(i);
        Serial.print(" velocity: ");
        Serial.print(velocity);
        Serial.print(" pressed, instant_derivative: ");
        Serial.print("instant_derivative: ");
        Serial.print(instant_derivative, 4);
        Serial.print(", derivative: ");
        Serial.print(derivatives[i], 4);
        Serial.print(", measures: ");
        Serial.print(current_measures[i]);
        Serial.print(" -> ");
        Serial.print(last_measures[i]);
        Serial.print(" delta:");
        Serial.print(
            (static_cast<float>(current_measures[i]) - last_measures[i]));
        Serial.println();

      } else if (current_measures[i] > kThresholdHigh && isPressed[i]) {
        isPressed[i] = false;

        // Send Note Off for previous note.
        MIDI.sendNoteOff(74 + i, 0, 1);
        Serial.print(">>> Key ");
        Serial.print(i);
        Serial.println(" released.");
      }
    }

    yield();
    heartbit.tick();
  }
}
