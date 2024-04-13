#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <MIDI.h>

#include <array>
#include <cassert>
#include <vector>

constexpr size_t kNotesCount = 12;
constexpr char const *kNotesNames[kNotesCount] = {
    "C",     "C#/Db", "D",     "D#/Eb", "E",     "F",
    "F#/Gb", "G",     "G#/Ab", "A",     "A#/Bb", "B",
};

// Returns the MIDI note index for a given not and octave.
enum { C, Cd, D, Dd, E, F, Fd, G, Gd, A, Ad, B };
constexpr uint8_t n(int note, int octave) {
  return (octave + 1) * kNotesCount + static_cast<int>(note);
}

// Maps measure position to a MIDI note index.
// 0: close/push, 1: open/pull
constexpr uint8_t keyboard_mapping[2][72] = {
    {
        /*00*/ 0,        n(Fd, 4), n(A, 6),  n(Ad, 4), n(A, 5),
        /*05*/ n(E, 2),  n(G, 3),  n(Ad, 2), n(Gd, 3), n(G, 4),
        /*10*/ n(E, 4),  n(G, 6),  n(B, 4),  n(E, 6),  n(A, 2),
        /*15*/ n(A, 3),  n(C, 4),  n(Cd, 2), n(A, 3),  n(F, 5),
        /*20*/ n(Gd, 4), n(E, 5),  n(F, 6),  n(E, 3),  n(Gd, 4),
        /*25*/ n(D, 4),  n(Dd, 4), n(Ad, 3), n(C, 4),  n(G, 4),
        /*30*/ n(B, 5),  n(Dd, 6), n(D, 2),  n(G, 2),  n(E, 4),
        /*35*/ n(C, 3),  n(B, 3),  n(Cd, 4), n(Cd, 5), n(Fd, 6),
        /*40*/ n(D, 6),  n(G, 2),  n(Ad, 3), n(F, 3),  n(Fd, 3),
        /*45*/ n(F, 4),  n(Fd, 5), n(Gd, 5), n(C, 6),  n(G, 5),
        /*50*/ n(E, 3),  n(B, 3),  n(Dd, 3), n(B, 2),  n(Dd, 4),
        /*55*/ n(A, 4),  n(Gd, 6), n(C, 5),  n(Cd, 6), n(Fd, 4),
        /*60*/ n(Cd, 4), n(Cd, 3), n(Fd, 2), n(D, 4),  n(E, 5),
        /*65*/ n(Ad, 5), n(D, 5),  n(Dd, 5), n(D, 3),  n(B, 4),
        /*70*/ n(F, 4),  n(F, 2),
    },
    {
        /*00*/ 0,        n(E, 4),  n(B, 6),  n(Ad, 5), n(Gd, 5),
        /*05*/ n(D, 2),  n(A, 3),  n(Ad, 2), n(Cd, 4), n(Gd, 4),
        /*10*/ n(Ad, 4), n(A, 6),  n(A, 4),  n(Cd, 6), n(E, 3),
        /*15*/ n(B, 3),  n(Dd, 3), n(Dd, 2), n(A, 3),  n(F, 5),
        /*20*/ n(G, 4),  n(D, 5),  n(F, 6),  n(B, 2),  n(A, 4),
        /*25*/ n(E, 4),  n(F, 3),  n(Ad, 3), n(Cd, 4), n(Fd, 4),
        /*30*/ n(A, 5),  n(Dd, 6), n(E, 2),  n(Gd, 2), n(Fd, 4),
        /*35*/ n(Ad, 3), n(B, 3),  n(D, 4),  n(B, 4),  n(G, 6),
        /*40*/ n(D, 6),  n(D, 3),  n(G, 3),  n(Fd, 3), n(G, 2),
        /*45*/ n(F, 4),  n(Cd, 5), n(Fd, 5), n(E, 6),  n(E, 5),
        /*50*/ n(Gd, 3), n(C, 4),  n(Cd, 3), n(Fd, 2), n(Dd, 4),
        /*55*/ n(Gd, 4), n(Gd, 6), n(C, 6),  n(B, 5),  n(G, 4),
        /*60*/ n(D, 4),  n(F, 4),  n(F, 2),  n(C, 4),  n(Dd, 5),
        /*65*/ n(Fd, 6), n(C, 5),  n(G, 5),  n(A, 2),  n(Dd, 4),
        /*70*/ n(C, 3),  n(C, 2),
    }};

constexpr uint8_t keyboard_channel[72] = {
    0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0,
    0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1,
    1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0,
};
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

template <typename... A>
struct TT {};
template <size_t N>
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

template <size_t N>
class MCP3008Array {
  SPIArray<N> spi_array_;

 public:
  typedef std::array<uint32_t, N * 8> Values;

  // Use digital pin index as visible on the Feather pinout.
  MCP3008Array(uint8_t clk, uint8_t mosi, std::array<uint8_t, N> miso,
               uint8_t cs)
      : spi_array_(clk, mosi, miso, cs) {}

  void begin() const { spi_array_.begin(); }
  // Returns the measures for all channels of all ADCs.
  // values[channel_idx * N + adc_idx]
  void read(Values &values) const {
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

MCP3008Array<9> adc_array(13, 12, {10, 9, 6, 5, 22, 21, 4, 25, 19}, 11);
uint32_t count = 0;

void setup() {
  usb_midi.setStringDescriptor("Bandoneon");

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
  static constexpr uint32_t kHeartbitPeriodMs = 1000;

 public:
  Heartbit() : last_display_ms_(millis()), tick_count_(0) {}
  bool tick() {
    tick_count_++;
    uint32_t delay = millis() - last_display_ms_;
    if (delay > kHeartbitPeriodMs) {
      last_display_ms_ += kHeartbitPeriodMs;
      Serial.print("Heartbit: scan frequency is ");
      Serial.print(tick_count_ * 1000 / delay);
      Serial.println(" Hz");
      tick_count_ = 0;
      return true;
    }
    return false;
  }
};

class Devices {
 public:
  struct Pinout {
    uint32_t p1_analog;
    uint32_t p2_analog;
    uint32_t expression_pedal_analog;
    uint32_t joy_x_analog;
    uint32_t joy_y_analog;
    uint32_t joy_bt_digital;
    uint32_t sustain_pedal_digital;
  } pinout_;

  struct Value {
    uint32_t p1;
    uint32_t p2;
    uint32_t expression_pedal;
    uint32_t joy_x;
    uint32_t joy_y;
    bool joy_bt;
    bool sustain_pedal;

    void print() {
      Serial.print("p1: ");
      Serial.print(p1);
      Serial.print(" p2: ");
      Serial.print(p2);
      Serial.print(" exp: ");
      Serial.print(expression_pedal);
      Serial.print(" sus: ");
      Serial.print(sustain_pedal);
      Serial.println();
      Serial.print("joy_x: ");
      Serial.print(joy_x);
      Serial.print(" y: ");
      Serial.print(joy_y);
      Serial.print(" bt: ");
      Serial.print(joy_bt);
      Serial.println();
    }
  };

  Devices(Pinout pinout) : pinout_(pinout) {
    pinMode(pinout_.p1_analog, INPUT);
    pinMode(pinout_.p2_analog, INPUT);
    pinMode(pinout_.expression_pedal_analog, INPUT);
    pinMode(pinout_.joy_x_analog, INPUT);
    pinMode(pinout_.joy_y_analog, INPUT);
    pinMode(pinout_.sustain_pedal_digital, INPUT_PULLUP);
    pinMode(pinout_.joy_bt_digital, INPUT_PULLUP);
  }

  void read(Value &value) {
    value.p1 = analogRead(pinout_.p1_analog);
    value.p2 = analogRead(pinout_.p2_analog);
    value.expression_pedal = analogRead(pinout_.expression_pedal_analog);
    value.joy_x = analogRead(pinout_.joy_x_analog);
    value.joy_y = analogRead(pinout_.joy_y_analog);
    value.joy_bt = digitalRead(pinout_.joy_bt_digital);
    value.sustain_pedal = digitalRead(pinout_.sustain_pedal_digital);
  }
};

Devices devices({
    .p1_analog = A1,
    .p2_analog = A0,
    .expression_pedal_analog = A2,
    .joy_x_analog = A3,
    .joy_y_analog = A4,
    .joy_bt_digital = 1,
    .sustain_pedal_digital = 0,
});

struct HysteresisFilter {
  const int32_t gap = 5;
  bool increasing = true;
  int32_t previous_value = 0;

  uint32_t apply(int32_t new_value) {
    if (increasing) {
      if (new_value > previous_value) {
        previous_value = new_value;
      } else if (new_value < previous_value - gap) {
        increasing = false;
        previous_value = new_value;
      }
    } else {
      if (new_value < previous_value) {
        previous_value = new_value;

      } else if (new_value > previous_value - gap) {
        increasing = true;
        previous_value = new_value;
      }
    }
    return previous_value;
  }
};

struct AxisMapper {
  uint32_t min;
  uint32_t max;

  uint32_t out_min;
  uint32_t out_max;

  uint32_t remap(uint32_t value) const {
    if (value < min) {
      return out_min;
    }
    if (value < max) {
      return out_min + (value - min) * (out_max - out_min) / (max - min);
    }
    return out_max;
  }
};

struct JoystickAxisMapper {
  uint32_t min;
  uint32_t max;
  uint32_t dead_min;  // inclusive
  uint32_t dead_max;

  uint32_t out_min;
  uint32_t out_max;
  uint32_t out_center;

  uint32_t remap(uint32_t value) const {
    if (value < min) {
      return out_min;
    }
    if (value < dead_min) {
      return out_center -
             (dead_min - value) * (out_center - out_min) / (dead_min - min);
    }
    if (value <= dead_max) {
      return out_center;
    }
    if (value < max) {
      return out_center +
             (value - dead_max) * (out_max - out_center) / (max - dead_max);
    }
    return out_max;
  }
};

constexpr AxisMapper expression_mapper{
    .min = 92,
    .max = 515,
    .out_min = 0x0,
    .out_max = 0x7f,
};

constexpr AxisMapper p1_mapper{
    .min = 3 + 1,
    .max = 1021 - 1,
    .out_min = 0x0,
    .out_max = 0x7f,
};

constexpr AxisMapper p2_mapper{
    .min = 3 + 1,
    .max = 1021 - 1,
    .out_min = 0x0,
    .out_max = 0x7f,
};

constexpr JoystickAxisMapper y_axis_mapper{.min = 3 + 1,
                                           .max = 1023 - 1,
                                           .dead_min = 515 - 2,
                                           .dead_max = 515 + 2,
                                           .out_min = 0x0,
                                           .out_max = 0x7f,
                                           .out_center = 0x40};

constexpr bool kPrintKey = false;
constexpr bool kPrintMidiEvents = false;

// Low pass filter on the derivate of the signal.
// The current derivate is merged in the previous one with a factor.
// 1 ignore the past, and 0 ignore the present.
constexpr float kDecayFactor = 0.25;
// Velocity profile.
constexpr float kMinVelocity = 0.26;
constexpr float kMaxVelocity = 10.47;

// Measure above which one a key is considered pressed.
constexpr int32_t kPressThreshold = 600;
constexpr int32_t kReleaseThreshold = 580;

constexpr int32_t kMeasuresSize = 16;

constexpr int32_t kMidiChannel = 1;

// Holds reading for each key, with some history.
decltype(adc_array)::Values measures[kMeasuresSize] = {};
// Time just before staring the corresponding reading.
int32_t measures_us[kMeasuresSize] = {};

std::array<bool, measures[0].size()> isPressed{};

template <size_t N>
void printKeyboardState(const std::array<uint32_t, N> &values) {
  Serial.print("=== Keyboard state ===");
  for (uint32_t i = 0; i < values.size(); i++) {
    if (i % (N / 8) == 0) Serial.println();
    size_t len = Serial.print(values[i] > 600 ? 1 : 0);
    for (int c = max(0, 5 - len); c--;) Serial.print(' ');
    Serial.print(',');
  }
  Serial.println();
}

void printNoteName(uint8_t midi_code) {
  const int note_idx = midi_code % kNotesCount;
  const int octave = midi_code / kNotesCount - 1;
  Serial.print(kNotesNames[note_idx]);
  Serial.print(octave);
}

void loop() {
  Heartbit heartbit;

  // Initialises all the measure histroy from an initial reading.
  for (uint32_t t = 1; t < kMeasuresSize; t++) {
    measures_us[t] = micros();
    adc_array.read(measures[t]);
  }

  uint32_t t1_id = 0;
  uint32_t previous_bend = 0x40;  // center
  uint32_t previous_expression = 0;
  uint32_t previous_p1 = 0;
  uint32_t previous_p2 = 0;
  HysteresisFilter p1_filter, p2_filter;

  for (;;) {
    // Capture time and measures.
    auto t1 = measures_us[t1_id] = micros();
    auto &measures_t1 = measures[t1_id];
    adc_array.read(measures_t1);

    // Use the oldest available record as reference point.
    auto t0_id = t1_id = (t1_id + 1) % kMeasuresSize;

    const auto t0 = measures_us[t0_id];
    const auto &measures_t0 = measures[t0_id];

    // The oldest value is also the next to write to.
    t1_id = t0_id;

    for (int i = 0; i < measures_t1.size(); i++) {
      if (measures_t1[i] > kPressThreshold && !isPressed[i]) {
        isPressed[i] = true;

        float derivate = 1000.f * (measures_t1[i] - measures_t0[i]) / (t1 - t0);
        // Clamp the derivative.
        derivate = min(max(derivate, min(kMinVelocity, kMaxVelocity)),
                       max(kMinVelocity, kMaxVelocity));
        // Compute the velocity from 1 to 127.
        const int32_t velocity = 1. + (derivate - kMinVelocity) /
                                          (kMaxVelocity - kMinVelocity) * 126.;

        // Send Note On for current position at full velocity (127) on
        // channel 1.
        MIDI.sendNoteOn(keyboard_mapping[1][i], velocity,
                        kMidiChannel + keyboard_channel[i]);
        if (kPrintKey) {
          Serial.print(">>> Key ");
          Serial.print(i);
          Serial.print(" mapping: ");
          Serial.print(keyboard_mapping[1][i]);
          Serial.print("=");
          printNoteName(keyboard_mapping[1][i]);

          Serial.print(" derivate: ");
          Serial.print(derivate);
          Serial.print(" velocity: ");
          Serial.print(velocity);
          Serial.print(", measures: ");
          Serial.print(measures_t1[i]);
          Serial.print(" -> ");
          Serial.print(measures_t0[i]);
          Serial.print(" delta:");
          Serial.print((static_cast<float>(measures_t1[i]) - measures_t0[i]));
          Serial.println();
        }
      } else if (measures_t1[i] < kReleaseThreshold && isPressed[i]) {
        isPressed[i] = false;

        // Send Note Off for previous note.
        MIDI.sendNoteOff(keyboard_mapping[1][i], 0,
                         kMidiChannel + keyboard_channel[i]);
        if (kPrintKey) {
          Serial.print(">>> Key ");
          Serial.print(i);
          Serial.println(" released.");
        }
      }
    }

    Devices::Value device_values;
    devices.read(device_values);

    const uint32_t bend = y_axis_mapper.remap(device_values.joy_y);
    if (bend != previous_bend) {
      MIDI.send(midi::MidiType::PitchBend, 0, bend, kMidiChannel);
      previous_bend = bend;
      if (kPrintMidiEvents) {
        Serial.print("joy: ");
        Serial.print(device_values.joy_y);
        Serial.print(" bend:");
        Serial.println(bend);
      }
    }

    const uint32_t expression =
        expression_mapper.remap(device_values.expression_pedal);
    if (expression != previous_expression) {
      MIDI.send(midi::MidiType::ControlChange, 1, expression, kMidiChannel);
      previous_expression = expression;
    }
    const uint32_t p1 = p1_mapper.remap(p1_filter.apply(device_values.p1));
    if (p1 != previous_p1) {
      MIDI.send(midi::MidiType::ControlChange, 12, p1, kMidiChannel);
      previous_p1 = p1;
      if (kPrintMidiEvents) {
        Serial.print("device_values.p1: ");
        Serial.print(device_values.p1);
        Serial.print(" p1: ");
        Serial.print(p1);
        Serial.println();
      }
    }

    const uint32_t p2 = p2_mapper.remap(p2_filter.apply(device_values.p2));
    if (p2 != previous_p2) {
      MIDI.send(midi::MidiType::ControlChange, 13, p2, kMidiChannel);
      previous_p2 = p2;
      if (kPrintMidiEvents) {
        Serial.print("device_values.p2: ");
        Serial.print(device_values.p2);
        Serial.print(" p2: ");
        Serial.print(p2);
        Serial.println();
      }
    }
    // Let the USB stack opeate if needed.
    yield();
    if (heartbit.tick()) {
      // device_values.print();
      printKeyboardState(measures_t1);
    }
  }
}
