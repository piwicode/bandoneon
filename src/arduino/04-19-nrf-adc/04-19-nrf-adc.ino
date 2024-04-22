#include <Adafruit_TinyUSB.h>  // for Serial
#include <Arduino.h>

#include <array>

#define DUMP(x)            \
  do {                     \
    Serial.print(#x ": "); \
    Serial.println(x);     \
  } while (0)

void setup() {
  // Initialises output to serial monitor.
  // Calls to Serial.print functions before this, typically in a static ctor,
  // will hang the system.
  Serial.begin(115200);
  while (!Serial) delay(10);  // for nrf52840 with native usb
  Serial.println("!! Program start: " __FILE__);

  // Lower the enable pin of the analog muxes.
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
}

constexpr uint32_t grey_code(uint32_t n) { return n ^ (n >> 1); }
constexpr uint32_t grey_mask(uint32_t n) {
  return grey_code(n) ^ grey_code((n + 15) % 16);
}
constexpr uint32_t log2(uint32_t n) {
  return n == 1 ? 0 : n == 2 ? 1 : n == 4 ? 2 : 3;
}
constexpr uint32_t grey_bit(uint32_t n) { return log2(grey_mask(n)); }

constexpr bool grey_is_set(uint32_t n) { return grey_mask(n) & grey_code(n); }

template <size_t CB>
class ChannelCycler {
 public:
  static constexpr size_t channel_count = 1 << CB;

  ChannelCycler(NRF_GPIO_Type *port, std::array<uint8_t, CB> pins) {
    // Standard drive for high and low.
    for (auto pin : pins) {
      port->PIN_CNF[pin] =
          ((uint32_t)GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
          ((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
          ((uint32_t)GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
          ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
          ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
    }
    for (uint32_t i = 0; i < channel_count; i++) {
      ops_[i] = OP{grey_code(i), grey_is_set(i) ? &port->OUTSET : &port->OUTCLR,
                   static_cast<uint32_t>(1 << pins[grey_bit(i)])};
    }
  }

  template <typename F>
  void for_each(F f) {
    for (auto &op : ops_) {
      op.write();
      f(op.channel);
    }
  }

 private:
  struct OP {
    uint32_t channel;
    volatile uint32_t *reg;
    uint32_t mask;
    void write() { *reg = mask; }
  } ops_[channel_count];
};

// Pinout
// https://cdn-learn.adafruit.com/assets/assets/000/114/673/large1024/circuitpython_Adafruit_Feather_nRF52840_Pinout.png?1662064111
enum AnalogPin : uint8_t {
  AIN0 = SAADC_CH_PSELP_PSELP_AnalogInput0,
  AIN1 = SAADC_CH_PSELP_PSELP_AnalogInput1,
  AIN2 = SAADC_CH_PSELP_PSELP_AnalogInput2,
  AIN3 = SAADC_CH_PSELP_PSELP_AnalogInput3,
  AIN4 = SAADC_CH_PSELP_PSELP_AnalogInput4,
  AIN5 = SAADC_CH_PSELP_PSELP_AnalogInput5,
  AIN6 = SAADC_CH_PSELP_PSELP_AnalogInput6,
};

template <size_t N>
class AnalogSequenceReader {
 public:
  typedef std::array<volatile int16_t, N> Values;

  AnalogSequenceReader(std::array<AnalogPin, N> analog_pins)
      : pins_(std::move(analog_pins)) {
    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_10bit;
    NRF_SAADC->ENABLE =
        (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos);

    // Use the internal referenc, rather than VDD.
    constexpr uint16_t saadcReference = SAADC_CH_CONFIG_REFSEL_Internal;
    // Gain applied to the input.
    constexpr uint16_t saadcGain = SAADC_CH_CONFIG_GAIN_Gain1_6;
    // Duration of the voltage sampling.
    constexpr uint16_t saadcSampleTime = SAADC_CH_CONFIG_TACQ_3us;
    // Disable over sampling.
    constexpr bool saadcBurst = SAADC_CH_CONFIG_BURST_Disabled;

    constexpr uint16_t config =
        ((SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESP_Pos) &
         SAADC_CH_CONFIG_RESP_Msk) |
        ((SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESN_Pos) &
         SAADC_CH_CONFIG_RESN_Msk) |
        ((saadcGain << SAADC_CH_CONFIG_GAIN_Pos) & SAADC_CH_CONFIG_GAIN_Msk) |
        ((saadcReference << SAADC_CH_CONFIG_REFSEL_Pos) &
         SAADC_CH_CONFIG_REFSEL_Msk) |
        ((saadcSampleTime << SAADC_CH_CONFIG_TACQ_Pos) &
         SAADC_CH_CONFIG_TACQ_Msk) |
        ((SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos) &
         SAADC_CH_CONFIG_MODE_Msk) |
        ((saadcBurst << SAADC_CH_CONFIG_BURST_Pos) & SAADC_CH_CONFIG_BURST_Msk);
    int c;
    // Enable channels and connect ot the input pin.
    // A channel is considered enabled if CH[n].PSELP is set. If more than one
    // channel, CH[n], is enabled, the ADC enters scan mode. In scan mode, one
    // SAMPLE task will trigger one conversion per enabled channel. T
    for (c = 0; c < N; c++) {
      NRF_SAADC->CH[c].PSELN = SAADC_CH_PSELP_PSELP_NC;
      NRF_SAADC->CH[c].PSELP = pins_[c];
      NRF_SAADC->CH[c].CONFIG = config;
    }

    for (; c < 8; c++) {
      NRF_SAADC->CH[c].PSELN = SAADC_CH_PSELP_PSELP_NC;
      NRF_SAADC->CH[c].PSELP = SAADC_CH_PSELP_PSELP_NC;
    }

    NRF_SAADC->RESULT.PTR = reinterpret_cast<uint32_t>(values_.data());
    // The size of the Result buffer is specified in the RESULT.MAXCNT register
    // and the ADC will generate an END event when it has filled up the Result
    // buffer.
    NRF_SAADC->RESULT.MAXCNT = values_.size();
    NRF_SAADC->TASKS_START = 0x01UL;
  }

#define STRINGIZE(x) STRINGIZE2(x)
#define STRINGIZE2(x) #x
#define LINE_STRING STRINGIZE(__LINE__)
#define CONSUME_EVENT(p)                                        \
  do {                                                          \
    int i = 640000;                                             \
    while (!(p)) {                                              \
      if (!i--) {                                               \
        Serial.println(" ** bail " #p "  L" LINE_STRING " **"); \
        break;                                                  \
      }                                                         \
    }                                                           \
    p = 0x00UL;                                                 \
  } while (0)

  const Values &read() {
    CONSUME_EVENT(NRF_SAADC->EVENTS_STARTED);

    NRF_SAADC->TASKS_SAMPLE = 0x01UL;
    // The end event occurs when the buffer is full. This is the only way to
    // detect that channels scans is complete, as EVENTS_DONE is triggered for
    // each sample collected. Waiting for N x EVENTS_DONE occurences is not
    // practical as it is easy to skip one because of an interuption.
    CONSUME_EVENT(NRF_SAADC->EVENTS_END);

    NRF_SAADC->TASKS_STOP = 0x01UL;
    CONSUME_EVENT(NRF_SAADC->EVENTS_STOPPED);

    // Restart the task to hide the EasyDMA setup latency.
    NRF_SAADC->TASKS_START = 0x01UL;

    return values_;
  }

 private:
  std::array<AnalogPin, N> pins_;
  Values values_ = {};
};

ChannelCycler<4> cycler(NRF_P0, {25, 24, 15, 13});
AnalogSequenceReader<5> ain({AIN2, AIN3, AIN6, AIN4, AIN0});

void pad(size_t width, size_t cnt) {
  for (int c = max(0, width - cnt); c--;) Serial.print(' ');
}

// https://stackoverflow.com/questions/1174169/function-passed-as-template-argument
void loop() {
  constexpr size_t N = 100;
  const uint32_t start_time = micros();
  for (int i = 0; i < N; i++) {
    cycler.for_each([](int channel) { ain.read(); });
  }
  const uint32_t end_time = micros();
  Serial.print(N * 1e6 / (end_time - start_time));

  Serial.print(" Hz  ");
  Serial.println("======================================");

  cycler.for_each([](int i) {
    auto &values = ain.read();
    Serial.print("ch[");
    pad(2, Serial.print(i));
    Serial.print("] ");
    for (auto value : values) {
      pad(6, Serial.print(value));
    }
    Serial.println();
  });

  delay(500);
}
