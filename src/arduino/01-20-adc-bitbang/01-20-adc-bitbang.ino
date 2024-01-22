#include <Arduino.h>

#include <array>
#include <cassert>
#include <vector>

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

class Clock {
  uint32_t half_period_cyc_;
  uint32_t last_cyc_;
  uint32_t overshoot_count_;

 public:
  Clock(uint32_t half_period_cyc)
      : half_period_cyc_(half_period_cyc),
        last_cyc_(DWT->CYCCNT),
        overshoot_count_(0) {}
  inline void delay() {
    const uint32_t now = DWT->CYCCNT;
    if (now - last_cyc_ >= half_period_cyc_) {
      last_cyc_ = now;  // We overshoot. We need to reset the counter.
      overshoot_count_++;
      return;
    }
    while (DWT->CYCCNT - last_cyc_ < half_period_cyc_) {
    }
    last_cyc_ += half_period_cyc_;
  }
  uint32_t overshoot_count() const { return overshoot_count_; }
  uint32_t half_period_cyc() const { return half_period_cyc_; }
};

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
  volatile PadValue *output_reg_;
  volatile PadValue *input_reg_;
  PadValue cs_mask_, clk_mask_, mosi_mask_;
  std::vector<PadValue> miso_masks_;

 public:
  uint32_t half_period_cyc_;
  SPIArray(uint8_t clk, uint8_t mosi, std::array<uint8_t, N> miso, uint8_t cs,
           uint32_t frequency_hz)
      : clk_(clk),
        mosi_(mosi),
        miso_(std::move(miso)),
        cs_(cs),
        half_period_cyc_(SystemCoreClock / frequency_hz / 2) {
    // Asserts that all specified pins are in the same port.
    auto port = digitalPinToPort(cs);
    assert(port == digitalPinToPort(clk));
    assert(port == digitalPinToPort(mosi));
    for (auto pin : miso) {
      assert(port == digitalPinToPort(pin));
    }

    // Get hardware mapping.
    output_reg_ = portOutputRegister(port);
    input_reg_ = portInputRegister(port);
    clk_mask_ = digitalPinToBitMask(clk_);
    mosi_mask_ = digitalPinToBitMask(mosi_);
    for (auto pin : miso_) {
      miso_masks_.push_back(digitalPinToBitMask(pin));
    }
    cs_mask_ = digitalPinToBitMask(cs_);
  }

  void show() {
    Serial.print("output_reg_: ");
    Serial.println((uint32_t)output_reg_, HEX);
    Serial.print("input_reg_: ");
    Serial.println((uint32_t)input_reg_, HEX);
    Serial.print("CS mask: ");
    Serial.println(cs_mask_, HEX);
    Serial.print("CLK mask: ");
    Serial.println(clk_mask_, HEX);
    Serial.print("MOSI mask: ");
    Serial.println(mosi_mask_, HEX);
    for (auto mask : miso_masks_) {
      Serial.print("MISO mask: ");
      Serial.println(mask, HEX);
    }
  }

  void begin() {
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
  inline void select(bool value) {
    // Fast digitalWrite(cs_, value ? LOW : HIGH);
    *output_reg_ = (*output_reg_ & ~cs_mask_) | (value ? 0 : cs_mask_);
  }

  template <unsigned int L>
  void transfer(const std::array<uint8_t, L> &wbuf, uint32_t bits_to_transmit,
                std::array<uint8_t, L * N> &rbuf) {
    // Only MSB first is supported.
    int r_idx = 0;
    int lastmosi = -1;
    Clock clk(half_period_cyc_);
    for (const uint8_t wbyte : wbuf) {
      uint8_t rbyte[N];
      for (uint8_t bit = 0x80; bit != 0x80 >> bits_to_transmit; bit >>= 1) {
        const bool mosi_value = (wbyte & bit) != 0;
        if (mosi_value != lastmosi) {
          // Fast digitalWrite(mosi_, mosi_value);
          *output_reg_ = (*output_reg_ & ~mosi_mask_) | (mosi_value ? mosi_mask_ : 0);
          lastmosi = mosi_value;
        }
        // Fast digitalWrite(clk_, HIGH);
        *output_reg_ |= clk_mask_;

        clk.delay();
        //delayCycles(half_period_cyc_);

        for (int i = 0; i < N; i++) {
          // Fast
          //bool bit = digitalRead(miso_[i]);
          bool bit = (*input_reg_ & miso_masks_[i]) != 0;
          rbyte[i] = (rbyte[i] << 1) | bit;
        }
        // Fast digitalWrite(clk_, LOW);
        *output_reg_ ^= clk_mask_;

        clk.delay();
        //delayCycles(half_period_cyc_);
      }
      for (int i = 0; i < N; i++) {
        rbuf[r_idx++] = rbyte[i];
      }
      bits_to_transmit -= 8;
    }
    // Serial.print("Overshoot count: ");
    // Serial.print(clk.overshoot_count());
    // Serial.print(" (");
    // Serial.print(clk.half_period_cyc());
    // Serial.println(" cycles)");
  }
};

template <unsigned int N>
class MCP3008Array {
  SPIArray<N> spi_array_;
  std::array<uint8_t, 3> w_buffer_ = {0x01, 0x00, 0x00};
  std::array<uint8_t, 3 * N> r_buffer_;

 public:
  MCP3008Array(uint8_t clk, uint8_t mosi, std::array<uint8_t, N> miso,
               uint8_t cs, uint32_t frequency_hz)
      : spi_array_(clk, mosi, miso, cs, frequency_hz) {}

  void begin() { spi_array_.begin(); }

  void read(uint8_t channel, std::array<uint16_t, N> &values) {
    assert(channel < 8);
    spi_array_.select(true);
    w_buffer_[1] = channel << 4;
    spi_array_.transfer(w_buffer_, 18, r_buffer_);
    spi_array_.select(false);
    for (int i = 0; i < spi_array_.size(); i++) {
      values[i] = (r_buffer_[i + N] & 0x03) << 8 | r_buffer_[i + 2 * N];
    }
  }

  void show() { spi_array_.show(); }
};

MCP3008Array<1> adc_array(10, 12, {11}, 13, 2.16e6);

std::array<uint16_t, 1> measures;

void setup() {
  // Enable logging through the serial port.
  Serial.begin(115200);
  adc_array.begin();
}

void loop() {
  adc_array.show();
  
  size_t sample_size = 1000;
  const uint32_t start_time = micros();
  uint32_t count = 0;
  while (micros() - start_time < 1000000) {
    adc_array.read(0, measures);
    count++;
  }
  Serial.print("Number of samples per seconds: ");
  Serial.println(count);

  Serial.print("[");
  Serial.print(count);
  Serial.print("] ");
  Serial.print("ADC0: ");
  Serial.println(measures[0]);
  delay(1000);
}
