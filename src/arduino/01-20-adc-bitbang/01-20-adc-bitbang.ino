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
  volatile PadValue *pad_;
  PadValue cs_mask_, clk_mask_, mosi_mask_;
  std::vector<PadValue> miso_masks_;
  uint32_t clk_period_us_;

 public:
  SPIArray(uint8_t clk, uint8_t mosi, std::array<uint8_t, N> miso, uint8_t cs,
           uint32_t clk_period_us)
      : clk_(clk),
        mosi_(mosi),
        miso_(std::move(miso)),
        cs_(cs),
        clk_period_us_(clk_period_us) {
    // Asserts that all specified pins are in the same port.
    auto port = digitalPinToPort(cs);
    assert(port == digitalPinToPort(clk));
    assert(port == digitalPinToPort(mosi));
    for (auto pin : miso) {
      assert(port == digitalPinToPort(pin));
    }

    // Get hardware mapping.
    pad_ = portOutputRegister(port);
    clk_mask_ = digitalPinToBitMask(clk_);
    mosi_mask_ = digitalPinToBitMask(mosi_);
    for (auto pin : miso_) {
      miso_masks_.push_back(digitalPinToBitMask(pin));
    }
    cs_mask_ = digitalPinToBitMask(cs_);
  }

  void begin() {
    pinMode(cs_, OUTPUT);
    digitalWrite(cs_, HIGH);

    pinMode(clk_, OUTPUT);
    // Only mode 0 is supported: idle low on mode 0 and 1
    digitalWrite(clk_, LOW);

    pinMode(mosi_, OUTPUT);
    digitalWrite(mosi_, HIGH);

    for (auto pin : miso_) {
      pinMode(pin, INPUT);
    }
  }

  int size() const { return miso_.size(); }
  void select(bool value) { digitalWrite(cs_, value ? LOW : HIGH); }

  template <unsigned int L>
  void transfer(const std::array<uint8_t, L> &wbuf,
                std::array<uint8_t, L * N> &rbuf) {
    // Only MSB first is supported.
    int r_idx = 0;
    for (const uint8_t wbyte : wbuf) {
      uint8_t rbyte = 0;
      for (uint8_t bit = 0x80; bit != 0; bit >>= 1) {
        digitalWrite(mosi_, (wbyte & bit) != 0);
        digitalWrite(clk_, HIGH);
        delayMicroseconds(clk_period_us_);

        bool rbit = digitalRead(miso_[0]);
        digitalWrite(clk_, LOW);
        delayMicroseconds(clk_period_us_);
        rbyte = (rbyte << 1) | rbit;
      }
      rbuf[r_idx++] = rbyte;
    }
  }
};

template <unsigned int N>
class MCP3008Array {
  SPIArray<N> spi_array_;
  std::array<uint8_t, 3> w_buffer_ = {0x01, 0x00, 0x00};
  std::array<uint8_t, 3 * N> r_buffer_;

 public:
  MCP3008Array(uint8_t clk, uint8_t mosi, std::array<uint8_t, N> miso,
               uint8_t cs, uint32_t clk_period_us)
      : spi_array_(clk, mosi, miso, cs, clk_period_us) {}

  void begin() { spi_array_.begin(); }

  void read(uint8_t channel, uint16_t *value) {
    assert(channel < 8);
    spi_array_.select(true);
    w_buffer_[1] = channel << 4;
    spi_array_.transfer(w_buffer_, r_buffer_);
    spi_array_.select(false);
    for (int i = 0; i < spi_array_.size(); i++) {
      *value++ = (r_buffer_[i * 3 + 1] & 0x03) << 8 | r_buffer_[i * 3 + 2];
    }
  }
};

MCP3008Array<1> adc_array(10, 12, {11}, 13, 1);

uint16_t measures[1] = {0};
uint32_t count = 0;

void setup() {
  // Enable logging through the serial port.
  Serial.begin(115200);
  adc_array.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  count++;
  Serial.print("[");
  Serial.print(count);
  Serial.print("] ");
  Serial.print("ADC0: ");
  adc_array.read(0, measures);
  Serial.println(measures[0]);
  delay(1000);
}
