#include <Arduino.h>

#include <array>
#include <cassert>
#include <vector>

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
  // Enable logging through the serial port.
  Serial.begin(115200);
  adc_array.begin();
}

void loop() {
  std::array<uint32_t, 5 * 8> measures;

  const uint32_t start_time = micros();
  adc_array.read(measures);
  const uint32_t end_time = micros();

  Serial.print("[");
  Serial.print(count++);
  Serial.print("] ");
  // Show sps
  Serial.print(measures.size() * 1e6 / (end_time - start_time));
  Serial.print(" sps | ");

  for (auto m : measures) {
    Serial.print(m);
    Serial.print(", ");
  }
  Serial.println();

  delay(200);
}
