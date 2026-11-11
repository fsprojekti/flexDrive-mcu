#include "encoders.h"
#include "config.h"
#include "esp_attr.h"
#include "soc/soc.h"
#include "soc/gpio_reg.h"

// Direct register read: fast and safe to call from IRAM ISR.
// (Kept outside the anonymous namespace: REG_READ's static-assert helper warns there.)
FORCE_INLINE_ATTR uint32_t readPin(uint8_t pin) {
  return pin < 32 ? (REG_READ(GPIO_IN_REG) >> pin) & 1u
                  : (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1u;
}

namespace {

struct Encoder {
  uint8_t pinA;
  uint8_t pinB;
  volatile int32_t  count;
  volatile uint32_t errors;
  volatile uint8_t  state;   // (A << 1) | B
};

Encoder enc[NUM_ENCODERS] = {
  {ENC1_PIN_A, ENC1_PIN_B, 0, 0, 0},
  {ENC2_PIN_A, ENC2_PIN_B, 0, 0, 0},
};

// index = (previousState << 2) | newState
// Sign convention is identical to the v1 firmware.
DRAM_ATTR const int8_t kTransition[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0,
};

void IRAM_ATTR encoderIsr(void* arg) {
  Encoder* e = static_cast<Encoder*>(arg);
  const uint8_t prev = e->state;
  const uint8_t now  = (uint8_t)((readPin(e->pinA) << 1) | readPin(e->pinB));

  const int8_t step = kTransition[(prev << 2) | now];
  if (step != 0) {
    e->count = e->count + step;
  } else if ((prev ^ now) == 0b11) {
    e->errors = e->errors + 1;  // both channels changed at once -> an edge was missed
  }
  e->state = now;
}

// GPIO34..39 have no internal pull-ups; INPUT_PULLUP would be ignored there.
uint8_t inputMode(uint8_t pin) {
  return (pin >= 34) ? INPUT : INPUT_PULLUP;
}

void setupEncoder(Encoder& e) {
  pinMode(e.pinA, inputMode(e.pinA));
  pinMode(e.pinB, inputMode(e.pinB));

  e.state = (uint8_t)((digitalRead(e.pinA) << 1) | digitalRead(e.pinB));
  attachInterruptArg(digitalPinToInterrupt(e.pinA), encoderIsr, &e, CHANGE);
  attachInterruptArg(digitalPinToInterrupt(e.pinB), encoderIsr, &e, CHANGE);
}

}  // namespace

void encodersBegin() {
  for (auto& e : enc) setupEncoder(e);
}

int32_t encoderCount(int idx) {
  return ENC_SIGN[idx] * enc[idx].count;   // 32-bit aligned read is atomic on ESP32
}

uint32_t encoderErrors(int idx) {
  return enc[idx].errors;
}
