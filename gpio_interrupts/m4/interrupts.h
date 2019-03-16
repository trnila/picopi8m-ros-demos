#pragma once
#include <stdint.h>

const int MAX_PINS = 160;

enum class IRQMode {
  Both = 0,
  Rising = 1,
  Falling = 2
};

void interrupts_init();
void enable(uint8_t port, uint8_t pin, IRQMode mode);
void disable(uint8_t port, uint8_t pin);

extern uint32_t counts[MAX_PINS];
extern uint8_t enabled[MAX_PINS];

