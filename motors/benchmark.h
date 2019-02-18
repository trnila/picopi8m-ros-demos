#pragma once
#include <stdint.h>
#include <stdio.h>
#include "core_cm4.h"

static volatile uint32_t print;
static volatile int tick;

void SysTick_Handler() {
  tick++;
  if(tick > 100) {
    tick = 0;
    print++;
  }
}

void benchmark() {
  SysTick_Config(SystemCoreClock / 100);
  volatile uint32_t counter;
  uint32_t myprint = 0;
  for(;;) {
    if(myprint != print) {
      printf("%d\r\n", counter);
      counter = 0;
      myprint++;
    }
    counter++;
  }
}
