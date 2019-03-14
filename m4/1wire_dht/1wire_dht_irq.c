#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"
#include "fsl_gpt.h"
#include "pin_mux.h"
#include "clock_config.h"

// dont forget to change irq function if you change GPIO port
const int PIN = 26;

// 2 irq for 8 bit in 5 bytes + 3 irqs in start sequence
const int RECEIVE_COMPLETE = 2 * 8 * 5 + 3;

volatile uint32_t tick;
uint8_t data[5];
volatile int state = 0;
volatile uint32_t prev;
volatile uint32_t times[80];

void BOARD_InitPins(void) {
  IOMUXC_SetPinMux(IOMUXC_SAI2_TXD0_GPIO4_IO26, 0U);
  IOMUXC_SetPinConfig(IOMUXC_SAI2_TXD0_GPIO4_IO26, 
      IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
      IOMUXC_SW_PAD_CTL_PAD_SRE(2U) |
      IOMUXC_SW_PAD_CTL_PAD_PUE_MASK); // pull up required for DHT!
}

void SysTick_Handler() {
  tick++;
}

void delay(uint32_t usec) {
  uint32_t dest = usec;
  uint32_t start = tick;
  while(tick - start < dest);
}

void GPIO4_Combined_16_31_IRQHandler() {
  GPIO_PortClearInterruptFlags(GPIO4, 1 << PIN); // GPIO4->ISR = 1 << PIN;
  int val = GPIO_PinRead(GPIO4, PIN);

  if(state < 3) {
    // complete start procedure
    // wait for 0, then 1 then again 0
    if((state & 1) == val) {
      state++;
      prev = tick;
    }
  } else {
    int i = state - 3;
    int b = i / 16;
    int diff = tick - prev;
    prev = tick;

    // if we are at falling edge, and high signal was bigger then 40us
    // store it as a 1
    if((i & 1) == 1) {
     data[b] <<= 1;
     if(diff > 40) {
        data[b] |= 1;
     } 
    }

    times[i] = diff;
    state++;

    // 2 irq for 8 bit in 5 bytes + 3 irqs in start sequence
    if(state >= RECEIVE_COMPLETE) {
      GPIO_DisableInterrupts(GPIO4, 1 << PIN);
    }
  }
}

/*
 * this example reads dht sensor value and prints them to the console
 * connect supply and data pin to the GPIO4_IO26
 */
int main(void) {
  BOARD_RdcInit();
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();
  BOARD_InitMemory();

  printf("dht22 irq example\r\n");

  // setup GPIO4_IO26
  gpio_pin_config_t conf;
  conf.direction = kGPIO_DigitalInput;
  conf.outputLogic = 0;
  conf.interruptMode = kGPIO_NoIntmode;
  GPIO_PinInit(GPIO4, PIN, &conf);

  uint32_t period_us = 1;
  uint32_t ticks = SystemCoreClock / 1000000UL * period_us;

  printf("SystemCoreClock = %u\r\n", SystemCoreClock);
  printf("ticks = %u\r\n", ticks);
  SysTick_Config(ticks);


  for(;;) {
    // 1) pull pin low for at least 1 ms - we choosed 5 ms
    conf.direction = kGPIO_DigitalOutput;
    GPIO_PinInit(GPIO4, PIN, &conf);
    delay(1000);

    // 2) activate pull up and complete receive in interrupts
    state = 0;
    conf.direction = kGPIO_DigitalInput;
    conf.interruptMode = kGPIO_IntRisingOrFallingEdge;
    GPIO_PinInit(GPIO4, PIN, &conf);
    GPIO_EnableInterrupts(GPIO4, 1 << PIN);
    EnableIRQ(GPIO4_Combined_16_31_IRQn);

    // wait for transmission
    delay(2000 * 1000);
    GPIO_DisableInterrupts(GPIO4, 1 << PIN);

    // dump captured times from transmission
    for(int i = 0; i < 80; i += 2) {
      if(i % 16 == 0 && i != 0) {
        printf("\r\n");
      }
      printf("%d %d   ", times[i], times[i + 1]);
    }
    printf("\r\n");

    if(state != RECEIVE_COMPLETE) {
      printf("failed to receive data from dht, current state %d\r\n", state);
      continue;
    }

    // | RH high | RH low | T high | T low | checksum |
    // checksum is sum of first 4 bytes modulo 256
    uint32_t sum = data[0] + data[1] + data[2] + data[3];
    if((sum & 0xFF) != data[4]) {
      printf("checksum %x not matches computed %x\r\n", data[4], sum & 0xFF);
    }

    printf("raw: ");
    for(int i = 0; i < 5; i++) {
      printf("%02x ", data[i]);
    }
    printf("\r\n");

    uint16_t humidity = (data[0] << 8) | data[1];
    int16_t temp = (data[2] << 8) | data[3];

    printf("temp = %d.%d ", temp / 10, temp % 10);
    printf("humidity = %d.%d\r\n", humidity / 10, humidity % 10);
    for(int i = 0; i < 5; i++) {
      data[i] = 0x00;
    }
  }

  for(;;);
}
