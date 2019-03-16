#include "fsl_gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "fsl_iomuxc.h"
#include "interrupts.h"
#include "pins.h"
#include <stdio.h>

static GPIO_Type* gpios[] = {
    GPIO1,
    GPIO2,
    GPIO3,
    GPIO4,
    GPIO5
};

uint32_t counts[MAX_PINS];
uint8_t enabled[MAX_PINS];

void handle(int port, int half) {
    GPIO_Type* gpio = gpios[port]; 

    // get which enabled pins generated interrupt
    int status = gpio->IMR & gpio->ISR;

    int mask = half ? 1 << 16 : 1;
    int offset = half * 16;
    for(int i = 0; i < 16; i++) {
        if(status & mask) {
            counts[port * 32 + offset + i]++;
        }
        mask <<= 1;
    }

    // clear interrupts only for our half
    gpio->ISR |= status & (0xFFFF << (offset));
}

void interrupts_init() {
    // enable all GPIO interrupts
    for(int irq = GPIO1_Combined_0_15_IRQn; irq <= GPIO5_Combined_16_31_IRQn; irq++) {
        EnableIRQ((IRQn_Type) irq);
    }
}

void enable(uint8_t port, uint8_t pin, IRQMode mode) {
    int num = (port - 1) * 32 + pin;
    configASSERT(num <= MAX_PINS);
    const struct pinmux_conf *pinmux = &pinmux_confs[num];

    IOMUXC_SetPinMux(
            pinmux->mux_register, 
            0x5,
            0x0,
            0x0,
            pinmux->config_register,
            0x0
    );
    IOMUXC_SetPinConfig(
            pinmux->mux_register, 
            0x5,
            0x0,
            0x0,
            pinmux->config_register,
            IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
            IOMUXC_SW_PAD_CTL_PAD_SRE(3U)
    );

    GPIO_Type* gport = gpios[port - 1];
    gpio_pin_config_t conf;
    conf.direction = kGPIO_DigitalInput;
    conf.outputLogic = 0;
    if(mode == IRQMode::Rising) {
      conf.interruptMode = kGPIO_IntRisingEdge;
    } else if(mode == IRQMode::Falling) {
      conf.interruptMode = kGPIO_IntFallingEdge;
    } else {
      conf.interruptMode = kGPIO_IntRisingOrFallingEdge;
    }
    GPIO_PinInit(gport, pin, &conf);
    GPIO_EnableInterrupts(gport, 1 << pin);

    enabled[num] = 1;
}

void disable(uint8_t port, uint8_t pin) {
    int num = (port - 1) * 32 + pin;
    GPIO_DisableInterrupts(gpios[port - 1], 1 << pin);
    enabled[num] = 0;
}

extern "C" void GPIO1_Combined_0_15_IRQHandler() {
  handle(0, 0);
}
extern "C" void GPIO1_Combined_16_31_IRQHandler() {
  handle(0, 1);
}

extern "C" void GPIO2_Combined_0_15_IRQHandler() {
  handle(1, 0);
}
extern "C" void GPIO2_Combined_16_31_IRQHandler() {
  handle(1, 1);
}

extern "C" void GPIO3_Combined_0_15_IRQHandler() {
  handle(2, 0);
}
extern "C" void GPIO3_Combined_16_31_IRQHandler() {
  handle(2, 1);
}

extern "C" void GPIO4_Combined_0_15_IRQHandler() {
  handle(3, 0);
}
extern "C" void GPIO4_Combined_16_31_IRQHandler() {
  handle(3, 1);
}

extern "C" void GPIO5_Combined_0_15_IRQHandler() {
  handle(4, 0);
}
extern "C" void GPIO5_Combined_16_31_IRQHandler() {
  handle(4, 1);
}
