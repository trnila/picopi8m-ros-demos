#include <stdio.h>
#include <stdint.h>
#include "fsl_debug_console.h"

uint32_t *stack;

void HardFault_Handler(void) __attribute__((naked));

void HardFault_Handler(void) {
    __asm volatile(
	    "mrs r0, MSP\n"
	    "ldr r1, =stack\n"
	    "str r0, [r1]\n"
    );
    printf("HARD FAULT\r\n");
    printf("run gdbserver.py\r\n");
    for(;;);
}

void do_hardfault() {
    SCB->CFSR = 0x2000000;
    SCB->CCR |= 0x10;
    volatile x = 1/0;
}
