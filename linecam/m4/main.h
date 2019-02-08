#pragma once
#include <stdint.h>
#include "fsl_gpio.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define PRINT_OUTPUT 0
#define CAMERA_POINTS 128

#define SI_PORT GPIO4
#define SI_PIN 23

#define CLK_PORT GPIO4
#define CLK_PIN 26

struct Bag {
  SemaphoreHandle_t publish_semaphore;
  SemaphoreHandle_t measure_semaphore;
  uint16_t *frame;
  uint16_t frame_size;
};

inline void SI_set(int state) {
    GPIO_PinWrite(SI_PORT, SI_PIN, state);
}

inline void CLK_set(int state) {
    GPIO_PinWrite(CLK_PORT, CLK_PIN, state);
}

inline char to_color(uint16_t val) {
    const char table[] = ".,;!vlLFE$";
    return table[val / 409];
}

inline void line_print(uint16_t* frame) {
#if PRINT_OUTPUT
        for(int i = 0; i < CAMERA_POINTS; i++) {
            printf("%c", to_color(frame[i]));
        }
        printf("\r\n");
#endif
}
