#pragma once
#include <stdint.h>
#include <stdio.h>

char to_color(uint16_t val) {
  const char table[] = ".,;!vlLFE$";
  return table[val / 409];
}

void line_print(uint16_t* frame, int len) {
  for(int i = 0; i < len; i++) {
    printf("%c", to_color(frame[i]));
    //printf("%d ", frame[i]);
  }
  printf("\n");
}
