#include <stdio.h>
#include <string.h>

#include "m4core.h"

int main(int argc, char** argv) {
  if(argc <= 1) {
    fprintf(stderr, "Usage: %s (start|stop|load firmware.bin)\n", argv[0]);
    return 1;
  }

  M4Core core;

  if(strcmp(argv[1], "stop") == 0) {
    core.stop();
  } else if(strcmp(argv[1], "start") == 0) {
    core.start();
  } else if(strcmp(argv[1], "load") == 0) {
    if(argc <= 2) {
      fprintf(stderr, "Missing path to firmware");
      return 1;
    }

    core.boot_firmware(argv[2]);
  } else {
    fprintf(stderr, "unknown action %s", argv[1]);
    return 1;
  }

  return 0;
}
