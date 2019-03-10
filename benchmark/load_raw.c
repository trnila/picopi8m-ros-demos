#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define SRC_M4RCR         0x3039000C
#define START_CLEAR_MASK  0xFFFFFF00
#define START_SET_MASK    0x000000AA
#define STOP_SET_MASK     0x00000001

#define PAGE_MASK         (4096 - 1)
#define SRC_M4RCR_SIZE    4
#define TCM_SIZE          0x3FFFF
#define TCM_ADDR          0x7e0000

uint8_t* do_map(int fd, uint64_t phys_addr, int size) {
  uint8_t* addr = (uint8_t*) mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, phys_addr & ~PAGE_MASK);
  if(addr == MAP_FAILED) {
    return 0;
  }

  return addr + (phys_addr & PAGE_MASK);
}

int main(int argc, char** argv) {
  if(argc <= 1) {
    fprintf(stderr, "Usage: %s firmware.bin\n", argv[0]);
    return -1;
  }

  int fd = open("/dev/mem", O_RDWR | O_SYNC);
  if(!fd) {
    perror("could not open /dev/mem: ");
    return -1;
  }

  uint32_t* ctrl_register = (uint32_t*) do_map(fd, SRC_M4RCR, SRC_M4RCR_SIZE); 
  if(!ctrl_register) {
    perror("could not map ctrl_register: ");
    return -1;
  }

  uint8_t* firmware_mapped = do_map(fd, TCM_ADDR, TCM_SIZE);
  if(!firmware_mapped) {
    perror("could not map TCM: ");
    return -1;
  }

  // stop core
  *ctrl_register |= STOP_SET_MASK;

  FILE* f = fopen(argv[1], "rb");
  if(!f) {
    perror("Could not open firmware: ");
    return -1;
  }

  fseek(f, 0, SEEK_END);
  int size = ftell(f);
  fseek(f, 0, SEEK_SET);

  if(size >= TCM_SIZE) {
    fprintf(stderr, "Firmware too big");
    return -1;
  }

  // copy it to our memory, because we have to make aligned access to TCM
  char firmware[TCM_SIZE];
  if(size != fread(firmware, 1, TCM_SIZE, f)) {
    perror("could not read firmware: ");
    return -1;
  }

  for(int i = 0; i < TCM_SIZE; i++) {
    if(i < size) {
      firmware_mapped[i] = firmware[i];
    } else {
      // zero the rest of TCM memory
      firmware_mapped[i] = 0;
    }
  }

  // finally start core
  *ctrl_register = (*ctrl_register & START_CLEAR_MASK) | START_SET_MASK;
  return 0;
}
