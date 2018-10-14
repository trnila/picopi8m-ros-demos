#include <stdexcept>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "m4core.h"

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

M4Core::M4Core() {
  int fd = open("/dev/mem", O_RDWR | O_SYNC);
  if(!fd) {
    throw std::runtime_error("Could not open /dev/mem");
  }

  ctrl_register = (uint32_t*) do_map(fd, SRC_M4RCR, SRC_M4RCR_SIZE); 
  if(!ctrl_register) {
    throw std::runtime_error("Could not map ctrl_register");
  }

  firmware_mapped = do_map(fd, TCM_ADDR, TCM_SIZE);
  if(!firmware_mapped) {
    throw std::runtime_error("Could not map firmware");
  }
}

M4Core::~M4Core() {

}

void M4Core::start() {
  system("rmmod imx_rpmsg");
  system("modprobe imx_rpmsg");
  *ctrl_register = (*ctrl_register & START_CLEAR_MASK) | START_SET_MASK;
}

void M4Core::stop() {
  *ctrl_register |= STOP_SET_MASK;
  system("rmmod imx_rpmsg");
}

void M4Core::boot_firmware(const char *firmware_path) {
  stop();

  FILE* f = fopen(firmware_path, "rb");
  if(!f) {
    throw std::runtime_error("Could not load firmware");
  }

  fseek(f, 0, SEEK_END);
  int size = ftell(f);
  fseek(f, 0, SEEK_SET);

  if(size >= TCM_SIZE) {
    fclose(f);
    throw std::runtime_error("Firmware wont fit in TCM");
  }

  char firmware[TCM_SIZE];
  if(size != fread(firmware, 1, TCM_SIZE, f)) {
    fclose(f);
    throw std::runtime_error("Firmware could not be read");
  }
  fclose(f);

  for(int i = 0; i < TCM_SIZE; i++) {
    if(i < size) {
      firmware_mapped[i] = firmware[i];
    } else {
      // zero the rest of TCM memory
      firmware_mapped[i] = 0;
    }
  }

  start();
}
