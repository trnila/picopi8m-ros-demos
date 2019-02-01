#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <stdint.h>

int fd;

#define PAGE_SIZE 4095
#define START 0xb8000000 
#define LEN 4096 * 100

#define COUNT 256

struct vring_desc {
  uint64_t addr;
  uint32_t len;
  uint16_t flags;
  uint16_t next;
};

struct vring_avail {
  uint16_t flags;
  uint16_t idx;
  uint16_t ring[COUNT];
};


struct vring_used_elem
{
  uint32_t id;
  uint32_t len;
};
struct vring_used
{
  uint16_t flags;
  uint16_t idx;
  struct vring_used_elem ring[COUNT];
};

struct ring {
  struct vring_desc desc[COUNT];
  struct vring_avail avail;

  struct vring_used tst;
};

void dump(uint64_t start, int dump_content) {
  struct ring* shm = (struct ring*) mmap(0, LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, start);

  //char *base = mmap(0, COUNT * shm->desc[0].len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, shm->desc[0].addr);
  for(int i = 0; i < COUNT; i++) {
    printf("addr = 0x%x len = %d flags = %d next = %d\n", shm->desc[i].addr, shm->desc[i].len, shm->desc[i].flags, shm->desc[i].next); 

    char a[128], b[128];
    char *ta = a, *tb = b;
    a[0] = b[0] = 0;
    //char *ptr = base + (shm->desc[i].addr - shm->desc[0].addr);
    if(shm->desc[i].addr && dump_content) {
      char *ptr = mmap(0, shm->desc[i].len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, shm->desc[i].addr & ~PAGE_SIZE) + ((uint64_t) shm->desc[i].addr & PAGE_SIZE);
      if(ptr != MAP_FAILED) {
        for(int j = 0; j < shm->desc[i].len; j++) {
          ta += sprintf(ta, "%02X ", ptr[j]);
          tb += sprintf(tb, "%c", isprint(ptr[j]) ? ptr[j] : '.');
          if((j + 1) % 16 == 0) {
            printf("%s    %s\n", a, b);
            a[0] = b[0] = 0;
            ta = a;
            tb = b;
            if((j + 1) / 16 == 1) {
              uint32_t *fields = (uint32_t *) ptr;
              printf(
                  "   src=0x%x dst=0x%x res=0x%x flags=0x%x\n",
                  fields[0], fields[1], fields[2], fields[3]
                  );
            }
          }
        }
      } else {
        printf("Failed to dump %X\n", shm->desc[i].addr);
      }
      munmap(ptr, shm->desc[i].len);
      printf("\n");
    }
  }

  printf("AVAILABLE\n");
  printf("\nflags = %d idx = %d\n", shm->avail.flags, shm->avail.idx);
  for(int i = 0; i <= COUNT; i++) {
    printf("  [%d] = %d\n", i, shm->avail.ring[i]);
  }
  printf("\n");

  printf("USED\n");
  struct vring_used *used = (char*) shm + 4096 * 2;
  printf("\nflags = %d idx = %d\n", used->flags, used->idx);
  for(int i = 0; i <= COUNT; i++) {
    printf("  [%d] = {idx = %d, len = %d}\n", i, used->ring[i].id, used->ring[i].len);
  }
  printf("\n");
}


int main(int argc, char** argv) {
  int dump_content = 1;
  uint32_t start = START;

  for(int i = 1; i < argc; i++) {
    if(strcmp(argv[i], "--no-content") == 0) {
      dump_content = 0;
    } else if(strcmp(argv[i], "--addr") == 0) {
      if(i + 1 >= argc) {
        fprintf(stderr, "missing address for --addr\n");
        exit(1);
      }
    
      start = strtol(argv[++i], NULL, 16);
    }  else {
      fprintf(stderr, "Unknown parameter: %s\n", argv[i]);
      exit(1);
    }
  }


  fd = open("/dev/mem", O_RDWR | O_SYNC);
  if (fd < 0) {
    return 1;
  }

  dump(start, dump_content);

  return 0;
}
