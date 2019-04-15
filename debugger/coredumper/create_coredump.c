#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <libelf.h>
#include <stdlib.h>

#define PAGE_MASK         (4096 - 1)
#define TCM_SIZE          (256 * 1024) 
#define TCM_A53_ADDR      0x7e0000
#define TCM_M4_ADDR       0x1ffe0000

uint8_t* do_map(int fd, uint64_t phys_addr, int size) {
  uint8_t* addr = (uint8_t*) mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, phys_addr & ~PAGE_MASK);
  if(addr == MAP_FAILED) {
    return 0;
  }

  return addr + (phys_addr & PAGE_MASK);
}

int main(int argc, char** argv) {
  int fd = open("/dev/mem", O_RDWR | O_SYNC);
  if(!fd) {
    perror("could not open /dev/mem: ");
    return -1;
  }

  // map TCM memory to our memory space
  uint8_t* firmware_mapped = do_map(fd, TCM_A53_ADDR, TCM_SIZE);
  if(!firmware_mapped) {
    perror("could not map TCM: ");
    return -1;
  }

  Elf           *e;
  Elf_Scn       *scn;
  Elf_Data      *data;
  Elf32_Ehdr    *ehdr;
  Elf32_Phdr    *phdr;

  int out = open("coredump", O_RDWR | O_CREAT | O_TRUNC);
  if(out < 0) {
    fprintf(stderr, "Could not create coredump file\n");
    exit(1);
  }

  if(elf_version(EV_CURRENT) == EV_NONE) {
    printf("elf_version: %s\n", elf_errmsg(-1));
    exit(1);
  }

  if((e = elf_begin(out, ELF_C_WRITE, NULL)) == NULL) {
    printf("elf_begin %s\n", elf_errmsg(-1));
    exit(1);
  }

  if ((ehdr = elf32_newehdr(e)) == NULL) {
    printf("elf32_newehdr %s\n",elf_errmsg(-1));
    exit(1);
  }

  // create ELF header
  ehdr->e_ident[EI_CLASS] = ELFCLASS32;
  ehdr->e_ident[EI_DATA] = ELFDATA2LSB;
  ehdr->e_ident[EI_VERSION] = EV_CURRENT;
  ehdr->e_ident[EI_OSABI] = ELFOSABI_NONE;
  ehdr->e_type = ET_CORE;
  ehdr->e_machine = EM_NONE;
  ehdr->e_version = EV_CURRENT;
  ehdr->e_entry = TCM_M4_ADDR;
  ehdr->e_phoff = sizeof(*ehdr);
  ehdr->e_ehsize = sizeof(*ehdr);
  ehdr->e_phentsize = sizeof(*phdr);
  ehdr->e_phnum = 1;


  // create program header for TCM memory
  if ((phdr = elf32_newphdr(e,1)) == NULL) {
    printf("elf32_newphdr %s\n", elf_errmsg(-1));
  }
  phdr->p_type = PT_LOAD;
  phdr->p_offset = ehdr->e_phoff + sizeof(*phdr) * ehdr->e_phnum;;
  phdr->p_vaddr = TCM_M4_ADDR;
  phdr->p_paddr = TCM_M4_ADDR;
  phdr->p_filesz = TCM_SIZE;
  phdr->p_memsz = TCM_SIZE;
  phdr->p_flags = PF_R | PF_W | PF_X;
  phdr->p_align = 0;

  // create section and place content of TCM memory
  if ((scn = elf_newscn(e)) == NULL) {
    printf("elf32_newscn %s\n",elf_errmsg(-1));
    exit(1);
  }

  if ((data = elf_newdata(scn)) == NULL) {
    printf("elf32_newdata %s\n",elf_errmsg(-1));
    exit(1);
  }

  data->d_align = 4;
  data->d_off = 0LL;
  data->d_buf = firmware_mapped;
  data->d_type = ELF_T_WORD; // code
  data->d_size = TCM_SIZE;
  data->d_version = EV_CURRENT;

  // save to the elf file
  if (elf_update(e, ELF_C_WRITE) < 0) {
    printf("elf32_update: %s\n",elf_errmsg(-1));
    exit(1);
  }
  elf_end(e);

  return 0;
}
