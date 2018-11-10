ifneq ($(KERNELRELEASE),)
obj-m  := ping_bench.o mu_bench.o

mu_bench-y := mu.o common.o
ping_bench-y := ping.o common.o

else
KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean
endif
