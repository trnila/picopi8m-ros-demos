ifneq ($(KERNELRELEASE),)
obj-m  := ping_bench.o

else
KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD
endif