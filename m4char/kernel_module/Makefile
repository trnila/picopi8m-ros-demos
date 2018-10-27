ifneq ($(KERNELRELEASE),)
obj-m  := rpmsg_m4char.o

else
KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean


endif
