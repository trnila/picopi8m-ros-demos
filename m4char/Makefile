LDFLAGS=-pthread

all: module m4 run_m4 main count_recv

.PHONY: module m4 run_m4 clean

module:
	rmmod rpmsg_m4char.ko || true
	make -C kernel_module
	insmod kernel_module/rpmsg_m4char.ko

m4:
	cd m4 && m4build

run_m4:
	m4ctl load ./m4/build/debug/strupper.bin 

clean:
	rm -f main
	make -C kernel_module clean
	rm -rf ./m4/build

