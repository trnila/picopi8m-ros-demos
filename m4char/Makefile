LDFLAGS=-pthread

all: module m4 spam count_recv

.PHONY: module m4 run_m4 clean

module:
	make -C kernel_module

m4:
	cd m4 && m4build

run:
	rmmod rpmsg_m4char.ko || true
	insmod kernel_module/rpmsg_m4char.ko
	m4ctl start ./m4/build/debug/strupper

clean:
	rm -f main
	make -C kernel_module clean
	rm -rf ./m4/build

