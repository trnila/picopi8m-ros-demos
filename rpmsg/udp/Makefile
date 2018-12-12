all: module multicast6_recv multicast4_recv broadcast_recv

module:
	make -C kernel_module

run: all
	rmmod rpmsg_udp || true
	insmod kernel_module/rpmsg_udp.ko

	cd m4 && m4run
