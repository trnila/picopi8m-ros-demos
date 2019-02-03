BINS=multicast6_recv multicast4_recv broadcast_recv

all: module $(BINS)

module:
	make -C kernel_module

run: all
	rmmod rpmsg_udp || true
	insmod kernel_module/rpmsg_udp.ko

	cd m4 && m4run

clean:
	rm -f $(BINS)
	make -C m4/build/debug clean || true
	make -C kernel_module clean || true
