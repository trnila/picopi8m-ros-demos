#!/bin/bash
set -e

unload_modules() {
	rmmod ping_bench || true
	rmmod mu_bench || true
	rmmod imx_rpmsg_tty || true
	rmmod rpmsg_char || true
	rmmod imx_rpmsg || true
}

rm -rf ./measurements
mkdir measurements

make

# benchmark kernel
unload_modules
insmod kernel_module/ping_bench.ko
(cd m4 && m4run ping_zerocopy)
sleep 1;
cp /proc/ping_benchmark measurements/kernel

pings_count=$(cat measurements/kernel | wc -l)

# benchmark imx_rpmsg_tty
unload_modules
modprobe imx_rpmsg_tty
(cd m4 && m4run ping_zerocopy)
sleep 1 # wait for /dev/ttyRPMSG
./tty "$pings_count" > measurements/tty

# benchmark rpmsg_char
unload_modules
modprobe rpmsg_char
(cd m4 && m4run ping_zerocopy)
sleep 1
./rpmsg_char "$pings_count" > measurements/rpmsg_char

# benchmark mu
unload_modules
insmod ./kernel_module/mu_bench.ko
(cd m4 && m4run mu --no-rpmsg)
sleep 1
cp /proc/mu_benchmark measurements/mu

# unload all modules
unload_modules

# merge to csv
columns="kernel mu rpmsg_char tty"
(cd measurements; echo ${columns// /,}; paste -d, $columns) > measurements/all.csv

echo OK
