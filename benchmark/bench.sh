#!/bin/bash
set -e

unload_modules() {
	rmmod ping_bench || true
	rmmod imx_rpmsg_tty || true
	rmmod rpmsg_char || true
}

rm -r ./measurements
mkdir measurements

make

# load m4 application
(cd m4 && m4run)

# benchmark kernel
unload_modules
insmod measurements/ping_bench.ko
sleep 1;
cat /proc/ping_benchmark > measurements/kernel

pings_count=$(cat measurements/kernel | wc -l)

# benchmark imx_rpmsg_tty
unload_modules
modprobe imx_rpmsg_tty
(cd m4 && m4run)
sleep 1 # wait for /dev/ttyRPMSG
./tty "$pings_count" > measurements/tty

# benchmark rpmsg_char
unload_modules
modprobe rpmsg_char
(cd m4 && m4run)
sleep 1
./rpmsg_char "$pings_count" > measurements/rpmsg_char


# merge to csv
columns="kernel rpmsg_char tty"
(cd measurements; echo ${columns// /,}; paste -d, $columns) > measurements/all.csv
