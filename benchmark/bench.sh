#!/bin/sh
make -C kernel_module
rmmod ping_bench.ko
insmod ./kernel_module/ping_bench.ko

dmesg -C
cd m4
m4run ping_zerocopy
dmesg  | grep "ping response" | grep -oP "[0-9]+ ns"
