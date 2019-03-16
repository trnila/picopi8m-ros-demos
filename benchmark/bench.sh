#!/bin/bash
set -e

TOTAL=10000

#make
rm -rf ./measurements
mkdir measurements

## Benchmark MU
# unload imx8m-cm4 remoteproc device
echo "Benchmarking MU..."
echo imx8m-cm4 > /sys/bus/platform/drivers/imx-rproc/unbind || true
rmmod mu_bench || true
insmod kernel_module/mu_bench.ko
./load_raw ./m4/build/debug/mu.bin
cat /proc/mu_bench > measurements/mu
rmmod mu_bench
echo imx8m-cm4 > /sys/bus/platform/drivers/imx-rproc/bind

## Benchmark rpmsg_m4char
echo "Benchmarking rpmsg_m4char..."
m4ctl start ./m4/build/debug/ping_m4char
./bench_m4char "$TOTAL" > measurements/m4char

## Benchmark rpmsg_tty
echo "Benchmarking tty..."
modprobe imx_rpmsg_tty || true
m4ctl start ./m4/build/debug/ping_tty
./bench_tty "$TOTAL" > measurements/tty

## Benchmark rostopic
echo "Benchmarking rostopic..."
rosrun rosserial_rpmsg serial_node.py /dev/m4char &
NODE_PID=$!
sleep 6
m4ctl start ./benchmark_ros_m4/m4/build/debug/bench_rostopic
./benchmark_ros_m4/build/devel/lib/benchmark_ros_m4/benchmark $TOTAL > measurements/rostopic
kill $NODE_PID

# merge to csv
columns="mu m4char tty rostopic"
(cd measurements; echo ${columns// /,}; paste -d, $columns) > measurements/all.csv

echo OK
