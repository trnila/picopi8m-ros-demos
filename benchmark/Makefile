TARGETS=bench_tty bench_m4char load_raw

all: $(TARGETS)
	make -C kernel_module
	(cd m4; m4build)
	(cd benchmark_ros_m4 && mkdir -p build && cd build && cmake .. && make)

clean:
	rm -f $(TARGETS)
	rm -rf m4/build
	make -C kernel_module clean || true
	rm -rf benchmark_ros_m4/build benchmark_ros_m4/m4/build
