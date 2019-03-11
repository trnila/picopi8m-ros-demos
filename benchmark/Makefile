TARGETS=bench_tty bench_m4char load_raw

all: $(TARGETS)
	make -C kernel_module
	(cd m4; m4build)

clean:
	rm -f $(TARGETS)
	rm -rf m4/build
	make -C kernel_module clean || true
