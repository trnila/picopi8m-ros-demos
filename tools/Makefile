TARGETS=m4ctl dump_rpmsg

all: $(TARGETS)

m4ctl: ../m4ctrl/src/m4ctl.cpp ../m4ctrl/src/m4core.cpp
	$(CXX) $^ -o $@ -I ../m4ctrl/src/


clean:
	rm -f $(TARGETS)
