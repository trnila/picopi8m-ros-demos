#include <stdint.h>

class M4Core {
public:
	M4Core(bool manage_rpmsg = true);
	~M4Core();
	void start();
	void boot_firmware(const char *firmware_path);
	void stop();
private:
	bool manage_rpmsg;
	uint32_t *ctrl_register;
	uint8_t *firmware_mapped;
};
