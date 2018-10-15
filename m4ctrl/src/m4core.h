#include <stdint.h>

class M4Core {
public:
	M4Core();
	~M4Core();
	void start();
	void boot_firmware(const char *firmware_path);
	void stop();
private:
	uint32_t *ctrl_register;
	uint8_t *firmware_mapped;
};
