#pragma once
#include <stdint.h>

struct Measurement {
  float position; // measured/observed position with artifical error
  float velocity;
  float acceleration;

  float real_position; // position without error, just for visualization
};

enum CommandType {
	CMD_PAUSE,
  CMD_SIGMA, // standard deviation of artifical error
};

struct Command {
	enum CommandType type;
	union {
		uint8_t u8;
		float f32;
	};
};

