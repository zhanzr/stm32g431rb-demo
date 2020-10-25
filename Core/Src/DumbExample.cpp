#include "DumbExample.h"

auto AverageThreeBytes(int8_t a, int8_t b, int8_t c) -> int8_t {
	return static_cast<int8_t>(((int16_t)a + (int16_t)b + (int16_t)c) / 3);
}
