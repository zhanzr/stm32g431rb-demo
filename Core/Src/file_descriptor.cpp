#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>

#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "dma.h"
#include "fmac.h"
#include "rng.h"
#include "usart.h"
#include "gpio.h"

#include "stm32g4xx_nucleo.h"

namespace std {
  int fputc(int ch, FILE *stream) {
		return (ITM_SendChar(ch));
  }
}
