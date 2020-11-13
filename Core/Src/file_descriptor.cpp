#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cassert>

#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "dma.h"
#include "fmac.h"
#include "rng.h"
#include "usart.h"
#include "gpio.h"

#include "stm32g4xx_nucleo.h"

#include <rt_sys.h>
 
#include "RTE_Components.h"
 
#ifdef RTE_Compiler_IO_STDOUT_EVR
#include "EventRecorder.h"
#endif
 
#ifdef RTE_Compiler_IO_File_FS
#include "rl_fs_lib.h"
#endif

#if (defined(RTE_Compiler_IO_STDIN)  || \
     defined(RTE_Compiler_IO_STDOUT) || \
     defined(RTE_Compiler_IO_STDERR) || \
     defined(RTE_Compiler_IO_File))
#define RETARGET_SYS
 
/* IO device file handles. */
#define FH_STDIN    0x8001
#define FH_STDOUT   0x8002
#define FH_STDERR   0x8003  
#endif

extern "C" {
extern int _sys_write (FILEHANDLE fh, const uint8_t *buf, uint32_t len, int mode);
}

namespace std {
  int fputc(int ch, FILE* stream) {
		return _sys_write(FH_STDOUT, reinterpret_cast<const uint8_t*>(&ch), 1, 0);
  }
}
