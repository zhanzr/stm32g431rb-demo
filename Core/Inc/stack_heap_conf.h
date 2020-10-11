#ifndef __STACK_HEAP_CONF_H__

#define HEAP_BASE 0x20100000  /* Example memory addresses */
#define STACK_BASE 0x20200000
#define HEAP_SIZE ((STACK_BASE-HEAP_BASE)/2)
#define STACK_SIZE ((STACK_BASE-HEAP_BASE)/2)

#endif // __STACK_HEAP_CONF_H__
