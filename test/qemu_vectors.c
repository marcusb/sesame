/* QEMU Vector Table for Cortex-M3 */
#include <stdint.h>

extern void _start(void);
extern void __StackTop(void);
extern void SVC_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);

/* Default handler for unexpected interrupts */
void Default_Handler(void) { while (1); }

__attribute__((section(".isr_vector"))) void (*const __isr_vector[])(void) = {
    __StackTop,
    _start,
    Default_Handler, /* NMI */
    Default_Handler, /* HardFault */
    Default_Handler, /* MemManage */
    Default_Handler, /* BusFault */
    Default_Handler, /* UsageFault */
    0,
    0,
    0,
    0, /* Reserved */
    SVC_Handler,
    Default_Handler, /* Debug Monitor */
    0,               /* Reserved */
    PendSV_Handler,
    SysTick_Handler,
};
