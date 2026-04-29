/* QEMU Vector Table for Cortex-M3 */
#include <stdint.h>
#include <stdio.h>

extern void _start(void);
extern void __StackTop(void);
extern void SVC_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);
extern void HardFault_Handler(void);
extern void EthernetISR(void);

/* Default handler for unexpected interrupts */
void Default_Handler(void) {
    uint32_t icsr = *(volatile uint32_t*)0xE000ED04;
    printf("DEFAULT HANDLER for IRQ: %lu\n", icsr & 0xFF);
    while (1);
}
void NMI_Handler(void) {
    printf("NMI\n");
    while (1);
}
void MemManage_Handler(void) {
    printf("MEM MANAGE\n");
    while (1);
}
void BusFault_Handler(void) {
    printf("BUS FAULT\n");
    while (1);
}
void UsageFault_Handler(void) {
    printf("USAGE FAULT\n");
    while (1);
}

extern void EthernetISR(void);

__attribute__((section(".isr_vector"))) void (*const __isr_vector[128])(
    void) = {
    __StackTop,
    _start,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0,
    0,
    0,
    0, /* Reserved */
    SVC_Handler,
    Default_Handler, /* Debug Monitor */
    0,               /* Reserved */
    PendSV_Handler,
    SysTick_Handler,
    /* IRQs */
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler, /* IRQ 12 */
    EthernetISR,     /* IRQ 13 (Index 29) */
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
};
