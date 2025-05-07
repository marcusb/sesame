#include "fsl_debug_console.h"
#include "wm_os.h"

#ifdef USE_BACKTRACE
#include "backtrace.h"

// size of the Cortex-M4 exception stack frame
#define FAULT_FRAME_LEN_FP (26 * 4)
#define FAULT_FRAME_LEN_NOFP (8 * 4)

#define BACKTRACE_DEPTH 20

static backtrace_t backtrace[BACKTRACE_DEPTH];
#endif

struct fault_frame {
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t IP;
    uint32_t LR;
    uint32_t PC;
    uint32_t PSR;
};

struct callee_registers {
    uint32_t r8;
    uint32_t r9;
    uint32_t r10;
    uint32_t r11;

    uint32_t r4;
    uint32_t r5;
    uint32_t r6;
    uint32_t r7;
};

__attribute__((__used__)) void HardFault_IRQHandler_C(
    const struct fault_frame *fault_frame,
    const struct callee_registers *callee_registers,
    uint32_t exception_return) {
    PRINTF("\r\n***Hard fault***\r\n");
#ifdef USE_BACKTRACE
    // Initialize the stack frame of the fault site. The original stack frame
    // is just above the exception stack frame.
    int fault_frame_len =
        exception_return & 0x10 ? FAULT_FRAME_LEN_FP : FAULT_FRAME_LEN_NOFP;
    backtrace_frame_t frame;
    frame.sp = (uint32_t)fault_frame + fault_frame_len;
    frame.fp = frame.sp;
    frame.lr = fault_frame->LR;
    frame.pc = fault_frame->PC;

    int count = _backtrace_unwind(backtrace, BACKTRACE_DEPTH, &frame);
    for (int i = 0; i < count; ++i) {
        PRINTF("%s@%p\r\n", backtrace[i].name, backtrace[i].address);
    }
#endif

    PRINTF(
        "\r\nr0  = 0x%08x\r\nr1  = 0x%08x\r\nr2  = 0x%08x\r\n"
        "r3  = 0x%08x\r\nr12 = 0x%08x\r\nlr = 0x%08x\r\npc = 0x%08x"
        "\r\npsr = 0x%08x\r\n",
        fault_frame->r0, fault_frame->r1, fault_frame->r2, fault_frame->r3,
        fault_frame->IP, fault_frame->LR, fault_frame->PC, fault_frame->PSR);

    PRINTF("Task name: %s\r\n", pcTaskGetName(NULL));

    uint32_t hfsr = SCB->HFSR;
    PRINTF("HFSR: 0x%08x\r\n", hfsr);
    if (hfsr & SCB_HFSR_FORCED_Msk) {
        hfsr = SCB->CFSR;
        PRINTF("CFSR: 0x%08x\r\n", hfsr);
        PRINTF("MMSR: 0x%02x\r\n", hfsr & 0xff);

        // if MMARVALID bit is set, read the address that caused the fault
        if (hfsr & SCB_CFSR_MEMFAULTSR_Msk) {
            PRINTF("MMAR : 0x%08x\r\n", SCB->MMFAR);
        }
        PRINTF("BFSR: 0x%02x\r\n", (hfsr >> 8) & 0xff);

        // if BFARVALID bit is set, read the address that caused the fault
        if (hfsr & SCB_CFSR_BUSFAULTSR_Msk) {
            PRINTF("BFAR: 0x%08x\r\n", SCB->BFAR);
        }
        PRINTF("UFSR: 0x%04x\r\n", hfsr >> 16);
    }
    for (;;);
}
