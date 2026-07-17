#include "fsl_debug_console.h"
#include "fsl_common.h"
#include <zephyr/sys/printk.h>
#include <stdarg.h>

int DbgConsole_Printf(const char *formatString, ...)
{
    va_list args;
    va_start(args, formatString);
    vprintk(formatString, args);
    va_end(args);
    return 0;
}

int DbgConsole_BlockingPrintf(const char *formatString, ...)
{
    va_list args;
    va_start(args, formatString);
    vprintk(formatString, args);
    va_end(args);
    return 0;
}

int DbgConsole_Putchar(int ch)
{
    printk("%c", ch);
    return ch;
}

int DbgConsole_Scanf(char *formatString, ...)
{
    return 0;
}

int DbgConsole_Getchar(void)
{
    return -1;
}

status_t DbgConsole_Flush(void)
{
    return kStatus_Success;
}

status_t DbgConsole_Init(uint8_t instance, uint32_t baudRate, serial_port_type_t device, uint32_t clkSrcFreq)
{
    return kStatus_Success;
}

status_t DbgConsole_Deinit(void)
{
    return kStatus_Success;
}

status_t DbgConsole_EnterLowpower(void)
{
    return kStatus_Success;
}

status_t DbgConsole_ExitLowpower(void)
{
    return kStatus_Success;
}
