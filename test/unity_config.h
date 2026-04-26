#pragma once
/* Defined in test_runner.c — routes through semihosting. */
extern void unity_putchar(char c);
extern void unity_flush(void);
#define UNITY_OUTPUT_CHAR(a) unity_putchar((char)(a))
#define UNITY_OUTPUT_FLUSH() unity_flush()
