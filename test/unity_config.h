#pragma once
/* Defined in test_runner.c — routes through the SDK debug console. */
extern void unity_putchar(char c);
#define UNITY_OUTPUT_CHAR(a) unity_putchar((char)(a))
