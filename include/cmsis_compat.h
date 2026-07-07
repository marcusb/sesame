/* CMSIS v2 -> v5 compatibility for SDK driver code */
#ifndef CMSIS_COMPAT_H
#define CMSIS_COMPAT_H

/* NVIC->IPR[] (v2) -> NVIC->ISPR[]/ICPR[] (v5) */
/* SCB->SHP (v2) -> SCB->SHP (v5, same name) */
/* SCB->SHPR (v2) -> SCB->SHP (v5) */

/* Workaround: patch the structs after CMSIS includes */
#ifdef __CMSIS_COMPILER_H
/* Redirect IPR access to ISPR/ICPR */
#define IPR ISPR
#endif

#endif /* CMSIS_COMPAT_H */
