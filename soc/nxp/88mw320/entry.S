.section	".text.reset.entry", "ax", %progbits
	.global	zephyr_reset_handler
	.thumb
	.thumb_func
	.type	zephyr_reset_handler, %function

zephyr_reset_handler:
#ifdef CONFIG_BOARD_MARVELL_MW302_RAM
	ldr	r0, =__start
	bx	r0
#else
	/* Flash (XIP) build: minimal trampoline to __start.
	   LDR literal is 4 bytes after BX. Total: 8 bytes. */
	ldr	r0, 2f
	bx	r0
	.balign 4
2:	.word	__start
#endif

	.size	zephyr_reset_handler, . - zephyr_reset_handler

#ifndef CONFIG_BOARD_MARVELL_MW302_RAM
	.section	".ramfunc.entry", "ax", %progbits
	.global	ram_trampoline
	.thumb
	.thumb_func
ram_trampoline:
	ldr	r0, =__start
	bx	r0
#endif
