SECTION_DATA_PROLOGUE(.wlan_data, (NOLOAD),)
{
    __sram1_bss_start = .;
    *(.wlan_data)
    *(.wlan_data.*)
    *wlan.c.obj(.bss .bss.* .noinit .noinit.*)
    *wifi-sdio.c.obj(.bss .bss.* .noinit .noinit.*)
    *wifi-mem.c.obj(.bss .bss.* .noinit .noinit.*)
    *mempool.c.obj(.noinit.kheap_buf__system_heap .noinit.*)
    *(.sram1_log_buf)
    *(.sram1_log_buf.*)
    __sram1_bss_end = .;
} GROUP_NOLOAD_LINK_IN(SRAM1, SRAM1)
