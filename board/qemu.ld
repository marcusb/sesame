/* Linker file for QEMU (generic Cortex-M) */

ENTRY(_start)

STACK_SIZE = 0x0800;
FAULT_STACK_SIZE = 0x0400;

MEMORY
{
  m_interrupts          (RX)  : ORIGIN = 0x00000000, LENGTH = 0x00000400
  m_text                (RX)  : ORIGIN = 0x00000400, LENGTH = 0x00200000
  m_data                (RW)  : ORIGIN = 0x20000000, LENGTH = 0x00100000
}

SECTIONS
{
  .interrupts :
  {
    . = ALIGN(4);
    __VECTOR_TABLE = .;
    __Vectors = .;
    KEEP(*(.isr_vector))
    KEEP(*(.interrupts))
    . = ALIGN(4);
  } > m_interrupts

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.rodata)
    *(.rodata*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
  } > m_text

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > m_text
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } > m_text

  .ctors : {
    __CTOR_LIST__ = .;
    KEEP (*crtbegin.o(.ctors))
    KEEP (*crtbegin?.o(.ctors))
    KEEP (*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
    __CTOR_END__ = .;
  } > m_text

  .dtors : {
    __DTOR_LIST__ = .;
    KEEP (*crtbegin.o(.dtors))
    KEEP (*crtbegin?.o(.dtors))
    KEEP (*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
    __DTOR_END__ = .;
  } > m_text

  .preinit_array : {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } > m_text

  .init_array : {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } > m_text

  .fini_array : {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } > m_text

  .data :
  {
    . = ALIGN(4);
    __data_start__ = .;
    *(.data)
    *(.data*)
    . = ALIGN(8);
    __tdata_start__ = .;
    *(.tdata .tdata*)
    . = ALIGN(8);
    __data_end__ = .;
  } > m_data AT> m_text

  __etext = LOADADDR(.data);

  .bss :
  {
    . = ALIGN(8);
    __tbss_start__ = .;
    *(.tbss .tbss*)
    . = ALIGN(4);
    __bss_start__ = .;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
  } > m_data

  .nvram (NOLOAD) :
  {
    . = ALIGN(4);
    *(.nvram*)
    *(.nvram_uninit*)
    . = ALIGN(4);
  } > m_data

  __end__ = ALIGN(__bss_end__, 8);
  PROVIDE(end = __end__);
  __HeapBase = __end__;
  __fault_stack = ORIGIN(m_data) + LENGTH(m_data);
  __StackTop = __fault_stack - FAULT_STACK_SIZE;
  __StackLimit = __StackTop - STACK_SIZE;
  __HeapLimit = __StackLimit;

  .stack __StackLimit :
  {
    . = ALIGN(8);
    . += STACK_SIZE;
    __StackTop_alias = .;
  } > m_data

  /* picolibc and SDK symbols */
  PROVIDE(__data_start = __data_start__);
  PROVIDE(__data_size = __data_end__ - __data_start__);
  PROVIDE(__data_source = __etext);
  PROVIDE(__bss_start = __bss_start__);
  PROVIDE(__bss_size = __bss_end__ - __bss_start__);
  PROVIDE(__tls_base = __tdata_start__);
  PROVIDE(__arm32_tls_tcb_offset = 8);
  
  /* SDK specific symbols */
  __data_end__ = __data_end__;
  __bss_end__ = __bss_end__;
  __DATA_ROM = __etext;
  __DATA_RAM = __data_start__;
  __DATA_END = __data_end__;
  __START_BSS = __bss_start__;
  __END_BSS = __bss_end__;
  
  __HeapBase_sram0 = 0;
  __HeapLimit_sram0 = 0;
}
