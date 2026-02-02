/* GNU ld script for ATSAM3X8E (Arduino Due)
 * FLASH: 512K @ 0x00080000
 * RAM  :  96K @ 0x20070000
 */

ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x00080000, LENGTH = 512K
  RAM   (rwx) : ORIGIN = 0x20070000, LENGTH = 96K
}

/* Top of stack */
_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
  /* Vector table */
  .isr_vector :
  {
    KEEP(*(.isr_vector))
  } > FLASH

  /* Code + rodata */
  .text :
  {
    *(.text*)
    *(.rodata*)
    *(.glue_7)
    *(.glue_7t)
    KEEP(*(.init))
    KEEP(*(.fini))
  } > FLASH

  .ARM.exidx :
  {
    *(.ARM.exidx*)
  } > FLASH

  /* Initialized data */
  .data : AT (ADDR(.text) + SIZEOF(.text) + SIZEOF(.ARM.exidx))
  {
    _sdata = .;
    *(.data*)
    _edata = .;
  } > RAM

  _sidata = LOADADDR(.data);

  /* Zero-init */
  .bss (NOLOAD) :
  {
    _sbss = .;
    *(.bss*)
    *(COMMON)
    _ebss = .;
  } > RAM

  /DISCARD/ :
  {
    *(.note*)
    *(.comment*)
  }
}
