/* Linker script for Seeed Studio XIAO STM32C5 (STM32C5A3CG)
 * 1024 KB Flash, 256 KB SRAM
 *
 * STM32C5 boots from internal Flash at 0x08000000.
 * In production or UF2 environments, offsets can be adjusted if using a bootloader.
 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}

/* This is the stack configuration */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
