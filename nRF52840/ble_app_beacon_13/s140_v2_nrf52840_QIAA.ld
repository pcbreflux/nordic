/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x21000, LENGTH = 0xdf000
  RAM (rwx) :  ORIGIN = 0x20001930, LENGTH = 0x3e6d0
}

SECTIONS
{
  . = ALIGN(4);
  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
} INSERT AFTER .data;

INCLUDE "nrf5x_common.ld"
