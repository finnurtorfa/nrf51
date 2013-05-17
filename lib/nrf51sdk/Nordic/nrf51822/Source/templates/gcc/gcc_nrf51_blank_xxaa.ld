/* Linker script to configure memory regions. */
SEARCH_DIR(.)
GROUP(-lgcc -lc -lcs3 -lcs3unhosted)

MEMORY
{
  rom (rx) : ORIGIN = 0x00000000, LENGTH = 256K
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 16K
}

INCLUDE "gcc_nrf51_common.ld"
