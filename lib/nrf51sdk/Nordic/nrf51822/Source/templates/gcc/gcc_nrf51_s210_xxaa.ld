/* Linker script to configure memory regions. */
SEARCH_DIR(.)
GROUP(-lgcc -lc -lcs3 -lcs3unhosted)

MEMORY
{
  rom (rx) : ORIGIN = 0x0000A000, LENGTH = 216K
  ram (rwx) : ORIGIN = 0x20000800, LENGTH = 14K
}
INCLUDE "gcc_nrf51_common.ld"
