/* Linker script to configure memory regions. */
SEARCH_DIR(.)
GROUP(-lgcc -lc -lcs3 -lcs3unhosted)

MEMORY
{
  rom (rx) : ORIGIN = 0x14000, LENGTH = 0x2C000 /* 80 kB is taken by S110, 176 kB available for application. */
  ram (rwx) : ORIGIN = 0x20002000, LENGTH = 0x2000 /* 8 kB, 8 kB is taken by S110. */
}
INCLUDE "gcc_nrf51_common.ld"
