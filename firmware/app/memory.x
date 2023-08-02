MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  MBR                               : ORIGIN = 0x00000000, LENGTH = 4K
  SOFTDEVICE                        : ORIGIN = 0x00001000, LENGTH = 152K
  FLASH                             : ORIGIN = 0x00026000, LENGTH = 160K
  BOOTLOADER                        : ORIGIN = 0x00077000, LENGTH = 24K
  STORAGE                           : ORIGIN = 0x0007E000, LENGTH = 4k
  RAM                               : ORIGIN = 0x2000BAF0, LENGTH = 17680

  /*DFU                               : ORIGIN = 0x0004E000, LENGTH = 164K
  BOOTLOADER_STATE                  : ORIGIN = 0x0007D000, LENGTH = 4K*/
  DFU                               : ORIGIN = 0x00000000, LENGTH = 164K
  BOOTLOADER_STATE                  : ORIGIN = 0x0007D000, LENGTH = 4K
}

__bootloader_state_start = ORIGIN(BOOTLOADER_STATE);
__bootloader_state_end = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE);

__bootloader_dfu_start = ORIGIN(DFU);
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU);
