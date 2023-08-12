MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  MBR                               : ORIGIN = 0x00000000, LENGTH = 4K
  SOFTDEVICE                        : ORIGIN = 0x00001000, LENGTH = 152K
  FLASH                             : ORIGIN = 0x00026000, LENGTH = 324K
  BOOTLOADER                        : ORIGIN = 0x00077000, LENGTH = 32K

  DFU                               : ORIGIN = 0x00000000, LENGTH = 328K
  BOOTLOADER_STATE                  : ORIGIN = 0x003FF000, LENGTH = 4K

  RAM                               : ORIGIN = 0x2000BAF0, LENGTH = 17680
}

__bootloader_state_start = ORIGIN(BOOTLOADER_STATE);
__bootloader_state_end = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE);

__bootloader_dfu_start = ORIGIN(DFU);
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU);
