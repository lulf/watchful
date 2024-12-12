MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  MBR                               : ORIGIN = 0x00000000, LENGTH = 4K
  SOFTDEVICE                        : ORIGIN = 0x00001000, LENGTH = 148K
  FLASH                             : ORIGIN = 0x00039000, LENGTH = 284K
  BOOTLOADER                        : ORIGIN = 0x00077000, LENGTH = 32K
  BOOTLOADER_STATE                  : ORIGIN = 0x0007F000, LENGTH = 4K

  RAM                               : ORIGIN = 0x2000BAF0, LENGTH = 17680
}
