MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  MBR                               : ORIGIN = 0x00000000, LENGTH = 4K
  SOFTDEVICE                        : ORIGIN = 0x00001000, LENGTH = 152K
  FLASH                             : ORIGIN = 0x00026000, LENGTH = 356K
  RAM                               : ORIGIN = 0x2000BAF0, LENGTH = 17680
}
