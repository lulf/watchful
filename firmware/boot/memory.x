MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  MBR                               : ORIGIN = 0x00000000, LENGTH = 4K
  SOFTDEVICE                        : ORIGIN = 0x00001000, LENGTH = 148K
  ACTIVE                            : ORIGIN = 0x00026000, LENGTH = 324K
  FLASH                             : ORIGIN = 0x00077000, LENGTH = 32K
  BOOTLOADER_STATE                  : ORIGIN = 0x0007F000, LENGTH = 4K

  DFU                               : ORIGIN = 0x00000000, LENGTH = 328K

  RAM                         (rwx) : ORIGIN = 0x20000008, LENGTH = 0xfff8
  /* NOTE: Disable when building reloader */
  uicr_bootloader_start_address (r) : ORIGIN = 0x10001014, LENGTH = 0x4
}

__bootloader_state_start = ORIGIN(BOOTLOADER_STATE);
__bootloader_state_end = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE);

__bootloader_active_start = ORIGIN(ACTIVE);
__bootloader_active_end = ORIGIN(ACTIVE) + LENGTH(ACTIVE);

__bootloader_dfu_start = ORIGIN(DFU);
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU);

__bootloader_start = ORIGIN(FLASH);

/* NOTE: Disable when building reloader */
SECTIONS
{
  .uicr_bootloader_start_address :
  {
    LONG(__bootloader_start)
  } > uicr_bootloader_start_address
}
