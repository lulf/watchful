MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  FLASH                             : ORIGIN = 0x00026000, LENGTH = 16K
  RELOADER                          : ORIGIN = 0x00039000, LENGTH = 284K

  RAM                         (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}

__reloader_start = ORIGIN(RELOADER);
