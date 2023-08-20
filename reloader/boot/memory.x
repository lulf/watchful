MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  FLASH                             : ORIGIN = 0x00008020, LENGTH = 32K
/*  FLASH                             : ORIGIN = 0x00000000, LENGTH = 32K*/
  RELOADER                          : ORIGIN = 0x00026000, LENGTH = 343K

  RAM                         (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}

__reloader_start = ORIGIN(RELOADER);
