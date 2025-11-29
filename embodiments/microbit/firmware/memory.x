/* Memory layout for BBC micro:bit V2 (nRF52833) - With SoftDevice S113 */

MEMORY
{
  /* micro:bit V2 (nRF52833) - SoftDevice S113 reserves:
   * FLASH: 0x00000000 - 0x00026000 (152KB) for SoftDevice
   * RAM:   0x20000000 - 0x20008000 (32KB) for SoftDevice
   */
  FLASH : ORIGIN = 0x00026000, LENGTH = 512K - 152K  /* Application starts after SoftDevice */
  RAM   : ORIGIN = 0x20008000, LENGTH = 128K - 32K   /* Application RAM after SoftDevice */
}

/* This is where the call stack will be allocated. */
/* The stack is of the full descending type. */
/* NOTE: You may need to adjust this based on your application's requirements */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);


