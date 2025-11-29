/* Memory layout for BBC micro:bit V2 (nRF52833) - Bare Metal (no SoftDevice)
 * 
 * We're using TrouBLE/nrf-sdc which doesn't require SoftDevice binary blob.
 * Firmware starts at 0x00000000 (full 512KB flash available).
 */

MEMORY
{
  FLASH : ORIGIN = 0x00000000, LENGTH = 512K  /* Full flash available (no SoftDevice) */
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K   /* Full RAM available (no SoftDevice) */
}

/* This is where the call stack will be allocated. */
/* The stack is of the full descending type. */
/* NOTE: You may need to adjust this based on your application's requirements */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);


