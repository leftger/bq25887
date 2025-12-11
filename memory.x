/* Memory layout tuned for the STM32WBA65RI.
 *
 * Verify these regions against the latest reference manual for your specific
 * device revision; adjust origins or lengths if ST updates the silicon. As of
 * the current datasheet, the device exposes:
 *   - 2 MiB of program flash starting at 0x0800_0000
 *   - 512 KiB of SRAM starting at 0x2000_0000
 */

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 512K
}

/* Provide stack start and heap/stack boundary symbols expected by many
 * Cortex-M crates. The stack grows downward from the top of RAM. */
PROVIDE(_stack_start = ORIGIN(RAM) + LENGTH(RAM));

/* Optional: allow the runtime to place the heap directly after `.bss`. */
PROVIDE(_heap_start = __bss_end__);
PROVIDE(_heap_end = ORIGIN(RAM) + LENGTH(RAM));
