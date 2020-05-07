/* Define memory regions. */
MEMORY
{
	rom (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 96K
	ram2 (rwx): ORIGIN = 0x10000000, LENGHT = 32k
}

/* Include the common ld script. */
INCLUDE cortex-m-generic.ld
