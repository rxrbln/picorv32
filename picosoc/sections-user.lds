
MEMORY {
    RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 0x20000 /* 128 KB */
}

SECTIONS {
    /* The program code and other data goes into FLASH */
    .text :
    {
        . = ALIGN(4);
        *(.start)          /* .text start sections (code) */
	*(.fastcode)

        *(.text)           /* .text sections (code) */
        *(.text*)          /* .text* sections (code) */
        *(.rodata)         /* .rodata sections (constants, strings, etc.) */
        *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
        *(.srodata)        /* .rodata sections (constants, strings, etc.) */
        *(.srodata*)       /* .rodata* sections (constants, strings, etc.) */

        *(.data)           /* .data sections */
        *(.data*)          /* .data* sections */
        *(.sdata)          /* .sdata sections */
        *(.sdata*)         /* .sdata* sections */

        *(.bss)
        *(.bss*)
        *(.sbss)
        *(.sbss*)
        *(COMMON)

        . = ALIGN(4);
    } >RAM
}
