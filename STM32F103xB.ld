/*
 * Copyright (C) 2015  Zubax Robotics  <info@zubax.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

 /*
  * Based on a ChibiOS demo app.
  */

MEMORY
{
    flash : org = 0x08008000, len = 95k    /* 32K for bootloader, 1K for configs */
    ram   : org = 0x20000100, len = 20224   /* First 256 bytes are reserved for bootloader/app communication */
}


__ram_start__          = ORIGIN(ram);
__ram_size__           = LENGTH(ram);
__ram_end__            = __ram_start__ + __ram_size__;

ENTRY(Reset_Handler)

PROVIDE(DeviceSignatureStorage = ORIGIN(flash) - 256);

SECTIONS
{
    . = 0;
    _text = .;

    startup : ALIGN(16) SUBALIGN(16)
    {
        FILL(0xDEADC0DE);
        KEEP(*(.vectors))
        . = ALIGN(8);               /* Note that the app descriptor must be aligned at 8 bytes boundary! */
        KEEP(*(.app_descriptor));
        . = ALIGN(8);
    } > flash

    constructors : ALIGN(4) SUBALIGN(4)
    {
        FILL(0xDEADC0DE);
        PROVIDE(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE(__init_array_end = .);
    } > flash

    destructors : ALIGN(4) SUBALIGN(4)
    {
        FILL(0xDEADC0DE);
        PROVIDE(__fini_array_start = .);
        KEEP(*(.fini_array))
        KEEP(*(SORT(.fini_array.*)))
        PROVIDE(__fini_array_end = .);
    } > flash

    /*
     * This padding section must be inserted before every section with greater alignment.
     */
    .padding1 :
    {
        /*
         * Filling is important: if it is not specified, gaps between aligned sections will be filled with zero when
         * exporting binaries, and will be skipped (i.e. set to 0xFF) when flashing .elf directly during debugging.
         * The discrepancy causes the firmware CRC to mismatch, preventing the bootloader from running it.
         * Assigning a well-defined fill byte eliminates the discrepancy (binary files will also contain 0xFF or
         * whatever), allowing to flash the firmware directly with an SWD debugger during development.
         * See http://www.math.utah.edu/docs/info/ld_3.html
         * Also http://blog.hrbacek.info/2013/12/28/filling-unused-memory-area-using-gnu-linker-script
         */
        FILL(0xDEADC0DE);
        /* Now, we need some dummy data here, otherwise the alignment above will be ignored. */
        BYTE(0xFF);
        . = ALIGN(64);  /* Text sometimes gets aligned at >32, not sure why */
    } > flash

    .text : ALIGN(16) SUBALIGN(16)
    {
        FILL(0xDEADC0DE);
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
        *(.glue_7t)
        *(.glue_7)
        *(.gcc*)
        /*
         * Note that SOME DEBUGGERS WILL REFUSE TO LOAD THE ELF if it contains odd-sized sections!
         * This issue can be observed, for example, with DroneCode Probe.
         */
        . = ALIGN(32);
    } > flash

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > flash

    .ARM.exidx :
    {
        PROVIDE(__exidx_start = .);
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        PROVIDE(__exidx_end = .);
     } > flash

    .eh_frame_hdr :
    {
        *(.eh_frame_hdr)
    } > flash

    .eh_frame : ONLY_IF_RO
    {
        *(.eh_frame)
    } > flash

    .textalign : ONLY_IF_RO
    {
        FILL(0xDEADC0DE);
        BYTE(0xFF);
        . = ALIGN(8);
    } > flash

    . = ALIGN(4);
    _etext = .;
    _textdata = _etext;
    _textdata_start = _etext;

    .mstack :
    {
        . = ALIGN(8);
        __main_stack_base__ = .;
        . += __main_stack_size__;
        . = ALIGN(8);
        __main_stack_end__ = .;
    } > ram

    .pstack :
    {
        __process_stack_base__ = .;
        __main_thread_stack_base__ = .;
        . += __process_stack_size__;
        . = ALIGN(8);
        __process_stack_end__ = .;
        __main_thread_stack_end__ = .;
    } > ram

    .data : ALIGN(4)
    {
        . = ALIGN(4);
        PROVIDE(_data_start = .);
        *(.data)
        *(.data.*)
        *(.ramtext)
        . = ALIGN(4);
        PROVIDE(_data_end = .);
    } > ram AT > flash

    .bss : ALIGN(4)
    {
        . = ALIGN(4);
        PROVIDE(_bss_start = .);
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
        PROVIDE(_bss_end = .);
        PROVIDE(end = .);
    } > ram

    .ram (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        *(.ram)
        *(.ram.*)
        . = ALIGN(4);
        __ram_free__ = .;
    } > ram
}

/* Heap default boundaries, it is defaulted to be the non-used part of ram region.*/
__heap_base__   = __ram_free__;
__heap_end__    = __ram_end__;
