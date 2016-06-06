/**
 * Copyright (c) 2015-2016 Motorola Mobility, LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
OUTPUT_FORMAT("elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_Start)

MEMORY {
    flash (rx) : ORIGIN = 0x08000000, LENGTH = 32K
    sram (rwx) : ORIGIN = 0x20000000, LENGTH = 48K  /* Leave 16K for dmesg */
}

_estack = 0x20016fff;                     /* 48K of usable RAM after dmesg */

SECTIONS {
    . = ALIGN(4);
    .flash_vector : {
        KEEP(*(.flash_vector))
        KEEP(*(.text._Start))
    } > flash

    /*
     *  ISR
     */
    . = ALIGN(4);
    _sf_isrv = LOADADDR(.reset);
    .reset : {
        _sr_isrv = .;
        KEEP(*(.isr_vector))
        . = ALIGN(8);
    } > sram AT > flash

    /*
     *  TEXT
     */
    . = ALIGN(4);
    .text : {
        *(.text .text.*)
        *(.fixup)
        *(.gnu.warning)
        *(.glue_7)
        *(.glue_7t)
        . = ALIGN(4);
        *(.rodata .rodata*)

        . = ALIGN(4);
        __preinit_array_start = ABSOLUTE(.);
        *(.preinit_array*)
        __preinit_array_end = ABSOLUTE(.);

        __init_array_start = ABSOLUTE(.);
        *(.init_array.* .init_array*)
        __init_array_end = ABSOLUTE(.);

        __fini_array_start =  ABSOLUTE(.);
        *(.fini_array.* .fini_array*)
        __fini_array_end =  ABSOLUTE(.);

        KEEP (*(.init))
        KEEP (*(.fini))
        KEEP(*(.id_section))
        . = ALIGN(8);
    } > sram AT > flash

    . = ALIGN(4);
    .ARM.extab : {
        *(.ARM.extab*)
        . = ALIGN(8);
    } > sram AT > flash

    . = ALIGN(4);
    .ARM.exidx : {
        __exidx_start = ABSOLUTE(.);
        *(.ARM.exidx*)
        __exidx_end = ABSOLUTE(.);
	    __read_only_end = ABSOLUTE(.);
        . = ALIGN(8);
    } > sram AT > flash

    /*
     *  DATA
     */
    . = ALIGN(4);
    .data : {
        _sdata = ABSOLUTE(.);
        _sidata = _sdata;
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        _edata = ABSOLUTE(.);
        . = ALIGN(8);
    } > sram AT > flash

    /*
     *  BSS
     */
    . = ALIGN(4);
    .bss : {
        _sbss = ABSOLUTE(.);
        __bss_start__ = _sbss;
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        _ebss = ABSOLUTE(.);
        __bss_end__ = _ebss;
    } > sram

    /DISCARD/ : {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }
}

