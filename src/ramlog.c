/**
 * Copyright (c) 2016 Motorola Mobility.
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

#include <config.h>
#include <debug.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "boot_main.h"

#define RAMLOG_VALIDITY    (0x0F1E2D3C)

struct ramlog_dev_s
{
    int               rl_validity;  /* Allows check if ramlog is valid */
    volatile uint16_t rl_head;      /* The head index (where data is added) */
    volatile uint16_t rl_tail;      /* The tail index (where data is removed) */
    uint16_t          rl_unused;    /* Field not used by muc-loader */
    uint32_t          rl_bufsize;   /* Size of the RAM buffer */
    char             *rl_buffer;    /* Circular RAM buffer */
};

static struct ramlog_dev_s *rl_dev = (struct ramlog_dev_s *)CONFIG_RAMLOG_ADDR;

void ramlog_addchar(char ch)
{
    int nexthead;

    if (rl_dev->rl_validity != RAMLOG_VALIDITY) return;

    /* Calculate the write index AFTER the next byte is written */
    nexthead = rl_dev->rl_head + 1;
    if (nexthead >= rl_dev->rl_bufsize) {
        nexthead = 0;
    }

    /* Would the next write overflow the circular buffer? */
    if (nexthead == rl_dev->rl_tail) {
        /* Yes, so move tail to drop oldest byte from buffer */
        if (++rl_dev->rl_tail >= rl_dev->rl_bufsize) {
          rl_dev->rl_tail = 0;
        }
    }

    rl_dev->rl_buffer[rl_dev->rl_head] = ch;
    rl_dev->rl_head = nexthead;
}

void ramlog_write(const char *buffer, uint32_t len)
{
    int nwritten;

    for (nwritten = 0; nwritten < len; nwritten++) {
        ramlog_addchar(buffer[nwritten]);
    }
}

void ramlog_writehex8(uint8_t num)
{
    int digit;
    int nibble;

    for (digit = 1; digit >= 0; digit--) {
        nibble = (num >> (digit << 2)) & 0x0f;
        if (nibble < 10) {
            ramlog_addchar('0' + nibble);
        } else {
            ramlog_addchar('a' + (nibble - 10));
        }
    }
}

void ramlog_reason(void)
{
    ramlog_write("--[MuC Loader]-- Reason: ", 25);
    ramlog_writehex8(get_flash_reason());
    ramlog_addchar('\n');
}

