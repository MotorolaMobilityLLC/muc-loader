/**
 * Copyright (c) 2015 Google Inc.
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

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Determine if a value is a power of 2
 *
 * @param x the value to check.
 *
 * @returns true if it is a power of 2, false otherwise
 */
bool is_power_of_2(uint32_t x);

/**
 * @brief Determine if an address is block-aligned
 *
 * @param location The address to check
 * @param block_size The size of a block (must be a power of 2)
 *
 * @returns true if it is block-aligned, false otherwise
 */
bool block_aligned(uint32_t address, uint32_t block_size);

/**
 * @brief Round up an address to the next block boundary
 *
 * @param location The address to check
 * @param block_size The size of a block (must be a power of 2)
 *
 * @returns A block-aligned address
 */
uint32_t next_block_boundary(uint32_t address, uint32_t block_size);

/**
 * @brief Check a range of bytes for a constant fill
 *
 * @param buf Points to the start of the region to check
 * @param len The number of bytes to check
 * @param fill_byte The constant byte to check against
 *
 * @returns True if the buffer is filled with a constan byte, false otherwise.
 */
bool is_constant_fill(uint8_t * buf, uint32_t len, uint8_t fill_byte);

void delay(volatile uint32_t steps);

#define TIMING_BUG_DELAY_LENGTH (0xfffff)

#define DISJOINT_OR(x, y)   (!x ? y : x)
