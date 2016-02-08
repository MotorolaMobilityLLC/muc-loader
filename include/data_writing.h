/*
 * Copyright (c) 2016 Motorola Mobility, LLC.
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
#ifndef __COMMON_INCLUDE_DATA_WRITING_H
#define __COMMON_INCLUDE_DATA_WRITING_H

#include <stdint.h>

typedef int (*data_writing_init)(void);
typedef int (*data_writing_erase_all)(void);
typedef int (*data_writing_erase)(uint32_t dst, uint32_t length);
typedef int (*data_writing_write)(uint32_t dst, void *src, uint32_t length);
typedef int (*data_writing_verify)(uint32_t dst, void *src, uint32_t length);
typedef int (*data_writing_finish)(void);
typedef size_t (*data_writing_get_capacity)(void);
typedef size_t (*data_writing_get_erase_size)(void);

typedef struct {
    data_writing_init init;
    data_writing_erase_all erase_all;
    data_writing_erase erase;
    data_writing_write write;
    data_writing_verify verify;
    data_writing_finish finish;
    data_writing_get_capacity get_capacity;
    data_writing_get_erase_size get_erase_size;
} data_write_ops;

#endif /* __COMMON_INCLUDE_DATA_WRITING_H */
