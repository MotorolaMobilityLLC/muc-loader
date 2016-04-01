/**
 * Copyright (c) 2015 Motorola Mobility.
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

#ifndef __COMMON_INCLUDE_DATALINK_H
#define __COMMON_INCLUDE_DATALINK_H

#include "stm32l4xx_hal.h"

/* Data Link request types */
#define DL_MUC_OP_BUS_CONFIG        0x00
#define DL_MUC_OP_BUS_CONFIG_RESP   0x80

typedef enum {
    initial,
    full
} e_armDMAtype;

typedef enum {
    datalink,
    control,
    firmware
} e_protocol_type;

extern e_armDMAtype dl_get_dma_type(void);
extern void dl_set_dma_type(e_armDMAtype type);
extern void dl_set_protocol_type(e_protocol_type t);
extern e_protocol_type dl_get_protocol_type(void);
extern int process_sent_complete(void);
extern void dl_init(void);
extern int datalink_send(uint8_t *buf, size_t len, msg_sent_cb cb, void *ctx);
extern uint16_t datalink_get_max_payload_size(void);
extern void dl_spi_error_handler(SPI_HandleTypeDef *_hspi);
extern void dl_spi_transfer_complete(SPI_HandleTypeDef *_hspi);
extern void setup_exchange(void);
#endif
