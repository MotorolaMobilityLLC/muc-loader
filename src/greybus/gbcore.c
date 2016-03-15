/**
 * Copyright (c) 2015 Google Inc.
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

#include <stddef.h>
#include <string.h>
#include "debug.h"

#include "greybus.h"
#include "network.h"

extern gb_operation_header *gb_op_hdr;   /* TODO: factor out shared globals */

static uint16_t greybus_id_count = 1;
uint16_t greybus_get_next_id(void)
{
    uint16_t next = greybus_id_count++;

    if (!next)
        next = 1;
    return next;
}

uint16_t greybus_get_max_payload_size(void)
{
    return network_get_max_payload_size() - sizeof(struct gb_operation_hdr);
}

static int greybus_send_message(uint32_t cport,
                                uint16_t id,
                                uint8_t type,
                                uint8_t status,
                                const unsigned char *payload_data,
                                uint16_t payload_size,
                                msg_sent_cb cb)
{
    struct gb_operation_msg *msg = (struct gb_operation_msg *)greybus_get_operation_header();
    unsigned char *payload = &msg->data[0];

#ifdef CONFIG_DEBUG_GREYBUS_CORE
    dbgprint("greybus_send_message\r\n");
    dbgprintx32("  - id = ", id, "\r\n");
    dbgprintx32("  - type = ", type, "\r\n");
    dbgprintx32("  - status = ", status, "\r\n");
    dbgprintx32("  - gb_op_hdr = ", (uint32_t)msg, "\r\n");
    dbgprintx32("  - payload = ", (uint32_t) payload, "\r\n");
    dbgprintx32("  - callback = ", (uint32_t) cb, "\r\n");
#endif

    msg->hdr.size    = sizeof(struct gb_operation_hdr) + payload_size;
    msg->hdr.id      = id;
    msg->hdr.type    = type;
    msg->hdr.status  = status;
    msg->hdr.padding = 0;

    if (payload_size != 0 && payload_data != NULL) {
        memcpy(payload, payload_data, payload_size);
    }

    return network_send(cport, (uint8_t *)msg, msg->hdr.size, cb);
}

int greybus_send_request(uint32_t cport,
                         uint16_t id,
                         uint8_t type,
                         const unsigned char *payload_data,
                         uint16_t payload_size,
                         msg_sent_cb cb)
{
    return greybus_send_message(cport,
                                id,
                                type,
                                0,
                                payload_data,
                                payload_size,
                                cb);
}

int greybus_send_response(uint32_t cport,
                        gb_operation_header *op_header,
                        uint8_t status,
                        const unsigned char *payload_data,
                        uint16_t payload_size,
                        msg_sent_cb cb)
{
    return greybus_send_message(cport,
                                op_header->id,
                                op_header->type | GB_TYPE_RESPONSE,
                                status,
                                payload_data,
                                payload_size,
                                cb);
}
