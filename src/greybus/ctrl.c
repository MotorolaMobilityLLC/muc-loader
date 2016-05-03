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
#include "boot_main.h"
#include "debug.h"
#include "greybus.h"
#include "manifest.h"

uint32_t gbfw_cportid = 0;

static int gbctrl_get_version(uint32_t cportid,
                            gb_operation_header *op_header) {
    unsigned char payload[2] = {GREYBUS_MAJOR_VERSION,
                                GREYBUS_MINOR_VERSION};

    return greybus_send_response(cportid,
                               op_header,
                               GB_OP_SUCCESS,
                               payload,
                               sizeof(payload),
                               NULL);
}

static int gbctrl_probe_ap(uint32_t cportid,
                         gb_operation_header *op_header) {
    uint16_t payload[1] = {0};

    return greybus_send_response(cportid,
                               op_header,
                               GB_OP_SUCCESS,
                               (unsigned char*)payload,
                               sizeof(payload),
                               NULL);
}

static int gbctrl_get_manifest_size(uint32_t cportid,
                                  gb_operation_header *op_header) {
    uint16_t payload[1] = { get_manifest_size() };

    return greybus_send_response(cportid,
                               op_header,
                               GB_OP_SUCCESS,
                               (unsigned char*)payload,
                               sizeof(payload),
                               NULL);
}

static bool manifest_fetched = false;

bool manifest_fetched_by_ap(void) {
    return manifest_fetched;
}

static int gbctrl_get_manifest(uint32_t cportid,
                             gb_operation_header *op_header) {
    int rc;
    rc = greybus_send_response(cportid,
                             op_header,
                             GB_OP_SUCCESS,
                             get_manifest(),
                             get_manifest_size(),
                             NULL);
    if (rc) {
        return rc;
    }

    manifest_fetched = true;
    return 0;
}

static int gbctrl_connected(uint32_t cportid,
                          gb_operation_header *op_header) {
    uint16_t *payload = (uint16_t *)(op_header + 1);

    if (op_header->size != sizeof(gb_operation_header) + sizeof(*payload)) {
        greybus_send_response(cportid,
                            op_header,
                            GB_OP_INVALID,
                            NULL,
                            0,
                            NULL);
        return -1;
    }

    gbfw_cportid = *payload;
    return greybus_send_response(cportid,
                               op_header,
                               GB_OP_SUCCESS,
                               NULL,
                               0,
                               NULL);
}

static int gbctrl_disconnected(uint32_t cportid,
                             gb_operation_header *op_header) {
    uint16_t *payload = (uint16_t *)(op_header + 1);

    if (op_header->size != sizeof(gb_operation_header) + sizeof(*payload) ||
        *payload != gbfw_cportid) {
        greybus_send_response(cportid,
                            op_header,
                            GB_OP_INVALID,
                            NULL,
                            0,
                            NULL);
        return -1;
    }

    return greybus_send_response(cportid,
                               op_header,
                               GB_OP_SUCCESS,
                               NULL,
                               0,
                               NULL);
}

static int gbctrl_unimplemented(uint32_t cportid,
        gb_operation_header *op_header)
{
    int rv = GB_OP_SUCCESS;

    if (op_header->id) {
        rv = greybus_send_response(cportid,
            op_header,
            GB_OP_INVALID,
            NULL,
            0,
            NULL);
    }

    return rv;
}

int control_cport_handler(uint32_t cportid,
                          void *data,
                          size_t len)
{
    int rc = 0;
    if (len < sizeof(gb_operation_header)) {
        dbgprint("control_cport_handler: RX data length error\r\n");
        return -1;
    }

    gb_operation_header *op_header = (gb_operation_header *)data;

    switch (op_header->type) {
    case GB_CTRL_OP_VERSION:
        rc = gbctrl_get_version(cportid, op_header);
        break;
    case GB_CTRL_OP_PROBE_AP:
        rc = gbctrl_probe_ap(cportid, op_header);
        break;
    case GB_CTRL_OP_GET_MANIFEST_SIZE:
        rc = gbctrl_get_manifest_size(cportid, op_header);
        break;
    case GB_CTRL_OP_GET_MANIFEST:
        rc = gbctrl_get_manifest(cportid, op_header);
        break;
    case GB_CTRL_OP_CONNECTED:
        rc = gbctrl_connected(cportid, op_header);
        break;
    case GB_CTRL_OP_DISCONNECTED:
        rc = gbctrl_disconnected(cportid, op_header);
        break;
    default:
        rc  = gbctrl_unimplemented(cportid, op_header);
        break;
    }

    return rc;
}
