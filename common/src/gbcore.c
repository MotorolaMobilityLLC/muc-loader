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
#include "gbfirmware.h"

uint32_t gbfw_cportid = 0;

/* Manifest with control and firmware protocol generated with manifesto */
static unsigned char manifest[] = {
  0x58, 0x00, 0x00, 0x01, 0x08, 0x00, 0x01, 0x00, 0x01, 0x02, 0x00, 0x00,
  0x1c, 0x00, 0x02, 0x00, 0x16, 0x01, 0x4d, 0x6f, 0x74, 0x6f, 0x72, 0x6f,
  0x6c, 0x61, 0x20, 0x4d, 0x6f, 0x62, 0x69, 0x6c, 0x69, 0x74, 0x79, 0x2c,
  0x20, 0x4c, 0x4c, 0x43, 0x10, 0x00, 0x02, 0x00, 0x08, 0x02, 0x46, 0x69,
  0x72, 0x6d, 0x77, 0x61, 0x72, 0x65, 0x00, 0x00, 0x08, 0x00, 0x04, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x08, 0x00, 0x04, 0x00, 0x01, 0x00, 0x01, 0x15, 0x08, 0x00, 0x03, 0x00,
  0x01, 0xff, 0x00, 0x00, 0x0a, 0xca, 0xa5, 0xcc, 0xac, 0xf8, 0xb2, 0xa4,
  0x0b, 0xfd, 0x01, 0x96, 0xa8, 0x96, 0xba, 0xdc, 0x04, 0x7f, 0x61, 0x37,
  0xdc, 0x62, 0x6c, 0x4c, 0x97, 0xd3, 0x06, 0x16, 0xe3, 0x8c, 0x86, 0xc3,
  0xb6, 0x2c, 0x31, 0x3f, 0xf8, 0xf4, 0x31, 0x74, 0xc9, 0xb1, 0x3b, 0xb2,
  0x47, 0xe9, 0x65, 0x4c, 0x76, 0xb5, 0x0f, 0x79, 0xfa, 0xbe, 0x9f, 0xf7,
  0xad, 0xa2, 0x7b, 0xdf, 0x13, 0xeb, 0x29, 0xac, 0x67, 0xb5, 0x04, 0xbb,
  0xf5, 0x9f, 0xb3, 0x1b, 0xd2, 0xb0, 0xd1, 0xfb, 0x81, 0x59, 0x05, 0x74,
  0xdf, 0xf9, 0x5a, 0x4b, 0x6e, 0xa0, 0xcc, 0x39, 0x0d, 0x84, 0x9b, 0x81,
  0x32, 0x0e, 0xa5, 0x28, 0xb2, 0x97, 0xab, 0x24, 0x92, 0x59, 0xdc, 0xaf,
  0x61, 0x1f, 0xc9, 0x06, 0xc6, 0xc8, 0x54, 0xb2, 0xc3, 0xb1, 0xdf, 0x6b,
  0x34, 0x22, 0x62, 0x2f, 0x0b, 0x72, 0x12, 0x55, 0x1e, 0x07, 0x86, 0x97,
  0x30, 0xf6, 0xf3, 0xa6, 0xd7, 0x73, 0xa1, 0x9b, 0xf8, 0x4b, 0x85, 0x71,
  0x8b, 0xf5, 0x48, 0xeb, 0x68, 0xfa, 0x3f, 0xb7, 0x03, 0xa1, 0x19, 0x7a,
  0xc8, 0x33, 0x27, 0xd2, 0x7d, 0xb9, 0xbf, 0x4c, 0xa0, 0xac, 0x82, 0xb9,
  0x7f, 0xa3, 0x19, 0xbb, 0x74, 0x49, 0xca, 0x9c, 0x1f, 0xfe, 0x22, 0x21,
  0x7f, 0x07, 0x82, 0xba, 0x8c, 0xe1, 0x78, 0x8d, 0xd1, 0x7e, 0x85, 0xfa,
  0x0e, 0x6a, 0x4e, 0x83, 0xf2, 0x0a, 0xed, 0x08, 0x3a, 0x3d, 0x24, 0x45,
  0x50, 0xab, 0xd3, 0x33, 0x9d, 0xb7, 0x98, 0xfe, 0xa5, 0xcc, 0xbd, 0x78,
  0xfe, 0xd3, 0x51, 0x5d, 0x7a, 0xc8, 0x6a, 0x3f, 0xe3, 0x6b, 0xbd, 0x46,
  0xad, 0xc8, 0x78, 0xb0, 0x66, 0x32, 0x93, 0x4e, 0x8c, 0xf6, 0x0c, 0x18,
  0x6b, 0x27, 0xd3, 0x69, 0xaa, 0x4f, 0xd5, 0xda, 0x26, 0x50, 0xa1, 0x64,
  0x3d, 0xcf, 0xbc, 0xb0, 0x3d, 0xd8, 0x9e, 0x90
};

static int greybus_send_message(uint32_t cport,
                                uint16_t id,
                                uint8_t type,
                                uint8_t status,
                                unsigned char *payload_data,
                                uint16_t payload_size) {
    /**
     * the payload_size are all pretty small for Greybus Control protocol
     * and the firmware downloading protocol on the boot ROM side.
     * So we can use the variable sized array here and not worrying
     * about running out of stack space
     */
    struct mods_spi_msg *spi_msg = (struct mods_spi_msg *)&aTxBuffer[0];

    gb_operation_header *msg_header = (gb_operation_header *)&spi_msg->m_msg.gb_op_hdr;
    unsigned char *payload = &spi_msg->m_msg.gb_op_hdr[sizeof(gb_operation_header)];

    msg_header->size    = sizeof(gb_operation_header) + payload_size;
    msg_header->id      = id;
    msg_header->type    = type;
    msg_header->status  = status;
    msg_header->padding = 0;

    if (payload_size != 0 && payload_data != NULL) {
        memcpy(payload, payload_data, payload_size);
    }
    return chip_unipro_send(cport, spi_msg, sizeof(spi_msg));
}

int dl_send_message(uint8_t id,
                                uint8_t status,
                                unsigned char *payload_data,
                                uint16_t payload_size) {
    /**
     * the payload_size are all pretty small for Greybus Control protocol
     * and the firmware downloading protocol on the boot ROM side.
     * So we can use the variable sized array here and not worrying
     * about running out of stack space
     */
    struct mods_spi_msg *spi_msg = (struct mods_spi_msg *)&aTxBuffer[0];
    unsigned char *payload = &spi_msg->dl_msg.dl_pl[0];

    spi_msg->dl_msg.mesg_id = id | status;

    if (payload_size != 0 && payload_data != NULL) {
        memcpy(payload, payload_data, payload_size);
    }

    spi_msg->hdr_bits = MSG_TYPE_DL;
    spi_msg->hdr_bits |= HDR_BIT_VALID;

    respReady = true;

    return 0;
}

int greybus_send_request(uint32_t cport,
                         uint16_t id,
                         uint8_t type,
                         unsigned char *payload_data,
                         uint16_t payload_size) {
    return greybus_send_message(cport,
                                id,
                                type,
                                0,
                                payload_data,
                                payload_size);
}

int greybus_op_response(uint32_t cport,
                        gb_operation_header *op_header,
                        uint8_t status,
                        unsigned char *payload_data,
                        uint16_t payload_size) {
    return greybus_send_message(cport,
                                op_header->id,
                                op_header->type | GB_TYPE_RESPONSE,
                                status,
                                payload_data,
                                payload_size);
}

static int gbctrl_get_version(uint32_t cportid,
                            gb_operation_header *op_header) {
    unsigned char payload[2] = {GREYBUS_MAJOR_VERSION,
                                GREYBUS_MINOR_VERSION};

    return greybus_op_response(cportid,
                               op_header,
                               GB_OP_SUCCESS,
                               payload,
                               sizeof(payload));
}

static int gbctrl_probe_ap(uint32_t cportid,
                         gb_operation_header *op_header) {
    uint16_t payload[1] = {0};

    return greybus_op_response(cportid,
                               op_header,
                               GB_OP_SUCCESS,
                               (unsigned char*)payload,
                               sizeof(payload));
}

static int gbctrl_get_manifest_size(uint32_t cportid,
                                  gb_operation_header *op_header) {
    uint16_t payload[1] = {sizeof(manifest)};

    return greybus_op_response(cportid,
                               op_header,
                               GB_OP_SUCCESS,
                               (unsigned char*)payload,
                               sizeof(payload));
}

static bool manifest_fetched = false;

bool manifest_fetched_by_ap(void) {
    return manifest_fetched;
}

static int gbctrl_get_manifest(uint32_t cportid,
                             gb_operation_header *op_header) {
    int rc;

    rc = greybus_op_response(cportid,
                             op_header,
                             GB_OP_SUCCESS,
                             manifest,
                             sizeof(manifest));
    if (rc) {
        return rc;
    }

    manifest_fetched = true;
    return 0;
}

static int gbctrl_connected(uint32_t cportid,
                          gb_operation_header *op_header) {
    int rc;
    uint16_t *payload = (uint16_t *)(op_header + 1);

    if (op_header->size != sizeof(gb_operation_header) + sizeof(*payload)) {
        greybus_op_response(cportid,
                            op_header,
                            GB_OP_INVALID,
                            NULL,
                            0);
        return -1;
    }

    rc = greybus_cport_connect();
    gbfw_cportid = *payload;
    if (rc != 0) {
        greybus_op_response(cportid,
                            op_header,
                            GB_OP_UNKNOWN_ERROR,
                            NULL,
                            0);
        return -1;
    }
    return greybus_op_response(cportid,
                               op_header,
                               GB_OP_SUCCESS,
                               NULL,
                               0);
}

static int gbctrl_disconnected(uint32_t cportid,
                             gb_operation_header *op_header) {
    int rc;
    uint16_t *payload = (uint16_t *)(op_header + 1);

    if (op_header->size != sizeof(gb_operation_header) + sizeof(*payload) ||
        *payload != gbfw_cportid) {
        greybus_op_response(cportid,
                            op_header,
                            GB_OP_INVALID,
                            NULL,
                            0);
        return -1;
    }

    rc = greybus_cport_disconnect();
    if (rc != 0) {
        greybus_op_response(cportid,
                            op_header,
                            GB_OP_UNKNOWN_ERROR,
                            NULL,
                            0);
        return -1;
    }
    return greybus_op_response(cportid,
                               op_header,
                               GB_OP_SUCCESS,
                               NULL,
                               0);
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
        dbgprint("VER\r\n");
        rc = gbctrl_get_version(cportid, op_header);
        break;
    case GB_CTRL_OP_PROBE_AP:
        dbgprint("PROBEAP\r\n");
        rc = gbctrl_probe_ap(cportid, op_header);
        break;
    case GB_CTRL_OP_GET_MANIFEST_SIZE:
        dbgprint("MANIFSIZE\r\n");
        rc = gbctrl_get_manifest_size(cportid, op_header);
        break;
    case GB_CTRL_OP_GET_MANIFEST:
        dbgprint("MANIFEST\r\n");
        rc = gbctrl_get_manifest(cportid, op_header);
        break;
    case GB_CTRL_OP_CONNECTED:
        dbgprint("CONN\r\n");
        rc = gbctrl_connected(cportid, op_header);
        break;
    case GB_CTRL_OP_DISCONNECTED:
        dbgprint("DISCON\r\n");
        rc = gbctrl_disconnected(cportid, op_header);
        break;
    default:
        dbgprint("default\r\n");
        greybus_op_response(cportid,
                            op_header,
                            GB_OP_INVALID,
                            NULL,
                            0);
        break;
    }

    return rc;
}

static int modsctrl_get_ids(uint32_t cportid,
                            gb_operation_header *op_header) {
    struct gb_control_get_ids_response get_ids_resp;

    get_ids_resp.fw_version = MOD_BOOT_VERSION;
    get_chip_id(&get_ids_resp.unipro_mfg_id, &get_ids_resp.unipro_prod_id);
    get_board_id(&get_ids_resp.ara_vend_id, &get_ids_resp.ara_prod_id);
    get_chip_uid(&get_ids_resp.uid_high, &get_ids_resp.uid_low);

    return greybus_op_response(cportid,
                               op_header,
                               GB_OP_SUCCESS,
                               (unsigned char *)&get_ids_resp,
                               sizeof(get_ids_resp));
}

static int modsctrl_get_version(uint32_t cportid,
                            gb_operation_header *op_header) {
    unsigned char payload[2] = {MB_CONTROL_VERSION_MAJOR,
                                MB_CONTROL_VERSION_MINOR};

    return greybus_op_response(cportid,
                               op_header,
                               GB_OP_SUCCESS,
                               payload,
                               sizeof(payload));
}

int mods_control_handler(uint32_t cportid,
                          void *data,
                          size_t len)
{
    int rc = 0;
    if (len < sizeof(gb_operation_header)) {
        dbgprint("mods_control_handler: RX data length error\r\n");
        return -1;
    }

    gb_operation_header *op_header = (gb_operation_header *)data;

    switch (op_header->type) {
    case MB_CONTROL_TYPE_GET_IDS:
        dbgprint("GET_ID\r\n");
        rc = modsctrl_get_ids(cportid, op_header);
        break;
    case MB_CONTROL_TYPE_PROTOCOL_VERSION:
        dbgprint("NODVER\r\n");
        rc = modsctrl_get_version(cportid, op_header);
        break;
    default:
        dbgprint("default\r\n");
        greybus_op_response(cportid,
                            op_header,
                            GB_OP_INVALID,
                            NULL,
                            0);
        break;
    }

    return rc;
}
