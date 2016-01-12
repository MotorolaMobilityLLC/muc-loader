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

#include "stm32l4xx_hal.h"

#include <version.h>

/* Version of the Greybus control protocol we support */
#define MB_CONTROL_VERSION_MAJOR              0x00
#define MB_CONTROL_VERSION_MINOR              0x02

/* Greybus control request types */
#define MB_CONTROL_TYPE_INVALID               0x00
#define MB_CONTROL_TYPE_PROTOCOL_VERSION      0x01
#define MB_CONTROL_TYPE_GET_IDS               0x02
#define MB_CONTROL_TYPE_REBOOT                0x03
#define MB_CONTROL_TYPE_PORT_CONNECTED        0x04
#define MB_CONTROL_TYPE_PORT_DISCONNECTED     0x05
#define MB_CONTROL_TYPE_SLAVE_POWER           0x06
#define MB_CONTROL_TYPE_ROOT_VERSION          0x07

/* Valid modes for the reboot request */
#define MB_CONTROL_REBOOT_MODE_RESET          0x01
#define MB_CONTROL_REBOOT_MODE_BOOTLOADER     0x02

/* Valid modes for the slave power request */
#define MB_CONTROL_SLAVE_POWER_ON             0x01
#define MB_CONTROL_SLAVE_POWER_OFF            0x02
#define MB_CONTROL_SLAVE_POWER_FLASH_MODE     0x03

enum slave_pwrctrl_mode {
    SLAVE_PWRCTRL_POWER_ON          = 0x01,
    SLAVE_PWRCTRL_POWER_OFF         = 0x02,
    SLAVE_PWRCTRL_POWER_FLASH_MODE  = 0x03,
};

/* Control protocol version request has no payload */
struct mb_control_proto_version_response {
    uint8_t      major;
    uint8_t      minor;
} __attribute__ ((packed));

/* Control protocol get_ids request has no payload */
struct mb_control_get_ids_response {
    uint32_t    unipro_mfg_id;
    uint32_t    unipro_prod_id;
    uint32_t    ara_vend_id;
    uint32_t    ara_prod_id;
    uint64_t    uid_low;
    uint64_t    uid_high;
    uint32_t    fw_version;
    uint32_t    slave_mask;
} __attribute__ ((packed));

struct mb_control_reboot_request {
    uint8_t      mode;
} __attribute__ ((packed));

struct mb_control_connected_request {
    uint8_t    cport_id;
} __attribute__ ((packed));

struct mb_control_disconnected_request {
    uint8_t    cport_id;
} __attribute__ ((packed));

struct mb_control_power_ctrl_request {
    uint32_t    slave_id;
    uint8_t      mode;
} __attribute__ ((packed));
/* Control protocol slave power response has no payload */

struct mb_control_root_ver_response {
    uint8_t      version;
} __attribute__ ((packed));

static int modsctrl_get_version(uint32_t cportid,
                            gb_operation_header *op_header)
{
    unsigned char payload[2] = {MB_CONTROL_VERSION_MAJOR,
                                MB_CONTROL_VERSION_MINOR};

    return greybus_op_response(cportid,
                               op_header,
                               GB_OP_SUCCESS,
                               payload,
                               sizeof(payload));
}

static int modsctrl_get_ids(uint32_t cportid,
                            gb_operation_header *op_header)
{
    struct mb_control_get_ids_response get_ids_resp;

    get_ids_resp.fw_version = (CONFIG_VERSION_MAJOR << 16 | CONFIG_VERSION_MINOR);
    get_chip_id(&get_ids_resp.unipro_mfg_id, &get_ids_resp.unipro_prod_id);
    get_board_id(&get_ids_resp.ara_vend_id, &get_ids_resp.ara_prod_id);
    get_chip_uid(&get_ids_resp.uid_high, &get_ids_resp.uid_low);
#ifdef MOD_SLAVE_APBE
    get_ids_resp.slave_mask = 1;
    dbgprint("MODCTRL:SLAVE MASK SET\r\n");
#else
    get_ids_resp.slave_mask = 0;
#endif

    return greybus_op_response(cportid,
                               op_header,
                               GB_OP_SUCCESS,
                               (unsigned char *)&get_ids_resp,
                               sizeof(get_ids_resp));
}

static int modsctrl_reboot(uint32_t cportid,
        struct gb_operation_msg *msg)
{
    int rv = GB_OP_SUCCESS;

    struct mb_control_reboot_request *req =
        (struct mb_control_reboot_request *)msg->data;

    switch (req->mode) {
        case MB_CONTROL_REBOOT_MODE_RESET:
            dbgprint("REBOOT_RESET\r\n");
        break;
        case MB_CONTROL_REBOOT_MODE_BOOTLOADER:
            dbgprint("REBOOT_BOOTLOADER\r\n");
            if (CheckFlashMode() == BOOT_STATE_NORMAL) {
                set_request_flash();
            }
        break;
        default:
            dbgprintx32("REBOOT ", req->mode, "\r\n");
        break;
    }
    HAL_NVIC_SystemReset();
    return rv;
}

static int modsctrl_unimplemented(uint32_t cportid,
        gb_operation_header *op_header)
{
    int rv = GB_OP_SUCCESS;

    dbgprintx32("unimplemented: ", op_header->id, "\r\n");
    if (op_header->id) {
        rv = greybus_op_response(cportid,
            op_header,
            GB_OP_INVALID,
            NULL,
            0);
    }

    return rv;
}

static int modsctrl_root_version(uint32_t cportid,
        gb_operation_header *op_header)
{
    struct mb_control_root_ver_response resp = { CONFIG_ROOT_VERSION };

    return greybus_op_response(cportid,
            op_header,
            GB_OP_SUCCESS,
            (unsigned char *)&resp,
            sizeof(resp));
}

static int modsctrl_slave_power(uint32_t cportid,
        struct gb_operation_msg *msg)
{
    int ret = 0;
    struct mb_control_power_ctrl_request *req =
        (struct mb_control_power_ctrl_request *)msg->data;

#ifdef MOD_SLAVE_APBE
    ret = slave_pwrctrl_set_mode(req->mode);
#else
    ret = modsctrl_unimplemented(cportid, (gb_operation_header *)msg);
#endif

    return ret ? GB_OP_UNKNOWN_ERROR : GB_OP_SUCCESS;
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
    case MB_CONTROL_TYPE_PROTOCOL_VERSION:
        dbgprint("MODCTRL:VER\r\n");
        rc = modsctrl_get_version(cportid, op_header);
        break;
    case MB_CONTROL_TYPE_GET_IDS:
        dbgprint("MODCTRL:GET_IDS\r\n");
        rc = modsctrl_get_ids(cportid, op_header);
        break;
    case MB_CONTROL_TYPE_REBOOT:
        dbgprint("MODCTRL:REBOOT\r\n");
        rc = modsctrl_reboot(cportid,
                (struct gb_operation_msg *)op_header);
        break;
    case MB_CONTROL_TYPE_PORT_CONNECTED:
        dbgprint("MODCTRL:CONNECTED\r\n");
        rc = modsctrl_unimplemented(cportid, op_header);
        break;
    case MB_CONTROL_TYPE_PORT_DISCONNECTED:
        dbgprint("MODCTRL:DISCONNECTED\r\n");
        rc = modsctrl_unimplemented(cportid, op_header);
        break;
    case MB_CONTROL_TYPE_SLAVE_POWER:
        dbgprint("MODCTRL:SLAVE_POWER\r\n");
        rc = modsctrl_slave_power(cportid, (struct gb_operation_msg *)op_header);
        break;
    case MB_CONTROL_TYPE_ROOT_VERSION:
        dbgprint("MODCTRL:ROOT_VERSION\r\n");
        rc = modsctrl_root_version(cportid, op_header);
        break;
    default:
        dbgprintx32("MODCTRL:default: ", op_header->type, "\r\n");
        rc = modsctrl_unimplemented(cportid, op_header);
        break;
    }

    return rc;
}
