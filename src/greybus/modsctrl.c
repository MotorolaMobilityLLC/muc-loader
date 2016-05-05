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

#include "stm32l4xx_hal.h"
#include <apbe.h>

#include <version.h>
#include <config.h>

/* Version of the Greybus control protocol we support */
#define MB_CONTROL_VERSION_MAJOR              0x00
#define MB_CONTROL_VERSION_MINOR              0x07

/* Greybus control request types */
#define MB_CONTROL_TYPE_INVALID               0x00
#define MB_CONTROL_TYPE_PROTOCOL_VERSION      0x01
#define MB_CONTROL_TYPE_GET_IDS               0x02
#define MB_CONTROL_TYPE_REBOOT                0x03
#define MB_CONTROL_TYPE_PORT_CONNECTED        0x04
#define MB_CONTROL_TYPE_PORT_DISCONNECTED     0x05
#define MB_CONTROL_TYPE_SLAVE_POWER           0x06
#define MB_CONTROL_TYPE_GET_ROOT_VER          0x07
#define MB_CONTROL_TYPE_RTC_SYNC              0x08
#define MB_CONTROL_TYPE_SLAVE_STATE           0x09
#define MB_CONTROL_TYPE_SET_CURRENT_LIMIT     0x0a
#define MB_CONTROL_TYPE_CAPABLITY_CHANGED     0x0b
#define MB_CONTROL_TYPE_GET_PWRUP_REASON      0x0c

/* Valid modes for the reboot request */
#define MB_CONTROL_REBOOT_MODE_RESET          0x01
#define MB_CONTROL_REBOOT_MODE_BOOTLOADER     0x02

/* Valid modes for the slave power request */
#define MB_CONTROL_SLAVE_POWER_ON             0x01
#define MB_CONTROL_SLAVE_POWER_OFF            0x02
#define MB_CONTROL_SLAVE_POWER_FLASH_MODE     0x03

/* Control protocol version request has no payload */
struct mb_control_proto_version_response {
    uint8_t     major;
    uint8_t     minor;
} __attribute__ ((packed));

/* Control protocol get_ids request has no payload */
#define MB_CONTROL_FW_VER_STR_SZ              32
struct mb_control_get_ids_response {
    uint32_t    unipro_mfg_id;
    uint32_t    unipro_prod_id;
    uint32_t    ara_vend_id;
    uint32_t    ara_prod_id;
    uint64_t    uid_low;
    uint64_t    uid_high;
    uint32_t    fw_version;
    uint32_t    slave_mask;
    char        fw_version_str[MB_CONTROL_FW_VER_STR_SZ];
    uint8_t     fw_vendor_updates;
} __attribute__ ((packed));

/* Control protocol reboot request */
struct mb_control_reboot_request {
    uint8_t     mode;
} __attribute__ ((packed));

/* Control protocol [dis]connected request */
struct mb_control_connected_request {
    uint8_t     cport_id;
} __attribute__ ((packed));
/* Control protocol [dis]connected response has no payload */

/* Control protocol slave power request */
struct mb_control_power_ctrl_request {
    uint32_t    slave_id;
    uint8_t     mode;
} __attribute__ ((packed));
/* Control protocol slave power response has no payload */

/* Control protocol get root version response */
struct mb_control_root_ver_response {
    uint8_t     version;
} __attribute__ ((packed));

/* Control protocol RTC sync request */
struct mb_control_rtc_sync_request {
    uint64_t    nsec;
} __attribute__ ((packed));
/* Control protocol RTC sync has no response */

/* Control protocol current limit request */
struct mb_control_current_limit_request {
    uint64_t    limit;
} __attribute__ ((packed));
/* Control protocol current limit response has no payload */

struct mb_control_get_pwrup_reason_response {
    uint32_t    reason;
} __attribute__ ((packed));

/* conditionally respond to a message */
static inline int
modsctrl_null_resp(uint32_t cportid, struct gb_operation_msg *msg,  uint8_t status)
{
    int rv = GB_OP_SUCCESS;

    if (msg->hdr.id) {
        rv = greybus_send_response(cportid, &msg->hdr, status, NULL, 0, NULL);
    }
    return rv;
}

static int modsctrl_get_version(uint32_t cportid, struct gb_operation_msg *msg)
{
    unsigned char payload[2] = {MB_CONTROL_VERSION_MAJOR,
                                MB_CONTROL_VERSION_MINOR};

    return greybus_send_response(cportid,
                               &msg->hdr,
                               GB_OP_SUCCESS,
                               payload,
                               sizeof(payload),
                               NULL);
}

/* copy a string from src to target */
/* NOTE: no boundary checking; do not use for inputted strings. */
static inline char *cpstr(char *tgt, const char *src)
{
    size_t i;

    for (i = 0; src[i] != '\0'; i++) {
        tgt[i] = src[i];
    }

    return &tgt[i];
}

static int modsctrl_get_ids(uint32_t cportid, struct gb_operation_msg *msg)
{
    struct mb_control_get_ids_response get_ids_resp;

    get_ids_resp.fw_version = (CONFIG_VERSION_MAJOR << 16 | CONFIG_VERSION_MINOR);
    get_chip_id(&get_ids_resp.unipro_mfg_id, &get_ids_resp.unipro_prod_id);
    get_board_id(&get_ids_resp.ara_vend_id, &get_ids_resp.ara_prod_id);
    get_chip_uid(&get_ids_resp.uid_high, &get_ids_resp.uid_low);
#ifdef CONFIG_SLAVE_APBE
    get_ids_resp.slave_mask = 1;
    dbgprint("MODCTRL:SLAVE MASK SET\r\n");
#else
    get_ids_resp.slave_mask = 0;
#endif

    cpstr(get_ids_resp.fw_version_str, CONFIG_VERSION_STRING " " CONFIG_VERSION_BUILD);

#ifdef CONFIG_GREYBUS_MODS_SUPPORT_VENDOR_UPDATES
    get_ids_resp.fw_vendor_updates = true;
#else
    get_ids_resp.fw_vendor_updates = false;
#endif

    return greybus_send_response(cportid,
                               &msg->hdr,
                               GB_OP_SUCCESS,
                               (unsigned char *)&get_ids_resp,
                               sizeof(get_ids_resp),
                               NULL);
}

static int modsctrl_reboot(uint32_t cportid, struct gb_operation_msg *msg)
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

static inline int
modsctrl_connected(uint32_t cportid, struct gb_operation_msg *msg)
{
    return modsctrl_null_resp(cportid, msg,  GB_OP_SUCCESS);
}

static inline int
modsctrl_disconnected(uint32_t cportid, struct gb_operation_msg *msg)
{
    return modsctrl_null_resp(cportid, msg,  GB_OP_SUCCESS);
}

static int modsctrl_slave_power(uint32_t cportid, struct gb_operation_msg *msg)
{
    int ret = 0;
#ifdef MOD_SLAVE_APBE
    struct mb_control_power_ctrl_request *req =
        (struct mb_control_power_ctrl_request *)msg->data;

    ret = slave_pwrctrl_set_mode(req->mode);
#else
    ret = modsctrl_null_resp(cportid, msg, GB_OP_INVALID);
#endif
    return ret ? GB_OP_UNKNOWN_ERROR : GB_OP_SUCCESS;
}

static int modsctrl_root_version(uint32_t cportid, struct gb_operation_msg *msg)
{
    struct mb_control_root_ver_response resp = { CONFIG_ROOT_VERSION };

    return greybus_send_response(cportid,
            &msg->hdr,
            GB_OP_SUCCESS,
            (unsigned char *)&resp,
            sizeof(resp),
            NULL);
}

static inline int
modsctrl_rtc_sync(uint32_t cportid, struct gb_operation_msg *msg)
{
    return modsctrl_null_resp(cportid, msg,  GB_OP_SUCCESS);
}

static inline int
modsctrl_set_current_limit(uint32_t cportid, struct gb_operation_msg *msg)
{
    return modsctrl_null_resp(cportid, msg,  GB_OP_SUCCESS);
}

static inline int
modsctrl_get_pwrup_reason(uint32_t cportid, struct gb_operation_msg *msg)
{
    struct mb_control_get_pwrup_reason_response resp;

    resp.reason = get_flash_reason();

    return greybus_send_response(cportid,
            &msg->hdr,
            GB_OP_SUCCESS,
            (unsigned char *)&resp,
            sizeof(resp),
            NULL);
}

int mods_control_handler(uint32_t cportid, void *data, size_t len)
{
    int rc = 0;
    struct gb_operation_msg *msg;

    if (len < sizeof(gb_operation_header)) {
        dbgprint("mods_control_handler: RX data length error\r\n");
        return -1;
    }

    msg = (struct gb_operation_msg *)data;
    switch (msg->hdr.type) {
    case MB_CONTROL_TYPE_PROTOCOL_VERSION:
        rc = modsctrl_get_version(cportid, msg);
        break;
    case MB_CONTROL_TYPE_GET_IDS:
        rc = modsctrl_get_ids(cportid, msg);
        break;
    case MB_CONTROL_TYPE_REBOOT:
        rc = modsctrl_reboot(cportid, msg);
        break;
    case MB_CONTROL_TYPE_PORT_CONNECTED:
        rc = modsctrl_connected(cportid, msg);
        break;
    case MB_CONTROL_TYPE_PORT_DISCONNECTED:
        rc = modsctrl_disconnected(cportid, msg);
        break;
    case MB_CONTROL_TYPE_SLAVE_POWER:
        rc = modsctrl_slave_power(cportid, msg);
        break;
    case MB_CONTROL_TYPE_GET_ROOT_VER:
        rc = modsctrl_root_version(cportid, msg);
        break;
    case MB_CONTROL_TYPE_RTC_SYNC:
        return modsctrl_rtc_sync(cportid, msg);
        break;
    case MB_CONTROL_TYPE_SET_CURRENT_LIMIT:
        return modsctrl_set_current_limit(cportid, msg);
        break;
    case MB_CONTROL_TYPE_GET_PWRUP_REASON:
        return modsctrl_get_pwrup_reason(cportid, msg);
        break;
    default:
        rc = modsctrl_null_resp(cportid, msg, GB_OP_INVALID);
        break;
    }

    return rc;
}
