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

#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include "debug.h"
#include "boot_main.h"
#include "greybus.h"
#include "gbfirmware.h"
#include "datalink.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_flash.h"
#include "tftf.h"

extern uint8_t responded_op;
extern int cport_connected;
extern int fw_cport_handler(uint32_t cportid, void *data, size_t len);

uint32_t agreed_pl_size = 0;
volatile uint8_t mesg_sent = false;

typedef enum {
    datalink,
    control,
    firmware
} e_protocol_type;

static e_protocol_type protocol_type = datalink;

void dl_init(void)
{
  protocol_type = datalink;
}

static int dl_send_message(uint8_t id,
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

struct dl_muc_bus_config_response payload = {24000000, MAX_NW_PL_SIZE};

static int dl_bus_config(struct dl_payload_msg *dl_msg) {
    uint16_t recved_max_pl_size;

    memcpy(&recved_max_pl_size, &dl_msg->dl_pl[0], sizeof(uint16_t));
    payload.max_bus_speed = 0;

    if(recved_max_pl_size < MAX_NW_PL_SIZE) {
        payload.sel_payload_size = recved_max_pl_size;
    } else {
        payload.sel_payload_size = MAX_NW_PL_SIZE;
    }

    agreed_pl_size = payload.sel_payload_size;
    return dl_send_message(dl_msg->mesg_id, GB_TYPE_RESPONSE, (uint8_t *)&payload,
                            sizeof(payload));
}

int dl_muc_handler(struct dl_payload_msg *dl_msg) {

    int rc = 0;

    if (dl_msg->mesg_id & GB_TYPE_RESPONSE) {
        return GB_FW_ERR_FAILURE;
    }

    responded_op = dl_msg->mesg_id;
    switch (dl_msg->mesg_id) {
    case DL_MUC_OP_BUS_CONFIG:
        dbgprint("BUSCGF\r\n");
        rc = dl_bus_config(dl_msg);
        responded_op = DL_MUC_OP_BUS_CONFIG_RESP;
        break;
    default:
        dbgprint("DLDFLT\r\n");
        responded_op = GB_FW_OP_INVALID;
        break;
    }

    return rc;
}

int process_mods_dl_msg(struct dl_payload_msg *dl_msg)
{
    int rc = 0;

    protocol_type = datalink;
    if(cport_connected == 0) {
        rc = dl_muc_handler(dl_msg);
    } else {
        dbgprint("NO DL HNDL\r\n");
    }
    return rc;
}

/* TODO: switch should be part of greybus core */
int process_mods_msg(struct mods_msg *m_msg)
{
    int rc = 0;

    /* poll until data cport connected */
    if (!manifest_fetched_by_ap() || (cport_connected == 0)) {
        protocol_type = control;
        if (m_msg->cport == MODS_CONTROL_CPORT) {
            rc = chip_unipro_receive(MODS_CONTROL_CPORT, mods_control_handler);
            if (rc == GB_FW_ERR_INVALID) {
               dbgprint("Greybus init failed\r\n");
               if (rc) {
                   goto protocol_error;
               }
            }
        } else {
           rc = chip_unipro_receive(CONTROL_CPORT, control_cport_handler);
           if (rc == GB_FW_ERR_INVALID) {
                dbgprint("Greybus init failed\r\n");
                if (rc) {
                    goto protocol_error;
                }
           }
        }
    } else if(responded_op != GB_FW_OP_READY_TO_BOOT) {
        protocol_type = firmware;
        rc = chip_unipro_receive(gbfw_cportid, fw_cport_handler);
        if (rc) {
            dbgprint("Greybus FW CPort handler failed\r\n");
            goto protocol_error;
        }
    } else {
        dbgprint("NO HNDL\r\n");
    }

    return rc;

protocol_error:
    cport_connected = 0;
    return rc;
}

int process_sent_complete(void)
{
    int rc;

    if (protocol_type == datalink) {
        dbgprint("protocol_type==datalink\r\n");
        if (responded_op == DL_MUC_OP_BUS_CONFIG_RESP) {
            armDMAtype = full;
            negotiated_pl_size = payload.sel_payload_size;
            return 0;
        }
    }

    if (protocol_type == firmware) {
        dbgprint("protocol_type==firmware\r\n");
        if (responded_op == GB_FW_OP_READY_TO_BOOT) {
            dbgprint("READY TO BOOT\r\n");
#if CONFIG_ROOT_VERSION == 0
            HAL_NVIC_SystemReset();
#endif
            return 0;
        }
    }

    if (responded_op != GB_FW_OP_AP_READY)
        return 0;
    if (mesg_sent == true)
        return 0;

    /* Fetch the firmware size. */
    dbgprint("FWSIZE\r\n");
    rc = gbfw_firmware_size(GBFW_STAGE_MIN);
    if (rc) {
        goto protocol_error;
    }
    mesg_sent = true;

    return 0;

protocol_error:
    cport_connected = 0;
    return rc;
}
