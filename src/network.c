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
#include <stdint.h>
#include <debug.h>

#include "greybus.h"
#include "network.h"
#include "datalink.h"

struct mods_nw_hdr {
    uint16_t  cport;
} __attribute__ ((packed));

struct mods_nw_msg
{
    struct mods_nw_hdr hdr;
    uint8_t   payload[0];
} __attribute__ ((packed));

uint16_t network_get_max_payload_size(void)
{
    return datalink_get_max_payload_size() - sizeof(struct mods_nw_hdr);
}

int network_send(uint32_t cport, uint8_t *buf, size_t len, msg_sent_cb cb)
{
    struct mods_nw_msg *nw = (struct mods_nw_msg *)&buf[-sizeof(struct mods_nw_hdr)];
    nw->hdr.cport = cport;

    return datalink_send((uint8_t *)nw, len + sizeof(struct mods_nw_hdr), cb, NULL);
}

/* TODO: switch should be part of greybus core */
int network_recv(const void *msg, size_t len)
{
    struct mods_nw_msg *m_msg = (struct mods_nw_msg *)msg;
    int rc = 0;

    switch(m_msg->hdr.cport) {
    case CONTROL_CPORT:
      rc = control_cport_handler(m_msg->hdr.cport, m_msg->payload,
                    len - sizeof(struct mods_nw_hdr));
      if (rc != GB_OP_SUCCESS) {
        dbgprint("mods_control_handler failed\r\n");
        goto protocol_error;
      }
      break;
    case FIRMWARE_CPORT:
      rc = fw_cport_handler(m_msg->hdr.cport, m_msg->payload,
                    len - sizeof(struct mods_nw_hdr));
      if (rc != GB_OP_SUCCESS) {
        dbgprint("fw_cport_handler failed\r\n");
        goto protocol_error;
      }
      break;
    case MODS_CONTROL_CPORT:
      rc = mods_control_handler(m_msg->hdr.cport, m_msg->payload,
                    len - sizeof(struct mods_nw_hdr));
      if (rc != GB_OP_SUCCESS) {
        dbgprint("mods_control_handler failed\r\n");
        goto protocol_error;
      }
      break;
    default:
      dbgprintx32("ERROR received message for cport ", m_msg->hdr.cport, "\r\n");
      break;
    }

protocol_error:
    return rc;
}

