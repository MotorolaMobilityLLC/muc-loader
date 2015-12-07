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
#include <stdbool.h>
#include "boot_main.h"
#include "debug.h"
#include "greybus.h"


/**
 * @brief send data down a CPort
 * @param cportid cport to send down
 * @param buf data buffer
 * @param len size of data to send
 * @param 0 on success, <0 on error
 */
int chip_unipro_send(unsigned int cportid, const void *buf, size_t len) {

    struct mods_spi_msg *spi_msg = (struct mods_spi_msg *)buf;

    if (spi_msg == NULL) {
        return -1;
    }

    spi_msg->hdr_bits |= HDR_BIT_VALID;
    spi_msg->hdr_bits |= MSG_TYPE_NW;
    spi_msg->m_msg.cport = cportid;

    respReady = true;

    return 0;
}

int chip_unipro_receive(unsigned int cportid, unipro_rx_handler handler) {
    struct mods_spi_msg *spi_msg = (struct mods_spi_msg *)&aRxBuffer[0];
    if (handler != NULL) {
        if(0 != handler(cportid,
                        spi_msg->m_msg.gb_op_hdr,
                        negotiated_pl_size - NW_HEADER_SIZE)) {
            dbgprint("RX handler returned error\r\n");
            return -1;
        }
    }
    return 0;
}

