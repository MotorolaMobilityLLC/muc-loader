/**
 * Copyright (c) 2015 Google Inc.
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

#ifndef __COMMON_INCLUDE_GREYBUS_H
#define __COMMON_INCLUDE_GREYBUS_H

#include <stddef.h>
#include <stdbool.h>

#define GREYBUS_MAJOR_VERSION        0
#define GREYBUS_MINOR_VERSION        1

typedef struct {
    uint16_t size;
    uint16_t id;
    uint8_t  type;
    uint8_t  status;
    uint16_t padding;
} __attribute__ ((packed)) gb_operation_header;

#define GB_TYPE_RESPONSE  0x80

#define GB_CTRL_OP_VERSION           0x01
#define GB_CTRL_OP_PROBE_AP          0x02
#define GB_CTRL_OP_GET_MANIFEST_SIZE 0x03
#define GB_CTRL_OP_GET_MANIFEST      0x04
#define GB_CTRL_OP_CONNECTED         0x05
#define GB_CTRL_OP_DISCONNECTED      0x06

#define GB_OP_SUCCESS                0x00
#define GB_OP_INVALID                0x06
#define GB_OP_UNKNOWN_ERROR          0xFE

#define GB_MAX_PAYLOAD_SIZE          (0x800 - 2*sizeof(gb_operation_header))

#define CONTROL_CPORT 0

#define MODS_CONTROL_CPORT 0xFFFF

/* Version of the Greybus control protocol we support */
#define MB_CONTROL_VERSION_MAJOR              0x00
#define MB_CONTROL_VERSION_MINOR              0x01

/* Greybus control request types */
#define MB_CONTROL_TYPE_INVALID               0x00
#define MB_CONTROL_TYPE_PROTOCOL_VERSION      0x01
#define MB_CONTROL_TYPE_GET_IDS               0x02
#define MB_CONTROL_TYPE_REBOOT                0x03

/* Valid modes for the reboot request */
#define MB_CONTROL_REBOOT_MODE_RESET          0x01
#define MB_CONTROL_REBOOT_MODE_BOOTLOADER     0x02

#define HDR_BIT_VALID  (0x01 << 7)  /* 1 = valid packet, 0 = dummy packet */
#define HDR_BIT_TYPE   (0x01 << 6)  /* SPI message type, datalink or network */
#define HDR_BIT_PKTS   (0x3F << 0)  /* How many additional packets to expect */

#define MSG_TYPE_DL    (0 << 6)     /* Packet for/from data link layer */
#define MSG_TYPE_NW    (1 << 6)     /* Packet for/from network layer */

struct __attribute__ ((packed)) mods_msg
{
    uint16_t  cport;
    uint8_t gb_op_hdr[];
};

struct __attribute__ ((packed)) dl_payload_msg
{
    uint8_t  mesg_id;
    uint8_t  dl_pl[];
};

#ifndef __GNUC__
#pragma anon_unions
#endif
struct __attribute__ ((packed))mods_spi_msg
{
    uint8_t  hdr_bits;
    uint8_t  reserved;
    union {
        struct  mods_msg m_msg;
        struct  dl_payload_msg dl_msg;
    };
};

/* Control protocol get_ids request has no payload */
struct __attribute__ ((packed)) gb_control_get_ids_response {
    uint32_t    unipro_mfg_id;
    uint32_t    unipro_prod_id;
    uint32_t    ara_vend_id;
    uint32_t    ara_prod_id;
    uint64_t    uid_low;
    uint64_t    uid_high;
    uint32_t    fw_version;
};

int control_cport_handler(uint32_t cportid,
                          void *data,
                          size_t len);

int mods_control_handler(uint32_t cportid,
                          void *data,
                          size_t len);

int greybus_op_response(uint32_t cport,
                        gb_operation_header *op_header,
                        uint8_t status,
                        unsigned char *payload_data,
                        uint16_t payload_size);

int greybus_send_request(uint32_t cport,
                         uint16_t id,
                         uint8_t type,
                         unsigned char *payload_data,
                         uint16_t payload_size);
int dl_send_message(uint8_t id,
                                uint8_t status,
                                unsigned char *payload_data,
                                uint16_t payload_size);
bool manifest_fetched_by_ap(void);
int process_mods_msg(struct mods_msg *m_msg);
int process_mods_dl_msg(struct dl_payload_msg *dl_msg);
int greybus_processing(void);
typedef enum {initial, full} e_armDMAtype;
typedef enum {datalink, control, firmware} e_protocol_type;

#define MAX_NW_PL_SIZE          2048 /* cport + GB header + DL payload */
#define MAX_DMA_BUF_SIZE        MAX_NW_PL_SIZE + DL_HEADER_BITS_SIZE
#define DL_HEADER_BITS_SIZE     2
#define NW_HEADER_SIZE          2    /* cport */
#define GB_HEADER_SIZE          (sizeof(gb_operation_header))
#define INITIAL_DMA_BUF_SIZE    32

#endif /* __COMMON_INCLUDE_GREYBUS_H */
