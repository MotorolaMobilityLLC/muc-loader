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

#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <chipapi.h>
#include "debug.h"
#include "boot_main.h"
#include "greybus.h"
#include "datalink.h"
#include "stm32l4xx_hal.h"
#include "tftf.h"
#include "stm32l4xx_flash.h"
#include "stm32_hal_mod.h"
#include "spi_flash_write.h"

/* Greybus FirmWare protocol version we support */
#define GB_FIRMWARE_VERSION_MAJOR   0x00
#define GB_FIRMWARE_VERSION_MINOR   0x01

/* Greybus FirmWare request types */
#define GB_FW_OP_INVALID            0x00
#define GB_FW_OP_PROTOCOL_VERSION   0x01
#define GB_FW_OP_FIRMWARE_SIZE      0x02
#define GB_FW_OP_GET_FIRMWARE       0x03
#define GB_FW_OP_READY_TO_BOOT      0x04
#define GB_FW_OP_AP_READY           0x05 /* Unidirectional request with no-payload */

/* Greybus FirmWare boot statuses */
#define GB_FW_BOOT_STATUS_INVALID   0x00
#define GB_FW_BOOT_STATUS_INSECURE  0x01
#define GB_FW_BOOT_STATUS_SECURE    0x02

/* Greybus FirmWare boot instructions */
#define GB_FW_BOOT_INSTR_FAILURE   0x00
#define GB_FW_BOOT_INSTR_OK        0x01

/* Greybus FirmWare error codes */
#define GB_FW_ERR_INVALID          (-1)
#define GB_FW_ERR_FAILURE          (-2)

/* Boot stage whose firmware we request */
#define GBFW_STAGE_MIN             0x01
#define GBFW_STAGE_MAX             0x03

#define GBFW_STAGE_BOOTLOADER      0x01
#define GBFW_STAGE_APBE_SPI_FLASH  0x02
#define GBFW_STAGE_MAIN            0x03

/* number of attempts to write flash before reset */
#define GBFW_MAX_RETRIES              3

/* Greybus FirmWare request and response payloads */
struct gbfw_protocol_version_request {
  uint8_t major, minor;
} __attribute__ ((__packed__));

struct gbfw_firmware_size_request {
  uint8_t stage;
} __attribute__ ((__packed__));

struct gbfw_firmware_size_response {
  size_t size;
} __attribute__ ((__packed__));

struct gbfw_get_firmware_request {
  uint32_t offset, size;
} __attribute__ ((__packed__));

struct gbfw_ready_to_boot_request {
  uint8_t status;
} __attribute__ ((__packed__));

#define MUCLOADER_BASE_ADDR     FLASH_BASE

static bool tftf_header_received = false;
static uint8_t tftf_buff[TFTF_HEADER_SIZE];
static uint16_t section_index = 0;

static struct fw_flash_data {
    uint32_t fw_flash_addr;
    uint32_t fw_request_size;
    uint32_t fw_offset;
    int32_t fw_remaining_size;
    uint32_t new_payload_size;
} fw_flash_data;

static uint8_t _gbfw_stage = 0x00;  /* current flashing stage */
static uint8_t _gbfw_updated_count = 0; /* number of stages successfully flashed */
extern uint32_t gbfw_cportid;  /* cport id on core for gbfw requests */

static int gbfw_ready_to_boot(uint8_t status);
static void gbfw_next_stage(void);

static int gbfw_get_version(uint32_t cportid, gb_operation_header *header)
{
    uint8_t payload[2] = {GB_FIRMWARE_VERSION_MAJOR, GB_FIRMWARE_VERSION_MINOR};

    return greybus_send_response(cportid, header, GB_OP_SUCCESS, payload,
                                 sizeof(payload), NULL);
}

static void gbfw_ap_ready_cb(int status, void *cntx)
{
    _gbfw_stage = 0;
    _gbfw_updated_count = 0;
    gbfw_next_stage();
}

static int gbfw_ap_ready(uint32_t cportid, gb_operation_header *header)
{
    return greybus_send_response(cportid, header, GB_OP_SUCCESS, NULL, 0, gbfw_ap_ready_cb);
}

static struct gbfw_firmware_size_response firmware_size_response;
static int gbfw_firmware_size_response(gb_operation_header *head,
                                       void *data, uint32_t len);

static int gbfw_firmware_size(uint8_t stage)
{
    int rc;
    struct gbfw_firmware_size_request req = {stage};
    uint16_t msg_id = greybus_get_next_id();

    rc = greybus_send_request(gbfw_cportid, msg_id, GB_FW_OP_FIRMWARE_SIZE,
                              (uint8_t*)&req, sizeof(req), NULL);
    if (rc) {
        return rc;
    }

    return 0;
}

static void gbfw_next_stage(void)
{
    uint8_t valid_stage_mask = 0x00;

    memset(&fw_flash_data, 0, sizeof(fw_flash_data));
    tftf_header_received = false;

    if (!chip_bootloader_is_readonly()) {
        valid_stage_mask |= 1 << GBFW_STAGE_BOOTLOADER;
    }
#ifdef CONFIG_APBE_FLASH
    valid_stage_mask |= 1 << GBFW_STAGE_APBE_SPI_FLASH;
#endif
    valid_stage_mask |= 1 << GBFW_STAGE_MAIN;

    while (_gbfw_stage < GBFW_STAGE_MAX) {
        if ((1 << (_gbfw_stage + 1)) & valid_stage_mask) {
            gbfw_firmware_size(++_gbfw_stage);
            return;
        }
        _gbfw_stage++;
    }

    if (_gbfw_stage >= GBFW_STAGE_MAX)
        gbfw_ready_to_boot(GB_FW_BOOT_STATUS_SECURE);
}

#ifdef CONFIG_APBE_FLASH
static bool gbfw_is_apbe_flash_stage(void)
{
   return _gbfw_stage == GBFW_STAGE_APBE_SPI_FLASH ? true : false;
}
#endif

static int gbfw_firmware_size_response(gb_operation_header *head, void *data,
                                       uint32_t len)
{
    size_t fw_size;

    if (head->status) {
#ifdef CONFIG_DEBUG_FIRMWARE
        dbgprint("no fw for current stage\r\n");
#endif
        gbfw_next_stage();
        return 0;
    }

    if (len < sizeof(struct gbfw_firmware_size_response)) {
        dbgprint("gbfw_firmware_size_response: bad response size\r\n");
        return GB_FW_ERR_INVALID;
    }
    memcpy(&firmware_size_response, data, sizeof(firmware_size_response));

    fw_size = firmware_size_response.size;

    if (fw_size > TFTF_HEADER_SIZE) {
        int rc;
        struct gbfw_get_firmware_request req = {0, TFTF_HEADER_SIZE};
        uint16_t msg_id = greybus_get_next_id();
#ifdef CONFIG_APBE_FLASH
        if (gbfw_is_apbe_flash_stage()) {
            req.offset = TFTF_HEADER_SIZE;
            firmware_size_response.size -= TFTF_HEADER_SIZE;
        }
#endif
        rc = greybus_send_request(gbfw_cportid, msg_id, GB_FW_OP_GET_FIRMWARE,
                                  (uint8_t*)&req, sizeof(req), NULL);
        if (rc) {
            return rc;
        }
        fw_flash_data.fw_request_size = TFTF_HEADER_SIZE;
    } else {
        dbgprint("skip empty firmware");
        gbfw_next_stage();
    }

    return 0;
}

static int gbfw_get_firmware(uint32_t offset, uint32_t size)
{
    int rc;
    struct gbfw_get_firmware_request req = {offset, size};
    uint16_t msg_id = greybus_get_next_id();

    rc = greybus_send_request(gbfw_cportid, msg_id, GB_FW_OP_GET_FIRMWARE,
                              (uint8_t*)&req, sizeof(req), NULL);
    if (rc) {
        return rc;
    }

    return 0;
}

static inline uint16_t gbfw_get_payload_size(void)
{
    return greybus_get_max_payload_size();
}

static int gbfw_get_firmware_response(gb_operation_header *header, void *data,
                                      uint32_t len)
{
    uint8_t *data_ptr;
    uint32_t flash_addr;
    uint32_t flash_data_size;
    uint32_t pl_data_size;
    int err = 0;
    int err_count = 0;

    tftf_header *tf_header = (tftf_header *)data;
    data_ptr = (uint8_t *)data;

    if (tftf_header_received == false) {
        if (!valid_tftf_header(tf_header)) {
            dbgprint("INVALID_TFTF");
            return GB_FW_ERR_INVALID;
        }

        memcpy(tftf_buff, data_ptr, sizeof(tftf_header));
        tftf_header_received = true;

        /* flash the raw code and any attached sections*/
        section_index = get_section_index(TFTF_SECTION_RAW_CODE, &tf_header->sections[0]);
        if (section_index == TFTF_SECTION_END) {
            dbgprint("No sections to flash");
            return GB_FW_ERR_INVALID;
        }

        if (set_flashing_flag()) {
            dbgprint("Failed to set flashing flag\r\n");
            HAL_NVIC_SystemReset();
        }

        /* erase image copy address and size */
        fw_flash_data.fw_flash_addr = tf_header->sections[section_index].section_load_address;
        fw_flash_data.fw_remaining_size = firmware_size_response.size - TFTF_HEADER_SIZE;
        fw_flash_data.fw_offset = 0;
        fw_flash_data.new_payload_size = 0;
#ifdef CONFIG_APBE_FLASH
        if (gbfw_is_apbe_flash_stage()) {
             /*
              * apbe stage 2 image is wrapped with a TFTF header with muc id's
              * and a muc signature section, need to skip requesting muc signature
              * and muc tftf header, so initialize offset to TFTF_HEADER_SIZE, and
              * remaining size to difference between total firmware size and apbe
              * image size.
              */
             fw_flash_data.fw_offset = TFTF_HEADER_SIZE;
             fw_flash_data.fw_remaining_size -= ((firmware_size_response.size - TFTF_HEADER_SIZE) -
                                                 spi_write_calc_total_len(data));
             do {
                 err = spi_write_to_flash_header(&spi_write_ops, data);
                 if (err) {
                     dbgprint("spi_write_to_flash_header error\r\n");
                     err_count++;
                 }
             } while (err && err_count < GBFW_MAX_RETRIES);

             if (err)
                 HAL_NVIC_SystemReset();
             else
                 err_count = 0;
        } else
#endif
        {
            if (_gbfw_stage == GBFW_STAGE_MAIN) {
                erase_tftf_header();
            }
            flash_erase(fw_flash_data.fw_flash_addr, fw_flash_data.fw_remaining_size);

            /* program the first packet */
            data_ptr += sizeof(tftf_header);

            flash_addr = fw_flash_data.fw_flash_addr + fw_flash_data.fw_offset;
            flash_data_size = fw_flash_data.fw_request_size - TFTF_HEADER_SIZE;

#ifdef CONFIG_DEBUG_FIRMWARE
            dbgprint("FLADR:");
            dbgprinthex32(flash_addr);
            dbgprint("\r\n");
#endif

             do {
                 err = program_flash_data(flash_addr, flash_data_size, data_ptr);
                 if (err) {
                     dbgprint("program_flash_data (hdr) error\r\n");
                     err_count++;
                }
             } while (err && err_count < GBFW_MAX_RETRIES);

             if (err)
                 HAL_NVIC_SystemReset();
             else
                 err_count = 0;
        }

        fw_flash_data.fw_offset += (fw_flash_data.fw_request_size - TFTF_HEADER_SIZE);
        fw_flash_data.fw_remaining_size -= (fw_flash_data.fw_request_size - TFTF_HEADER_SIZE);

#ifdef CONFIG_DEBUG_FIRMWARE
        dbgprint("off:");
        dbgprinthex32(fw_flash_data.fw_offset + TFTF_HEADER_SIZE);
        dbgprint("\r\n");
#endif

        /* Calculate the size for get firmware request */
        pl_data_size = gbfw_get_payload_size();

        /* data size should be multiple of uint64_t */
        fw_flash_data.fw_request_size =
                 pl_data_size - (pl_data_size % sizeof(uint64_t));
    } else {
        tf_header = (tftf_header *)tftf_buff;
#ifdef CONFIG_APBE_FLASH
        if (gbfw_is_apbe_flash_stage()) {
             do {
                 err = spi_write_to_flash_data(&spi_write_ops, data,
                                               fw_flash_data.new_payload_size);
                 if (err) {
                     dbgprint("spi_write_to_flash_data error\r\n");
                     err_count++;
                 }
             } while (err && err_count < GBFW_MAX_RETRIES);

             if (err)
                 HAL_NVIC_SystemReset();
             else
                 err_count = 0;

             flash_data_size = fw_flash_data.new_payload_size;
        } else
#endif
        {
            flash_addr = fw_flash_data.fw_flash_addr + fw_flash_data.fw_offset;
            flash_data_size = fw_flash_data.new_payload_size;

#ifdef CONFIG_DEBUG_FIRMWARE
            dbgprint("FLADR:");
            dbgprinthex32(flash_addr);
            dbgprint("\r\n");
            dbgprint("PLSZ:");
            dbgprinthex32(flash_data_size);
            dbgprint("\r\n");
#endif

            do {
                err = program_flash_data(flash_addr, flash_data_size, data_ptr);
                if (err) {
                    dbgprint("program_flash_data error\r\n");
                    err_count++;
                }
             } while (err && err_count < GBFW_MAX_RETRIES);

             if (err)
                 HAL_NVIC_SystemReset();
             else
                 err_count = 0;
        }

        fw_flash_data.fw_offset += flash_data_size;
        fw_flash_data.fw_remaining_size -= flash_data_size;

#ifdef CONFIG_DEBUG_FIRMWARE
        dbgprint("off:");
        dbgprinthex32(fw_flash_data.fw_offset + TFTF_HEADER_SIZE);
        dbgprint("\r\n");
        dbgprint("Rem:");
        dbgprinthex32(fw_flash_data.fw_remaining_size);
        dbgprint("\r\n");
#endif
    }

    if (fw_flash_data.fw_remaining_size > 0) {
        if (fw_flash_data.fw_remaining_size < fw_flash_data.fw_request_size) {
            fw_flash_data.new_payload_size = fw_flash_data.fw_remaining_size;
        } else {
            fw_flash_data.new_payload_size = fw_flash_data.fw_request_size;
        }

#ifdef CONFIG_DEBUG_FIRMWARE
        dbgprint("Req:");
        dbgprinthex32(fw_flash_data.new_payload_size);
        dbgprint("\r\n");
#endif
        gbfw_get_firmware(fw_flash_data.fw_offset + TFTF_HEADER_SIZE,
                          fw_flash_data.new_payload_size);
    } else {
        /* No more data to write */

#ifdef CONFIG_APBE_FLASH
        if (gbfw_is_apbe_flash_stage())
            spi_write_to_flash_finish(&spi_write_ops);
        else
#endif
        {

            program_flash_lock();
        }

        /* now that we are done flashing, if this is the main stage,
         * then we want to update the tf header section */
        if (_gbfw_stage == GBFW_STAGE_MAIN) {
            if (valid_tftf_header(tf_header)) {
                program_tftf_header(tftf_buff, sizeof(tftf_buff));
            }
        }
        _gbfw_updated_count++;
        gbfw_next_stage();
    }
    return 0;
}

static void gbfw_ready_to_boot_cb(int status, void *cntx)
{
#if CONFIG_ROOT_VERSION == 0
    HAL_NVIC_SystemReset();
#endif
}

static int gbfw_ready_to_boot(uint8_t status)
{
    int rc;
    struct gbfw_ready_to_boot_request req = {status};
    uint16_t msg_id = greybus_get_next_id();

    /* Erase the Flash Mode Barker */
    if (_gbfw_updated_count > 0) {
#ifdef CONFIG_DEBUG_FIRMWARE
        dbgprintx32("updated ", _gbfw_updated_count, " files\r\n");
#endif
        clr_flash_barker();
    }

    /* ready_to_boot currently doesn't respond so fake a response */
    rc = greybus_send_request(gbfw_cportid, msg_id, GB_FW_OP_READY_TO_BOOT,
                      (uint8_t*)&req, sizeof(req), gbfw_ready_to_boot_cb);

    return rc;
}

static int gbfw_ready_to_boot_response(gb_operation_header *header,
                                       void *data, uint32_t len)
{
    if (header->status) {
        dbgprint("gbfw_ready_to_boot_response(): got error status\r\n");
        return -header->status;
    }
    return 0;
}

int fw_cport_handler(uint32_t cportid, void *data, size_t len)
{
    int rc = 0;
    uint8_t *data_ptr;

    if (cportid != gbfw_cportid) {
        dbgprint("fw_cport_handler: incorrect CPort #\r\n");
        return GB_FW_ERR_INVALID;
    }

    if (len < sizeof(gb_operation_header)) {
        dbgprint("fw_cport_handler: RX data length error\r\n");
        return GB_FW_ERR_INVALID;
    }

    gb_operation_header *op_header = (gb_operation_header *)data;
    data_ptr = (uint8_t *)data;

    if (op_header->size > len) {
        dbgprint("fw_cport_handler: payload size > total size.\r\n");
        dbgprintx32("  total_size (len) = 0x", len, "\r\n");
        dbgprintx32("      payload_size = 0x", op_header->size, "\r\n");
        return GB_FW_ERR_INVALID;
    }

    if (op_header->type & GB_TYPE_RESPONSE && op_header->status) {
        dbgprintx32("fw_cport_handler: Greybus response, status 0x",
                   op_header->status, "\r\n");
    }
    /*
     * This works here because we're actually calling this handler
     * synchronously.  We set the constant and return to the recipient rather
     * than setting the constant in parallel with the recipient.
     */
    data_ptr += sizeof(gb_operation_header);
    len -= sizeof(gb_operation_header);
    switch (op_header->type) {
    case GB_FW_OP_PROTOCOL_VERSION:
        rc = gbfw_get_version(cportid, op_header);
        break;
    case GB_FW_OP_FIRMWARE_SIZE | GB_TYPE_RESPONSE:
        rc = gbfw_firmware_size_response(op_header, data_ptr, len);
        break;
    case GB_FW_OP_GET_FIRMWARE | GB_TYPE_RESPONSE:
        rc = gbfw_get_firmware_response(op_header, data_ptr, len);
        break;
    case GB_FW_OP_READY_TO_BOOT | GB_TYPE_RESPONSE:
        rc = gbfw_ready_to_boot_response(op_header, data_ptr, len);
        break;
    case GB_FW_OP_AP_READY:
        rc = gbfw_ap_ready(cportid, op_header);
        break;
    default:
        dbgprint("FWDFLT\r\n");
        break;
    }

    return rc;
}
