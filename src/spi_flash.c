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

#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include "debug.h"
#include "boot_main.h"
#include "stm32l4xx_hal.h"
#include "tftf.h"
#include "ffff.h"
#include <utils.h>
#include "spi_flash_write.h"
#include <stm32_hal_mod.h>

#define FLASH_START  0x0000

/* Buffer used for transmission, reception */
uint8_t aTxBuffer_spiFlash[MAX_DMA_BUF_SIZE];
uint8_t aRxBuffer_spiFlash[MAX_DMA_BUF_SIZE];
static SPI_HandleTypeDef hspi_flash;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

enum {
    TRANSFER_WAIT,
    TRANSFER_COMPLETE,
    TRANSFER_ERROR
};

static volatile uint32_t wTransferState = TRANSFER_WAIT;
static uint32_t dst = FLASH_START;
static uint32_t total_section_length;

void spi_flash_hal_init(void)
{
    device_spi_flash_init(&hspi_flash);
}

int spi_write_calc_total_len(void *data)
{
    tftf_header *tf_header = (tftf_header *)data;
    tftf_section_descriptor *section = tf_header->sections;
    uint32_t num_sections = 0;

    total_section_length = 0;
    while (section->section_type != TFTF_SECTION_END) {
        total_section_length += section->section_length;
        num_sections++;
        section++;
    }
#ifdef CONFIG_DEBUG_SPI_FLASH
    dbgprintx32("spi flash tftf num_sections ",  num_sections, "\r\n");
    dbgprintx32("spi flash tftf total_section_length ", total_section_length, "\r\n");
#endif

    return total_section_length;
}

int spi_write_to_flash_header(const data_write_ops *ops, void *data)
{
    int result = 0;
    tftf_header *tf_header = (tftf_header *)data;
    ffff_header tmp_header;
    const uint32_t header0_start = dst;
    const uint32_t header1_start = header0_start + sizeof(tmp_header);
    const uint32_t tftf_start = header1_start + sizeof(tmp_header);

    result = ops->init();
    if (result) {
        goto error;
    }

    memset(&tmp_header, 0, sizeof(tmp_header));
    memcpy(tmp_header.sentinel_value, FFFF_SENTINEL_VALUE,
                                          sizeof(tmp_header.sentinel_value));
    memcpy(tmp_header.build_timestamp, tf_header->build_timestamp,
                                            sizeof(tmp_header.build_timestamp));
    memcpy(tmp_header.flash_image_name, tf_header->firmware_package_name,
                                               sizeof(tmp_header.flash_image_name));
    tmp_header.flash_capacity = ops->get_capacity();
    tmp_header.erase_block_size = ops->get_erase_size();
    tmp_header.header_size = sizeof(tmp_header);
    tmp_header.flash_image_length =
                   2 * sizeof(tmp_header) + sizeof(*tf_header) + total_section_length;
    tmp_header.header_generation = 1;
    tmp_header.elements[0].element_type = FFFF_ELEMENT_STAGE_2_FW;
    tmp_header.elements[0].element_class = 0;
    tmp_header.elements[0].element_id = 1;
    tmp_header.elements[0].element_length = sizeof(*tf_header) + total_section_length;
    tmp_header.elements[0].element_location = tftf_start - dst;
    tmp_header.elements[0].element_generation = 1;
    tmp_header.elements[1].element_type = FFFF_ELEMENT_END;
    memcpy(tmp_header.trailing_sentinel_value, FFFF_SENTINEL_VALUE,
                                          sizeof(tmp_header.trailing_sentinel_value));

    ops->erase(dst, tmp_header.flash_image_length);
    ops->write(header0_start, &tmp_header, sizeof(tmp_header));
    result |= ops->verify(header0_start, &tmp_header, sizeof(tmp_header));

    ops->write(header1_start, &tmp_header, sizeof(tmp_header));
    result |= ops->verify(header1_start, &tmp_header, sizeof(tmp_header));

    ops->write(tftf_start, tf_header, sizeof(*tf_header));
    result |= ops->verify(tftf_start, tf_header, sizeof(*tf_header));

    dst = tftf_start + sizeof(*tf_header);
error:
    return result;
}

int spi_write_to_flash_data(const data_write_ops *ops, void *src, uint32_t len)
{
    int result = 0;

#ifdef CONFIG_DEBUG_SPI_FLASH
    dbgprintx32("spi_write_to_flash_data dst ", dst, "\r\n");
#endif
    ops->write(dst, src, len);
    result = ops->verify(dst, src, len);

    dst += len;
    return result;
}

void spi_write_to_flash_finish(const data_write_ops *ops)
{
    dbgprint("spi_write_to_flash_finish\r\n");
    ops->finish();
    dst = FLASH_START;
    total_section_length = 0;
}

static void Error_Handler(SPI_HandleTypeDef *hspi)
{
    wTransferState = TRANSFER_ERROR;
    /* reset spi */
    mods_muc_set_spi1_cs(PIN_SET);
    HAL_SPI_DeInit(hspi);
    dbgprint("SPI Flash DeInit\r\n");
    device_spi_flash_init(hspi);
    dbgprint("SPI Flash Re-Init\r\n");
    memset(aTxBuffer_spiFlash, 0, MAX_DMA_BUF_SIZE);
    memset(aRxBuffer_spiFlash, 0, MAX_DMA_BUF_SIZE);
}

void spi_flash_transfer_complete(SPI_HandleTypeDef *_hspi)
{
#ifdef CONFIG_DEBUG_SPI_FLASH
  dbgprint("spi_flash_transfer_complete\r\n");
#endif
  wTransferState = TRANSFER_COMPLETE;
}

void spi_flash_error_handler(SPI_HandleTypeDef *_hspi)
{
    dbgprint("spi_flash_transfer_error\r\n");
    Error_Handler(_hspi);
}

/**
 * @brief
 * Perform an spi flash receive and transfer using DMA. This Function
 * blocks until the exchange is complete.
 * @param dev pointer to structure of device data
 * @param txbuffer pointer to the data to send
 * @param rxbuffer pointer to the buffer to receive data into
 * @param nbytes number of bytes to exchange,
 * @return 0 on success, negative errno on error
 */
int chip_spi_exchange(void *txbuffer, void *rxbuffer, uint32_t nbytes) {

    uint32_t len;
    int remaining_bytes = nbytes;
    int buf_offset = 0;

    while (remaining_bytes > 0) {

        memset(aTxBuffer_spiFlash, 0, MAX_DMA_BUF_SIZE);
        memset(aRxBuffer_spiFlash, 0, MAX_DMA_BUF_SIZE);
        len = MIN(remaining_bytes, MAX_DMA_BUF_SIZE);
        memcpy(aTxBuffer_spiFlash, txbuffer + buf_offset, len);
        mods_muc_set_spi1_cs(PIN_RESET);

        if(HAL_SPI_TransmitReceive_DMA(&hspi_flash, (uint8_t*)aTxBuffer_spiFlash,
                (uint8_t *)aRxBuffer_spiFlash, len) != HAL_OK) {
            /* Transfer error in transmission process */
            dbgprint("chip_spi_exchange HAL_SPI_TransmitReceive_DMA error\r\n");
            Error_Handler(&hspi_flash);
            return -EIO;
        }
        /* wait for TxRx to complete */
        while (wTransferState == TRANSFER_WAIT) {
        }
        chip_delay(5);
        if (wTransferState == TRANSFER_COMPLETE) {
            memcpy(rxbuffer + buf_offset, aRxBuffer_spiFlash, len);
        } else {
           Error_Handler(&hspi_flash);
           return -EIO;
        }
        remaining_bytes -= len;
        buf_offset += len;
    }
    mods_muc_set_spi1_cs(PIN_SET);
    return 0;
}

void chip_delay(uint32_t msec)
{
    HAL_Delay(msec);
}
