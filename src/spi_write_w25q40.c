/*
 * Copyright (c) 2016 Motorola Mobility, LLC.
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

#include "chipapi.h"
#include "debug.h"
#include "data_writing.h"
#include <utils.h>
#include "stm32l4xx_hal.h"
#include "spi_flash_write.h"
#include <stm32_hal_mod.h>

#define JEDEC_ID_CMD (0x9f)
#define JEDEC_ID_CMD_SIZE (4)

#define WRITE_CMD (0x02)
#define WRITE_CMD_SIZE (4)

#define STATUS_CMD (0x05)
#define STATUS_CMD_SIZE (2)
#define STATUS_CMD2 (0x35)
#define STATUS_CMD_SIZE2 (2)
#define STATUS_BUSY (0x1)
#define STATUS_WRITE_ENABLE_LATCH (0x2)

#define WRITE_ENABLE_CMD (0x06)
#define WRITE_ENABLE_CMD_SIZE (1)

#define ERASE_ALL_CMD (0x60)
#define ERASE_ALL_CMD_SIZE (1)

#define ERASE_CMD (0x20)
#define ERASE_CMD_SIZE (4)

#define READ_CMD (0x03)
#define READ_CMD_SIZE (4)

struct spi_flash_device {
    uint8_t jedec_id[3];
    uint8_t pad;
    size_t address_width;
    size_t sector_size;
    size_t num_sectors;
    size_t page_size;
    size_t erase_size;
    uint32_t max_frequency;
    uint32_t program_delay;
    uint32_t erase_delay;
};

const static struct spi_flash_device SPI_FLASH_DEVICES[] = {
    /* w25q40bw */
    {
        .jedec_id = { 0xef, 0x50, 0x13 },
        .address_width = 3,
        .sector_size = 64*1024,
        .num_sectors = 8,
        .page_size = 256,
        .erase_size = 4*1024,
        .max_frequency = 24*1000*1000,
        .program_delay = 10,
        .erase_delay = 400,
    },
    /* w25q40bew */
    {
        .jedec_id = { 0xef, 0x60, 0x13 },
        .address_width = 3,
        .sector_size = 64*1024,
        .num_sectors = 8,
        .page_size = 256,
        .erase_size = 4*1024,
        .max_frequency = 24*1000*1000,
        .program_delay = 10,
        .erase_delay = 400,
    },
};

static const struct spi_flash_device *spi_flash_device;

static const struct spi_flash_device * spi_find_device(const uint8_t *jedec_id) {
    size_t i;
    for (i = 0; i < sizeof(SPI_FLASH_DEVICES)/sizeof(SPI_FLASH_DEVICES[0]); i++) {
        int result = memcmp(jedec_id, SPI_FLASH_DEVICES[i].jedec_id, sizeof(SPI_FLASH_DEVICES[i].jedec_id));
        if (!result) {
            return &SPI_FLASH_DEVICES[i];
        }
    }

    dbgprint("WARNING: JEDEC ID not found\n");
    return &SPI_FLASH_DEVICES[0];
}

static void spi_flash_write_address(uint32_t address, uint8_t buffer[]) {
    if (spi_flash_device->address_width == 3) {
        buffer[0] = (address >> 16) & 0x0ff;
        buffer[1] = (address >>  8) & 0x0ff;
        buffer[2] = (address >>  0) & 0x0ff;
    } else if (spi_flash_device->address_width == 4) {
        buffer[0] = (address >> 24) & 0x0ff;
        buffer[1] = (address >> 16) & 0x0ff;
        buffer[2] = (address >>  8) & 0x0ff;
        buffer[3] = (address >>  0) & 0x0ff;
    } else {
        dbgprint("ERROR: invalid address width");
    }
}

static int spi_flash_read_id(void) {
    uint8_t txbuf[JEDEC_ID_CMD_SIZE];

    memset(txbuf, 0, JEDEC_ID_CMD_SIZE);

    txbuf[0] = JEDEC_ID_CMD;
    chip_spi_exchange(txbuf, txbuf, JEDEC_ID_CMD_SIZE);
    dbgprintx32("spi_flash_read_id ..", txbuf[1], "\r\n");
    spi_flash_device = spi_find_device(&txbuf[1]);
    if (!spi_flash_device) {
        return -1;
    }

    return 0;
}

static int spi_flash_write_enable(void) {
    uint8_t txbuf[STATUS_CMD_SIZE];

    memset(txbuf, 0, STATUS_CMD_SIZE);
    txbuf[0] = WRITE_ENABLE_CMD;

    chip_spi_exchange(txbuf, txbuf, WRITE_ENABLE_CMD_SIZE);

    memset(txbuf, 0, STATUS_CMD_SIZE);
    txbuf[0] = STATUS_CMD;
    while (1) {
        chip_spi_exchange(txbuf, txbuf, STATUS_CMD_SIZE);

        if ((txbuf[1] & STATUS_WRITE_ENABLE_LATCH) == STATUS_WRITE_ENABLE_LATCH) {
            break;
        }
    }

    return 0;
}

static void spi_flash_wait_for_not_busy(void) {
    uint8_t txbuf[STATUS_CMD_SIZE];
    memset(txbuf, 0, STATUS_CMD_SIZE);
    txbuf[0] = STATUS_CMD;
    while (1) {
        chip_spi_exchange(txbuf, txbuf, STATUS_CMD_SIZE);
        if ((txbuf[1] & STATUS_BUSY) == 0) {
            break;
        }
    }
}

static int spi_flash_init(void) {
    int result;

    result = spi_flash_read_id();

    return result;
}

static int spi_flash_erase_all(void) {
    uint8_t txbuf[ERASE_ALL_CMD_SIZE];

    spi_flash_write_enable();
    memset(txbuf, 0, ERASE_CMD_SIZE);
    txbuf[0] = ERASE_ALL_CMD;
    chip_spi_exchange(txbuf, txbuf, ERASE_ALL_CMD_SIZE);

    spi_flash_wait_for_not_busy();
    return 0;
}

static int spi_flash_erase(uint32_t dst, uint32_t length) {
    uint8_t txbuf[ERASE_CMD_SIZE];
    uint32_t dst_end = dst + length;

#ifdef CONFIG_DEBUG_SPI_FLASH
    dbgprintx32("erase len ", length, "\n");
    dbgprintx32("erase dst ", dst, "\n");
    dbgprintx32("erase dst end ", (dst + length), "\n");
#endif
    dbgprintx32("erasing ", length, " bytes\r\n");
    while (dst < dst_end) {
        memset(txbuf, 0, ERASE_CMD_SIZE);
        spi_flash_write_enable();

        txbuf[0] = ERASE_CMD;
        spi_flash_write_address(dst, &txbuf[1]);
        chip_spi_exchange(txbuf, txbuf, ERASE_CMD_SIZE);

        chip_delay(spi_flash_device->erase_delay);

        spi_flash_wait_for_not_busy();
        dst += spi_flash_device->erase_size;
#ifdef CONFIG_DEBUG_SPI_FLASH
        dbgprintx32("erase dst addr", dst, "\r\n");
#endif
    }

    dbgprint("\n");
    return 0;
}

static int spi_flash_write(uint32_t _dst, void *_src, uint32_t length) {
    uint8_t txbuf[WRITE_CMD_SIZE + spi_flash_device->page_size];
    uint32_t dst = _dst;
    uint8_t *src = _src;
    uint8_t *src_end = src + length;

#ifdef CONFIG_DEBUG_SPI_FLASH
    dbgprintx32("write len ", length, "\r\n");
    dbgprintx32("write dst ", _dst, "\r\n");
    dbgprintx32("write dst end ", (_dst + length), "\r\n");
    dbgprintx32("write src ", ((uint32_t)_src), "\r\n");
    dbgprintx32("write src end ", (((uint32_t)src) + length), "\r\n");
#endif

    while (src < src_end) {
        memset(txbuf, 0, WRITE_CMD_SIZE + spi_flash_device->page_size);
        spi_flash_write_enable();

        uint32_t len = MIN(src_end - src, spi_flash_device->page_size);

        /* Align the destination on a page boundary. */
        uint32_t page_offset = (dst % spi_flash_device->page_size);
        if (page_offset) {
            len = MIN(src_end - src, spi_flash_device->page_size - page_offset);
        }

        txbuf[0] = WRITE_CMD;
#ifdef CONFIG_DEBUG_SPI_FLASH
        dbgprintx32("page dst ", dst, "\r\n");
#endif
        spi_flash_write_address(dst, &txbuf[1]);
        memcpy(&txbuf[WRITE_CMD_SIZE], src, len);

        chip_spi_exchange(txbuf, txbuf, (WRITE_CMD_SIZE + len));

        chip_delay(spi_flash_device->program_delay);

        spi_flash_wait_for_not_busy();

        dst += len;
        src += len;
    }

    return 0;
}

static int spi_flash_verify(uint32_t dst, void *src, uint32_t length) {
    uint8_t rxbuf[READ_CMD_SIZE + length];
    int result;

    spi_flash_wait_for_not_busy();

    memset(rxbuf, 0, (READ_CMD_SIZE + length));
    rxbuf[0] = READ_CMD;
    spi_flash_write_address(dst, &rxbuf[1]);

    chip_spi_exchange(rxbuf, rxbuf, (length + READ_CMD_SIZE));

    result = memcmp(&rxbuf[READ_CMD_SIZE], src, length);
    if (result) {
        dbgprintx32("ERROR: flash verify failed\n", result, "\r\n");
        return result;
    }

    return result;
}

static int spi_flash_finish(void) {
    spi_flash_device = NULL;
    return 0;
}

static size_t spi_flash_get_capacity(void) {
    return spi_flash_device ? spi_flash_device->sector_size * spi_flash_device->num_sectors : 0;
}

static size_t spi_flash_get_erase_size(void) {
    return spi_flash_device ? spi_flash_device->erase_size : 0;
}

const data_write_ops spi_write_ops = {
    .init = spi_flash_init,
    .erase_all = spi_flash_erase_all,
    .erase = spi_flash_erase,
    .write = spi_flash_write,
    .verify = spi_flash_verify,
    .finish = spi_flash_finish,
    .get_capacity = spi_flash_get_capacity,
    .get_erase_size = spi_flash_get_erase_size,
};
