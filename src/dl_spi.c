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
#include "stm32l4xx_hal.h"
#include "stm32_hal_mod.h"
#include "stm32l4xx_flash.h"
#include "tftf.h"

#include "datalink.h"
#include "network.h"

#define HDR_BIT_VALID  (0x01 << 7)  /* 1 = valid packet, 0 = dummy packet */
#define HDR_BIT_TYPE   (0x01 << 6)  /* SPI message type, datalink or network */
#define HDR_BIT_PKTS   (0x3F << 0)  /* How many additional packets to expect */

#define MSG_TYPE_DL    (0 << 6)     /* Packet for/from data link layer */
#define MSG_TYPE_NW    (1 << 6)     /* Packet for/from network layer */

SPI_HandleTypeDef hspi;                  /* TODO: factor out shared globals */

/* Buffer used for transmission */
uint8_t aTxBuffer[MAX_DMA_BUF_SIZE];
uint8_t aRxBuffer[MAX_DMA_BUF_SIZE];

struct spi_msg_hdr {
        uint8_t bits;
        uint8_t rsvd;
} __attribute__ ((packed));

struct spi_msg {
    struct spi_msg_hdr hdr;
    uint8_t payload[0];
} __attribute__ ((packed));

struct spi_dl_msg
{
    uint8_t  mesg_id;
    uint8_t  payload[0];
} __attribute__ ((packed));

struct dl_muc_bus_config_request {
    uint16_t max_payload_size;
} __attribute__ ((__packed__));

struct dl_muc_bus_config_response {
    uint32_t max_bus_speed;
    uint16_t sel_payload_size;
} __attribute__ ((__packed__));

static struct {
    bool            armDMA;
    uint16_t        payload_size;
    msg_sent_cb     sent_cb;
    void            *sent_ctx;
    bool            respReady;
} g_spi_data;

/* TODO: migrate to gbcore.c */
struct gb_operation_hdr *greybus_get_operation_header(void)
{
    return (struct gb_operation_hdr *)&aTxBuffer[DL_HEADER_BITS_SIZE + NW_HEADER_SIZE];
}

uint16_t datalink_get_max_payload_size(void)
{
     return g_spi_data.payload_size;
}

static inline msg_sent_cb dl_get_sent_cb(void)
{
    return g_spi_data.sent_cb;
}

static inline void dl_set_sent_cb(msg_sent_cb cb, void *ctx)
{
    g_spi_data.sent_cb  = cb;
    g_spi_data.sent_ctx = ctx;
}

static inline void dl_call_sent_cb(int status)
{
    msg_sent_cb cb = g_spi_data.sent_cb;
    void *ctx = g_spi_data.sent_ctx;

    dl_set_sent_cb(NULL, NULL);
    if (cb)
        cb(status, ctx);
}

void dl_init(void)
{
#ifdef CONFIG_DEBUG_DATALINK
    dbgprint("dl_init\r\n");
#endif
    mods_rfr_set(PIN_RESET);
    mods_muc_int_set(PIN_RESET);
    hspi.Instance->CR1 |= (SPI_CR1_SSM | SPI_CR1_SSI);
    g_spi_data.respReady = false;
    g_spi_data.armDMA = true;
    dl_set_sent_cb(NULL, NULL);
    g_spi_data.payload_size = INITIAL_DMA_BUF_SIZE;
}

/* called from network layer */
/* buf should point to the start of a network message */
int datalink_send(uint8_t *buf, size_t len, msg_sent_cb cb, void *ctx)
{
    struct spi_msg *dl = (struct spi_msg *)&buf[-sizeof(struct spi_msg_hdr)];

    dl->hdr.bits = MSG_TYPE_NW;
    dl->hdr.bits |= HDR_BIT_VALID;

    g_spi_data.respReady = true;
    dl_set_sent_cb(cb, ctx);

    return 0;
}

static int dl_send_dl_msg(uint8_t id,
                          uint8_t status,
                          unsigned char *payload_data,
                          uint16_t payload_size,
                          msg_sent_cb cb,
                          void *ctx)
{
    struct mods_spi_msg *spi_msg = (struct mods_spi_msg *)&aTxBuffer[0];
    unsigned char *payload = &spi_msg->dl_msg.dl_pl[0];

    spi_msg->dl_msg.mesg_id = id | status;

    if (payload_size != 0 && payload_data != NULL) {
        memcpy(payload, payload_data, payload_size);
    }

    spi_msg->hdr_bits = MSG_TYPE_DL;
    spi_msg->hdr_bits |= HDR_BIT_VALID;

    g_spi_data.respReady = true;
    dl_set_sent_cb(cb, ctx);

    return 0;
}

static void dl_bus_config_cb(int status, void *data)
{
    uint32_t pl_size = (uint32_t)data;

#ifdef CONFIG_DEBUG_DATALINK
    dbgprintx32("dl_bus_config_cb(0x", status, ",");
    dbgprintx32(" 0x", pl_size, ")\r\n");
#endif
    g_spi_data.payload_size = pl_size;
}

static int dl_bus_config(struct spi_dl_msg *dl_msg)
{
    uint16_t recved_max_pl_size;
    static struct dl_muc_bus_config_response payload = {
        15000000,
        MAX_NW_PL_SIZE
    };

    memcpy(&recved_max_pl_size, &dl_msg->payload[0], sizeof(uint16_t));
    payload.max_bus_speed = 0;

    if (recved_max_pl_size < MAX_NW_PL_SIZE) {
        payload.sel_payload_size = recved_max_pl_size;
    } else {
        payload.sel_payload_size = MAX_NW_PL_SIZE;
    }
#ifdef CONFIG_DEBUG_DATALINK
    dbgprintx32("dl_bus_config(0x", payload.max_bus_speed, ",");
    dbgprintx32(" 0x", payload.sel_payload_size, ")\r\n");
#endif

    return dl_send_dl_msg(dl_msg->mesg_id, GB_TYPE_RESPONSE, (uint8_t *)&payload,
                          sizeof(payload), dl_bus_config_cb,
                          (void *)(uint32_t)payload.sel_payload_size);
}

int dl_muc_handler(void *msg)
{
    struct spi_dl_msg *dl_msg = (struct spi_dl_msg *)msg;
    int rc = 0;

#ifdef CONFIG_DEBUG_DATALINK
    dbgprint("dl_muc_handler\r\n");
#endif
    if (dl_msg->mesg_id & GB_TYPE_RESPONSE) {
        return GB_FW_ERR_FAILURE;
    }

    switch (dl_msg->mesg_id) {
    case DL_MUC_OP_BUS_CONFIG:
        rc = dl_bus_config(dl_msg);
        break;
    default:
        dbgprint("DLDFLT\r\n");
        break;
    }

    return rc;
}

int dl_process_msg(void *msg)
{
    struct spi_msg *spi_msg = (struct spi_msg *)msg;

    if (spi_msg->hdr.bits & HDR_BIT_VALID) {
        if ((spi_msg->hdr.bits & HDR_BIT_TYPE) == MSG_TYPE_NW) {
            /* Send up to network layer */
            network_recv(spi_msg->payload, g_spi_data.payload_size);
        } else if ((spi_msg->hdr.bits & HDR_BIT_TYPE) == MSG_TYPE_DL) {
            /* handle at our level */
            (void)dl_muc_handler(spi_msg->payload);
        } else {
            return 0;
        }
    } else if (g_spi_data.respReady) {
        /* we were sending a message so handle it */
        mods_rfr_set(PIN_RESET);
        mods_muc_int_set(PIN_RESET);
        g_spi_data.respReady = false;
        dl_call_sent_cb(0);
    } else {
        dbgprint("UNEXPECTED MSG!!\r\n");
    }
  return 0;
}

static void Error_Handler(SPI_HandleTypeDef *_hspi)
{
  dbgprint("FTL\r\n");

  /* reset spi */
  mods_rfr_set(PIN_RESET);
  mods_muc_int_set(PIN_RESET);
  g_spi_data.respReady = false;

  HAL_SPI_DeInit(_hspi);
  mod_dev_base_spi_reset();
  MX_SPI_Init();
  memset(aTxBuffer, 0, MAX_DMA_BUF_SIZE);
  memset(aRxBuffer, 0, MAX_DMA_BUF_SIZE);
  g_spi_data.armDMA = true;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *_hspi)
{
  dbgprintx32("SPIERR : 0x", _hspi->ErrorCode, "\r\n");

  dl_call_sent_cb(-1);
  Error_Handler(_hspi);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *_hspi)
{
#ifdef CONFIG_DEBUG_DATALINK
  dbgprint("HAL_SPI_TxRxCpltCallback\r\n");
#endif

  /* Enable Software Slave Management to prevent spurious receives */
  _hspi->Instance->CR1 |= (SPI_CR1_SSM | SPI_CR1_SSI);

  memset(aTxBuffer, 0, MAX_DMA_BUF_SIZE);
  dl_process_msg((struct mods_spi_msg *)aRxBuffer);
  memset(aRxBuffer, 0, MAX_DMA_BUF_SIZE);

  g_spi_data.armDMA = true;
}

void setup_exchange(void)
{
  uint32_t buf_size;

  /* Start the Full Duplex Communication process */
  /* While the SPI in TransmitReceive process, user can transmit data through
     "aTxBuffer" buffer & receive data through "aRxBuffer" */
  if (g_spi_data.armDMA == true) {
    /* Response is ready, signal INT to base */
    if (g_spi_data.respReady == true) {
       mods_muc_int_set(PIN_SET);
    } else {
       mods_muc_int_set(PIN_RESET);
    }

    /* Wait for WAKE_N to arm DMA */
    if (mods_wake_n_get() == PIN_SET) {
      return;
    }

#ifdef CONFIG_DEBUG_DATALINK
    dbgprint("WKE-L\r\n");
    dbgprintx32("SR : 0x", hspi.Instance->SR, "\r\n");
#endif

    /* select DMA buffer size */
    buf_size =  g_spi_data.payload_size + DL_HEADER_BITS_SIZE;

    if (HAL_SPI_TransmitReceive_DMA(&hspi, (uint8_t*)aTxBuffer,
                                    (uint8_t *)aRxBuffer, buf_size) != HAL_OK) {
      /* Transfer error in transmission process */
      Error_Handler(&hspi);
    }

#ifdef CONFIG_DEBUG_DATALINK
    dbgprintx32("ARMED(0x", buf_size, ")\r\n");
#endif

    g_spi_data.armDMA = false;

    /* We are ready to receive, allow the hardware to manage the NSS */
    hspi.Instance->CR1 &= ~(SPI_CR1_SSM | SPI_CR1_SSI);
    mods_rfr_set(PIN_SET);
  }
}
