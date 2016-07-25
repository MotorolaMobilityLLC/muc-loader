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
#include "stm32l4xx_hal.h"
#include "stm32_hal_mod.h"
#include "stm32l4xx_flash.h"
#include "tftf.h"

#include "datalink.h"
#include "network.h"

/* HEADER BITS FORMAT
 *
 * |  F  |  E  |  D  |  C  |  B  |  A  |  9  |  8  |
 * |-----+-----+-----+-----+-----+-----+-----+-----|
 * |              RESERVED             | DMY | PKT1|
 *
 *
 * |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
 * |-----+-----+-----+-----+-----+-----+-----+-----|
 * |VALID|TYPE |< -----        PKTS        ----- > |
 */

/* Protocol version supported by this driver */
#define PROTO_VER           (2)

/* Protocol version change log */
#define PROTO_VER_PKT1      (1)     /* Version that added PKT1 bit */
#define PROTO_VER_ACK       (1)     /* Minimum version for ACK support */
#define PROTO_VER_DUMMY     (2)     /* Version that added DUMMY bit */

/*
 * Protocol version support macro for checking if the requested feature (f)
 * is supported by the attached MuC.
 */
#define BASE_SUPPORTS(d, f) ((d)->proto_ver >= PROTO_VER_##f)

#define HDR_BIT_DUMMY  (0x01 << 9)  /* 1 = dummy packet */
#define HDR_BIT_PKT1   (0x01 << 8)  /* 1 = first packet of message */
#define HDR_BIT_VALID  (0x01 << 7)  /* 1 = packet has valid payload */
#define HDR_BIT_TYPE   (0x01 << 6)  /* SPI message type, datalink or network */
#define HDR_BIT_PKTS   (0x3F << 0)  /* How many additional packets to expect */

#define MSG_TYPE_DL    (0 << 6)     /* Packet for/from data link layer */
#define MSG_TYPE_NW    (1 << 6)     /* Packet for/from network layer */

/* Possible values for bus config features */
#define DL_BIT_ACK     (1 << 0)     /* Flag to indicate ACKing is supported */

#define DL_NUM_TRIES          3     /* number of times to try sending a message */
#define DL_ACK_TIMEOUT_MS   100     /* 100 milliseconds */

enum ack
{
  ACK_NEEDED,
  ACK_NOT_NEEDED,
  ACK_ERROR,
};

static SPI_HandleTypeDef gb_hspi;

/* Buffer used for transmission */
uint8_t aTxBuffer[MAX_DMA_BUF_SIZE];
uint8_t aRxBuffer[MAX_DMA_BUF_SIZE];

struct spi_msg_hdr {
        uint16_t bits;
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

struct dl_muc_bus_config_req {
    uint16_t max_pl_size;            /* Maximum payload size supported by base */
    uint8_t  features;               /* See DL_BIT_* defines for values */
    uint8_t  version;                /* SPI msg format version of base */
} __attribute__ ((__packed__));

struct dl_muc_bus_config_resp {
    uint32_t max_speed;              /* Maximum bus speed supported by the mod */
    uint16_t pl_size;                /* Payload size that mod has selected to use */
    uint8_t  features;               /* See DL_BIT_* defines for values */
    uint8_t  version;                /* SPI msg format version supported by mod */
} __attribute__ ((__packed__));

static struct {
    bool            armDMA;
    uint16_t        payload_size;
    msg_sent_cb     sent_cb;
    void            *sent_ctx;
    bool            respReady;
    bool            ack_supported;   /* Core and mod support ACK'ing on success */
    int             tx_remaining;    /* Number of sends remaining */
    uint8_t         proto_ver;       /* Protocol version supported by base */
} g_spi_data;

/* TODO: migrate to gbcore.c */
struct gb_operation_hdr *greybus_get_operation_header(void)
{
    return (struct gb_operation_hdr *)&aTxBuffer[DL_HEADER_BITS_SIZE + NW_HEADER_SIZE];
}

static inline void dl_set_txp_hdr(uint8_t *buf, uint16_t bits)
{
    struct spi_msg_hdr *hdr = (struct spi_msg_hdr *)buf;

    hdr->bits = bits;
}

static inline void dl_setup_for_dummy_tx(void)
{
    memset(aTxBuffer, 0, MAX_DMA_BUF_SIZE);
    dl_set_txp_hdr(aTxBuffer, HDR_BIT_DUMMY);
}

static inline bool dl_is_msg_valid_rx(void *msg)
{
    struct spi_msg *spi_msg = (struct spi_msg *)msg;
    uint16_t valid_mask = (HDR_BIT_VALID | HDR_BIT_PKT1);

    return ((spi_msg->hdr.bits & valid_mask) == valid_mask);
}

static inline bool dl_is_msg_dummy(void *msg)
{
    struct spi_msg *spi_msg = (struct spi_msg *)msg;

    return (BASE_SUPPORTS(&g_spi_data, DUMMY)) ?
            !!(spi_msg->hdr.bits & HDR_BIT_DUMMY) :
              (spi_msg->hdr.bits == 0);
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
    device_spi_mod_init(&gb_hspi);
    mods_rfr_set(PIN_RESET);
    mods_muc_int_set(PIN_RESET);
    g_spi_data.respReady = false;
    g_spi_data.armDMA = true;
    g_spi_data.ack_supported = false;
    g_spi_data.tx_remaining = DL_NUM_TRIES;
    dl_set_sent_cb(NULL, NULL);
    g_spi_data.payload_size = INITIAL_DMA_BUF_SIZE;
    g_spi_data.proto_ver = 0;
}

void dl_exit(void)
{
#ifdef CONFIG_DEBUG_DATALINK
    dbgprint("dl_exit\r\n");
#endif
    mods_rfr_set(PIN_RESET);
    mods_muc_int_set(PIN_RESET);
    HAL_SPI_DMAStop(&gb_hspi);
    mod_dev_base_spi_reset();
}

/* called from network layer */
/* buf should point to the start of a network message */
int datalink_send(uint8_t *buf, size_t len, msg_sent_cb cb, void *ctx)
{
    dl_set_txp_hdr(&buf[-sizeof(struct spi_msg_hdr)],
        HDR_BIT_PKT1 | MSG_TYPE_NW | HDR_BIT_VALID);
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
    struct spi_msg *spi_msg = (struct spi_msg *)aTxBuffer;
    struct spi_dl_msg *spi_dl_msg = (struct spi_dl_msg *)spi_msg->payload;
    uint8_t *payload = spi_dl_msg->payload;

    spi_dl_msg->mesg_id = id | status;

    if (payload_size != 0 && payload_data != NULL) {
        memcpy(payload, payload_data, payload_size);
    }

    dl_set_txp_hdr(&aTxBuffer[0], MSG_TYPE_DL | HDR_BIT_VALID | HDR_BIT_PKT1);
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
    struct dl_muc_bus_config_req *req =
            (struct dl_muc_bus_config_req *)dl_msg->payload;
    static struct dl_muc_bus_config_resp resp = {
        15000000,
        MAX_NW_PL_SIZE
    };

    g_spi_data.proto_ver = req->version;

    resp.version = PROTO_VER;
    resp.features = 0;

#ifdef CONFIG_GREYBUS_MODS_ACK
    if (req->features & DL_BIT_ACK) {
        g_spi_data.ack_supported = true;
        resp.features |= DL_BIT_ACK;

      /* Bases that support ACKing must use PROTO_VER_ACK or later */
      if (!BASE_SUPPORTS(&g_spi_data, ACK))
        {
          dbgprint("ACK requires newer protocol version\r\n");
          g_spi_data.proto_ver = PROTO_VER_ACK;
        }
    }
#ifdef CONFIG_DEBUG_DATALINK
    dbgprintx32("ack_supported = ", g_spi_data.ack_supported, "\r\n");
#endif
#endif

    if (req->max_pl_size < MAX_NW_PL_SIZE) {
        resp.pl_size = req->max_pl_size;
    } else {
        resp.pl_size = MAX_NW_PL_SIZE;
    }
#ifdef CONFIG_DEBUG_DATALINK
    dbgprintx32("dl_bus_config(0x", resp.max_speed, ",");
    dbgprintx32(" 0x", resp.pl_size, ")\r\n");
#endif

    return dl_send_dl_msg(dl_msg->mesg_id, GB_TYPE_RESPONSE, (uint8_t *)&resp,
                          sizeof(resp), dl_bus_config_cb,
                          (void *)(uint32_t)resp.pl_size);
}

/**
 * @brief handle message at datalink layer
 * @param msg - datalink message
 * @returns 0 on success
 */
int dl_muc_handler(void *msg)
{
    struct spi_dl_msg *dl_msg = (struct spi_dl_msg *)msg;
    int rc = 0;

#ifdef CONFIG_DEBUG_DATALINK
    dbgprint("dl_muc_handler\r\n");
#endif
    if (dl_msg->mesg_id & GB_TYPE_RESPONSE) {
        return GB_OP_PROTOCOL_BAD;
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

static void dump(void)
{
  uint32_t buf_size =  g_spi_data.payload_size + DL_HEADER_BITS_SIZE;

  dbgprintx32("DR:     0x", gb_hspi.Instance->DR, "\r\n");
  dbgprintx32("INT:      ", mods_muc_int_get(), "\r\n");
  dbgprintx32("RFR:      ", mods_rfr_get(), "\r\n");
  dbgprintx32("buf_sz  0x", buf_size, "\r\n");
  dbgprintx32("respReady ", g_spi_data.respReady, "\r\n");
  dbgprintx32("armDMA    ", g_spi_data.armDMA, "\r\n");
}

/*
 * returns: true if the messages were acknowledged successfully
 *          (or the caller should behave as if they were) and
 *          can delete the message.
 */
static bool ack_handler(bool tx_attempted, enum ack ack_req)
{
#ifdef CONFIG_GREYBUS_MODS_ACK
    uint32_t start;
    uint8_t ack;
    uint8_t wake_n = 0;
    uint8_t timeout = 0;
#ifdef CONFIG_DEBUG_DATALINK
    uint32_t dbg_tx_rx = tx_attempted << 16 | (uint16_t)ack_req;
    dbgprintx32("ack_handler 0x", dbg_tx_rx, "\r\n");
#endif

    if (!g_spi_data.ack_supported)
        return true;

    start = mods_getms();

#ifdef CONFIG_DEBUG_DATALINK
    dbgprintx32("start 0x", start, "\r\n");
#endif

    /* Send ACK to base (if requested) */
    mods_ack_received((ack_req == ACK_NEEDED) ? 1 : 0);

    /* We transmitted something so look for ACK from base */
    mods_ack_transmitted_setup();
    if (tx_attempted) {
        do {
            ack = mods_ack_transmitted_get();
            wake_n = mods_wake_n_get();
            timeout = ((mods_getms() - start) >= DL_ACK_TIMEOUT_MS);
        } while (!ack && wake_n && !timeout);

        if (!ack && wake_n) {
           /*
            * Since both TX and RX (potentially) failed, need
            * to spin longer to ensure base does not see false ACK.
            */
            if (ack_req == ACK_ERROR) {
                do {
                    wake_n = mods_wake_n_get();
                    timeout = (mods_getms() - start) >= (2 * DL_ACK_TIMEOUT_MS);
                } while (wake_n && !timeout);
            }

            if (--g_spi_data.tx_remaining > 0) {
#ifdef CONFIG_DEBUG_DATALINK
                dbgprint("Retry: No ACK received\r\n");
#endif
                return false;
            } else {
#ifdef CONFIG_DEBUG_DATALINK
                dbgprint("Abort: No ACK received\r\n");
#endif
                g_spi_data.tx_remaining = DL_NUM_TRIES;
                return true;
            }
        } else if (g_spi_data.tx_remaining != DL_NUM_TRIES) {
#ifdef CONFIG_DEBUG_DATALINK
            dbgprint("Retry successful\r\n");
#endif
            g_spi_data.tx_remaining = DL_NUM_TRIES;
        }
    }

    if (ack_req == ACK_ERROR) {
        /* Must block long enough to ensure base does not see false ACK. */
        do {
            wake_n = mods_wake_n_get();
            timeout = ((mods_getms() - start) >= (2 * DL_ACK_TIMEOUT_MS));
        } while (wake_n && !timeout);

        /* If here, there either was no TX or the TX was successful and ACK'd,
         * so it is okay to return true.
         */
        return true;
    }
#endif
    return true;
}

static int dl_process_msg(void)
{
    struct spi_msg *spi_msg = (struct spi_msg *)aRxBuffer;
    bool rx_valid = dl_is_msg_valid_rx(aRxBuffer);
    bool rx_dummy = dl_is_msg_dummy(aRxBuffer);
    enum ack rx_ack_req;
    bool tx_retry = false;

    if (rx_valid) {
        rx_ack_req = ACK_NEEDED;
    } else if (!rx_dummy) {
        rx_ack_req = ACK_ERROR;
    } else {
        rx_ack_req = ACK_NOT_NEEDED;
    }

    tx_retry = !ack_handler(g_spi_data.respReady, rx_ack_req);

    if (rx_valid) {
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
        if (!tx_retry) {
            g_spi_data.respReady = false;
            dl_setup_for_dummy_tx();
        }
        dl_call_sent_cb(0);
    } else if (!rx_dummy) {
        dbgprint("UNEXPECTED MSG!!\r\n");
        dump();
    }

    memset(aRxBuffer, 0, MAX_DMA_BUF_SIZE);
    return 0;
}

static void Error_Handler(SPI_HandleTypeDef *_hspi)
{
  bool tx_retry = false;

  mods_rfr_set(PIN_RESET);
  mods_muc_int_set(PIN_RESET);
  _hspi->Instance->CR1 |= (SPI_CR1_SSM | SPI_CR1_SSI);

  tx_retry = !ack_handler(g_spi_data.respReady, ACK_ERROR);
  if (!tx_retry)
    g_spi_data.respReady = false;
  dbgprintx32("SPIERR : 0x", _hspi->ErrorCode, "\r\n");
  dump();
  HAL_SPI_DeInit(_hspi);
  mod_dev_base_spi_reset();
  device_spi_mod_init(_hspi);
  if (!tx_retry)
    dl_setup_for_dummy_tx();
  memset(aRxBuffer, 0, MAX_DMA_BUF_SIZE);
  g_spi_data.armDMA = true;
}

void dl_spi_error_handler(SPI_HandleTypeDef *_hspi)
{
  Error_Handler(_hspi);
  dl_call_sent_cb(-1);
}

void dl_spi_transfer_complete(SPI_HandleTypeDef *_hspi)
{
  mods_rfr_set(PIN_RESET);
  mods_muc_int_set(PIN_RESET);

  /* Enable Software Slave Management to prevent spurious receives */
  _hspi->Instance->CR1 |= (SPI_CR1_SSM | SPI_CR1_SSI);

#ifdef CONFIG_DEBUG_DATALINK
  dbgprint("TxRxCpltCB\r\n");
#endif

  dl_process_msg();

  g_spi_data.armDMA = true;
}

void setup_exchange(void)
{
  uint32_t buf_size;

  if (g_spi_data.armDMA == true) {
    if (g_spi_data.respReady == true) {
      /* we have something to send */
#ifdef CONFIG_DEBUG_DATALINK
      dbgprint("RSP\r\n");
#endif
      buf_size =  g_spi_data.payload_size + DL_HEADER_BITS_SIZE;
      if (HAL_SPI_TransmitReceive_DMA(&gb_hspi, (uint8_t*)aTxBuffer,
                                          (uint8_t *)aRxBuffer, buf_size) != HAL_OK) {
        /* Transfer error in transmission process */
        Error_Handler(&gb_hspi);
      }

#ifdef CONFIG_DEBUG_DATALINK
      dbgprintx32("ARMED(0x", buf_size, ")\r\n");
#endif
      g_spi_data.armDMA = false;

      /* We are ready to send, allow the hardware to manage the NSS */
      mods_spi_restore();
      gb_hspi.Instance->CR1 &= ~(SPI_CR1_SSM | SPI_CR1_SSI);
      mods_rfr_set(PIN_SET);
      mods_muc_int_set(PIN_SET);
    } else if (mods_wake_n_get() != PIN_SET) {
      /* host is telling us to setup to receive */
#ifdef CONFIG_DEBUG_DATALINK
      dbgprint("WKE-L\r\n");
#endif
      buf_size =  g_spi_data.payload_size + DL_HEADER_BITS_SIZE;
      if (HAL_SPI_TransmitReceive_DMA(&gb_hspi, (uint8_t*)aTxBuffer,
                                      (uint8_t *)aRxBuffer, buf_size) != HAL_OK) {
        /* Transfer error in transmission process */
        Error_Handler(&gb_hspi);
      }

#ifdef CONFIG_DEBUG_DATALINK
      dbgprintx32("ARMED(0x", buf_size, ")\r\n");
#endif
      g_spi_data.armDMA = false;

      /* We are ready to receive, allow the hardware to manage the NSS */
      mods_spi_restore();
      gb_hspi.Instance->CR1 &= ~(SPI_CR1_SSM | SPI_CR1_SSI);
      mods_rfr_set(PIN_SET);
    }
  }
}
