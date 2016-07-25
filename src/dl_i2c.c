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
#include "greybus.h"
#include "stm32l4xx_hal.h"
#include "stm32_hal_mod.h"
#include "stm32l4xx_flash.h"
#include "tftf.h"

#include "crc16_poly8005.h"
#include "datalink.h"
#include "network.h"

/* HEADER BITS FORMAT
 *
 * |  F  |  E  |  D  |  C  |  B  |  A  |  9  |  8  |
 * |-----+-----+-----+-----+-----+-----+-----+-----|
 * |           RESERVED          | ACK | DMY | PKT1|
 *
 *
 * |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
 * |-----+-----+-----+-----+-----+-----+-----+-----|
 * |VALID|TYPE |< -----        PKTS        ----- > |
 */

/* Protocol version supported by this driver */
#define PROTO_VER               (1)

/*
 * The bare minimum protocol version required by the driver. To remain backwards
 * compatible with every base ever shipped, this value should never be changed
 * and new features must only be enabled once determined that the base supports
 * them.
 */
#define MIN_PROTO_VER           (1)

#define HDR_BIT_ACK    (0x01 << 10) /* 1 = Base has ACK'd last packet sent */
#define HDR_BIT_DUMMY  (0x01 <<  9) /* 1 = dummy packet */
#define HDR_BIT_PKT1   (0x01 <<  8) /* 1 = first packet of message */
#define HDR_BIT_VALID  (0x01 <<  7) /* 1 = packet has valid payload */
#define HDR_BIT_TYPE   (0x01 <<  6) /* I2C message type, datalink or network */
#define HDR_BIT_PKTS   (0x3F <<  0) /* How many additional packets to expect */

#define MSG_TYPE_DL        (0 << 6) /* Packet for/from data link layer */
#define MSG_TYPE_NW        (1 << 6) /* Packet for/from network layer */

#define NUM_TRIES               (3) /* number of times to try sending a message */

#define I2C_TIMEOUT_MS       (1000)

/* Size of the header in bytes */
#define HDR_SIZE           sizeof(struct i2c_msg_hdr)

/* Size of the CRC in bytes */
#define CRC_SIZE           (2)

/* Macro to determine the packet size from the payload size */
#define PKT_SIZE(pl_size)  (pl_size + HDR_SIZE + CRC_SIZE)

/* All the possible states in the state machine */
enum dl_state_e
{
  DL_STATE_IDLE,                    /* 0 */
  DL_STATE_RX,                      /* 1 */
  DL_STATE_TX,                      /* 2 */
  DL_STATE_ACK_RX,                  /* 3 */
  DL_STATE_ACK_TX,                  /* 4 */
  DL_STATE_DUMMY,                   /* 5 */

  /* Add new states above here */
  DL_STATE__NUM_STATES,
};

struct i2c_msg_hdr
{
    uint16_t bits;
} __attribute__ ((packed));

struct i2c_msg
{
    struct i2c_msg_hdr hdr;
    uint8_t payload[0];
} __attribute__ ((packed));

struct i2c_dl_msg
{
    uint8_t  id;
    uint8_t  payload[0];
} __attribute__ ((packed));

struct dl_muc_bus_config_req
{
    uint16_t max_pl_size;            /* Maximum payload size supported by base */
    uint8_t  features;               /* See DL_BIT_* defines for values */
    uint8_t  version;                /* I2C msg format version of base */
} __attribute__ ((__packed__));

struct dl_muc_bus_config_resp
{
    uint32_t max_speed;              /* Maximum bus speed supported by the mod */
    uint16_t pl_size;                /* Payload size that mod has selected to use */
    uint8_t  features;               /* See DL_BIT_* defines for values */
    uint8_t  version;                /* I2C msg format version supported by mod */
} __attribute__ ((__packed__));

static struct i2c_data_s
{
    I2C_HandleTypeDef hi2c;
    uint16_t          pl_size;       /* Size of packet payload in bytes */
    msg_sent_cb       sent_cb;
    void             *sent_ctx;
    enum dl_state_e   txn_state;     /* Current transaction state */
    int               tx_remaining;  /* Number of sends remaining */
    uint8_t           proto_ver;     /* Protocol version supported by base */

    uint8_t           tx_buf[MAX_DMA_BUF_SIZE];
    uint8_t           rx_buf[MAX_DMA_BUF_SIZE];
} g_data;

struct state_funcs
{
  int (*start_tx)(struct i2c_data_s *priv);
  int (*start_rx)(struct i2c_data_s *priv);

  int (*stop_success)(struct i2c_data_s *priv);
  int (*stop_error)(struct i2c_data_s *priv);
};

/* TODO: migrate to gbcore.c */
struct gb_operation_hdr *greybus_get_operation_header(void)
{
    return (struct gb_operation_hdr *)&g_data.tx_buf[DL_HEADER_BITS_SIZE + NW_HEADER_SIZE];
}

static inline void set_txp_hdr(uint8_t *buf, uint16_t bits)
{
    struct i2c_msg_hdr *hdr = (struct i2c_msg_hdr *)buf;

    hdr->bits = bits;
}

static inline void setup_for_dummy_tx(struct i2c_data_s *priv)
{
    set_txp_hdr(priv->tx_buf, HDR_BIT_DUMMY);
}

static inline void set_ack_tx_hdr(struct i2c_data_s *priv)
{
    struct i2c_msg_hdr *hdr = (struct i2c_msg_hdr *)priv->tx_buf;
    hdr->bits = hdr->bits | HDR_BIT_ACK;
}

static inline bool dl_is_msg_valid(void *msg)
{
    struct i2c_msg *i2c_msg = (struct i2c_msg *)msg;
    uint16_t valid_mask = (HDR_BIT_VALID | HDR_BIT_PKT1);

    return ((i2c_msg->hdr.bits & valid_mask) == valid_mask);
}

static void set_crc(struct i2c_data_s *priv)
{
    uint16_t *crc = (uint16_t *)&priv->tx_buf[priv->pl_size + HDR_SIZE];
    *crc = crc16_poly8005(priv->tx_buf, priv->pl_size + HDR_SIZE,
                          CRC_INIT_VAL);
}

static inline void reset_tx_buf(struct i2c_data_s *priv)
{
    memset(priv->tx_buf, 0, MAX_DMA_BUF_SIZE);
    priv->tx_remaining = NUM_TRIES;
}

uint16_t datalink_get_max_payload_size(void)
{
    return g_data.pl_size;
}

static inline void dl_set_sent_cb(struct i2c_data_s *priv, msg_sent_cb cb,
                                  void *ctx)
{
    priv->sent_cb  = cb;
    priv->sent_ctx = ctx;
}

static void dl_call_sent_cb(struct i2c_data_s *priv, int status)
{
    msg_sent_cb cb = priv->sent_cb;
    void *ctx = priv->sent_ctx;

    dl_set_sent_cb(priv, NULL, NULL);
    if (cb)
        cb(status, ctx);
}

#ifdef CONFIG_DEBUG_DATALINK
static const char* state_name(enum dl_state_e state)
{
    switch (state) {
        case DL_STATE_IDLE:   return "IDLE";
        case DL_STATE_RX:     return "RX";
        case DL_STATE_TX:     return "TX";
        case DL_STATE_ACK_RX: return "ACK_RX";
        case DL_STATE_ACK_TX: return "ACK_TX";
        case DL_STATE_DUMMY:  return "DUMMY";
        default:              return "???";
    }
}
#endif

static void set_state(struct i2c_data_s *priv, enum dl_state_e state)
{
    if (priv->txn_state != state) {
#ifdef CONFIG_DEBUG_DATALINK
        dbgprint(state_name(priv->txn_state));
        dbgprint(" -> ");
        dbgprint(state_name(state));
        dbgprint("\r\n");
#endif
        priv->txn_state = state;
    }
}

void dl_init(void)
{
#ifdef CONFIG_DEBUG_DATALINK
    dbgprint("dl_init\r\n");
#endif

    device_i2c_mod_init(&g_data.hi2c);
    mods_rfr_set(PIN_SET);
    mods_muc_int_set(PIN_RESET);

    set_state(&g_data, DL_STATE_IDLE);
    dl_set_sent_cb(&g_data, NULL, NULL);
    reset_tx_buf(&g_data);

    g_data.pl_size = INITIAL_DMA_BUF_SIZE;
    g_data.proto_ver = MIN_PROTO_VER;
}

void dl_exit(void)
{
#ifdef CONFIG_DEBUG_DATALINK
    dbgprint("dl_exit\r\n");
#endif
    mods_rfr_set(PIN_RESET);
    mods_muc_int_set(PIN_RESET);
}

/* called from network layer */
/* buf should point to the start of a network message */
int datalink_send(uint8_t *buf, size_t len, msg_sent_cb cb, void *ctx)
{
    set_txp_hdr(&buf[-sizeof(struct i2c_msg_hdr)],
                HDR_BIT_PKT1 | MSG_TYPE_NW | HDR_BIT_VALID);
    dl_set_sent_cb(&g_data, cb, ctx);
    mods_muc_int_set(PIN_SET);

    return 0;
}

static int send_dl_msg(struct i2c_data_s *priv, uint8_t id, uint8_t *pl_data,
                       uint16_t pl_size, msg_sent_cb cb, void *ctx)
{
    struct i2c_msg *i2c_msg = (struct i2c_msg *)priv->tx_buf;
    struct i2c_dl_msg *i2c_dl_msg = (struct i2c_dl_msg *)i2c_msg->payload;
    uint8_t *payload = i2c_dl_msg->payload;

    i2c_dl_msg->id = id;

    if (pl_size > 0 && pl_data) {
        memcpy(payload, pl_data, pl_size);
    }

    set_txp_hdr(priv->tx_buf, MSG_TYPE_DL | HDR_BIT_VALID | HDR_BIT_PKT1);
    dl_set_sent_cb(priv, cb, ctx);
    mods_muc_int_set(PIN_SET);

    return 0;
}

static void dl_bus_config_cb(int status, void *data)
{
    struct i2c_data_s *priv = (struct i2c_data_s *)data;

#ifdef CONFIG_DEBUG_DATALINK
    dbgprintx32("dl_bus_config_cb(0x", status, ")\r\n");
#endif
    priv->pl_size = MAX_NW_PL_SIZE;
}

/**
 * @brief handle message at datalink layer
 * @param msg - datalink message
 * @returns 0 on success
 */
static int dl_msg_handler(struct i2c_data_s *priv, void *msg)
{
    struct i2c_dl_msg *dl_msg = (struct i2c_dl_msg *)msg;
    struct dl_muc_bus_config_req *req =
            (struct dl_muc_bus_config_req *)dl_msg->payload;
    struct dl_muc_bus_config_resp resp;

#ifdef CONFIG_DEBUG_DATALINK
    dbgprint("dl_msg_handler\r\n");
#endif

    /* Only BUS_CFG_REQ is supported */
    if (dl_msg->id != DL_MUC_OP_BUS_CONFIG) {
        dbgprintx32("Unknown ID (0x", dl_msg->id, ")\r\n");
        return -EINVAL;
    }

    if (req->version < MIN_PROTO_VER) {
        dbgprintx32("Unsupported protocol version (0x", req->version, ")\r\n");
        return -EPROTONOSUPPORT;
    }

    if (req->max_pl_size < MAX_NW_PL_SIZE) {
        dbgprintx32("Unsupported payload size (0x", req->max_pl_size, ")\r\n");
        return -EPROTONOSUPPORT;
    }

    priv->proto_ver = req->version;

    resp.max_speed = 400000;
    resp.version = PROTO_VER;
    resp.features = 0;
    resp.pl_size = MAX_NW_PL_SIZE;

    return send_dl_msg(priv, DL_MUC_OP_BUS_CONFIG_RESP, (uint8_t *)&resp,
                       sizeof(resp), dl_bus_config_cb, priv);
}

static int idle_start_tx(struct i2c_data_s *priv)
{
    if (dl_is_msg_valid(priv->tx_buf)) {
        mods_muc_int_set(PIN_RESET);
        set_state(priv, DL_STATE_TX);
    } else {
        /* Nothing to send! */
        setup_for_dummy_tx(priv);
        set_state(priv, DL_STATE_DUMMY);
    }

    set_crc(priv);

    return 0;
}

static int idle_start_rx(struct i2c_data_s *priv)
{
    set_state(priv, DL_STATE_RX);

    return 0;
}

static int idle_stop(struct i2c_data_s *priv)
{
    /* This will never happen */
    dbgprint("idle_stop: ???\r\n");

    return 0;
}

static int no_start(struct i2c_data_s *priv)
{
    /* This is unexpected, so return to IDLE and start state machine again */
    dbgprint("no_start: ???\r\n");
    set_state(priv, DL_STATE_IDLE);

    return -EAGAIN;
}

static int rx_stop_error(struct i2c_data_s *priv)
{
    set_state(priv, DL_STATE_IDLE);

    return 0;
}

static int rx_stop_success(struct i2c_data_s *priv)
{
    /* First verify valid CRC */
    uint16_t calc_crc = crc16_poly8005(priv->rx_buf, priv->pl_size + HDR_SIZE,
                                       CRC_INIT_VAL);
    uint16_t *rcvd_crc = (uint16_t *)&(priv->rx_buf[priv->pl_size + HDR_SIZE]);
    if (calc_crc != *rcvd_crc) {
        dbgprintx32("CRC mismatch: 0x", calc_crc, " != ");
        dbgprintx32("0x", *rcvd_crc, "\r\n");

        return rx_stop_error(priv);
    }

    struct i2c_msg *msg = (struct i2c_msg *)priv->rx_buf;
    enum dl_state_e next_state = DL_STATE_ACK_TX;

#ifdef CONFIG_DEBUG_DATALINK
    dbgprintx32("hdr->bits: 0x", msg->hdr.bits, "\r\n");
#endif

    if (!(msg->hdr.bits & HDR_BIT_VALID)) {
        /* Received a dummy or garbage packet - no processing to do! */

        if (!(msg->hdr.bits & HDR_BIT_DUMMY))
            dbgprint("garbage packet\r\n");

        next_state = DL_STATE_IDLE;
        goto done;
    }

    if (!(msg->hdr.bits & HDR_BIT_PKT1)) {
        dbgprint("1st pkt bit not set\r\n");
        goto done;
    }

    if ((msg->hdr.bits & HDR_BIT_TYPE) == MSG_TYPE_NW) {
        /* Send up to network layer */
        network_recv(msg->payload, priv->pl_size);
    } else {
        /* handle at our level */
        dl_msg_handler(priv, msg->payload);
    }

done:
    set_state(priv, next_state);

    return 0;
}

static int tx_stop_success(struct i2c_data_s *priv)
{
    set_state(priv, DL_STATE_ACK_RX);

    return 0;
}

static int tx_stop_error(struct i2c_data_s *priv)
{
    if (--priv->tx_remaining <= 0) {
        dbgprint("tx_stop_error: abort\r\n");

        reset_tx_buf(priv);
        dl_call_sent_cb(priv, -1);
    } else {
        dbgprint("tx_stop_error: retry\r\n");

        mods_muc_int_set(PIN_SET);
    }

    set_state(priv, DL_STATE_IDLE);

    return 0;
}

static int ack_rx_start_tx(struct i2c_data_s *priv)
{
    /* This is unexpected, so return to IDLE and start state machine again */
    set_state(priv, DL_STATE_IDLE);

    /* Assume the base was happy with last transmission */
    reset_tx_buf(priv);
    dl_call_sent_cb(priv, 0);

    return -EAGAIN;
}

static int ack_rx_start_rx(struct i2c_data_s *priv)
{
    /* Nothing to do */
    return 0;
}

static int ack_rx_stop_success(struct i2c_data_s *priv)
{
    struct i2c_msg_hdr *hdr = (struct i2c_msg_hdr *)priv->rx_buf;

    if (hdr->bits & HDR_BIT_ACK) {
        reset_tx_buf(priv);
        dl_call_sent_cb(priv, 0);
    } else if (--priv->tx_remaining <= 0) {
        dbgprint("ack_rx_stop_success: abort\r\n");

        reset_tx_buf(priv);
        dl_call_sent_cb(priv, -1);
    } else {
        dbgprint("ack_rx_stop_success: retry\r\n");

        mods_muc_int_set(PIN_SET);
    }

    /* It is possible the base has sent a valid packet that needs parsing */
    set_state(priv, DL_STATE_RX);

    return -EAGAIN;
}

static int ack_tx_start_tx(struct i2c_data_s *priv)
{
    mods_muc_int_set(PIN_RESET);

    if (!dl_is_msg_valid(priv->tx_buf)) {
        /* Nothing to send! */
        setup_for_dummy_tx(priv);
    }

    set_ack_tx_hdr(priv);
    set_crc(priv);

    return 0;
}

static int ack_tx_stop_success(struct i2c_data_s *priv)
{
    struct i2c_msg_hdr *hdr = (struct i2c_msg_hdr *)priv->tx_buf;

    /* It is possible we sent a valid packet with ACK */
    if (hdr->bits & HDR_BIT_VALID) {
        set_state(priv, DL_STATE_TX);
        return -EAGAIN;
    }

    reset_tx_buf(priv);
    set_state(priv, DL_STATE_IDLE);

    return 0;
}

static int ack_tx_stop_error(struct i2c_data_s *priv)
{
    struct i2c_msg_hdr *hdr = (struct i2c_msg_hdr *)priv->tx_buf;

    if (!(hdr->bits & HDR_BIT_VALID)) {
        reset_tx_buf(priv);
    } else if (--priv->tx_remaining <= 0) {
        dbgprint("ack_tx_stop_error: abort\r\n");

        reset_tx_buf(priv);
        dl_call_sent_cb(priv, -1);
    } else {
        dbgprint("ack_tx_stop_error: retry\r\n");

        mods_muc_int_set(PIN_SET);
    }

    set_state(priv, DL_STATE_IDLE);

    return 0;
}

static int dummy_stop(struct i2c_data_s *priv)
{
    reset_tx_buf(priv);
    set_state(priv, DL_STATE_IDLE);

    return 0;
}

static const struct state_funcs state_funcs_tbl[DL_STATE__NUM_STATES] =
{
  /* DL_STATE_IDLE */
  { idle_start_tx,   idle_start_rx,   idle_stop,           idle_stop         },

  /* DL_STATE_RX */
  { no_start,        no_start,        rx_stop_success,     rx_stop_error     },

  /* DL_STATE_TX */
  { no_start,        no_start,        tx_stop_success,     tx_stop_error     },

  /* DL_STATE_ACK_RX */
  { ack_rx_start_tx, ack_rx_start_rx, ack_rx_stop_success, tx_stop_error     },

  /* DL_STATE_ACK_TX */
  { ack_tx_start_tx, no_start,        ack_tx_stop_success, ack_tx_stop_error },

  /* DL_STATE_DUMMY */
  { no_start,        no_start,        dummy_stop,          dummy_stop        },
};

void setup_exchange(void)
{
    HAL_StatusTypeDef status;
    int ret;

    if (!(g_data.hi2c.Instance->ISR & I2C_ISR_ADDR)) {
        /* Nothing to do */
        return;
    }

    if (g_data.hi2c.Instance->ISR & I2C_ISR_DIR) {
        /* Read transfer, slave enters transmitter mode */

        do {
            ret = state_funcs_tbl[g_data.txn_state].start_tx(&g_data);
        } while (ret == -EAGAIN);

        status = HAL_I2C_Slave_Transmit(&g_data.hi2c, g_data.tx_buf,
                                        PKT_SIZE(g_data.pl_size),
                                        I2C_TIMEOUT_MS);
    } else {
        /* Write transfer, slave enters receiver mode */

        do {
            ret = state_funcs_tbl[g_data.txn_state].start_rx(&g_data);
        } while (ret == -EAGAIN);

        status = HAL_I2C_Slave_Receive(&g_data.hi2c, g_data.rx_buf,
                                       PKT_SIZE(g_data.pl_size),
                                       I2C_TIMEOUT_MS);
    }

    if (status == HAL_OK) {
        do {
            ret = state_funcs_tbl[g_data.txn_state].stop_success(&g_data);
        } while (ret == -EAGAIN);
    } else {
        dbgprintx32("I2C error: 0x", status, "\r\n");

        /* Toggle peripheral enable to get I2C back into known state */
        g_data.hi2c.Instance->CR1 &= ~I2C_CR1_PE;
        g_data.hi2c.Instance->CR1 |= I2C_CR1_PE;

        do {
            ret = state_funcs_tbl[g_data.txn_state].stop_error(&g_data);
        } while (ret == -EAGAIN);
    }
}
