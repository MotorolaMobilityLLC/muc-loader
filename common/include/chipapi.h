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

#ifndef __COMMON_INCLUDE_CHIPAPI_H
#define __COMMON_INCLUDE_CHIPAPI_H

#include <stdint.h>
#include <stddef.h>

void chip_init(void);

/* For debug output */
void chip_dbginit(void);
void chip_dbgputc(int);
void chip_dbgflush(void);

/* Used when CONFIG_GPIO=y */
#ifdef CONFIG_GPIO
void chip_gpio_init(void);
uint8_t chip_gpio_get_value(uint8_t which);
void chip_gpio_set_value(uint8_t which, uint8_t value);
void chip_gpio_direction_in(uint8_t which);
void chip_gpio_direction_out(uint8_t which, uint8_t value);
#else
static inline void chip_gpio_init(void) {}
static inline uint8_t chip_gpio_get_value(uint8_t which) {return 0;}
static inline void chip_gpio_set_value(uint8_t which, uint8_t value) {}
static inline void chip_gpio_direction_in(uint8_t which) {}
static inline void chip_gpio_direction_out(uint8_t which, uint8_t value) {}
#endif

#if defined(_SIMULATION) && ((BOOT_STAGE == 1) || (BOOT_STAGE == 3))
void chip_handshake_with_test_controller(void);
void chip_signal_boot_status(uint32_t status);
#endif

int chip_validate_data_load_location(void *base, uint32_t length);

void chip_reset_before_jump(void);
void chip_jump_to_image(uint32_t start_address);

void chip_unipro_init(void);
int chip_unipro_init_cport(uint32_t cportid);
int chip_unipro_recv_cport(uint32_t *cportid);

#define ATTR_LOCAL 0
#define ATTR_PEER  1
/**
 * @brief Perform a DME get request
 * @param attr DME attribute address
 * @param val destination to read into
 * @param selector attribute selector index, or NCP_SELINDEXNULL if none
 * @param peer 1 if peer access, 0 if local
 * @param result_code destination for access result
 * @return 0
 */
int chip_unipro_attr_read(uint16_t attr,
                          uint32_t *val,
                          uint16_t selector,
                          int peer,
                          uint32_t *result_code);

/**
 * @brief Perform a DME set request
 * @param attr DME attribute address
 * @param val value to write
 * @param selector attribute selector index, or NCP_SELINDEXNULL if none
 * @param peer 1 if peer access, 0 if local
 * @param result_code destination for access result
 * @return 0
 */
int chip_unipro_attr_write(uint16_t attr,
                           uint32_t val,
                           uint16_t selector,
                           int peer,
                           uint32_t *result_code);

/**
 * @brief send data down a CPort
 * @param cportid cport to send down
 * @param buf data buffer
 * @param len size of data to send
 * @return 0 on success, <0 on error
 */
int chip_unipro_send(unsigned int cportid, const void *buf, size_t len);

/**
 * @brief handler callback for UniPro data RX
 * @param cportid cport which received data
 * @param data pointer to the data buffer
 * @param len number of bytes of data received
 * @return 0 on success, <0 on error
 */
typedef int (*unipro_rx_handler)(uint32_t cportid,
                                 void *data,
                                 size_t len);

/**
 * @brief wait for data from a cport
 * @param cportid cport for the rx
 * @param handler rx handler callback, called before RX is restarted
 */
int chip_unipro_receive(unsigned int cportid, unipro_rx_handler handler);

/**
 * @brief advertise the boot status to the switch
 * @param boot_status
 * @param result_code destination for advertisement result
 * @return 0 on success, <0 on error
 */
int chip_advertise_boot_status(uint32_t boot_status, uint32_t *result_code);

/**
 * @brief advertise the boot type to the switch
 * @param result_code destination for advertisement result
 * @return 0 on success, <0 on error
 */
int chip_advertise_boot_type(uint32_t *result_code);

/**
 * @brief reset UniPro before signalling readiness to boot firmware to switch
 */
void chip_reset_before_ready(void);

/**
 * @brief check if a crypto key has been revoked
 * @param index of the key in public_keys array
 * @return 0 indicates the key is NOT revoked
 *         1 indicates the key has been revoked so should not be used.
 */
int chip_is_key_revoked(int index);

/*
 * @brief wait for unipro link up sequence to finish
 * This is called when boot ROM needs the link to be ready
 * so never return if the link is not ready
 */
void chip_wait_for_link_up(void);

#endif /* __COMMON_INCLUDE_CHIPAPI_H */
