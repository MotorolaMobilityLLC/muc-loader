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

#ifndef __COMMON_INCLUDE_TFTF_H
#define __COMMON_INCLUDE_TFTF_H

#include <stdint.h>
#include <stdbool.h>


#define TFTF_HEADER_SIZE                  512
/* following values are derived from TFTF_HEADER_SIZE */
#define TFTF_MAX_SECTIONS                 20
#define TFTF_RESERVED                     4

/**
 * @brief TFTF Sentinel value "TFTF"
 *
 * Note: string must be in reverse order so that it looks OK on a little-
 * endian dump.
 */
#define TFTF_SENTINEL_SIZE                4
#define TFTF_SENTINEL_VALUE               "TFTF"
#define TFTF_SENTINEL                     0x46544654
/* Compile-time test to verify consistency of _SIZE and _VALUE */
typedef char ___tftf_sentinel_test[(TFTF_SENTINEL_SIZE ==
                                    sizeof(TFTF_SENTINEL_VALUE) - 1) ?
                                   1 : 0];

/* Section types */
#define TFTF_SECTION_END                  0xFE
#define TFTF_SECTION_RAW_CODE             1
#define TFTF_SECTION_RAW_DATA             2
    #define DATA_ADDRESS_TO_BE_IGNORED    0xFFFFFFFF
#define TFTF_SECTION_COMPRESSED_CODE      3
#define TFTF_SECTION_COMPRESSED_DATA      4
#define TFTF_SECTION_MANIFEST             5
#define TFTF_SECTION_SIGNATURE            0x80
#define TFTF_SECTION_CERTIFICATE          0x81

typedef struct {
    uint8_t section_type;
    uint8_t section_class[3];
    uint32_t section_id;
    uint32_t section_length;
    uint32_t section_load_address;
    uint32_t section_expanded_length;
} __attribute__ ((packed)) tftf_section_descriptor;

#ifndef __GNUC__
#pragma anon_unions
#endif
typedef union {
    struct {
        char sentinel_value[TFTF_SENTINEL_SIZE];
        uint32_t header_size;
        char build_timestamp[16];
        char firmware_package_name[48];
        uint32_t package_type;
        uint32_t start_location;
        uint32_t unipro_mid;
        uint32_t unipro_pid;
        uint32_t ara_vid;
        uint32_t ara_pid;
        uint32_t reserved[TFTF_RESERVED];
        tftf_section_descriptor sections[TFTF_MAX_SECTIONS];
    };
    unsigned char buffer[TFTF_HEADER_SIZE];
} __attribute__ ((packed)) tftf_header;
/* Compile-time test hack to verify the header is TFTF_HEADER_SIZE bytes */
typedef char ___tftf_header_test[(sizeof (tftf_header) == TFTF_HEADER_SIZE) ?
                                 1 : -1];

typedef struct {
    uint32_t length;            /* total size of tftf_signature structure */
    uint32_t type;              /* Some ALGORITHM_TYPE_xxx from crypto.h */
    char key_name[96];
    unsigned char signature[256];
} __attribute__ ((packed)) tftf_signature;

typedef void (*image_entry_func)(void);

uint8_t get_section_index(uint8_t section_type, const tftf_section_descriptor *section);
bool valid_tftf_header(const tftf_header * header);
int validate_image_signature(const tftf_header *tftf, uint16_t *secureIndex);
int tftf_get_load_addr(const tftf_header *header, uint8_t section_type, uint32_t *addr);
uint32_t tftf_get_pid(const tftf_header *tftf);
uint32_t tftf_get_vid(const tftf_header *tftf);

#endif /* __COMMON_INCLUDE_TFTF_H */
