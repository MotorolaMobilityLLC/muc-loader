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
#include "tftf.h"
#include "debug.h"
#include "utils.h"
#include "crypto.h"
#include <errno.h>
#include "boot_main.h"

#define NEW_VALIDATION

/**
 * Crypto state is used when parsing TFTF image:
 * 1. When start to parse a TFTF image, the crypto state is set to INIT
 * 2. When the first unsigned section is found in the TFTF header, the crypto
 *    state is set to HASHING and the data in header (up to but not include
 *    the first unsigned section) is hashed. And in this state, data of all
 *    sections loaded were hashed, until the first unsigned section
 * 3. Before the first unsigned section data is loaded, the crypto state is
 *    set to HASHED, and hash digest is retrieved.
 * 4. After crypto state becomes HASHED, each signature section is used to
 *    verify the TFTF signed data. If any of the signature is able to verify
 *    the data, the crypto state is set to VERIFIED
 * At the end of processing a signed TFTF image, crypto state VERIFIED means
 * this is a trusted image. Crypto state HASHED means it is a corrupted image.
 */
typedef enum {
    CRYPTO_STATE_INIT,
    CRYPTO_STATE_HASHING,
    CRYPTO_STATE_HASHED,
    CRYPTO_STATE_VERIFIED
} crypto_processing_state;

typedef struct {
    tftf_header header;
    crypto_processing_state crypto_state;
    unsigned char hash[HASH_DIGEST_SIZE];
    tftf_signature signature;
    bool contain_signature;
} tftf_processing_state;

static const char tftf_sentinel[] = TFTF_SENTINEL_VALUE;

static tftf_processing_state tftf;

/*
 * Prototypes
 */

/**
 * @brief Determine if the TFTF section type is valid
 *
 * @param section_type The section type to check
 *
 * @returns True if a valid section type, false otherwise
 */
bool valid_tftf_type(uint32_t section_type) {
     return (((section_type >= TFTF_SECTION_RAW_CODE) &&
              (section_type <= TFTF_SECTION_MANIFEST)) ||
             (section_type == TFTF_SECTION_SIGNATURE) ||
             (section_type == TFTF_SECTION_CERTIFICATE) ||
             (section_type == TFTF_SECTION_END));
}

/**
 * @brief Validate a TFTF section descriptor
 *
 * @param section The TFTF section descriptor to validate
 * @param header The TFTF header to which it belongs
 * @param section_contains_start Pointer to a flag that will be set if the
 *        image entry point lies within this section. (Untouched if not)
 * @param end_of_sections Pointer to a flag that will be set if the section
 *        type is the end-of-section-table marker. (Untouched if not)
 *
 * @returns True if valid section, false otherwise
 */
bool valid_tftf_section(const tftf_section_descriptor * section,
                        const tftf_header * header,
                        bool * section_contains_start,
                        bool * end_of_sections) {
    uint32_t    section_start;
    uint32_t    section_end;
    uint32_t    other_section_start;
    uint32_t    other_section_end;
    const tftf_section_descriptor * other_section;

    if (!valid_tftf_type(section->section_type)) {
        dbgprint("BRE_TFTF_HEADER_TYPE\r\n");
        return false;
    }

    /* Is this the end-of-table marker? */
    if (section->section_type == TFTF_SECTION_END) {
        *end_of_sections = true;
        return true;
    }

    /*
     * Convert the section limits to absolute addresses to compare against
     * absolute addresses found in the TFTF header.
     */
    section_start = section->section_load_address;
    section_end = section_start + section->section_expanded_length;

    if (section_start == DATA_ADDRESS_TO_BE_IGNORED) {
        return true;
    }

    /* Verify the expanded/compressed lengths are sane */
    if (section->section_expanded_length < section->section_length) {
        dbgprint("BRE_TFTF_COMPRESSION_BAD\r\n");
        return false;
    }

    /* Does the section contain the entry point? */
    if ((header->start_location >= section_start) &&
        (header->start_location < section_end) &&
        (section->section_type == TFTF_SECTION_RAW_CODE)) {
        *section_contains_start = true;
    }

    /*
     * Check this section for collision against all following sections.
     * Since we're called in a scanning fashion from the start to the end of
     * the sections array, all sections before us have already validated that
     * they don't collide with us.
     *
     * Overlap is determined to be "non-disjoint" sections
     */
    for (other_section = section + 1;
         ((other_section < &header->sections[TFTF_MAX_SECTIONS]) &&
          (other_section->section_type != TFTF_SECTION_END) &&
          (other_section->section_load_address != DATA_ADDRESS_TO_BE_IGNORED));
         other_section++) {
        other_section_start = other_section->section_load_address;
        other_section_end = other_section_start +
                            other_section->section_expanded_length;
        if ((other_section->section_type != TFTF_SECTION_END) &&
            (!((other_section_end < section_start) ||
            (other_section_start >= section_end)))) {
            dbgprint("BRE_TFTF_COLLISION\r\n");
            return false;
        }
    }

    return true;
}

/**
 * @brief Validate a TFTF header
 *
 * @param header The TFTF header to validate
 *
 * @returns True if valid TFTF header, false otherwise
 */
bool valid_tftf_header(const tftf_header * header) {
    const tftf_section_descriptor * section;
    bool section_contains_start = false;
    bool end_of_sections = false;
    int i;

    /* Verify the sentinel */
    for (i = 0; i < TFTF_SENTINEL_SIZE; i++) {
        if (header->sentinel_value[i] != tftf_sentinel[i]) {
            dbgprint("BRE_TFTF_SENTINEL\r\n");
            return false;
        }
    }

    if (header->header_size != TFTF_HEADER_SIZE) {
        dbgprint("BRE_TFTF_HEADER_SIZE\r\n");
        return false;
    }

    /* Verify all of the sections */
    for (section = &header->sections[0];
         (section < &header->sections[TFTF_MAX_SECTIONS]) && !end_of_sections;
         section++) {
        if (!valid_tftf_section(section, header, &section_contains_start,
                                &end_of_sections)) {
            /* (valid_tftf_section took care of error reporting) */
            return false;
        }
    }
    if (!end_of_sections) {
        dbgprint("BRE_TFTF_NO_TABLE_END\r\n");
        return false;
    }

    /* Verify that, if this TFTF has a start address, it falls in one of our code sections. */
    if ((header->start_location != 0) && !section_contains_start) {
        dbgprint("BRE_TFTF_START_NOT_IN_CODE\r\n");
        return false;
    }

    /*
     * Verify that the remainder of the header (i.e., unused section
     * descriptors and the padding) is zero-filled
     */
    if (!is_constant_fill((uint8_t *)section,
                          (uint32_t)&header[1] - (uint32_t)section,
                          0x00)) {
        dbgprint("BRE_TFTF_NON_ZERO_PAD\r\n");
        return false;
    }

    return true;
}

uint8_t get_section_index(uint8_t section_type, const tftf_section_descriptor *section)
{
    uint16_t ndx = 0;

    for (ndx = 0; ndx < TFTF_MAX_SECTIONS; ndx++) {
        if (section->section_type == section_type) {
            return ndx;
        }
        if (section->section_type == TFTF_SECTION_END) {
            return TFTF_SECTION_END;
        }
        section++;
    }
    return TFTF_SECTION_END;
}

int validate_image_signature(const tftf_header *tf_header, uint16_t *secureIndex)
{
    tftf_section_descriptor *section;
    uint32_t unipro_mid = 0;
    uint32_t unipro_pid = 0;
    uint32_t ara_vid;
    uint32_t ara_pid;
    uint16_t sIndex;

    crypto_init();

    memcpy(&tftf.header, tf_header, sizeof(tftf_header));

    tftf.crypto_state = CRYPTO_STATE_INIT;
    tftf.contain_signature = false;

    if(!valid_tftf_header(tf_header))
    {
        return -1;
    }

    get_chip_id(&unipro_mid, &unipro_pid);
    get_board_id(&ara_vid, &ara_pid);
    if (((tftf.header.unipro_mid != 0) &&
         (tftf.header.unipro_mid != unipro_mid)) ||
        ((tftf.header.unipro_pid != 0) &&
         (tftf.header.unipro_pid != unipro_pid)) ||
        ((tftf.header.ara_vid != 0) &&
         (tftf.header.ara_vid != ara_vid)) ||
        ((tftf.header.ara_pid != 0) &&
         (tftf.header.ara_pid != ara_pid))) {
        return -1;
    }

    /*
     * Process the TFTF sections
     */
    section = &tftf.header.sections[0];
    while(1) {
        if ((uint32_t)section - (uint32_t)&tftf.header >= TFTF_HEADER_SIZE) {
            return -1;
        }

        if (section->section_type == TFTF_SECTION_END) {
            break;
        }

        switch (section->section_type) {
        case TFTF_SECTION_SIGNATURE:
            tftf.contain_signature = true;
            /* fall through */
        case TFTF_SECTION_CERTIFICATE:
            if (tftf.crypto_state == CRYPTO_STATE_INIT) {
                uint32_t header_hash_len;

                /**
                 * Found the first section of type 0x80 and above (i.e.,
                 * signature or certificate), start by hashing all of the
                 * header up to but not including the first unsigned section
                 */
                hash_start();
                tftf.crypto_state = CRYPTO_STATE_HASHING;
                header_hash_len = (uint32_t)section - (uint32_t)&tftf.header;
                hash_update((unsigned char *)&tftf.header, header_hash_len);
            }
            break;

        case TFTF_SECTION_COMPRESSED_CODE:
        case TFTF_SECTION_COMPRESSED_DATA:
            return -1;

        default:
            if (tftf.crypto_state == CRYPTO_STATE_HASHING) {
                return -1;
            }
            break;
        }
        section++;
    }

    if (!tftf.contain_signature) {
        dbgprint("no signature\r\n");
        return -1;
    }

    /* the header is validated */
    section = &tftf.header.sections[0];
    sIndex = get_section_index(TFTF_SECTION_RAW_CODE, section);
    if(sIndex == TFTF_SECTION_END)
    {
        return -1;
    }

    section = &tftf.header.sections[sIndex];
    hash_update((unsigned char *)section->section_load_address, section->section_length);

    if (tftf.crypto_state == CRYPTO_STATE_HASHING) {
        hash_final(tftf.hash);
        tftf.crypto_state = CRYPTO_STATE_HASHED;
    }

    memcpy((uint8_t *)&tftf.signature, (uint8_t *)(section->section_load_address + section->section_length),
            sizeof(tftf.signature));

    if (tftf.crypto_state == CRYPTO_STATE_HASHED) {
        if (verify_signature(tftf.hash, &tftf.signature) == 0) {
            tftf.crypto_state = CRYPTO_STATE_VERIFIED;
        } else {
            sIndex = TFTF_SECTION_END;
            return -1;
        }
    } else {
        sIndex = TFTF_SECTION_END;
        return -1;
    }

    *secureIndex = sIndex;
    return 0;
}

uint32_t tftf_get_pid(const tftf_header *header)
{
    if (header)
        return header->ara_pid;

    return 0xffffffff;
}

uint32_t tftf_get_vid(const tftf_header *header)
{
    if (header)
        return header->ara_vid;

    return 0xffffffff;
}

int tftf_get_load_addr(const tftf_header *header, uint8_t section_type, uint32_t *addr)
{
    const tftf_section_descriptor *section;
    bool section_contains_start = false;
    bool end_of_sections = false;
    uint16_t sIndex = 0;

    sIndex = get_section_index(section_type, &header->sections[0]);
    if(sIndex == TFTF_SECTION_END)
        return -1;
    section = &header->sections[sIndex];

    if (!valid_tftf_section(section, header, &section_contains_start, &end_of_sections))
        return -1;

    *addr = section->section_load_address;

    return 0;
}
