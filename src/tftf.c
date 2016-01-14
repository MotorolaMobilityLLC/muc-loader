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
bool valid_tftf_section(tftf_section_descriptor * section,
                        tftf_header * header,
                        bool * section_contains_start,
                        bool * end_of_sections) {
    uint32_t    section_start;
    uint32_t    section_end;
    uint32_t    other_section_start;
    uint32_t    other_section_end;
    tftf_section_descriptor * other_section;

    if (!valid_tftf_type(section->section_type)) {
        dbgprint("BRE_TFTF_HEADER_TYPE");
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
        dbgprint("BRE_TFTF_COMPRESSION_BAD");
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
            dbgprint("BRE_TFTF_COLLISION");
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
bool valid_tftf_header(tftf_header * header) {
    tftf_section_descriptor * section;
    bool section_contains_start = false;
    bool end_of_sections = false;
    int i;

    /* Verify the sentinel */
    for (i = 0; i < TFTF_SENTINEL_SIZE; i++) {
        if (header->sentinel_value[i] != tftf_sentinel[i]) {
            dbgprint("BRE_TFTF_SENTINEL");
            return false;
        }
    }

    if (header->header_size != TFTF_HEADER_SIZE) {
        dbgprint("BRE_TFTF_HEADER_SIZE");
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
        dbgprint("BRE_TFTF_NO_TABLE_END");
        return false;
    }

    /* Verify that, if this TFTF has a start address, it falls in one of our code sections. */
    if ((header->start_location != 0) && !section_contains_start) {
        dbgprint("BRE_TFTF_START_NOT_IN_CODE");
        return false;
    }

    /*
     * Verify that the remainder of the header (i.e., unused section
     * descriptors and the padding) is zero-filled
     */
    if (!is_constant_fill((uint8_t *)section,
                          (uint32_t)&header[1] - (uint32_t)section,
                          0x00)) {
        dbgprint("BRE_TFTF_NON_ZERO_PAD");
        return false;
    }

    return true;
}

uint8_t get_section_index(uint8_t section_type, tftf_section_descriptor *section)
{
    uint16_t index_count = 0;
    while(section->section_type != section_type) {
        if (section->section_type == TFTF_SECTION_END) {
            if ( index_count == 0) {
                return TFTF_SECTION_END;
            } else {
                break;
            }
        }
        index_count++;
        section++;
    }
    return index_count;
}
