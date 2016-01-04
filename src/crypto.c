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
#include "bootrom.h"
#include "tftf.h"
#include "debug.h"
#include "crypto.h"

#include "../vendors/MIRACL/bootrom.c"

void (*sha256_init_func)(sha256 *sh);
void (*sha256_process_func)(sha256 *sh,int byte);
void (*sha256_hash_func)(sha256 *sh,char hash[32]);
int (*rsa2048_verify_func)(char digest[], char signature[], char public_key[]);

#ifndef _NOCRYPTO
static sha256 shctx;
#endif

/**
 * @brief Initialize the SHA hash
 *
 * @param none
 *
 * @returns Nothing
 */
void hash_start(void) {
#ifndef _NOCRYPTO
    sha256_init_func(&shctx);
#endif
}


/**
 * @brief Add data to the SHA hash
 *
 * @param data Pointer to the run of data to add to the hash.
 * @param datalen The length in bytes of the data run.
 *
 * @returns Nothing
 */
void hash_update(unsigned char *data, uint32_t datalen) {
#ifndef _NOCRYPTO
    uint32_t i;
    for (i = 0; i < datalen; i++) {
        sha256_process_func(&shctx, data[i]);
    }
#endif
}


/**
 * @brief Finalize the SHA hash and return the digest
 *
 * @param digest Pointer to the digest buffer
 *
 * @returns Nothing
 */
void hash_final(unsigned char *digest) {
#ifndef _NOCRYPTO
    sha256_hash_func(&shctx,(char*)digest);
#endif
}

static int find_public_key(tftf_signature *signature, const unsigned char **key) {
    uint32_t *ps, *pk;
    int i, k;
    uint32_t size = (sizeof(public_keys[0]) -
                     sizeof(public_keys[0].key)) /
                    sizeof(uint32_t);

    ps = (uint32_t *)&(signature->type);

    for (k = 0; k < number_of_public_keys; k++) {
        if (chip_is_key_revoked(k)) {
            dbgprintx32("Key ", k, " revoked\n");
            continue;
        }
        pk = (uint32_t *)&public_keys[k];
        for (i = 0; i < size; i++) {
            if (ps[i] != pk[i]) {
                break;
            }
        }
        if (i >= size) {
            dbgprint("Found pub. key for this sig.\n");
            *key = public_keys[k].key;
            return 0;
        }
    }

    dbgprint("Failed to find pub. key for this sig.\n");
    return -1;
}

/**
 * @brief Verify a SHA digest against a signature
 *
 * @param digest The SHA digest obtained from hash-final.
 * @param signature A pointer to the TFTF signature block.
 *
 * @returns 0 if the digest verifies, non-zero otherwise
 */
int verify_signature(unsigned char *digest, tftf_signature *signature) {
#ifdef _NOCRYPTO
/* little ending 32bit word for "FAIL" */
#define _SIM_KEYNAME_FAILURE_SENTINEL 0x4C494146
    uint32_t *pname = (uint32_t *)(signature->key_name);
    if (*pname == _SIM_KEYNAME_FAILURE_SENTINEL) {
        return -1;
    }
    return 0;
#endif
    int ret;
    const unsigned char *public_key;

    if (find_public_key(signature, &public_key)) {
        return -1;
    }

    ret = rsa2048_verify_func((char *)digest,
                              (char *)public_key,
                              (char *)signature->signature) ? 0 : -1;

    if (ret) {
        dbgprint("Signature failed\n");
    } else {
        dbgprint("Signature verified\n");
#if BOOT_STAGE == 1
        communication_area *p = (communication_area *)&_communication_area;
        memcpy(p->stage_2_firmware_identity,
               digest,
               sizeof(p->stage_2_firmware_identity));
        memcpy(p->stage_2_validation_key_name,
               signature->key_name,
               sizeof(p->stage_2_validation_key_name));
#endif
    }

    return ret;
}

void crypto_init(void) {
#if BOOT_STAGE == 1
    set_shared_function(SHARED_FUNCTION_SHA256_INIT, shs256_init);
    set_shared_function(SHARED_FUNCTION_SHA256_PROCESS, shs256_process);
    set_shared_function(SHARED_FUNCTION_SHA256_HASH, shs256_hash);
    set_shared_function(SHARED_FUNCTION_RSA2048_VERIFY, rsa_verify);
#endif
    sha256_init_func = get_shared_function(SHARED_FUNCTION_SHA256_INIT);
    sha256_process_func = get_shared_function(SHARED_FUNCTION_SHA256_PROCESS);
    sha256_hash_func = get_shared_function(SHARED_FUNCTION_SHA256_HASH);
    rsa2048_verify_func = get_shared_function(SHARED_FUNCTION_RSA2048_VERIFY);
}
