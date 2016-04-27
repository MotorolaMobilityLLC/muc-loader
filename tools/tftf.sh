#
# Copyright (c) 2016 Motorola Mobility, LLC.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
source .version
source .config

BIN=$1

MAJOR=$(grep CONFIG_VERSION_MAJOR .version | cut -f2 -d"=" )
MINOR=$(grep CONFIG_VERSION_MINOR .version | cut -f2 -d"=" )
printf -v VERSION  '%08x' $(($(($(grep CONFIG_VERSION_MAJOR .version | cut -f2 -d"=" )<<16)) + $(grep CONFIG_VERSION_MINOR .version | cut -f2 -d"=" )))

function tftf_muc()
{
    CHP_MFG=${CONFIG_ARCH_UNIPRO_MFG##0x}
    CHP_PID=${CONFIG_ARCH_UNIPRO_PID##0x}
    BRD_VID=${CONFIG_ARCH_BOARDID_VID##0x}
    BRD_PID=${CONFIG_ARCH_BOARDID_PID##0x}
    TFTF_FILENAME="upd-${CHP_MFG}-${CHP_PID}-${BRD_VID}-${BRD_PID}-01.tftf"
    start_address="$(grep ' _Start$' out/System.map  | cut -d\  -f1)"
    load_address="$(grep g_flashVectors out/System.map| cut -d\  -f1)"

    create-tftf -v \
          --code ${BIN} \
          --out out/${TFTF_FILENAME} \
          --start "0x${start_address}" \
          --load  "0x${load_address} " \
          --unipro-mfg ${CONFIG_ARCH_UNIPRO_MFG} \
          --unipro-pid ${CONFIG_ARCH_UNIPRO_PID} \
          --ara-vid ${CONFIG_ARCH_BOARDID_VID}\
          --ara-pid ${CONFIG_ARCH_BOARDID_PID} \
          --ara-stage "01" \
          --ara-reserved-tftf 0x${VERSION}
}

tftf_muc ${BIN}
