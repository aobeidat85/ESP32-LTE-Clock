# SPDX-License-Identifier: GPL-3.0-only
from functools import lru_cache
import numpy as np

def crc16(bits):
    reg=0
    for bit in bits:
        top=(reg>>15)^int(bit)
        reg=(reg<<1)&65535
        if top: reg^=0x1021
    return np.array([(reg>>i)&1 for i in range(15,-1,-1)])

@lru_cache(maxsize=1)
def trellis():
    outputs=np.empty((64,2,3),int)
    for state in range(64):
        for bit in (0,1):
            reg=(state<<1)|bit
            # Reverse the standard's 133/171/165 taps for this LSB-newest register.
            outputs[state,bit]=[(reg&p).bit_count()%2 for p in (0x6d,0x4f,0x57)]
    return outputs
