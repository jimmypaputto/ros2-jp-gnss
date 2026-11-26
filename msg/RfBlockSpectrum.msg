# Jimmy Paputto 2026
# Spectrum of one RF block (UBX-MON-SPAN)

uint8 id                 # RF block index, matches RfBlock.band
uint8[] data             # amplitude bins as sent by the receiver
uint32 span              # Hz
uint32 resolution        # Hz
uint32 center_freq       # Hz
uint8 gain               # receiver programmable gain, dB
