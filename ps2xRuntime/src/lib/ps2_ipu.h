#pragma once
// The IPU (MPEG decoder) behind PS2Memory. The decoder itself is Play!'s CIPU (src/lib/ipu,
// BSD-2-Clause); this layer feeds it from DMA channel 4 (toIPU), drains it to channel 3
// (fromIPU) and maps its registers and FIFOs. Kept out of ps2_memory.h so the generated
// code, which includes that header, does not depend on it.
#include <cstdint>

class PS2Memory;

namespace ps2_ipu
{
    bool isRegister(uint32_t address);                 // IPU_CMD/CTRL/BP/TOP and the FIFOs
    uint32_t read32(PS2Memory &memory, uint32_t address);
    void write32(PS2Memory &memory, uint32_t address, uint32_t value);
    void readOutFifo(PS2Memory &memory, uint8_t qword[16]);
    void writeInFifo(PS2Memory &memory, const uint8_t qword[16]);

    // A CHCR write that set STR on channel 3 (0x1000B000) or 4 (0x1000B400).
    void kick(PS2Memory &memory, uint32_t channelBase, uint32_t chcr);

    // Moves data between the DMA channels and the decoder and runs the current command as
    // far as the input allows. Cheap when the IPU is idle.
    void pump(PS2Memory &memory);
}
