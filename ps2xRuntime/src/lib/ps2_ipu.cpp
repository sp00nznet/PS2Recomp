#include "ps2_ipu.h"

#include "runtime/ps2_memory.h"
#include "ipu/IPU.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <cstring>
#include <memory>
#include <vector>

namespace
{
    constexpr uint32_t kFromIpu = 0x1000B000u; // DMA channel 3
    constexpr uint32_t kToIpu = 0x1000B400u;   // DMA channel 4
    constexpr uint32_t kMadr = 0x10u, kQwc = 0x20u, kTadr = 0x30u;
    constexpr uint32_t kIntcIpu = 8u;
    constexpr uint32_t kRamMask = 0x01FFFFFFu;
    constexpr uint32_t kSprMask = 0x3FFFu;

    struct Channel
    {
        bool active = false;
        bool chain = false;
        bool lastTag = false; // the tag whose data is being sent ends the chain
        uint32_t chcr = 0;
        uint32_t madr = 0;
        uint32_t qwc = 0;
        uint32_t tadr = 0;
        std::vector<uint32_t> callStack;
    };

    struct State
    {
        PS2Memory *memory = nullptr;
        std::unique_ptr<CIPU> ipu;
        Channel to;
        Channel from;
        uint8_t *pioOut = nullptr; // set while the guest reads IPU_OUT_FIFO directly
        bool pioOutFilled = false;
    };

    State g_state;

    // ponytail: PS2X_TRACE_IPU=N prints the first N IPU commands, kicks and completions.
    bool trace()
    {
        static int budget = [] {
            const char *v = std::getenv("PS2X_TRACE_IPU");
            return v ? std::atoi(v) : 0;
        }();
        return budget > 0 && budget-- > 0;
    }

    uint8_t *guestPointer(PS2Memory &memory, uint32_t address)
    {
        if (address & 0x80000000u)
            return memory.getScratchpad() + (address & kSprMask);
        return memory.getRDRAM() + (address & kRamMask);
    }

    void storeChannel(PS2Memory &memory, uint32_t base, const Channel &channel)
    {
        memory.writeIORegister(base + kMadr, channel.madr);
        memory.writeIORegister(base + kQwc, channel.qwc);
        memory.writeIORegister(base + kTadr, channel.tadr);
    }

    // ponytail: completes the transfer when the data is done; DMA stalls and the
    // per-tag IRQ are not modelled beyond ending the chain on TIE+IRQ.
    void finish(PS2Memory &memory, uint32_t base, Channel &channel, uint32_t bit)
    {
        if (trace())
            std::cerr << "[ipu] dma ch" << bit << " done madr=0x" << std::hex << channel.madr << std::dec << std::endl;
        channel.active = false;
        storeChannel(memory, base, channel);
        memory.signalDmacCompletion(bit);
    }

    // Source-chain DMAtag (EE manual): QWC 0-15, ID 28-30, IRQ 31, ADDR 32-62, SPR 63.
    bool nextTag(PS2Memory &memory, Channel &channel)
    {
        if (channel.lastTag)
            return false;
        uint64_t tag = 0;
        std::memcpy(&tag, guestPointer(memory, channel.tadr), sizeof(tag));
        const uint32_t qwc = static_cast<uint32_t>(tag & 0xFFFFu);
        const uint32_t id = static_cast<uint32_t>((tag >> 28) & 7u);
        const bool irq = ((tag >> 31) & 1u) != 0u;
        const uint32_t addr = static_cast<uint32_t>(tag >> 32) & 0xFFFFFFF0u; // bit 31 = scratchpad
        const bool tie = (channel.chcr & (1u << 7)) != 0u;
        channel.qwc = qwc;
        const uint32_t after = channel.tadr + 16u;
        switch (id)
        {
        case 0: // refe
            channel.madr = addr;
            channel.lastTag = true;
            break;
        case 1: // cnt
            channel.madr = after;
            channel.tadr = after + qwc * 16u;
            break;
        case 2: // next
            channel.madr = after;
            channel.tadr = addr;
            break;
        case 3: // ref
        case 4: // refs
            channel.madr = addr;
            channel.tadr = after;
            break;
        case 5: // call
            channel.madr = after;
            channel.callStack.push_back(after + qwc * 16u);
            channel.tadr = addr;
            break;
        case 6: // ret
            channel.madr = after;
            if (channel.callStack.empty())
                channel.lastTag = true;
            else
            {
                channel.tadr = channel.callStack.back();
                channel.callStack.pop_back();
            }
            break;
        default: // end
            channel.madr = after;
            channel.lastTag = true;
            break;
        }
        if (irq && tie)
            channel.lastTag = true;
        // CHCR keeps the upper 16 bits of the last tag.
        channel.chcr = (channel.chcr & 0xFFFFu) | static_cast<uint32_t>(tag & 0xFFFF0000u);
        return true;
    }

    uint32_t receiveFromIpu(const void *data, uint32_t qwc)
    {
        State &state = g_state;
        if (state.pioOut)
        {
            if (state.pioOutFilled || qwc == 0u)
                return 0u;
            std::memcpy(state.pioOut, data, 16u);
            state.pioOutFilled = true;
            return 1u;
        }
        Channel &from = state.from;
        if (!from.active || !state.memory)
            return 0u;
        const uint32_t n = std::min(qwc, from.qwc);
        std::memcpy(guestPointer(*state.memory, from.madr), data, n * 16u);
        from.madr += n * 16u;
        from.qwc -= n;
        if (from.qwc == 0u)
            finish(*state.memory, kFromIpu, from, 3u);
        else
            storeChannel(*state.memory, kFromIpu, from);
        return n;
    }

    CIPU &ipu(PS2Memory &memory)
    {
        State &state = g_state;
        state.memory = &memory;
        if (!state.ipu)
        {
            // The decoder outlives any one PS2Memory (tests make several), so the IRQ goes to
            // whichever memory last touched the IPU rather than to a captured reference.
            state.ipu = std::make_unique<CIPU>([]() {
                if (g_state.memory)
                    g_state.memory->raiseIntc(kIntcIpu);
            });
            state.ipu->Reset();
            state.ipu->SetDMA3ReceiveHandler(receiveFromIpu);
        }
        return *state.ipu;
    }

    bool feed(PS2Memory &memory, CIPU &decoder)
    {
        Channel &to = g_state.to;
        bool progressed = false;
        while (to.active)
        {
            if (to.qwc == 0u)
            {
                if (!to.chain || !nextTag(memory, to))
                {
                    finish(memory, kToIpu, to, 4u);
                    return true;
                }
                continue;
            }
            const uint32_t taken = decoder.ReceiveDMA4(to.madr & (0x80000000u | kRamMask), to.qwc, false, memory.getRDRAM(), memory.getScratchpad());
            if (taken == 0u)
                break; // input FIFO full
            to.madr += taken * 16u;
            to.qwc -= taken;
            progressed = true;
        }
        if (progressed && to.active)
            storeChannel(memory, kToIpu, to);
        return progressed;
    }
}

namespace ps2_ipu
{
    bool isRegister(uint32_t address)
    {
        return (address >= 0x10002000u && address < 0x10002040u) ||
               (address >= 0x10007000u && address < 0x10007020u);
    }

    void pump(PS2Memory &memory)
    {
        if (!g_state.ipu && !g_state.to.active && !g_state.from.active)
            return;
        CIPU &decoder = ipu(memory);
        for (int guard = 0; guard < 4096; ++guard)
        {
            bool progressed = feed(memory, decoder);
            if (decoder.WillExecuteCommand())
            {
                decoder.CountTicks(0x7FFFFFFF); // no decode latency
                const bool wasBusy = decoder.WillExecuteCommand();
                decoder.ExecuteCommand();
                progressed = progressed || (wasBusy && !decoder.WillExecuteCommand());
            }
            if (decoder.HasPendingOUTFIFOData() && g_state.from.active)
            {
                decoder.FlushOUTFIFOData();
                progressed = true;
            }
            if (!progressed)
                break;
        }
    }

    uint32_t read32(PS2Memory &memory, uint32_t address)
    {
        pump(memory);
        return ipu(memory).GetRegister(address);
    }

    void write32(PS2Memory &memory, uint32_t address, uint32_t value)
    {
        if (address == 0x10002000u && trace())
            std::cerr << "[ipu] cmd 0x" << std::hex << value << std::dec << std::endl;
        ipu(memory).SetRegister(address, value);
        pump(memory);
    }

    void readOutFifo(PS2Memory &memory, uint8_t qword[16])
    {
        CIPU &decoder = ipu(memory);
        pump(memory);
        std::memset(qword, 0, 16);
        g_state.pioOut = qword;
        g_state.pioOutFilled = false;
        if (decoder.HasPendingOUTFIFOData())
            decoder.FlushOUTFIFOData();
        g_state.pioOut = nullptr;
        pump(memory);
    }

    void writeInFifo(PS2Memory &memory, const uint8_t qword[16])
    {
        CIPU &decoder = ipu(memory);
        for (uint32_t i = 0; i < 4; ++i)
        {
            uint32_t word = 0;
            std::memcpy(&word, qword + i * 4u, 4u);
            decoder.SetRegister(0x10007010u + i * 4u, word);
        }
        pump(memory);
    }

    void kick(PS2Memory &memory, uint32_t channelBase, uint32_t chcr)
    {
        Channel next;
        next.madr = memory.readIORegister(channelBase + kMadr);
        next.qwc = memory.readIORegister(channelBase + kQwc) & 0xFFFFu;
        next.tadr = memory.readIORegister(channelBase + kTadr);
        next.chcr = chcr;
        next.chain = ((chcr >> 2) & 3u) == 1u && channelBase == kToIpu;
        next.active = true;
        Channel &channel = channelBase == kToIpu ? g_state.to : g_state.from;
        channel = next;
        if (trace())
            std::cerr << "[ipu] kick 0x" << std::hex << channelBase << " chcr=0x" << chcr << " madr=0x" << channel.madr
                      << " qwc=" << std::dec << channel.qwc << " tadr=0x" << std::hex << channel.tadr << std::dec << std::endl;
        ipu(memory);
        pump(memory);
    }
}
