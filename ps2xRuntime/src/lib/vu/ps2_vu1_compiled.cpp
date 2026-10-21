// Registry and run-time support for ahead-of-time compiled VU1 microprograms
// (see runtime/ps2_vu1_compiled.h and tools/vu1_recomp.py).
#include "runtime/ps2_vu1_compiled.h"
#include "runtime/ps2_vu1_compiled_run.h"
#include "runtime/gs/ps2_gif_arbiter.h"
#include "runtime/ps2_memory.h"

#include <cfenv>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace vu1c
{
    namespace
    {
        std::vector<const Program *> &programs()
        {
            static std::vector<const Program *> list;
            return list;
        }

        bool g_submitPackets = true;
        uint64_t g_packetHash = 0;
    }

    void registerProgram(const Program *program) { programs().push_back(program); }

    // Same polynomials as ps2_vu1_lower.cpp.
    float eatan(float value)
    {
        constexpr float k[] = {0.999999344348907f, -0.333298563957214f, 0.199465364217758f, -0.13085337519646f,
                               0.096420042216778f, -0.055909886956215f, 0.021861229091883f, -0.004054057877511f};
        const float sq = value * value;
        float p = k[7];
        for (int i = 6; i >= 0; --i)
            p = k[i] + sq * p;
        return 0.785398185253143f + value * p;
    }
    float esin(float value)
    {
        constexpr float k[] = {1.0f, -0.166666567325592f, 0.008333025500178f, -0.000198074136279f, 0.000002601886990f};
        const float sq = value * value;
        float p = k[4];
        for (int i = 3; i >= 0; --i)
            p = k[i] + sq * p;
        return value * p;
    }
    float eexp(float value)
    {
        constexpr float k[] = {0.249998688697815f, 0.031257584691048f, 0.002591371303424f,
                               0.000171562001924f, 0.000005430199963f, 0.000000690600018f};
        float p = k[5];
        for (int i = 4; i >= 0; --i)
            p = k[i] + value * p;
        p = 1.0f + value * p;
        p *= p;
        p *= p;
        return p != 0.0f ? 1.0f / p : std::numeric_limits<float>::max();
    }

    // Walks GIF tags from qwordAddress up to EOP (same length rules as VU1Interpreter::progressXgkick)
    // and sends the packet on PATH1 at once.
    uint32_t xgkick(Ctx &c, uint32_t qwordAddress)
    {
        static uint8_t packet[0x10000];
        uint32_t src = (qwordAddress * 16u) & 0x3FFFu, bytes = 0;
        for (;;)
        {
            if (bytes + 16 > sizeof(packet))
                break;
            uint64_t tag;
            std::memcpy(&tag, c.data + src, 8);
            const uint32_t nloop = tag & 0x7FFF, format = (tag >> 58) & 3;
            uint32_t nreg = (tag >> 60) & 0xF;
            if (nreg == 0)
                nreg = 16;
            uint64_t len = 16;
            if (format == 0)
                len += uint64_t(nloop) * nreg * 16;
            else if (format == 1)
                len += ((uint64_t(nloop) * nreg + 1) & ~1ull) * 8;
            else if (format == 2)
                len += uint64_t(nloop) * 16;
            else
                break;
            if (bytes + len > sizeof(packet))
                break;
            for (uint64_t i = 0; i < len; i += 16)
            {
                std::memcpy(packet + bytes, c.data + src, 16);
                bytes += 16;
                src = (src + 16) & 0x3FFFu;
            }
            if ((tag >> 15) & 1)
                break;
        }
        if (g_submitPackets && c.mem)
            c.mem->submitGifPacket(GifPathId::Path1, packet, bytes);
        else
            for (uint32_t i = 0; i < bytes; ++i)
                g_packetHash = (g_packetHash ^ packet[i]) * 1099511628211ull;
        return bytes / 16;
    }

    void flushAll(Ctx &c)
    {
        c.cyc += 64; // past every latency (the longest, EATAN, is 54)
        commitFlags(c);
        commitQ(c);
        commitP(c);
        c.flagHead = c.flagTail;
    }

    const Program *find(const uint8_t *code, uint64_t generation)
    {
        static uint64_t cachedGeneration = ~0ull;
        static const Program *cached = nullptr;
        if (generation != cachedGeneration)
        {
            cachedGeneration = generation;
            cached = nullptr;
            for (const Program *p : programs())
                if (std::memcmp(code, p->code, p->codeBytes) == 0)
                {
                    cached = p;
                    break;
                }
        }
        return cached;
    }

    RunResult run(const Program *program, VU1State &state, uint8_t *data, PS2Memory *mem, uint32_t pc)
    {
        Ctx c;
        c.s = &state;
        c.data = data;
        c.mem = mem;
        c.pc = pc & 0x3FFFu;
        c.workingClip = state.clip;
        const int previousRounding = std::fegetround();
        std::fesetround(FE_TOWARDZERO);
        const bool ended = program->run(c);
        std::fesetround(previousRounding);
        state.pc = c.pc;
        state.cycles += c.cyc;
        state.ebit = false;
        state.haltAfterDelaySlot = false;
        state.branchPending = false;
        return {ended, c.pc, c.cyc};
    }

    const char *name(const Program *program) { return program->name; }
    bool usesFlags(const Program *program) { return program->flags; }

    void setPacketCapture(bool capture, uint64_t *hashOut)
    {
        if (hashOut)
            *hashOut = g_packetHash;
        g_submitPackets = !capture;
        g_packetHash = 1469598103934665603ull;
    }
}
