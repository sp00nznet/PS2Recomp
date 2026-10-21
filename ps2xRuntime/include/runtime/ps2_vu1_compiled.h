// Ahead-of-time compiled VU1 microprograms.
//
// tools/vu1_recomp.py turns a VU1 code image (dumped with PS2X_VU1_DUMP) into C++ that calls
// P<upper, lower, pc, flags>() once per instruction pair. Each P instantiation decodes its
// words at compile time, so what is left at run time is the arithmetic plus a small scoreboard.
//
// Timing follows VU1Interpreter (ps2_vu1_core.cpp), which this mirrors rule for rule:
//  - VF/VI/ACC reads interlock: a pair stalls until the registers it reads are ready, so
//    results are written to the register file at issue and later readers always see them.
//    Within one pair both halves read the old registers.
//  - Q (FDIV), P (EFU), MAC/status and clip flags do not interlock: they become visible
//    latency cycles after issue, so programs can (and do) read the previous value on purpose.
//  - A VI written by an integer op is read as its old value by a branch in the next pair.
// Differences from the interpreter: XGKICK copies its whole packet at issue (the interpreter
// copies a qword every two cycles), and D/T halt bits are not supported (vu1_recomp.py refuses
// images that use them). PS2X_VU1_VERIFY=1 runs both and reports any difference.
#pragma once

#include "runtime/ps2_vu1.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <type_traits>
#include <utility>

class PS2Memory;

#if defined(_MSC_VER)
#define VU1C_INLINE __forceinline
#else
#define VU1C_INLINE inline __attribute__((always_inline))
#endif

namespace vu1c
{
    struct Ctx
    {
        VU1State *s = nullptr;
        uint8_t *data = nullptr; // 16 KB VU1 data memory
        PS2Memory *mem = nullptr;
        uint32_t pc = 0;  // in: entry pc; out: resume pc
        uint32_t cyc = 0; // cycles since this run started
        bool br = false;  // a branch in the previous pair was taken
        bool eb = false;  // the previous pair had the E bit: stop after this one
        uint32_t target = 0;

        uint32_t vfReady[32][4]{};
        uint32_t viReady[16]{};
        uint32_t accReady[4]{};

        // Branch read of a VI written in the pair before (see readBranchVi).
        uint8_t bkReg = 0;
        int32_t bkVal = 0;

        struct Flag
        {
            uint32_t ready, issue, mac, status, extraSticky, clip;
            bool writesMac, writesStatus, writesSticky, writesClip;
        };
        Flag flags[16]{};
        uint32_t flagHead = 0, flagTail = 0; // ring, in issue order
        uint32_t workingClip = 0;

        bool fdivValid = false;
        uint32_t fdivReady = 0;
        float fdivValue = 0;
        uint32_t fdivDi = 0;

        struct Efu
        {
            bool valid;
            uint32_t ready;
            float value;
        };
        Efu efu[2]{};
        uint32_t efuResourceReady = 0;
        uint32_t xgkickBusyUntil = 0;
    };

    // One compiled code image. code/codeBytes is what VU1 code memory must hold for run() to apply.
    struct Program
    {
        const char *name;
        const uint32_t *code;
        uint32_t codeBytes;
        bool (*run)(Ctx &); // false: hand back to the interpreter at c.pc
        bool flags;         // MAC/status flags are computed (the image reads them)
    };

    // Called from generated files' static initialisers.
    void registerProgram(const Program *program);
    // Defined in ps2_vu1_compiled.cpp.
    uint32_t xgkick(Ctx &c, uint32_t qwordAddress); // returns qwords sent
    void flushAll(Ctx &c);

    // ---- decode helpers (constexpr mirrors of ps2_vu1_detail.h) ----
    constexpr uint8_t dest(uint32_t i) { return (i >> 21) & 0xF; }
    constexpr uint8_t ft(uint32_t i) { return (i >> 16) & 0x1F; }
    constexpr uint8_t fs(uint32_t i) { return (i >> 11) & 0x1F; }
    constexpr uint8_t fd(uint32_t i) { return (i >> 6) & 0x1F; }
    constexpr uint8_t vit(uint32_t i) { return (i >> 16) & 0xF; }
    constexpr uint8_t vis(uint32_t i) { return (i >> 11) & 0xF; }
    constexpr uint8_t vid(uint32_t i) { return (i >> 6) & 0xF; }
    constexpr int32_t imm11(uint32_t i) { return static_cast<int32_t>(i << 21) >> 21; }
    constexpr int32_t imm15(uint32_t i) { return static_cast<int16_t>((i & 0x7FF) | ((i >> 10) & 0x7800)); }
    constexpr uint8_t lane(uint32_t component) { return static_cast<uint8_t>(8u >> component); }
    constexpr uint8_t upperSpecial(uint32_t i) { return (i & 3u) | ((i >> 4) & 0x7Cu); }

    enum Pipe : uint8_t
    {
        PNone,
        PFmac,
        PFdiv,
        PEfu,
        PIalu,
        PLsu,
        PBranch,
        PXgkick
    };

    struct VfAccess
    {
        uint8_t reg = 0, lanes = 0;
    };

    // Same fields and rules as VU1Interpreter::InstructionUsage / decode*Usage.
    struct Usage
    {
        Pipe pipe = PNone;
        uint8_t latency = 0, vfLatency = 0, viLatency = 0;
        VfAccess vfRead[2]{};
        uint8_t vfReadCount = 0;
        VfAccess vfWrite{};
        uint16_t viRead = 0, viWrite = 0;
        uint8_t accRead = 0, accWrite = 0;
        bool waitQ = false, waitP = false, delaysNextBranchRead = false, reserved = false;

        constexpr void read(uint8_t reg, uint8_t lanes)
        {
            if (reg == 0 || lanes == 0)
                return;
            for (uint8_t k = 0; k < vfReadCount; ++k)
                if (vfRead[k].reg == reg)
                {
                    vfRead[k].lanes |= lanes;
                    return;
                }
            if (vfReadCount < 2)
                vfRead[vfReadCount++] = {reg, lanes};
        }
        constexpr void write(uint8_t reg, uint8_t lanes)
        {
            if (reg != 0 && lanes != 0)
                vfWrite = {reg, lanes};
        }
        constexpr void rvi(uint8_t r)
        {
            if (r)
                viRead |= static_cast<uint16_t>(1u << r);
        }
        constexpr void wvi(uint8_t r)
        {
            if (r)
                viWrite |= static_cast<uint16_t>(1u << r);
        }
        constexpr uint8_t readLanes(uint8_t reg) const
        {
            for (uint8_t k = 0; k < vfReadCount; ++k)
                if (vfRead[k].reg == reg)
                    return vfRead[k].lanes;
            return 0;
        }
    };

    constexpr bool upperReadsAcc(uint8_t op)
    {
        return (op >= 0x08 && op <= 0x0F) || op == 0x21 || op == 0x23 || op == 0x25 || op == 0x27 || op == 0x29 ||
               op == 0x2D || op == 0x2E;
    }

    constexpr Usage upperUsage(uint32_t u)
    {
        Usage x;
        x.pipe = PFmac;
        x.latency = 4;
        const uint8_t op = u & 0x3F, d = dest(u);
        if (op <= 0x2F)
        {
            x.read(fs(u), d);
            x.write(fd(u), d);
            if (op <= 0x1B)
                x.read(ft(u), lane(op & 3));
            else if (op >= 0x28)
                x.read(ft(u), op == 0x2E ? 0xE : d);
            if (upperReadsAcc(op))
                x.accRead = d;
            return x;
        }
        if (op >= 0x3C)
        {
            const uint8_t sp = upperSpecial(u);
            const bool toAcc = sp <= 0x0F || (sp >= 0x18 && sp <= 0x1C) || sp == 0x1E || (sp >= 0x20 && sp <= 0x2A) ||
                               (sp >= 0x2C && sp <= 0x2E);
            if (toAcc)
            {
                x.read(fs(u), d);
                if (sp <= 0x1B)
                    x.read(ft(u), lane(sp & 3));
                else if (sp >= 0x28 && sp <= 0x2E)
                    x.read(ft(u), sp == 0x2E ? 0xE : d);
                x.accWrite = d;
                if ((sp >= 0x08 && sp <= 0x0F) || sp == 0x21 || sp == 0x23 || sp == 0x25 || sp == 0x27 || sp == 0x29 ||
                    sp == 0x2D)
                    x.accRead = d;
            }
            else if ((sp >= 0x10 && sp <= 0x17) || sp == 0x1D)
            {
                x.read(fs(u), d);
                x.write(ft(u), d);
            }
            else if (sp == 0x1F)
            {
                x.read(fs(u), 0xE);
                x.read(ft(u), 0x1);
            }
            else if (sp != 0x2F && sp != 0x30)
                x.reserved = true;
            return x;
        }
        x.reserved = true;
        return x;
    }

    constexpr Usage lowerUsage(uint32_t l)
    {
        Usage x;
        if (l == 0 || l == 0x8000033Cu)
            return x;
        const uint8_t hi = (l >> 25) & 0x7F, vfT = ft(l), vfS = fs(l), viT = vit(l), viS = vis(l), viD = vid(l),
                      d = dest(l);
        switch (hi)
        {
        case 0x00: x.pipe = PLsu; x.latency = 4; x.rvi(viS); x.write(vfT, d); return x;
        case 0x01: x.pipe = PLsu; x.latency = 1; x.rvi(viT); x.read(vfS, d); return x;
        case 0x04: x.pipe = PLsu; x.latency = 4; x.rvi(viS); x.wvi(viT); return x;
        case 0x05: x.pipe = PLsu; x.latency = 1; x.rvi(viS); x.rvi(viT); return x;
        case 0x08:
        case 0x09: x.pipe = PIalu; x.latency = 1; x.delaysNextBranchRead = true; x.rvi(viS); x.wvi(viT); return x;
        case 0x10:
        case 0x12:
        case 0x13: x.pipe = PIalu; x.latency = 1; x.wvi(1); return x;
        case 0x11: x.pipe = PFmac; x.latency = 4; return x;
        case 0x14:
        case 0x16:
        case 0x17: x.pipe = PIalu; x.latency = 1; x.wvi(viT); return x;
        case 0x15: x.pipe = PFmac; x.latency = 4; return x;
        case 0x18:
        case 0x1A:
        case 0x1B: x.pipe = PIalu; x.latency = 1; x.rvi(viS); x.wvi(viT); return x;
        case 0x1C: x.pipe = PIalu; x.latency = 1; x.wvi(viT); return x;
        case 0x20: x.pipe = PBranch; return x;
        case 0x21: x.pipe = PBranch; x.latency = 1; x.wvi(viT); return x;
        case 0x24: x.pipe = PBranch; x.rvi(viS); return x;
        case 0x25: x.pipe = PBranch; x.latency = 1; x.rvi(viS); x.wvi(viT); return x;
        case 0x28:
        case 0x29: x.pipe = PBranch; x.rvi(viS); x.rvi(viT); return x;
        case 0x2C:
        case 0x2D:
        case 0x2E:
        case 0x2F: x.pipe = PBranch; x.rvi(viS); return x;
        case 0x40: break;
        default: x.reserved = true; return x;
        }
        const uint8_t direct = l & 0x3F;
        if (direct == 0x30 || direct == 0x31 || direct == 0x34 || direct == 0x35)
        {
            x.pipe = PIalu; x.latency = 1; x.delaysNextBranchRead = true;
            x.rvi(viS); x.rvi(viT); x.wvi(viD);
            return x;
        }
        if (direct == 0x32)
        {
            x.pipe = PIalu; x.latency = 1; x.delaysNextBranchRead = true;
            x.rvi(viS); x.wvi(viT);
            return x;
        }
        if (direct < 0x3C)
        {
            x.reserved = true;
            return x;
        }
        const uint8_t sp = upperSpecial(l);
        const uint8_t fsfLane = lane((l >> 21) & 3), ftfLane = lane((l >> 23) & 3);
        switch (sp)
        {
        case 0x30:
        case 0x31: x.pipe = PFmac; x.latency = 4; x.read(vfS, sp == 0x31 ? 0xF : d); x.write(vfT, d); break;
        case 0x34:
        case 0x36:
            x.pipe = PLsu; x.latency = 4; x.viLatency = 1; x.delaysNextBranchRead = true;
            x.rvi(viS); x.wvi(viS); x.write(vfT, d);
            break;
        case 0x35:
        case 0x37:
            x.pipe = PLsu; x.latency = 1; x.delaysNextBranchRead = true;
            x.rvi(viT); x.wvi(viT); x.read(vfS, d);
            break;
        case 0x38: x.pipe = PFdiv; x.latency = 7; x.read(vfS, fsfLane); x.read(vfT, ftfLane); break;
        case 0x39: x.pipe = PFdiv; x.latency = 7; x.read(vfT, ftfLane); break;
        case 0x3A: x.pipe = PFdiv; x.latency = 13; x.read(vfS, fsfLane); x.read(vfT, ftfLane); break;
        case 0x3B: x.pipe = PFdiv; x.waitQ = true; break;
        case 0x3C: x.pipe = PIalu; x.latency = 1; x.delaysNextBranchRead = true; x.read(vfS, fsfLane); x.wvi(viT); break;
        case 0x3D: x.pipe = PFmac; x.latency = 4; x.rvi(viS); x.write(vfT, d); break;
        case 0x3E: x.pipe = PLsu; x.latency = 4; x.rvi(viS); x.wvi(viT); break;
        case 0x3F: x.pipe = PLsu; x.latency = 1; x.rvi(viS); x.rvi(viT); break;
        case 0x40:
        case 0x41: x.pipe = PFmac; x.latency = 4; x.write(vfT, d); break;
        case 0x42:
        case 0x43: x.pipe = PIalu; x.latency = 1; x.read(vfS, fsfLane); break;
        case 0x64: x.pipe = PFmac; x.latency = 4; x.write(vfT, d); break;
        case 0x68:
        case 0x69: x.pipe = PIalu; x.latency = 1; x.wvi(viT); break;
        case 0x6C: x.pipe = PXgkick; x.latency = 2; x.rvi(viS); break;
        case 0x70: x.pipe = PEfu; x.latency = 11; x.read(vfS, 0xE); break;
        case 0x71:
        case 0x72: x.pipe = PEfu; x.latency = 18; x.read(vfS, 0xE); break;
        case 0x73: x.pipe = PEfu; x.latency = 24; x.read(vfS, 0xE); break;
        case 0x74: x.pipe = PEfu; x.latency = 54; x.read(vfS, 0xC); break;
        case 0x75: x.pipe = PEfu; x.latency = 54; x.read(vfS, 0xA); break;
        case 0x76: x.pipe = PEfu; x.latency = 12; x.read(vfS, 0xF); break;
        case 0x77: x.pipe = PEfu; x.latency = 18; x.read(vfS, fsfLane); break;
        case 0x78:
        case 0x7A: x.pipe = PEfu; x.latency = 12; x.read(vfS, fsfLane); break;
        case 0x79: x.pipe = PEfu; x.latency = 29; x.read(vfS, fsfLane); break;
        case 0x7B: x.pipe = PEfu; x.waitP = true; break;
        case 0x7C: x.pipe = PEfu; x.latency = 54; x.read(vfS, fsfLane); break;
        case 0x7D: x.pipe = PEfu; x.latency = 44; x.read(vfS, fsfLane); break;
        default: x.reserved = true; break;
        }
        return x;
    }

    // ---- arithmetic helpers (same rules as VU1Interpreter) ----
    VU1C_INLINE uint32_t bitsOf(float f)
    {
        uint32_t b;
        std::memcpy(&b, &f, 4);
        return b;
    }
    VU1C_INLINE float floatOf(uint32_t b)
    {
        float f;
        std::memcpy(&f, &b, 4);
        return f;
    }
    VU1C_INLINE float norm(float v) // normalizeOperand
    {
        uint32_t b = bitsOf(v);
        const uint32_t e = (b >> 23) & 0xFF;
        if (e == 0)
            b &= 0x80000000u;
        else if (e == 0xFF)
            b = (b & 0x80000000u) | 0x7F7FFFFFu;
        return floatOf(b);
    }
    VU1C_INLINE float normResult(float v, uint32_t &laneFlags) // normalizeResult
    {
        uint32_t b = bitsOf(v);
        const uint32_t sign = b & 0x80000000u, mag = b & 0x7FFFFFFFu, e = (b >> 23) & 0xFF;
        laneFlags = sign ? 2u : 0u;
        if (mag == 0)
            laneFlags |= 1u;
        else if (e == 0)
        {
            laneFlags |= 5u;
            b = sign;
        }
        else if (e == 0xFF)
        {
            laneFlags |= 8u;
            b = sign | 0x7F7FFFFFu;
        }
        return floatOf(b);
    }
    VU1C_INLINE float normResult(float v)
    {
        uint32_t ignored;
        return normResult(v, ignored);
    }
    // normalizeFmacExactResult: fix the value and return Z/S/U/O lane flags from the exact result.
    VU1C_INLINE uint8_t exactFix(float &value, double exact)
    {
        const bool neg = std::signbit(exact);
        const double mag = std::fabs(exact);
        uint8_t f = neg ? 2u : 0u;
        const uint32_t sign = neg ? 0x80000000u : 0u;
        if (mag == 0.0)
        {
            f |= 1u;
            value = floatOf(sign);
        }
        else if (mag > static_cast<double>(std::numeric_limits<float>::max()))
        {
            f |= 8u;
            value = floatOf(sign | 0x7F7FFFFFu);
        }
        else if (mag < static_cast<double>(std::numeric_limits<float>::min()))
        {
            f |= 5u;
            value = floatOf(sign);
        }
        return f;
    }
    VU1C_INLINE int32_t floatToInt(float v, float scale)
    {
        const double s = static_cast<double>(v) * static_cast<double>(scale);
        if (s >= static_cast<double>(std::numeric_limits<int32_t>::max()))
            return std::numeric_limits<int32_t>::max();
        if (s <= static_cast<double>(std::numeric_limits<int32_t>::min()))
            return std::numeric_limits<int32_t>::min();
        return static_cast<int32_t>(s);
    }
    float eatan(float v);
    float esin(float v);
    float eexp(float v);

    // ---- pipeline commits (lazy: done right before something reads the committed value) ----
    VU1C_INLINE void commitFlags(Ctx &c)
    {
        VU1State &s = *c.s;
        while (c.flagHead != c.flagTail && c.flags[c.flagHead & 15].ready <= c.cyc)
        {
            const Ctx::Flag &e = c.flags[c.flagHead & 15];
            if (e.writesMac)
                s.mac = e.mac;
            if (e.writesStatus)
            {
                const uint32_t cur = e.status & 0xF;
                s.status = (s.status & 0xFF0u) | cur | ((cur | e.extraSticky) << 6);
            }
            if (e.writesSticky)
                s.status = (s.status & 0x03Fu) | (e.status & 0xFC0u);
            if (e.writesClip)
                s.clip = e.clip;
            ++c.flagHead;
        }
    }
    VU1C_INLINE void commitQ(Ctx &c)
    {
        if (c.fdivValid && c.fdivReady <= c.cyc)
        {
            c.s->q = c.fdivValue;
            c.s->status = (c.s->status & 0xFCFu) | c.fdivDi | (c.fdivDi << 6);
            c.fdivValid = false;
        }
    }
    VU1C_INLINE void commitP(Ctx &c)
    {
        // Oldest first: both slots can be due when commits are lazy.
        Ctx::Efu *a = &c.efu[0], *b = &c.efu[1];
        if (a->valid && b->valid && b->ready < a->ready)
            std::swap(a, b);
        for (Ctx::Efu *e : {a, b})
            if (e->valid && e->ready <= c.cyc)
            {
                c.s->p = e->value;
                e->valid = false;
            }
    }
    VU1C_INLINE Ctx::Flag &pushFlag(Ctx &c)
    {
        Ctx::Flag &e = c.flags[c.flagTail++ & 15];
        e = {};
        e.issue = c.cyc;
        e.ready = c.cyc + 4;
        return e;
    }
    VU1C_INLINE void queueP(Ctx &c, float value, uint32_t latency)
    {
        value = normResult(value);
        Ctx::Efu &e = !c.efu[0].valid ? c.efu[0] : c.efu[1];
        e = {true, c.cyc + latency, value};
        c.efuResourceReady = c.cyc + (latency ? latency - 1 : 0);
    }
    VU1C_INLINE void queueQ(Ctx &c, float value, uint32_t latency, uint32_t di)
    {
        c.fdivValid = true;
        c.fdivReady = c.cyc + latency;
        c.fdivValue = normResult(value);
        c.fdivDi = di & 0x30u;
    }
    VU1C_INLINE int32_t branchVi(const Ctx &c, uint8_t reg)
    {
        if (reg == 0)
            return 0;
        return c.bkReg == reg ? c.bkVal : c.s->vi[reg];
    }
    template <uint8_t D>
    VU1C_INLINE void applyDest(float *dst, const float *r)
    {
        if constexpr ((D & 8) != 0) dst[0] = r[0];
        if constexpr ((D & 4) != 0) dst[1] = r[1];
        if constexpr ((D & 2) != 0) dst[2] = r[2];
        if constexpr ((D & 1) != 0) dst[3] = r[3];
    }
    VU1C_INLINE void storeQ(Ctx &c, uint32_t addr, const uint32_t w[4], uint8_t mask)
    {
        uint8_t *p = c.data + addr;
        for (uint32_t k = 0; k < 4; ++k)
            if (mask & lane(k))
                std::memcpy(p + k * 4, &w[k], 4);
    }

    // ---- upper (FMAC) ----
    enum UKind : uint8_t
    {
        UAdd, USub, UMadd, UMsub, UMax, UMini, UMul, UOpmsub, UOpmula, UItof, UFtoi, UAbs, UClip, UNop
    };
    enum USrc : uint8_t
    {
        SBc, SQ, SI, SVt
    };
    struct UDec
    {
        UKind kind = UNop;
        USrc src = SVt;
        uint8_t bc = 0;
        bool toAcc = false;
        uint8_t dst = 0; // VF register for non-ACC results
        uint8_t scaleIdx = 0;
    };
    constexpr UDec decodeUpper(uint32_t u)
    {
        UDec x;
        const uint8_t op = u & 0x3F;
        x.dst = fd(u);
        if (op < 0x3C)
        {
            const UKind byGroup[7] = {UAdd, USub, UMadd, UMsub, UMax, UMini, UMul};
            if (op <= 0x1B)
            {
                x.kind = byGroup[op >> 2];
                x.src = SBc;
                x.bc = op & 3;
                return x;
            }
            switch (op)
            {
            case 0x1C: x.kind = UMul; x.src = SQ; break;
            case 0x1D: x.kind = UMax; x.src = SI; break;
            case 0x1E: x.kind = UMul; x.src = SI; break;
            case 0x1F: x.kind = UMini; x.src = SI; break;
            case 0x20: x.kind = UAdd; x.src = SQ; break;
            case 0x21: x.kind = UMadd; x.src = SQ; break;
            case 0x22: x.kind = UAdd; x.src = SI; break;
            case 0x23: x.kind = UMadd; x.src = SI; break;
            case 0x24: x.kind = USub; x.src = SQ; break;
            case 0x25: x.kind = UMsub; x.src = SQ; break;
            case 0x26: x.kind = USub; x.src = SI; break;
            case 0x27: x.kind = UMsub; x.src = SI; break;
            case 0x28: x.kind = UAdd; break;
            case 0x29: x.kind = UMadd; break;
            case 0x2A: x.kind = UMul; break;
            case 0x2B: x.kind = UMax; break;
            case 0x2C: x.kind = USub; break;
            case 0x2D: x.kind = UMsub; break;
            case 0x2E: x.kind = UOpmsub; break;
            case 0x2F: x.kind = UMini; break;
            default: break;
            }
            return x;
        }
        const uint8_t sp = upperSpecial(u);
        x.dst = ft(u);
        x.toAcc = true;
        if (sp <= 0x0F || (sp >= 0x18 && sp <= 0x1B))
        {
            const UKind byGroup[7] = {UAdd, USub, UMadd, UMsub, UNop, UNop, UMul};
            x.kind = byGroup[sp >> 2];
            x.src = SBc;
            x.bc = sp & 3;
            return x;
        }
        x.toAcc = false;
        if (sp >= 0x10 && sp <= 0x13) { x.kind = UItof; x.scaleIdx = sp & 3; return x; }
        if (sp >= 0x14 && sp <= 0x17) { x.kind = UFtoi; x.scaleIdx = sp & 3; return x; }
        x.toAcc = true;
        switch (sp)
        {
        case 0x1C: x.kind = UMul; x.src = SQ; break;
        case 0x1D: x.kind = UAbs; x.toAcc = false; break;
        case 0x1E: x.kind = UMul; x.src = SI; break;
        case 0x1F: x.kind = UClip; x.toAcc = false; break;
        case 0x20: x.kind = UAdd; x.src = SQ; break;
        case 0x21: x.kind = UMadd; x.src = SQ; break;
        case 0x22: x.kind = UAdd; x.src = SI; break;
        case 0x23: x.kind = UMadd; x.src = SI; break;
        case 0x24: x.kind = USub; x.src = SQ; break;
        case 0x25: x.kind = UMsub; x.src = SQ; break;
        case 0x26: x.kind = USub; x.src = SI; break;
        case 0x27: x.kind = UMsub; x.src = SI; break;
        case 0x28: x.kind = UAdd; break;
        case 0x29: x.kind = UMadd; break;
        case 0x2A: x.kind = UMul; break;
        case 0x2C: x.kind = USub; break;
        case 0x2D: x.kind = UMsub; break;
        case 0x2E: x.kind = UOpmula; break;
        default: x.kind = UNop; x.toAcc = false; break;
        }
        return x;
    }

    // Computes the upper result into out[] (applied to the register file after the lower op).
    // Returns true when the result goes through the FMAC flag path.
    template <uint32_t U, bool F>
    VU1C_INLINE void upper(Ctx &c, float out[4])
    {
        constexpr UDec x = decodeUpper(U);
        constexpr uint8_t D = dest(U);
        constexpr uint8_t S = fs(U), T = ft(U);
        VU1State &s = *c.s;
        if constexpr (x.kind == UNop)
            return;
        else if constexpr (x.kind == UClip)
        {
            const uint32_t w = bitsOf(s.vf[T][3]);
            const int32_t limit = (w & 0x7F800000u) ? static_cast<int32_t>(w & 0x7FFFFFFFu) : 0x007FFFFF;
            uint32_t flags = 0;
            for (uint32_t k = 0; k < 3; ++k)
            {
                const uint32_t b = bitsOf(s.vf[S][k]);
                if (static_cast<int32_t>(b) > limit) flags |= 1u << (k * 2);
                if (static_cast<int32_t>(b ^ 0x80000000u) > limit) flags |= 2u << (k * 2);
            }
            c.workingClip = ((c.workingClip << 6) | flags) & 0xFFFFFFu;
            Ctx::Flag &e = pushFlag(c);
            e.clip = c.workingClip;
            e.writesClip = true;
            return;
        }
        else if constexpr (x.kind == UItof)
        {
            constexpr float div[4] = {1.0f, 16.0f, 4096.0f, 32768.0f};
            for (uint32_t k = 0; k < 4; ++k)
                if (D & lane(k))
                {
                    int32_t iv;
                    std::memcpy(&iv, &s.vf[S][k], 4);
                    out[k] = x.scaleIdx == 0 ? static_cast<float>(iv) : static_cast<float>(iv) / div[x.scaleIdx];
                }
            return;
        }
        else if constexpr (x.kind == UFtoi)
        {
            constexpr float mul[4] = {1.0f, 16.0f, 4096.0f, 32768.0f};
            for (uint32_t k = 0; k < 4; ++k)
                if (D & lane(k))
                {
                    const int32_t iv = floatToInt(norm(s.vf[S][k]), mul[x.scaleIdx]);
                    std::memcpy(&out[k], &iv, 4);
                }
            return;
        }
        else if constexpr (x.kind == UAbs)
        {
            for (uint32_t k = 0; k < 4; ++k)
                if (D & lane(k))
                    out[k] = std::fabs(norm(s.vf[S][k]));
            return;
        }
        else
        {
            if constexpr (x.src == SQ)
                commitQ(c);
            float vs[4], vt[4], acc[4];
            for (uint32_t k = 0; k < 4; ++k)
            {
                vs[k] = norm(s.vf[S][k]);
                if constexpr (x.src == SBc)
                    vt[k] = norm(s.vf[T][x.bc]);
                else if constexpr (x.src == SQ)
                    vt[k] = norm(s.q);
                else if constexpr (x.src == SI)
                    vt[k] = norm(s.i);
                else
                    vt[k] = norm(s.vf[T][k]);
                acc[k] = norm(s.acc[k]);
            }
            if constexpr (x.kind == UOpmsub || x.kind == UOpmula)
            {
                // OPMSUB: acc - cross; OPMULA: cross. Operands are permuted (y,z,x).
                float a[4] = {vs[1], vs[2], vs[0], 0}, b[4] = {norm(s.vf[T][2]), norm(s.vf[T][0]), norm(s.vf[T][1]), 0};
                for (uint32_t k = 0; k < 4; ++k)
                {
                    vs[k] = a[k];
                    vt[k] = b[k];
                }
            }
            if constexpr (x.kind == UMax || x.kind == UMini)
            {
                for (uint32_t k = 0; k < 4; ++k)
                    if (D & lane(k))
                        out[k] = x.kind == UMax ? (vs[k] > vt[k] ? vs[k] : vt[k]) : (vs[k] < vt[k] ? vs[k] : vt[k]);
                return;
            }
            uint8_t laneFlags[4]{};
            uint32_t extraSticky = 0;
            for (uint32_t k = 0; k < 4; ++k)
            {
                if (!(D & lane(k)))
                    continue;
                float r;
                if constexpr (x.kind == UAdd) r = vs[k] + vt[k];
                else if constexpr (x.kind == USub) r = vs[k] - vt[k];
                else if constexpr (x.kind == UMul || x.kind == UOpmula) r = vs[k] * vt[k];
                else if constexpr (x.kind == UMadd) r = acc[k] + vs[k] * vt[k];
                else r = acc[k] - vs[k] * vt[k];
                if ((x.kind == UOpmsub || x.kind == UOpmula) && k == 3)
                    r = 0.0f;
                if constexpr (F)
                {
                    // Flags come from the exact result (VU1Interpreter::calculateFmacExactResult).
                    double e;
                    if constexpr (x.kind == UAdd) e = double(vs[k]) + double(vt[k]);
                    else if constexpr (x.kind == USub) e = double(vs[k]) - double(vt[k]);
                    else if constexpr (x.kind == UMul || x.kind == UOpmula) e = double(vs[k]) * double(vt[k]);
                    else if constexpr (x.kind == UMadd) e = double(acc[k]) + double(vs[k]) * double(vt[k]);
                    else e = double(acc[k]) - double(vs[k]) * double(vt[k]);
                    if ((x.kind == UOpmsub || x.kind == UOpmula) && k == 3)
                        e = 0.0;
                    laneFlags[k] = exactFix(r, e);
                    if constexpr (x.kind == UMadd || x.kind == UMsub || x.kind == UOpmsub)
                        if (!(x.kind == UOpmsub && k == 3))
                        {
                            float p = vs[k] * vt[k];
                            extraSticky |= exactFix(p, double(vs[k]) * double(vt[k])) & 0xFu;
                        }
                }
                else
                {
                    // Without flags, the exact path only ever zeroes underflow and clamps overflow;
                    // round-toward-zero never produces infinity, so this is the same value.
                    const uint32_t b = bitsOf(r);
                    if (((b >> 23) & 0xFF) == 0)
                        r = floatOf(b & 0x80000000u);
                }
                out[k] = r;
            }
            if constexpr (F)
            {
                uint32_t mac = 0, status = 0;
                for (uint32_t k = 0; k < 4; ++k)
                {
                    if (!(D & lane(k)))
                        continue;
                    const uint32_t lf = laneFlags[k], ln = lane(k);
                    if (lf & 1) mac |= ln;
                    if (lf & 2) mac |= ln << 4;
                    if (lf & 4) mac |= ln << 8;
                    if (lf & 8) mac |= ln << 12;
                    status |= lf;
                }
                Ctx::Flag &fe = pushFlag(c);
                fe.mac = mac;
                fe.status = status;
                fe.extraSticky = extraSticky;
                fe.writesMac = fe.writesStatus = true;
            }
        }
    }

    // ---- lower ----
    template <uint32_t L, uint32_t PC>
    VU1C_INLINE void lower(Ctx &c)
    {
        VU1State &s = *c.s;
        constexpr uint8_t hi = (L >> 25) & 0x7F, D = dest(L), vfT = ft(L), vfS = fs(L), viT = vit(L), viS = vis(L),
                          viD = vid(L);
        constexpr uint32_t pcNext = PC + 8;
        constexpr uint32_t brTarget = (PC + 8 + imm11(L) * 8) & 0x3FFF;
        constexpr uint8_t sp = upperSpecial(L);
        constexpr uint32_t comp = (L >> 21) & 3;
        auto addr = [](int32_t qw) { return (static_cast<uint32_t>(qw) * 16u) & 0x3FFFu; };
        auto firstLane = []() constexpr { return (D & 8) ? 0 : (D & 4) ? 1 : (D & 2) ? 2 : 3; };
        if constexpr (L == 0 || L == 0x8000033Cu)
            return;
        else if constexpr (hi == 0x00) // LQ
        {
            float tmp[4];
            std::memcpy(tmp, c.data + addr(s.vi[viS] + imm11(L)), 16);
            applyDest<D>(s.vf[vfT], tmp);
        }
        else if constexpr (hi == 0x01) // SQ
        {
            uint32_t w[4];
            std::memcpy(w, s.vf[vfS], 16);
            storeQ(c, addr(s.vi[viT] + imm11(L)), w, D);
        }
        else if constexpr (hi == 0x04) // ILW
        {
            uint32_t v;
            std::memcpy(&v, c.data + addr(s.vi[viS] + imm11(L)) + firstLane() * 4, 4);
            if constexpr (viT != 0) s.vi[viT] = static_cast<int16_t>(v & 0xFFFF);
        }
        else if constexpr (hi == 0x05) // ISW
        {
            const uint32_t v = static_cast<uint16_t>(s.vi[viT] & 0xFFFF);
            const uint32_t w[4] = {v, v, v, v};
            storeQ(c, addr(s.vi[viS] + imm11(L)), w, D);
        }
        else if constexpr (hi == 0x08) { if constexpr (viT != 0) s.vi[viT] = static_cast<int16_t>(s.vi[viS] + imm15(L)); }
        else if constexpr (hi == 0x09) { if constexpr (viT != 0) s.vi[viT] = static_cast<int16_t>(s.vi[viS] - imm15(L)); }
        else if constexpr (hi == 0x10) { commitFlags(c); s.vi[1] = (s.clip & 0xFFFFFF) == (L & 0xFFFFFF) ? 1 : 0; }
        else if constexpr (hi == 0x11) // FCSET
        {
            c.workingClip = L & 0xFFFFFF;
            for (uint32_t k = c.flagHead; k != c.flagTail; ++k)
                if (c.flags[k & 15].issue == c.cyc)
                    c.flags[k & 15].writesClip = false;
            Ctx::Flag &e = pushFlag(c);
            e.clip = c.workingClip;
            e.writesClip = true;
        }
        else if constexpr (hi == 0x12) { commitFlags(c); s.vi[1] = (s.clip & L & 0xFFFFFF) ? 1 : 0; }
        else if constexpr (hi == 0x13) { commitFlags(c); s.vi[1] = ((s.clip | (L & 0xFFFFFF)) & 0xFFFFFF) == 0xFFFFFF ? 1 : 0; }
        else if constexpr (hi == 0x14 || hi == 0x15 || hi == 0x16 || hi == 0x17)
        {
            constexpr uint16_t imm12 = static_cast<uint16_t>((((L >> 21) & 1) << 11) | (L & 0x7FF));
            if constexpr (hi == 0x15) // FSSET
            {
                for (uint32_t k = c.flagHead; k != c.flagTail; ++k)
                    if (c.flags[k & 15].issue == c.cyc)
                        c.flags[k & 15].writesStatus = false;
                Ctx::Flag &e = pushFlag(c);
                e.status = imm12 & 0xFC0u;
                e.writesSticky = true;
            }
            else
            {
                commitFlags(c);
                commitQ(c);
                const uint32_t st = s.status & 0xFFF;
                if constexpr (viT != 0)
                    s.vi[viT] = hi == 0x14 ? (st == imm12 ? 1 : 0) : hi == 0x16 ? static_cast<int32_t>(st & imm12)
                                                                                 : static_cast<int32_t>(st | imm12);
            }
        }
        else if constexpr (hi == 0x18 || hi == 0x1A || hi == 0x1B)
        {
            commitFlags(c);
            const uint32_t m = static_cast<uint16_t>(s.vi[viS]);
            if constexpr (viT != 0)
                s.vi[viT] = hi == 0x18 ? ((s.mac & 0xFFFF) == m ? 1 : 0) : hi == 0x1A ? static_cast<int32_t>(s.mac & m)
                                                                                      : static_cast<int32_t>(s.mac | m);
        }
        else if constexpr (hi == 0x1C) { commitFlags(c); if constexpr (viT != 0) s.vi[viT] = static_cast<int32_t>(s.clip & 0xFFF); }
        else if constexpr (hi == 0x20) { c.br = true; c.target = brTarget; }
        else if constexpr (hi == 0x21) { if constexpr (viT != 0) s.vi[viT] = static_cast<int32_t>((PC + 16) / 8); c.br = true; c.target = brTarget; }
        else if constexpr (hi == 0x24 || hi == 0x25)
        {
            const uint32_t t = (static_cast<uint32_t>(static_cast<uint16_t>(branchVi(c, viS))) * 8u) & 0x3FFF;
            if constexpr (hi == 0x25 && viT != 0) s.vi[viT] = static_cast<int32_t>((PC + 16) / 8);
            c.br = true;
            c.target = t;
        }
        else if constexpr (hi >= 0x28 && hi <= 0x2F)
        {
            const int16_t a = static_cast<int16_t>(branchVi(c, viS));
            bool take;
            if constexpr (hi == 0x28) take = a == static_cast<int16_t>(branchVi(c, viT));
            else if constexpr (hi == 0x29) take = a != static_cast<int16_t>(branchVi(c, viT));
            else if constexpr (hi == 0x2C) take = a < 0;
            else if constexpr (hi == 0x2D) take = a > 0;
            else if constexpr (hi == 0x2E) take = a <= 0;
            else take = a >= 0;
            c.br = take;
            c.target = brTarget;
        }
        else if constexpr (hi == 0x40 && (L & 0x3F) < 0x3C)
        {
            constexpr uint8_t fn = L & 0x3F;
            if constexpr (fn == 0x30 && viD != 0) s.vi[viD] = static_cast<int16_t>(s.vi[viS] + s.vi[viT]);
            else if constexpr (fn == 0x31 && viD != 0) s.vi[viD] = static_cast<int16_t>(s.vi[viS] - s.vi[viT]);
            else if constexpr (fn == 0x32 && viT != 0) s.vi[viT] = static_cast<int16_t>(s.vi[viS] + (static_cast<int32_t>(((L >> 6) & 0x1F) << 27) >> 27));
            else if constexpr (fn == 0x34 && viD != 0) s.vi[viD] = s.vi[viS] & s.vi[viT];
            else if constexpr (fn == 0x35 && viD != 0) s.vi[viD] = s.vi[viS] | s.vi[viT];
        }
        else if constexpr (sp == 0x30) { float t[4]; std::memcpy(t, s.vf[vfS], 16); applyDest<D>(s.vf[vfT], t); }
        else if constexpr (sp == 0x31) { float t[4] = {s.vf[vfS][1], s.vf[vfS][2], s.vf[vfS][3], s.vf[vfS][0]}; applyDest<D>(s.vf[vfT], t); }
        else if constexpr (sp == 0x34) // LQI
        {
            float t[4];
            std::memcpy(t, c.data + addr(static_cast<uint16_t>(s.vi[viS])), 16);
            applyDest<D>(s.vf[vfT], t);
            if constexpr (viS != 0) s.vi[viS] = static_cast<int16_t>(s.vi[viS] + 1);
        }
        else if constexpr (sp == 0x35) // SQI
        {
            uint32_t w[4];
            std::memcpy(w, s.vf[vfS], 16);
            storeQ(c, addr(static_cast<uint16_t>(s.vi[viT])), w, D);
            if constexpr (viT != 0) s.vi[viT] = static_cast<int16_t>(s.vi[viT] + 1);
        }
        else if constexpr (sp == 0x36) // LQD
        {
            if constexpr (viS != 0) s.vi[viS] = static_cast<int16_t>(s.vi[viS] - 1);
            float t[4];
            std::memcpy(t, c.data + addr(static_cast<uint16_t>(s.vi[viS])), 16);
            applyDest<D>(s.vf[vfT], t);
        }
        else if constexpr (sp == 0x37) // SQD
        {
            if constexpr (viT != 0) s.vi[viT] = static_cast<int16_t>(s.vi[viT] - 1);
            uint32_t w[4];
            std::memcpy(w, s.vf[vfS], 16);
            storeQ(c, addr(static_cast<uint16_t>(s.vi[viT])), w, D);
        }
        else if constexpr (sp == 0x38) // DIV
        {
            const float num = norm(s.vf[vfS][(L >> 21) & 3]), den = norm(s.vf[vfT][(L >> 23) & 3]);
            uint32_t di = 0;
            float r;
            if (den == 0.0f)
            {
                di = num == 0.0f ? 0x10u : 0x20u;
                r = std::signbit(num) != std::signbit(den) ? -std::numeric_limits<float>::max() : std::numeric_limits<float>::max();
            }
            else
                r = num / den;
            queueQ(c, r, 7, di);
        }
        else if constexpr (sp == 0x39) // SQRT
        {
            const float v = norm(s.vf[vfT][(L >> 23) & 3]);
            queueQ(c, std::sqrt(std::fabs(v)), 7, v < 0.0f ? 0x10u : 0u);
        }
        else if constexpr (sp == 0x3A) // RSQRT
        {
            const float num = norm(s.vf[vfS][(L >> 21) & 3]), rad = norm(s.vf[vfT][(L >> 23) & 3]);
            const float den = std::sqrt(std::fabs(rad));
            uint32_t di = rad < 0.0f ? 0x10u : 0u;
            float r;
            if (den != 0.0f)
                r = num / den;
            else
            {
                di = num == 0.0f ? 0x10u : 0x20u;
                r = std::signbit(num) ? -std::numeric_limits<float>::max() : std::numeric_limits<float>::max();
            }
            queueQ(c, r, 13, di);
        }
        else if constexpr (sp == 0x3B) { commitQ(c); } // WAITQ (the stall already happened)
        else if constexpr (sp == 0x3C) { uint32_t v; std::memcpy(&v, &s.vf[vfS][comp], 4); if constexpr (viT != 0) s.vi[viT] = static_cast<int16_t>(v & 0xFFFF); }
        else if constexpr (sp == 0x3D)
        {
            const int32_t v = static_cast<int16_t>(s.vi[viS] & 0xFFFF);
            float t[4];
            for (auto &x : t) std::memcpy(&x, &v, 4);
            applyDest<D>(s.vf[vfT], t);
        }
        else if constexpr (sp == 0x3E) // ILWR
        {
            uint32_t v;
            std::memcpy(&v, c.data + addr(static_cast<uint16_t>(s.vi[viS])) + firstLane() * 4, 4);
            if constexpr (viT != 0) s.vi[viT] = static_cast<int16_t>(v & 0xFFFF);
        }
        else if constexpr (sp == 0x3F) // ISWR
        {
            const uint32_t v = static_cast<uint16_t>(s.vi[viT] & 0xFFFF);
            const uint32_t w[4] = {v, v, v, v};
            storeQ(c, addr(static_cast<uint16_t>(s.vi[viS])), w, D);
        }
        else if constexpr (sp == 0x40 || sp == 0x41)
        {
            if constexpr (sp == 0x40)
            {
                const uint32_t xb = (s.r >> 4) & 1, yb = (s.r >> 22) & 1;
                s.r = (((s.r << 1) ^ xb ^ yb) & 0x007FFFFFu) | 0x3F800000u;
            }
            const float v = floatOf(s.r);
            const float t[4] = {v, v, v, v};
            applyDest<D>(s.vf[vfT], t);
        }
        else if constexpr (sp == 0x42) { s.r = 0x3F800000u | (bitsOf(s.vf[vfS][comp]) & 0x007FFFFFu); }
        else if constexpr (sp == 0x43) { s.r = 0x3F800000u | ((s.r ^ bitsOf(s.vf[vfS][comp])) & 0x007FFFFFu); }
        else if constexpr (sp == 0x64) { commitP(c); const float t[4] = {s.p, s.p, s.p, s.p}; applyDest<D>(s.vf[vfT], t); }
        else if constexpr (sp == 0x68) { if constexpr (viT != 0) s.vi[viT] = static_cast<int32_t>(s.top & 0x3FF); }
        else if constexpr (sp == 0x69) { if constexpr (viT != 0) s.vi[viT] = static_cast<int32_t>(s.itop & 0x3FF); }
        else if constexpr (sp == 0x6C) // XGKICK: the next one waits for this transfer (one qword per two cycles)
        {
            const uint32_t qw = xgkick(c, static_cast<uint16_t>(s.vi[viS]));
            c.xgkickBusyUntil = c.cyc + (qw ? 2 * qw - 1 : 0);
        }
        else if constexpr (sp >= 0x70 && sp <= 0x73)
        {
            const float x = norm(s.vf[vfS][0]), y = norm(s.vf[vfS][1]), z = norm(s.vf[vfS][2]);
            const float sum = x * x + y * y + z * z;
            if constexpr (sp == 0x70) queueP(c, sum, 11);
            else if constexpr (sp == 0x71) queueP(c, sum != 0.0f ? 1.0f / sum : sum, 18);
            else if constexpr (sp == 0x72) queueP(c, std::sqrt(sum), 18);
            else { const float len = std::sqrt(sum); queueP(c, len != 0.0f ? 1.0f / len : len, 24); }
        }
        else if constexpr (sp == 0x74) { const float x = norm(s.vf[vfS][0]), y = norm(s.vf[vfS][1]); queueP(c, x != 0.0f ? eatan(y / x) : 0.0f, 54); }
        else if constexpr (sp == 0x75) { const float x = norm(s.vf[vfS][0]), z = norm(s.vf[vfS][2]); queueP(c, x != 0.0f ? eatan(z / x) : 0.0f, 54); }
        else if constexpr (sp == 0x76)
        {
            float sum = 0.0f;
            for (uint32_t k = 0; k < 4; ++k) sum += norm(s.vf[vfS][k]);
            queueP(c, sum, 12);
        }
        else if constexpr (sp == 0x77)
        {
            float r = norm(s.vf[vfS][comp]);
            if (r >= 0.0f) { r = std::sqrt(r); if (r != 0.0f) r = 1.0f / r; }
            queueP(c, r, 18);
        }
        else if constexpr (sp == 0x78) { const float v = norm(s.vf[vfS][comp]); queueP(c, v >= 0.0f ? std::sqrt(v) : v, 12); }
        else if constexpr (sp == 0x79) { queueP(c, esin(norm(s.vf[vfS][comp])), 29); }
        else if constexpr (sp == 0x7A) { const float v = norm(s.vf[vfS][comp]); queueP(c, v != 0.0f ? 1.0f / v : v, 12); }
        else if constexpr (sp == 0x7B) { commitP(c); } // WAITP
        else if constexpr (sp == 0x7C) { queueP(c, eatan(norm(s.vf[vfS][comp])), 54); }
        else if constexpr (sp == 0x7D) { queueP(c, eexp(norm(s.vf[vfS][comp])), 44); }
        (void)pcNext;
    }

    // r = max(r, a[n]) / a[n] = v for each set bit n of M. M is a constant, so this folds to
    // straight-line code (lambdas here were not inlined by MSVC).
    template <uint32_t M, size_t... I>
    VU1C_INLINE void maxOverImpl(const uint32_t *a, uint32_t &r, std::index_sequence<I...>)
    {
        ((((M >> I) & 1u) ? void(r = std::max(r, a[I])) : void()), ...);
    }
    template <uint32_t M, size_t N>
    VU1C_INLINE void maxOver(const uint32_t *a, uint32_t &r)
    {
        maxOverImpl<M>(a, r, std::make_index_sequence<N>{});
    }
    template <uint32_t M, size_t... I>
    VU1C_INLINE void setOverImpl(uint32_t *a, uint32_t v, std::index_sequence<I...>)
    {
        ((((M >> I) & 1u) ? void(a[I] = v) : void()), ...);
    }
    template <uint32_t M, size_t N>
    VU1C_INLINE void setOver(uint32_t *a, uint32_t v)
    {
        setOverImpl<M>(a, v, std::make_index_sequence<N>{});
    }
    // Lane mask (x = 8 ... w = 1) to component bits (x = bit 0 ... w = bit 3).
    constexpr uint32_t components(uint8_t lanes)
    {
        return ((lanes >> 3) & 1) | ((lanes >> 1) & 2) | ((lanes << 1) & 4) | ((lanes << 3) & 8);
    }

    template <Usage X>
    VU1C_INLINE void stall(const Ctx &c, uint32_t &r)
    {
        if constexpr (X.vfReadCount > 0)
            maxOver<components(X.vfRead[0].lanes), 4>(c.vfReady[X.vfRead[0].reg], r);
        if constexpr (X.vfReadCount > 1)
            maxOver<components(X.vfRead[1].lanes), 4>(c.vfReady[X.vfRead[1].reg], r);
        maxOver<X.viRead & 0xFFFEu, 16>(c.viReady, r);
        maxOver<components(X.accRead), 4>(c.accReady, r);
    }

    // One instruction pair at PC. Branch decisions are left in c.br / c.target for the
    // generated code, which acts on them after the delay slot.
    template <uint32_t U, uint32_t L, uint32_t PC, bool F>
    VU1C_INLINE void P(Ctx &c)
    {
        constexpr bool iBit = (U & 0x80000000u) != 0;
        constexpr Usage uu = upperUsage(U);
        constexpr Usage lu = iBit ? Usage{} : lowerUsage(L);
        static_assert(!uu.reserved && !lu.reserved, "vu1_recomp.py must not emit reserved instructions");

        uint32_t r = c.cyc;
        stall<uu>(c, r);
        stall<lu>(c, r);
        if constexpr (lu.pipe == PFdiv)
            if (c.fdivValid)
                r = std::max(r, c.fdivReady);
        if constexpr (lu.pipe == PEfu)
            r = std::max(r, c.efuResourceReady);
        if constexpr (lu.waitP)
            for (const Ctx::Efu &e : c.efu)
                if (e.valid)
                    r = std::max(r, e.ready);
        if constexpr (lu.pipe == PXgkick)
            r = std::max(r, c.xgkickBusyUntil);
        c.cyc = r;
        // Everything with ready <= cyc is now visible; Q must be committed before a new DIV replaces it.
        if constexpr (lu.pipe == PFdiv)
            commitQ(c);
        if constexpr (lu.pipe == PEfu)
            commitP(c);

        VU1State &s = *c.s;
        float upperOut[4];
        upper<U, F>(c, upperOut);

        constexpr int lowerViWrite = [] {
            for (int reg = 1; reg < 16; ++reg)
                if (lu.viWrite & (1u << reg))
                    return reg;
            return 0;
        }();
        int32_t oldVi = 0;
        if constexpr (lowerViWrite != 0 && lu.delaysNextBranchRead)
            oldVi = s.vi[lowerViWrite];

        constexpr uint8_t uReg = uu.vfWrite.reg;
        if constexpr (iBit)
            s.i = norm(floatOf(L));
        else if constexpr (uReg != 0 && lu.vfWrite.reg == uReg)
        {
            // The lower write to the upper's destination is dropped (VU1Interpreter::suppressedLowerVf).
            float keep[4];
            std::memcpy(keep, s.vf[uReg], 16);
            lower<L, PC>(c);
            std::memcpy(s.vf[uReg], keep, 16);
        }
        else
            lower<L, PC>(c);

        // Upper results land after the lower op read the old registers.
        constexpr UDec ux = decodeUpper(U);
        if constexpr (uReg != 0)
            applyDest<uu.vfWrite.lanes>(s.vf[uReg], upperOut);
        if constexpr (uu.accWrite != 0)
            applyDest<uu.accWrite>(s.acc, upperOut);
        (void)ux;

        // Scoreboard: when each written register becomes readable.
        if constexpr (lu.vfWrite.reg != 0 && lu.vfWrite.reg != uReg)
            setOver<components(lu.vfWrite.lanes), 4>(c.vfReady[lu.vfWrite.reg], r + (lu.vfLatency ? lu.vfLatency : lu.latency));
        if constexpr (uReg != 0)
            setOver<components(uu.vfWrite.lanes), 4>(c.vfReady[uReg], r + (uu.vfLatency ? uu.vfLatency : uu.latency));
        setOver<lu.viWrite & 0xFFFEu, 16>(c.viReady, r + (lu.viLatency ? lu.viLatency : lu.latency));
        setOver<components(uu.accWrite), 4>(c.accReady, r + 1);

        if constexpr (lowerViWrite != 0 && lu.delaysNextBranchRead)
        {
            c.bkReg = static_cast<uint8_t>(lowerViWrite);
            c.bkVal = oldVi;
        }
        else
            c.bkReg = 0;

        s.vf[0][0] = s.vf[0][1] = s.vf[0][2] = 0.0f;
        s.vf[0][3] = 1.0f;
        s.vi[0] = 0;
        c.cyc = r + 1;
    }
}
