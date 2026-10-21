// What the runtime needs to run a compiled VU1 program (kept apart from the template-heavy
// ps2_vu1_compiled.h so ps2_runtime.cpp does not have to parse it).
#pragma once

#include "runtime/ps2_vu1.h"

#include <cstdint>

class PS2Memory;

namespace vu1c
{
    struct Program;
    struct RunResult
    {
        bool ended;     // false: continue in the interpreter at pc
        uint32_t pc;
        uint32_t cycles;
    };

    // The compiled program matching VU1 code memory, or nullptr. Re-matched only when generation changes.
    const Program *find(const uint8_t *code, uint64_t generation);
    const char *name(const Program *program);
    bool usesFlags(const Program *program); // MAC/status are tracked
    RunResult run(const Program *program, VU1State &state, uint8_t *data, PS2Memory *mem, uint32_t pc);
    // Verification support: while capturing, XGKICK packets are hashed instead of sent.
    void setPacketCapture(bool capture, uint64_t *hashOut);
}
