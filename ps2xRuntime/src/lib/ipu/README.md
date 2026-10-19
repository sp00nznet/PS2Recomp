# IPU decoder (from Play!)

The MPEG-2 decoder behind the IPU registers. It is Play!'s `CIPU`, by Jean-Philip
Desjardins, under the BSD-2-Clause licence in `LICENSE.Play.txt` (Play- and
Play--Framework carry the same text).

| Files | Upstream | Commit |
|-------|----------|--------|
| `IPU*.cpp/h` | https://github.com/jpd002/Play- `Source/ee/` | d44bfa1814b729d4a3ed3c6c3a945432d5ee5bbe |
| `mpeg2/*`, `idct/*`, `BitStream`, `MemStream`, `Stream`, `PlayTypes.h` (was `Types.h`), `Convertible.h`, `EndianUtils.h`, `Singleton.h`, `PtrMacro.h`, `alloca_def.h`, `maybe_unused.h` | https://github.com/jpd002/Play--Framework `include/`, `src/` | 3368a5a7d9cb18af4591e51f37fe9505496f70c2 |

Changes made here:

- Save-state code removed.
- The INTC and DMAC hooks are gone: the interrupt is a `std::function` callback, and
  DMA channels 3/4 are fed from `../ps2_ipu.cpp`.
- Play's logging is a no-op `CLog` stub.
- `EE_SPR_SIZE` is a local constant.
- `Types.h` is renamed `PlayTypes.h`, because `ps2xRuntime/include/types.h` shadowed it.
- `idct/Interface.h` gains a missing include.

`../ps2_ipu.cpp` is the glue: register and FIFO mapping, toIPU source-chain tags and
fromIPU output. It is new code, not Play's.
