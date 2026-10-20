// PS2X_PROFILE=<startVsync>:<seconds>[:<file>] samples every host thread about once a
// millisecond, once the guest reaches <startVsync>, and writes exe-relative addresses of the
// sampled stacks to <file> (default ps2x_profile.txt). Symbolize them with the linker map:
//   self <rva> <n>    leaf frame in the exe
//   incl <rva> <n>    anywhere on the stack (once per sample)
//   outside <n>       leaf outside the exe (waits, CRT, drivers)
// No PDB is needed: x64 unwind data walks the stack.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <tlhelp32.h>

#include <algorithm>
#include <chrono>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace
{
    constexpr int kMaxFrames = 48;

    // Runs while the target is suspended: no allocation here, the target may hold the heap lock.
    int walk(CONTEXT ctx, uintptr_t base, uintptr_t end, uintptr_t *out)
    {
        int n = 0;
        for (int depth = 0; depth < kMaxFrames && ctx.Rip; ++depth)
        {
            if (ctx.Rip >= base && ctx.Rip < end)
                out[n++] = ctx.Rip - base;
            else if (depth == 0)
                out[n++] = 0; // leaf outside the exe
            DWORD64 imageBase = 0;
            PRUNTIME_FUNCTION fn = RtlLookupFunctionEntry(ctx.Rip, &imageBase, nullptr);
            if (!fn)
            {
                // leaf function without unwind data: return address is at rsp
                ctx.Rip = *reinterpret_cast<DWORD64 *>(ctx.Rsp);
                ctx.Rsp += 8;
                continue;
            }
            void *handlerData = nullptr;
            DWORD64 establisher = 0;
            RtlVirtualUnwind(UNW_FLAG_NHANDLER, imageBase, ctx.Rip, fn, &ctx, &handlerData, &establisher, nullptr);
        }
        return n;
    }
}

void ps2StartProfiler(std::function<uint64_t()> vsync)
{
    const char *spec = std::getenv("PS2X_PROFILE");
    if (!spec)
        return;
    unsigned long long start = 0, seconds = 30;
    char file[512] = "ps2x_profile.txt";
    std::sscanf(spec, "%llu:%llu:%511s", &start, &seconds, file);
    std::thread([vsync, start, seconds, path = std::string(file)]()
                {
        while (vsync() < start)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const unsigned long long startVsync = vsync();
        std::fprintf(stderr, "[profile] sampling from vsync %llu for %llus\n", startVsync, seconds);

        const auto base = reinterpret_cast<uintptr_t>(GetModuleHandleW(nullptr));
        const auto *dos = reinterpret_cast<const IMAGE_DOS_HEADER *>(base);
        const auto *nt = reinterpret_cast<const IMAGE_NT_HEADERS *>(base + dos->e_lfanew);
        const uintptr_t end = base + nt->OptionalHeader.SizeOfImage;
        const DWORD self = GetCurrentThreadId();

        std::unordered_map<uintptr_t, uint64_t> selfHits, inclHits;
        uint64_t outside = 0, samples = 0;
        std::vector<HANDLE> threads;
        auto refresh = [&]() {
            for (HANDLE h : threads)
                CloseHandle(h);
            threads.clear();
            HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
            THREADENTRY32 te{sizeof(te)};
            for (BOOL ok = Thread32First(snap, &te); ok; ok = Thread32Next(snap, &te))
                if (te.th32OwnerProcessID == GetCurrentProcessId() && te.th32ThreadID != self)
                    if (HANDLE h = OpenThread(THREAD_SUSPEND_RESUME | THREAD_GET_CONTEXT | THREAD_QUERY_INFORMATION, FALSE, te.th32ThreadID))
                        threads.push_back(h);
            CloseHandle(snap);
        };

        const auto stop = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
        uintptr_t frames[kMaxFrames];
        std::unordered_set<uintptr_t> seen;
        for (int tick = 0; std::chrono::steady_clock::now() < stop; ++tick)
        {
            if (tick % 500 == 0)
                refresh();
            for (HANDLE h : threads)
            {
                if (SuspendThread(h) == static_cast<DWORD>(-1))
                    continue;
                CONTEXT ctx{};
                ctx.ContextFlags = CONTEXT_CONTROL | CONTEXT_INTEGER;
                const int n = GetThreadContext(h, &ctx) ? walk(ctx, base, end, frames) : 0;
                ResumeThread(h);
                if (n == 0)
                    continue;
                ++samples;
                if (frames[0] == 0)
                {
                    ++outside;
                    continue; // waiting threads dominate otherwise; only count busy stacks
                }
                ++selfHits[frames[0]];
                seen.clear();
                for (int i = 0; i < n; ++i)
                    if (frames[i] && seen.insert(frames[i]).second)
                        ++inclHits[frames[i]];
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        FILE *f = std::fopen(path.c_str(), "w");
        if (!f)
        {
            std::fprintf(stderr, "[profile] cannot write %s\n", path.c_str());
            return;
        }
        std::fprintf(f, "samples %llu\noutside %llu\n", static_cast<unsigned long long>(samples),
                     static_cast<unsigned long long>(outside));
        for (const auto &[rva, count] : selfHits)
            std::fprintf(f, "self %llx %llu\n", static_cast<unsigned long long>(rva), static_cast<unsigned long long>(count));
        for (const auto &[rva, count] : inclHits)
            std::fprintf(f, "incl %llx %llu\n", static_cast<unsigned long long>(rva), static_cast<unsigned long long>(count));
        std::fclose(f);
        std::fprintf(stderr, "[profile] wrote %s (%llu samples), vsync %llu -> %llu\n", path.c_str(),
                     static_cast<unsigned long long>(samples), startVsync, static_cast<unsigned long long>(vsync())); })
        .detach();
}
#else
void ps2StartProfiler(std::function<uint64_t()>) {}
#endif
