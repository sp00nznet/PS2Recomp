// A host crash inside generated code otherwise ends the process silently (no PDB, and
// WER is often off). This prints the exception and the guest pc/ra the last dispatch ran on.
#include <cstdint>
#include <cstdio>

uint32_t ps2CrashGuestPc();
uint32_t ps2CrashGuestRa();

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

namespace
{
    LONG WINAPI report(EXCEPTION_POINTERS *info)
    {
        const EXCEPTION_RECORD *record = info->ExceptionRecord;
        const auto base = reinterpret_cast<uintptr_t>(GetModuleHandleW(nullptr));
        const auto at = reinterpret_cast<uintptr_t>(record->ExceptionAddress);
        std::fprintf(stderr, "[crash] exception 0x%08lX at exe+0x%llX", record->ExceptionCode,
                     static_cast<unsigned long long>(at - base));
        if (record->ExceptionCode == EXCEPTION_ACCESS_VIOLATION && record->NumberParameters >= 2)
            std::fprintf(stderr, " (%s 0x%llX)", record->ExceptionInformation[0] ? "write" : "read",
                         static_cast<unsigned long long>(record->ExceptionInformation[1]));
        std::fprintf(stderr, " guest pc=0x%08X ra=0x%08X\n", ps2CrashGuestPc(), ps2CrashGuestRa());
        std::fflush(stderr);
        return EXCEPTION_CONTINUE_SEARCH;
    }
}

void ps2InstallCrashReport()
{
    SetUnhandledExceptionFilter(report);
}
#else
void ps2InstallCrashReport() {}
#endif
