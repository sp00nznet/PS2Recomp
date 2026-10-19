#pragma once

#include <array>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace ps2x::iop
{
    class IopHost;
}

namespace ps2x::iop::detail
{
    struct IopCpuState;
    class IopMemory;

    // Devices behind SIO2: a DualShock 2 on ports 0/1 and a PS2 memory card on ports 2/3.
    // The game's own sio2man stays loaded; its transfer entry points are answered here, so
    // no SIO2 register, DMA or interrupt emulation is needed.
    class IopSio2
    {
    public:
        IopSio2(IopHost &host, IopMemory &memory) noexcept;

        void reset();
        [[nodiscard]] bool dispatchImport(uint16_t ordinal, IopCpuState &cpu);

        // One SIO2 command addressed to a port. Exposed for tests.
        [[nodiscard]] bool exchange(int port, const uint8_t *in, size_t size, uint8_t *out);

    private:
        struct Pad
        {
            bool config = false;
            bool analog = false;
            bool pressure = false;
        };

        struct Card
        {
            std::fstream file;
            bool present = false;
            uint8_t terminator = 0x66u;
            uint32_t page = 0u;
            uint32_t offset = 0u;
        };

        bool transfer(uint32_t descriptor);
        void exchangePad(int port, const uint8_t *in, size_t size, uint8_t *out);
        void exchangeCard(Card &card, const uint8_t *in, size_t size, uint8_t *out);
        void openCards();

        IopHost &m_host;
        IopMemory &m_memory;
        std::array<Pad, 2> m_pads{};
        std::array<Card, 2> m_cards{};
        bool m_cardsOpened = false;
        uint32_t m_traceBudget = 0u;
    };
}
