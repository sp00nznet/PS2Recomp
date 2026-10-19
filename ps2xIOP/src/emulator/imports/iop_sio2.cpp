#include "iop_sio2.h"

#include "../core/iop_cpu.h"
#include "../core/iop_memory.h"
#include "ps2x/iop/iop_host.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <sstream>

// Device behaviour follows the public DualShock 2 command set and the PS2 memory card
// command layout that ps2sdk's mcman (AFL 2.0) builds and checks: each reply carries 0x2B
// right after the command's input bytes, then any payload, and ends with the terminator.
namespace ps2x::iop::detail
{
    namespace
    {
        // sio2_transfer_data_t (ps2sdk sio2man.h)
        constexpr uint32_t kStat6c = 0x00u;
        constexpr uint32_t kRegData = 0x28u;
        constexpr uint32_t kStat70 = 0x24u;
        constexpr uint32_t kStat74 = 0x68u;
        constexpr uint32_t kInSize = 0x6Cu;
        constexpr uint32_t kOutSize = 0x70u;
        constexpr uint32_t kIn = 0x74u;
        constexpr uint32_t kOut = 0x78u;
        constexpr uint32_t kInDma = 0x7Cu;
        constexpr uint32_t kOutDma = 0x88u;

        constexpr uint8_t kInsertedTerminator = 0x66u;
        constexpr uint32_t kConnected = 0x1100u;
        constexpr uint32_t kNoDevice = 0x1D100u;

        constexpr uint32_t kPageData = 512u;
        constexpr uint32_t kPageRaw = 528u; // data + ECC spare
        constexpr uint32_t kBlockPages = 16u;
        constexpr uint32_t kCardPages = 0x4000u; // 8 MB

        uint8_t xorBytes(const uint8_t *data, size_t size)
        {
            uint8_t value = 0u;
            for (size_t i = 0; i < size; ++i)
                value ^= data[i];
            return value;
        }
    }

    IopSio2::IopSio2(IopHost &host, IopMemory &memory) noexcept
        : m_host(host), m_memory(memory)
    {
    }

    void IopSio2::reset()
    {
        m_pads = {};
        for (Card &card : m_cards)
        {
            // A freshly inserted card answers with 0x66 until a driver sets its own
            // terminator: mcman reads 0x66 as "card changed", and the SDK's mc2 driver
            // only treats a card as newly inserted when it does not already answer 0x55.
            card.terminator = kInsertedTerminator;
            card.page = 0u;
            card.offset = 0u;
        }
        if (const char *trace = std::getenv("PS2X_TRACE_SIO2"))
            m_traceBudget = static_cast<uint32_t>(std::max(1, std::atoi(trace)));
    }

    void IopSio2::openCards()
    {
        if (m_cardsOpened)
            return;
        m_cardsOpened = true;
        // ponytail: raw 8 MB images with ECC (16384 x 528 bytes); no other card sizes.
        const char *paths[2] = {std::getenv("PS2X_MEMCARD1"), std::getenv("PS2X_MEMCARD2")};
        for (size_t i = 0; i < m_cards.size(); ++i)
        {
            if (!paths[i] || !*paths[i])
                continue;
            m_cards[i].file.open(paths[i], std::ios::in | std::ios::out | std::ios::binary);
            m_cards[i].present = m_cards[i].file.is_open();
            m_host.log(m_cards[i].present ? LogLevel::Info : LogLevel::Warning,
                       std::string("[SIO2] memory card ") + std::to_string(i + 1) + ": " + paths[i] +
                           (m_cards[i].present ? "" : " (cannot open)"));
        }
    }

    bool IopSio2::dispatchImport(uint16_t ordinal, IopCpuState &cpu)
    {
        switch (ordinal)
        {
        case 25: // sio2_transfer
        case 51: // sio2_transfer2
            cpu.gpr[2] = transfer(cpu.gpr[4]) ? 1u : 0u;
            return true;
        // Transfer init/reset only serialise access to the hardware; our transfer is atomic.
        case 23: case 24: case 26:
        case 46: case 47: case 48: case 49: case 50: case 52:
        case 61: case 62: case 63:
            cpu.gpr[2] = 0u;
            return true;
        default:
            return false;
        }
    }

    bool IopSio2::transfer(uint32_t td)
    {
        openCards();
        const auto word = [&](uint32_t offset)
        { return m_memory.read32(td + offset); };

        std::vector<uint8_t> inPio(word(kInSize));
        if (!inPio.empty())
            (void)m_memory.readRam(word(kIn), inPio.data(), inPio.size());
        const uint32_t inDma = word(kInDma), inSlice = word(kInDma + 4u) * 4u;
        const uint32_t outDma = word(kOutDma), outSlice = word(kOutDma + 4u) * 4u;
        std::vector<uint8_t> outPio;

        uint32_t stat = kConnected;
        size_t pioCursor = 0;
        for (uint32_t i = 0; i < 16u; ++i)
        {
            const uint32_t reg = word(kRegData + i * 4u);
            if (reg == 0u)
                break;
            const int port = static_cast<int>(reg & 3u);
            const size_t sendSize = (reg >> 8) & 0x1FFu;
            const size_t recvSize = (reg >> 18) & 0x1FFu;
            const size_t size = std::max(sendSize, recvSize);

            std::vector<uint8_t> in(size, 0u), out(size, 0xFFu);
            if (inDma != 0u)
                (void)m_memory.readRam(inDma + i * inSlice, in.data(), sendSize);
            else
            {
                const size_t n = std::min(sendSize, inPio.size() - std::min(pioCursor, inPio.size()));
                std::copy_n(inPio.begin() + pioCursor, n, in.begin());
                pioCursor += sendSize;
            }

            if (!exchange(port, in.data(), size, out.data()))
                stat = kNoDevice;

            // Routine pad polls (0x42) are not traced; everything else is.
            if (m_traceBudget > 0u && !(port < 2 && in[1] == 0x42u))
            {
                --m_traceBudget;
                std::ostringstream line;
                line << "[SIO2] port=" << port << std::hex;
                for (size_t b = 0; b < std::min<size_t>(size, 12u); ++b)
                    line << ' ' << static_cast<int>(in[b]);
                line << " ->";
                for (size_t b = 0; b < std::min<size_t>(size, 12u); ++b)
                    line << ' ' << static_cast<int>(out[b]);
                m_host.log(LogLevel::Info, line.str());
            }

            if (outDma != 0u)
                (void)m_memory.writeRam(outDma + i * outSlice, out.data(), recvSize);
            else
                outPio.insert(outPio.end(), out.begin(), out.begin() + recvSize);
        }

        const uint32_t outSize = word(kOutSize);
        if (outSize != 0u && word(kOut) != 0u)
        {
            outPio.resize(outSize, 0xFFu);
            (void)m_memory.writeRam(word(kOut), outPio.data(), outSize);
        }
        m_memory.write32(td + kStat6c, stat);
        m_memory.write32(td + kStat70, 0xFu);
        m_memory.write32(td + kStat74, 0u);
        return true;
    }

    bool IopSio2::exchange(int port, const uint8_t *in, size_t size, uint8_t *out)
    {
        if (size < 2u)
            return false;
        if (port < 2)
        {
            // Pad 1 is always plugged in; pad 2 only if the host reports state for it.
            uint8_t probe[6];
            if (port != 0 && !m_host.readPad(port, probe))
                return false;
            exchangePad(port, in, size, out);
            return true;
        }
        Card &card = m_cards[static_cast<size_t>(port & 1)];
        if (!card.present)
            return false;
        exchangeCard(card, in, size, out);
        return true;
    }

    void IopSio2::exchangePad(int port, const uint8_t *in, size_t size, uint8_t *out)
    {
        Pad &pad = m_pads[static_cast<size_t>(port & 1)];
        std::fill(out, out + size, 0x00u);
        const uint8_t command = in[1];
        out[0] = 0xFFu;
        out[1] = pad.config ? 0xF3u : pad.pressure ? 0x79u : pad.analog ? 0x73u : 0x41u;
        if (size > 2u)
            out[2] = 0x5Au;
        const auto reply = [&](std::initializer_list<uint8_t> bytes)
        {
            size_t i = 3u;
            for (const uint8_t b : bytes)
                if (i < size)
                    out[i++] = b;
        };
        const uint8_t arg = size > 3u ? in[3] : 0u;

        if (!pad.config && (command == 0x42u || command == 0x43u))
        {
            // Buttons are active low; sticks centred unless the host says otherwise.
            uint8_t state[6] = {0xFFu, 0xFFu, 0x80u, 0x80u, 0x80u, 0x80u};
            (void)m_host.readPad(port, state);
            reply({state[0], state[1], state[2], state[3], state[4], state[5]});
            if (pad.pressure)
            {
                // Pressure bytes: right, left, up, down, triangle, circle, cross, square, L1, R1, L2, R2.
                static constexpr uint8_t kBits[12] = {5, 7, 4, 6, 12, 13, 14, 15, 10, 11, 8, 9};
                const uint16_t buttons = static_cast<uint16_t>(state[0] | (state[1] << 8));
                for (size_t i = 0; i < 12u && 9u + i < size; ++i)
                    out[9u + i] = (buttons & (1u << kBits[i])) ? 0x00u : 0xFFu;
            }
            if (command == 0x43u && arg == 1u)
                pad.config = true;
            return;
        }

        switch (command)
        {
        case 0x43: // leave config mode
            pad.config = arg != 0u;
            break;
        case 0x44: // set analog/digital
            pad.analog = arg != 0u;
            if (!pad.analog)
                pad.pressure = false;
            break;
        case 0x45: // query model: DualShock 2
            reply({0x03u, 0x02u, static_cast<uint8_t>(pad.analog ? 1u : 0u), 0x02u, 0x01u, 0x00u});
            break;
        case 0x46:
            if (arg == 0u)
                reply({0x00u, 0x00u, 0x01u, 0x02u, 0x00u, 0x0Au});
            else
                reply({0x00u, 0x00u, 0x01u, 0x01u, 0x01u, 0x14u});
            break;
        case 0x47:
            reply({0x00u, 0x00u, 0x02u, 0x00u, 0x01u, 0x00u});
            break;
        case 0x4C:
            reply({0x00u, 0x00u, 0x00u, static_cast<uint8_t>(arg == 0u ? 0x04u : 0x07u), 0x00u, 0x00u});
            break;
        case 0x4D: // vibration map
            reply({0x00u, 0x01u, 0xFFu, 0xFFu, 0xFFu, 0xFFu});
            break;
        case 0x4F: // enable pressure-sensitive buttons
            pad.analog = true;
            pad.pressure = true;
            reply({0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x5Au});
            break;
        case 0x41:
            if (pad.pressure)
                reply({0xFFu, 0xFFu, 0x03u, 0x00u, 0x00u, 0x5Au});
            else
                reply({0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x5Au});
            break;
        case 0x40:
            reply({0x00u, 0x00u, 0x02u, 0x00u, 0x00u, 0x5Au});
            break;
        default:
            break;
        }
    }

    void IopSio2::exchangeCard(Card &card, const uint8_t *in, size_t size, uint8_t *out)
    {
        std::fill(out, out + size, 0xFFu);
        const uint8_t command = in[1];
        const auto ack = [&](size_t at)
        {
            if (at + 1u < size)
            {
                out[at] = 0x2Bu;
                out[at + 1u] = card.terminator;
            }
        };
        const auto rawOffset = [&]() -> std::streamoff
        { return static_cast<std::streamoff>(card.page) * kPageRaw + card.offset; };

        switch (command)
        {
        case 0x21: // set erase address
        case 0x22: // set write address
        case 0x23: // set read address
            if (size >= 6u)
                card.page = static_cast<uint32_t>(in[2] | (in[3] << 8) | (in[4] << 16) | (in[5] << 24));
            card.offset = 0u;
            ack(size - 2u);
            break;
        case 0x26: // get specs
            if (size >= 13u)
            {
                out[2] = 0x2Bu; // card flags: ECC, bad-block list
                out[3] = static_cast<uint8_t>(kPageData);
                out[4] = static_cast<uint8_t>(kPageData >> 8);
                out[5] = static_cast<uint8_t>(kBlockPages);
                out[6] = 0u;
                out[7] = static_cast<uint8_t>(kCardPages);
                out[8] = static_cast<uint8_t>(kCardPages >> 8);
                out[9] = static_cast<uint8_t>(kCardPages >> 16);
                out[10] = static_cast<uint8_t>(kCardPages >> 24);
                out[11] = xorBytes(out + 3, 8u);
                out[12] = card.terminator;
            }
            break;
        case 0x27: // set terminator
            if (size > 2u)
                card.terminator = in[2];
            ack(size - 2u);
            break;
        case 0x28: // get terminator
            if (size >= 5u)
            {
                out[2] = 0x2Bu;
                out[3] = card.terminator;
                out[4] = card.terminator;
            }
            break;
        case 0x42: // write data
        case 0x43: // read data
        {
            const size_t count = size > 2u ? in[2] : 0u;
            if (size < count + 6u)
                break;
            if (command == 0x43u)
            {
                out[3] = 0x2Bu;
                card.file.clear();
                card.file.seekg(rawOffset());
                card.file.read(reinterpret_cast<char *>(out + 4), static_cast<std::streamsize>(count));
                out[4u + count] = xorBytes(out + 4, count);
                out[5u + count] = card.terminator;
            }
            else
            {
                card.file.clear();
                card.file.seekp(rawOffset());
                card.file.write(reinterpret_cast<const char *>(in + 3), static_cast<std::streamsize>(count));
                card.file.flush();
                ack(4u + count);
            }
            card.offset += static_cast<uint32_t>(count);
            break;
        }
        case 0x82: // erase the block holding the erase address
        {
            const std::vector<char> blank(kPageRaw * kBlockPages, static_cast<char>(0xFF));
            card.file.clear();
            card.file.seekp(static_cast<std::streamoff>(card.page - card.page % kBlockPages) * kPageRaw);
            card.file.write(blank.data(), static_cast<std::streamsize>(blank.size()));
            card.file.flush();
            ack(size - 2u);
            break;
        }
        default: // probe (0x11), 0x12, read/write end (0x81), 0xBF, 0xF3, auth traffic
            ack(size - 2u);
            break;
        }
    }
}
