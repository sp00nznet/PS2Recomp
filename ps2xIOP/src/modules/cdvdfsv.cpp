#include "module_factories.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace ps2x::iop::detail
{
    namespace
    {
        // cdvdfsv's file service. libcdvd's sceCdSearchFile() binds this sid and
        // sends { sceCdlFILE result; u32 pad; char name[]; }: the result struct is
        // filled in place and the 4-byte reply is found/not-found.
        constexpr uint32_t kCdvdFsvSid = 0x80000597u;
        constexpr uint32_t kSearchFileFunction = 0x00u;
        constexpr uint32_t kNameOffset = 0x24u;
        constexpr uint32_t kSectorSize = 2048u;

        // Pseudo-LSNs. We serve loose files rather than an ISO image, so a
        // searched file gets a synthetic sector address and any later read is
        // translated back through this map. Starting well past the volume
        // descriptors keeps a zero or tiny LSN reading as "unset".
        constexpr uint32_t kFirstLsn = 0x1000u;

        struct sceCdlFILE
        {
            uint32_t lsn;
            uint32_t size;
            char name[16];
            uint8_t date[8];
        };
        static_assert(sizeof(sceCdlFILE) == 32, "sceCdlFILE must be 32 bytes");

        // "\0FLIST.DIR;1" -> "0FLIST.DIR": drop the leading separator and the
        // ISO9660 version suffix the guest appends.
        std::string normalizeIsoName(const char *raw)
        {
            std::string out(raw ? raw : "");
            const size_t semi = out.find(';');
            if (semi != std::string::npos)
            {
                out.resize(semi);
            }
            for (char &c : out)
            {
                if (c == '\\')
                {
                    c = '/';
                }
            }
            while (!out.empty() && out.front() == '/')
            {
                out.erase(out.begin());
            }
            return out;
        }

        class CdvdFsvService final : public IopService
        {
        public:
            explicit CdvdFsvService(IopHost &host)
                : m_host(host)
            {
            }

            [[nodiscard]] std::string_view name() const override
            {
                return "cdvdfsv";
            }

            [[nodiscard]] std::span<const uint32_t> sids() const override
            {
                return kSids;
            }

            void reset() override
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_byLsn.clear();
                m_byName.clear();
                m_nextLsn = kFirstLsn;
                m_searches = 0u;
                m_hits = 0u;
            }

            [[nodiscard]] RpcResult handleRpc(const RpcRequest &request) override
            {
                RpcResult result;
                if (request.sid != kCdvdFsvSid)
                {
                    return result;
                }

                result.handled = true;
                result.resultAddress = request.receive.address;

                if (request.function != kSearchFileFunction ||
                    request.send.address == 0u ||
                    request.send.size <= kNameOffset)
                {
                    reply(request, 0);
                    return result;
                }

                const uint32_t maxName = request.send.size - kNameOffset;
                std::vector<char> raw(static_cast<size_t>(maxName) + 1u, '\0');
                if (!m_host.readGuest(request.send.address + kNameOffset, raw.data(), maxName))
                {
                    reply(request, 0);
                    return result;
                }
                raw[maxName] = '\0';
                const std::string wanted = normalizeIsoName(raw.data());

                uint32_t lsn = 0u;
                uint64_t size = 0u;
                const bool found = !wanted.empty() && lookup(wanted, lsn, size);

                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    ++m_searches;
                    if (found)
                    {
                        ++m_hits;
                    }
                }

                if (!found)
                {
                    std::ostringstream message;
                    message << "cdvdfsv: searchFile miss for '" << wanted << "'";
                    m_host.log(LogLevel::Warning, message.str());
                    reply(request, 0);
                    return result;
                }

                sceCdlFILE file{};
                file.lsn = lsn;
                file.size = static_cast<uint32_t>(std::min<uint64_t>(size, 0xFFFFFFFFull));
                const size_t slash = wanted.find_last_of('/');
                const std::string base =
                    (slash == std::string::npos) ? wanted : wanted.substr(slash + 1u);
                std::strncpy(file.name, base.c_str(), sizeof(file.name) - 1u);
                (void)m_host.writeGuest(request.send.address, &file, sizeof(file));

                {
                    std::ostringstream message;
                    message << "cdvdfsv: searchFile '" << wanted << "' lsn=" << lsn
                            << " size=" << size;
                    m_host.log(LogLevel::Info, message.str());
                }

                reply(request, 1);
                return result;
            }

            void appendDebugMetrics(std::vector<DebugMetric> &metrics) const override
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                metrics.push_back({"search_requests", m_searches, false});
                metrics.push_back({"search_hits", m_hits, false});
                metrics.push_back({"mapped_files", static_cast<uint32_t>(m_byLsn.size()), false});
            }

        private:
            struct Entry
            {
                uint32_t lsn = 0u;
                uint64_t size = 0u;
                std::string path;
            };

            void reply(const RpcRequest &request, int32_t value)
            {
                if (request.receive.address != 0u && request.receive.size >= sizeof(int32_t))
                {
                    (void)m_host.writeGuest(request.receive.address, &value, sizeof(value));
                }
            }

            bool lookup(const std::string &wanted, uint32_t &lsn, uint64_t &size)
            {
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    const auto it = m_byName.find(wanted);
                    if (it != m_byName.end())
                    {
                        lsn = it->second.lsn;
                        size = it->second.size;
                        return true;
                    }
                }

                const std::string hostPath = m_host.translateGuestPath(wanted);
                const uint64_t handle = m_host.openHostFile(hostPath);
                if (handle == 0u)
                {
                    return false;
                }
                uint64_t fileSize = 0u;
                const bool ok = m_host.hostFileSize(handle, fileSize);
                m_host.closeHostFile(handle);
                if (!ok)
                {
                    return false;
                }

                std::lock_guard<std::mutex> lock(m_mutex);
                Entry entry;
                entry.lsn = m_nextLsn;
                entry.size = fileSize;
                entry.path = wanted;
                const uint64_t sectors = (fileSize + kSectorSize - 1u) / kSectorSize;
                m_nextLsn += static_cast<uint32_t>(std::max<uint64_t>(sectors, 1ull));
                m_byName.emplace(wanted, entry);
                m_byLsn.emplace(entry.lsn, entry);
                lsn = entry.lsn;
                size = fileSize;
                return true;
            }

            inline static constexpr std::array<uint32_t, 1> kSids{kCdvdFsvSid};

            IopHost &m_host;
            mutable std::mutex m_mutex;
            std::unordered_map<uint32_t, Entry> m_byLsn;
            std::unordered_map<std::string, Entry> m_byName;
            uint32_t m_nextLsn = kFirstLsn;
            uint32_t m_searches = 0u;
            uint32_t m_hits = 0u;
        };
    }

    std::unique_ptr<IopService> createCdvdFsvService(IopHost &host)
    {
        return std::make_unique<CdvdFsvService>(host);
    }
}
