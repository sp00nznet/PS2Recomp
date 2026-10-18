#include "module_factories.h"

#include <array>
#include <cstdint>
#include <algorithm>
#include <cstring>
#include <mutex>

namespace ps2x::iop::detail
{
    namespace
    {
        // LOADFILE, the IOP's module loader. Titles that link Sony's libsif
        // directly (rather than going through the SDK HLE) load their IOP
        // modules with RPCs to this sid: fn 0 loads a module from a path,
        // fn 6 from a buffer already copied into IOP heap. Both reply
        // { int32 module_id; int32 module_result; }. Our IOP is HLE, so there is
        // nothing to load: a module's RPC services are separate IopServices,
        // and here a load just succeeds with a fresh id.
        //
        // fn 0xFF returns the 4-byte LOADFILE version, which libsif compares
        // with the version it was built against and refuses to load on a
        // mismatch.
        // ponytail: one fixed version ("2800", SDK 2.8). A title built against
        // another SDK fails the check; make it a profile setting when one does.
        constexpr uint32_t kLoadFileSid = 0x80000006u;
        constexpr uint32_t kModLoadFunction = 0u;
        constexpr uint32_t kModBufLoadFunction = 6u;
        constexpr uint32_t kVersionFunction = 0xFFu;
        constexpr char kVersion[4] = {'2', '8', '0', '0'};
        constexpr uint32_t kFirstModuleId = 0x100u;
        constexpr std::array<uint32_t, 1> kSids{kLoadFileSid};

        class LoadFileService final : public IopService
        {
        public:
            explicit LoadFileService(IopHost &host)
                : m_host(host)
            {
            }

            [[nodiscard]] std::string_view name() const override
            {
                return "loadfile";
            }

            [[nodiscard]] std::span<const uint32_t> sids() const override
            {
                return kSids;
            }

            void reset() override
            {
            }

            [[nodiscard]] RpcResult handleRpc(const RpcRequest &request) override
            {
                RpcResult result;
                if (request.sid != kLoadFileSid)
                {
                    return result;
                }
                result.handled = true;
                result.resultAddress = request.receive.address;

                if (request.function == kVersionFunction)
                {
                    write(request, kVersion, sizeof(kVersion));
                    return result;
                }

                int32_t reply[2] = {-1, -1};
                if (request.function == kModLoadFunction || request.function == kModBufLoadFunction)
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    reply[0] = static_cast<int32_t>(m_nextId++);
                    reply[1] = 0;
                }
                write(request, reply, sizeof(reply));
                return result;
            }

        private:
            void write(const RpcRequest &request, const void *data, size_t size)
            {
                if (request.receive.address != 0u)
                {
                    (void)m_host.writeGuest(request.receive.address, data,
                                            std::min<size_t>(size, request.receive.size));
                }
            }

            IopHost &m_host;
            std::mutex m_mutex;
            uint32_t m_nextId = kFirstModuleId;
        };
    }

    std::unique_ptr<IopService> createLoadFileService(IopHost &host)
    {
        return std::make_unique<LoadFileService>(host);
    }
}
