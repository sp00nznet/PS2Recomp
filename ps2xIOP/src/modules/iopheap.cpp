#include "module_factories.h"

#include <array>
#include <cstdint>

namespace ps2x::iop::detail
{
    namespace
    {
        // The IOP heap service. sceSifAllocIopHeap / sceSifFreeIopHeap bind this
        // sid and send one u32 (the size, or the address to free); the reply is
        // one u32 (the IOP address, 0 on failure; 0 for a successful free).
        // Titles that link libsif directly reach this over RPC instead of the
        // SDK-level HLE, so without it their first IOP allocation fails.
        constexpr uint32_t kIopHeapSid = 0x80000003u;
        constexpr uint32_t kAllocFunction = 1u;
        constexpr uint32_t kFreeFunction = 2u;
        constexpr std::array<uint32_t, 1> kSids{kIopHeapSid};

        class IopHeapService final : public IopService
        {
        public:
            explicit IopHeapService(IopHost &host)
                : m_host(host)
            {
            }

            [[nodiscard]] std::string_view name() const override
            {
                return "iopheap";
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
                if (request.sid != kIopHeapSid)
                {
                    return result;
                }
                result.handled = true;
                result.resultAddress = request.receive.address;

                uint32_t arg = 0u;
                if (request.send.address == 0u || request.send.size < sizeof(arg) ||
                    !m_host.readGuest(request.send.address, &arg, sizeof(arg)))
                {
                    reply(request, 0u);
                    return result;
                }

                uint32_t value = 0u;
                if (request.function == kAllocFunction)
                {
                    value = m_host.allocateIopHeap(arg);
                }
                else if (request.function == kFreeFunction)
                {
                    value = m_host.freeIopHeap(arg) ? 0u : 0xFFFFFFFFu;
                }
                reply(request, value);
                return result;
            }

        private:
            void reply(const RpcRequest &request, uint32_t value)
            {
                if (request.receive.address != 0u && request.receive.size >= sizeof(value))
                {
                    (void)m_host.writeGuest(request.receive.address, &value, sizeof(value));
                }
            }

            IopHost &m_host;
        };
    }

    std::unique_ptr<IopService> createIopHeapService(IopHost &host)
    {
        return std::make_unique<IopHeapService>(host);
    }
}
