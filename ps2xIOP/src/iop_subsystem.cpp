#include "ps2x/iop/iop_subsystem.h"

#include "iop_service.h"
#include "iop_module_manager.h"
#include "emulator/iop_emulator.h"
#include "module_factories.h"
#include "ps2x/iop/ps2_path.h"

#include <algorithm>
#include <cstring>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace ps2x::iop
{
    namespace
    {
        // LOADFILE (rom0:LOADFILE), the IOP's module loader. A title that links
        // Sony's libsif itself loads its IRX modules with raw RPCs to this sid
        // instead of a stubbed sceSifLoadModule, so the subsystem answers them by
        // loading the module for real. Requests (PS2SDK loadfile.h):
        //   fn 0 MOD_LOAD     { s32 arg_len; s32 modres; char path[252]; char args[252]; }
        //   fn 6 MOD_BUF_LOAD { u32 iop_ptr; s32 arg_len; char unused[252]; char args[252]; }
        // Both reply { s32 module_id_or_error; s32 module_start_result; }.
        // fn 0xFF returns the 4-byte version libsif checks before any load.
        // ponytail: fixed version "2800" (SDK 2.8); make it configurable when a
        // title built against another SDK refuses it.
        constexpr uint32_t kLoadFileSid = 0x80000006u;
        constexpr uint32_t kLfModLoad = 0u;
        constexpr uint32_t kLfModBufLoad = 6u;
        constexpr uint32_t kLfVersion = 0xFFu;
        constexpr uint32_t kLfPathMax = 252u;
        constexpr uint32_t kLfArgMax = 252u;

        struct LoadFileRequest
        {
            uint32_t word0; // arg_len (fn 0) or IOP buffer address (fn 6)
            uint32_t word1; // modres (fn 0) or arg_len (fn 6)
            char path[kLfPathMax];
            char args[kLfArgMax];
        };
    }

    class IopSubsystem::Impl
    {
    public:
        explicit Impl(IopHost &hostRef)
            : host(hostRef),
              emulator(hostRef)
        {
            coreServices.emplace_back(detail::createMcservService(host));
            coreServices.emplace_back(detail::createDbcmanService(host));
            coreServices.emplace_back(detail::createCdvdFsvService(host));
            coreServices.emplace_back(detail::createIopHeapService(host));
            coreServices.emplace_back(detail::createLibSdService(host));
            refreshServiceModuleKeys();
            rebuildRoutes();
        }

        bool serviceActive(const detail::IopService &service) const
        {
            return moduleManager.isLoaded(service.moduleAliases());
        }

        void refreshServiceModuleKeys()
        {
            std::vector<std::string> keys;
            for (const auto &service : coreServices)
            {
                for (std::string_view alias : service->moduleAliases())
                    keys.emplace_back(alias);
            }
            moduleManager.setServiceModuleKeys(std::move(keys));
        }

        void rebuildRoutes()
        {
            routes.clear();
            lastError.clear();
            for (const auto &service : coreServices)
            {
                if (!serviceActive(*service))
                    continue;
                for (const uint32_t sid : service->sids())
                {
                    if (!routes.emplace(sid, service.get()).second)
                    {
                        std::ostringstream out;
                        out << "duplicate IOP SID 0x" << std::hex << sid << " in core services";
                        lastError = out.str();
                        routes.clear();
                        return;
                    }
                }
            }
        }

        void recordLoadOutcome(std::string_view path, bool hle)
        {
            constexpr size_t maxOutcomes = 32u;
            if (loadOutcomes.size() >= maxOutcomes || !loggedLoadPaths.emplace(path).second)
                return;
            std::string message = hle ? "[IOP:HLE] fallback module='" : "[IOP:load-failed] module='";
            message.append(path);
            message += hle ? "' physical IRX unavailable; using registered HLE provider"
                           : "' no HLE provider accepted the module; physical IRX was not loaded";
            loadOutcomes.push_back(message);
            host.log(hle ? LogLevel::Info : LogLevel::Warning, message);
        }

        IopHost &host;
        detail::ServiceList coreServices;
        std::unordered_map<uint32_t, detail::IopService *> routes;
        std::vector<std::string> loadOutcomes;
        std::unordered_set<std::string> loggedLoadPaths;
        std::string lastError;
        detail::IopModuleManager moduleManager;
        detail::IopEmulator emulator;
    };

    IopSubsystem::IopSubsystem(IopHost &host)
        : m_impl(std::make_unique<Impl>(host))
    {
    }

    IopSubsystem::~IopSubsystem() = default;
    IopSubsystem::IopSubsystem(IopSubsystem &&) noexcept = default;
    IopSubsystem &IopSubsystem::operator=(IopSubsystem &&) noexcept = default;

    void IopSubsystem::reset()
    {
        m_impl->moduleManager.reset();
        m_impl->loadOutcomes.clear();
        m_impl->loggedLoadPaths.clear();
        for (auto &service : m_impl->coreServices)
        {
            if (service)
            {
                service->reset();
            }
        }
        m_impl->emulator.reset();
        m_impl->refreshServiceModuleKeys();
        m_impl->rebuildRoutes();
    }

    ModuleLoadResult IopSubsystem::loadModule(std::string_view path, const void *arguments, uint32_t argumentSize)
    {
        const ParsedPs2Path parsed = parsePs2Path(path);
        if (!parsed)
            return {true, -1, -1};

        if (parsed.device != Ps2PathDevice::Rom0)
        {
            ModuleLoadResult physical = m_impl->emulator.loadModule(path, arguments, argumentSize);
            if (physical.moduleId > 0)
            {
                m_impl->moduleManager.observePhysicalLoad(physical.moduleId, path);
                m_impl->rebuildRoutes();
                return physical;
            }
        }

        ModuleLoadResult hle = m_impl->moduleManager.loadHle(path);
        if (hle.moduleId > 0)
        {
            m_impl->rebuildRoutes();
            if (parsed.device != Ps2PathDevice::Rom0)
                m_impl->recordLoadOutcome(path, true);
        }
        else
        {
            m_impl->recordLoadOutcome(path, false);
        }
        return hle;
    }

    ModuleLoadResult IopSubsystem::loadModuleBuffer(uint32_t guestAddress, const void *arguments, uint32_t argumentSize)
    {
        return m_impl->emulator.loadModuleBuffer(guestAddress, arguments, argumentSize);
    }

    bool IopSubsystem::stopModule(int32_t moduleId, int32_t *result)
    {
        if (m_impl->moduleManager.stopHle(moduleId, result))
        {
            m_impl->rebuildRoutes();
            return true;
        }
        if (!m_impl->emulator.stopModule(moduleId, result))
            return false;
        m_impl->moduleManager.observePhysicalStop(moduleId);
        m_impl->rebuildRoutes();
        return true;
    }

    void IopSubsystem::runEeCycles(uint64_t eeCycles) noexcept
    {
        m_impl->emulator.runEeCycles(eeCycles);
    }

    RpcAbi IopSubsystem::selectRpcAbi(const RpcAbiRequest &request) const
    {
        for (const auto &service : m_impl->coreServices)
        {
            if (service && m_impl->serviceActive(*service))
            {
                const RpcAbi selected = service->selectRpcAbi(request);
                if (selected != RpcAbi::RuntimeDefault)
                {
                    return selected;
                }
            }
        }
        return RpcAbi::RuntimeDefault;
    }

    bool IopSubsystem::canBindRpc(uint32_t sid) const noexcept
    {
        if (sid == kLoadFileSid || m_impl->routes.find(sid) != m_impl->routes.end())
        {
            return true;
        }
        return m_impl->emulator.hasRpcServer(sid);
    }

    RpcResult IopSubsystem::handleRpc(const RpcRequest &request)
    {
        const auto route = m_impl->routes.find(request.sid);
        detail::IopService *hle = route != m_impl->routes.end() ? route->second : nullptr;

        RpcResult emulated = m_impl->emulator.handleRpc(request);
        if (emulated.handled)
        {
            return emulated;
        }
        if (request.sid == kLoadFileSid && !hle)
        {
            return handleLoadFile(request);
        }
        if (!hle)
        {
            return emulated;
        }
        return hle->handleRpc(request);
    }

    RpcResult IopSubsystem::handleLoadFile(const RpcRequest &request)
    {
        RpcResult result;
        result.handled = true;
        result.resultAddress = request.receive.address;
        IopHost &host = m_impl->host;
        auto reply = [&](const void *data, size_t size)
        {
            if (request.receive.address != 0u)
                (void)host.writeGuest(request.receive.address, data, std::min<size_t>(size, request.receive.size));
        };

        if (request.function == kLfVersion)
        {
            constexpr char version[4] = {'2', '8', '0', '0'};
            reply(version, sizeof(version));
            return result;
        }

        int32_t out[2] = {-1, -1};
        LoadFileRequest in{};
        const bool known = request.function == kLfModLoad || request.function == kLfModBufLoad;
        if (known && host.readGuest(request.send.address, &in, std::min<size_t>(sizeof(in), request.send.size)))
        {
            in.path[kLfPathMax - 1u] = 0;
            const uint32_t argLen = std::min(request.function == kLfModLoad ? in.word0 : in.word1, kLfArgMax);
            const void *args = argLen ? in.args : nullptr;
            const ModuleLoadResult loaded = request.function == kLfModLoad
                                                ? loadModule(in.path, args, argLen)
                                                : m_impl->emulator.loadModuleIopBuffer(in.word0, args, argLen);
            out[0] = loaded.moduleId;
            out[1] = loaded.startResult;

            std::ostringstream message;
            message << "[IOP:loadfile] fn=" << request.function;
            if (request.function == kLfModLoad)
                message << " path='" << in.path << "'";
            else
                message << " iop=0x" << std::hex << in.word0 << std::dec;
            message << " id=" << out[0] << " start=" << out[1];
            host.log(out[0] > 0 ? LogLevel::Info : LogLevel::Warning, message.str());
        }
        reply(out, sizeof(out));
        return result;
    }

    void IopSubsystem::onSifTransfer(const SifTransfer &transfer)
    {
        for (auto &service : m_impl->coreServices)
        {
            if (service && m_impl->serviceActive(*service))
            {
                service->onSifTransfer(transfer);
            }
        }
        m_impl->emulator.onSifTransfer(transfer);
    }

    uint32_t IopSubsystem::allocateMemory(uint32_t size, uint32_t alignment)
    {
        return m_impl->emulator.allocateMemory(size, alignment);
    }

    bool IopSubsystem::freeMemory(uint32_t address)
    {
        return m_impl->emulator.freeMemory(address);
    }

    bool IopSubsystem::readMemory(uint32_t address, void *destination, size_t size) const
    {
        return m_impl->emulator.readMemory(address, destination, size);
    }

    bool IopSubsystem::writeMemory(uint32_t address, const void *source, size_t size)
    {
        return m_impl->emulator.writeMemory(address, source, size);
    }

    bool IopSubsystem::zeroMemory(uint32_t address, size_t size)
    {
        return m_impl->emulator.zeroMemory(address, size);
    }

    bool IopSubsystem::isMemoryRange(uint32_t address, size_t size) const
    {
        return m_impl->emulator.isMemoryRange(address, size);
    }

    DebugSnapshot IopSubsystem::debugSnapshot() const
    {
        DebugSnapshot snapshot;
        snapshot.emulatorCycles = m_impl->emulator.cycles();
        snapshot.emulatorInstructions = m_impl->emulator.instructions();
        snapshot.emulatorLoadedModules = m_impl->emulator.loadedModuleCount();
        snapshot.emulatorThreads = m_impl->emulator.threadCount();
        snapshot.emulatorRpcServers = m_impl->emulator.rpcServerCount();
        snapshot.diagnostics = m_impl->loadOutcomes;
        if (!m_impl->lastError.empty())
        {
            snapshot.diagnostics.push_back(m_impl->lastError);
        }

        for (const auto &service : m_impl->coreServices)
        {
            DebugService row;
            row.name = service->name();
            row.sids.assign(service->sids().begin(), service->sids().end());
            row.active = m_impl->serviceActive(*service);
            service->appendDebugMetrics(row.metrics);
            snapshot.services.push_back(std::move(row));
        }
        return snapshot;
    }
}
