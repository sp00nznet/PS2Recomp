#pragma once

#include "runtime/gs/gs_backend.h"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

// Runs another raster backend on its own thread. Calls that only change GS memory (draws, CLUT
// loads, transfers in, flushes) are queued in order; calls that return something or read VRAM
// wait for the queue to drain and then call the wrapped backend directly. The guest can only
// see GS memory through those, so FINISH does not wait.
class GSThreadedBackend final : public GSRasterBackend
{
public:
    explicit GSThreadedBackend(std::unique_ptr<GSRasterBackend> inner);
    ~GSThreadedBackend() override;

    void Initialize(uint8_t *vram, uint32_t vramSize) override;
    void Reset() override;

    void Submit(const GSPrimitiveBatch &batch) override;
    void LoadClut(const GSTex0Reg &tex0, const GSTexClutReg &texclut) override;

    void BeginTransfer(const GSTransferCommand &command) override;
    void UploadImage(const uint8_t *data, uint32_t sizeBytes) override;

    void Flush() override;
    void TextureFlush() override;
    void Sync(GSSyncReason reason) override;
    PresentationFrame Present(const GSPresentationRequest &request) override;

    bool ClearFramebuffer(const GSContext &context, uint32_t rgba) override;
    uint32_t ConsumeLocalToHostBytes(uint8_t *dst, uint32_t maxBytes) override;

    uint32_t ReadVram(uint32_t psm, uint32_t base, uint32_t bw, uint32_t x, uint32_t y) const override;
    void WriteVram(uint32_t psm, uint32_t base, uint32_t bw, uint32_t x, uint32_t y, uint32_t value) override;
    void SnapshotVram(std::vector<uint8_t> &out) const override;
    GSTransferSnapshot GetTransferSnapshot() const override;

private:
    enum class Kind : uint8_t
    {
        Submit,
        LoadClut,
        BeginTransfer,
        UploadImage,
        Flush,
        TextureFlush,
        Sync,
        WriteVram,
    };
    struct Command
    {
        Kind kind;
        GSSyncReason reason;
        GSPrimitiveBatch batch;
        GSTex0Reg tex0;
        GSTexClutReg texclut;
        GSTransferCommand transfer;
        std::vector<uint8_t> data;
        uint32_t psm, base, bw, x, y, value;
    };
    static constexpr uint32_t kRing = 1u << 14;

    Command &beginPush();
    void endPush();
    void drain() const; // returns once every queued command has run; then hold m_exec to use m_inner
    void worker();

    std::unique_ptr<GSRasterBackend> m_inner;
    std::vector<Command> m_ring;
    std::atomic<uint64_t> m_pushed{0};
    std::atomic<uint64_t> m_done{0};
    mutable std::mutex m_exec;
    std::mutex m_wakeMutex;
    std::condition_variable m_wake;
    std::atomic<bool> m_sleeping{false};
    std::atomic<bool> m_quit{false};
    std::thread m_thread;
};
