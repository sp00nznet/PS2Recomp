#include "runtime/gs/gs_threaded_backend.h"

#include <chrono>

GSThreadedBackend::GSThreadedBackend(std::unique_ptr<GSRasterBackend> inner)
    : m_inner(std::move(inner)), m_ring(kRing)
{
    m_thread = std::thread([this]() { worker(); });
}

GSThreadedBackend::~GSThreadedBackend()
{
    drain();
    m_quit = true;
    {
        std::lock_guard<std::mutex> lock(m_wakeMutex);
        m_wake.notify_one();
    }
    m_thread.join();
}

// ponytail: one producer (the thread feeding the GS frontend) is assumed; a second one would
// need a lock here.
GSThreadedBackend::Command &GSThreadedBackend::beginPush()
{
    const uint64_t slot = m_pushed.load(std::memory_order_relaxed);
    while (slot - m_done.load(std::memory_order_acquire) >= kRing)
        std::this_thread::yield();
    return m_ring[slot % kRing];
}

void GSThreadedBackend::endPush()
{
    m_pushed.fetch_add(1, std::memory_order_release);
    if (m_sleeping.load(std::memory_order_acquire))
    {
        std::lock_guard<std::mutex> lock(m_wakeMutex);
        m_wake.notify_one();
    }
}

void GSThreadedBackend::drain() const
{
    const uint64_t target = m_pushed.load(std::memory_order_acquire);
    while (m_done.load(std::memory_order_acquire) < target)
        std::this_thread::yield();
}

void GSThreadedBackend::worker()
{
    for (;;)
    {
        uint64_t next = m_done.load(std::memory_order_relaxed);
        if (next == m_pushed.load(std::memory_order_acquire))
        {
            if (m_quit)
                return;
            std::unique_lock<std::mutex> lock(m_wakeMutex);
            m_sleeping.store(true, std::memory_order_release);
            m_wake.wait_for(lock, std::chrono::milliseconds(2), [&]()
                            { return m_quit || m_pushed.load(std::memory_order_acquire) != next; });
            m_sleeping.store(false, std::memory_order_release);
            continue;
        }
        std::lock_guard<std::mutex> exec(m_exec);
        const uint64_t end = m_pushed.load(std::memory_order_acquire);
        for (; next != end; ++next)
        {
            Command &c = m_ring[next % kRing];
            switch (c.kind)
            {
            case Kind::Submit: m_inner->Submit(c.batch); break;
            case Kind::LoadClut: m_inner->LoadClut(c.tex0, c.texclut); break;
            case Kind::BeginTransfer: m_inner->BeginTransfer(c.transfer); break;
            case Kind::UploadImage: m_inner->UploadImage(c.data.data(), static_cast<uint32_t>(c.data.size())); break;
            case Kind::Flush: m_inner->Flush(); break;
            case Kind::TextureFlush: m_inner->TextureFlush(); break;
            case Kind::Sync: m_inner->Sync(c.reason); break;
            case Kind::WriteVram: m_inner->WriteVram(c.psm, c.base, c.bw, c.x, c.y, c.value); break;
            }
            m_done.store(next + 1, std::memory_order_release);
        }
    }
}

void GSThreadedBackend::Initialize(uint8_t *vram, uint32_t vramSize)
{
    drain();
    std::lock_guard<std::mutex> exec(m_exec);
    m_inner->Initialize(vram, vramSize);
}

void GSThreadedBackend::Reset()
{
    drain();
    std::lock_guard<std::mutex> exec(m_exec);
    m_inner->Reset();
}

void GSThreadedBackend::Submit(const GSPrimitiveBatch &batch)
{
    Command &c = beginPush();
    c.kind = Kind::Submit;
    c.batch = batch;
    endPush();
}

void GSThreadedBackend::LoadClut(const GSTex0Reg &tex0, const GSTexClutReg &texclut)
{
    Command &c = beginPush();
    c.kind = Kind::LoadClut;
    c.tex0 = tex0;
    c.texclut = texclut;
    endPush();
}

void GSThreadedBackend::BeginTransfer(const GSTransferCommand &command)
{
    Command &c = beginPush();
    c.kind = Kind::BeginTransfer;
    c.transfer = command;
    endPush();
}

void GSThreadedBackend::UploadImage(const uint8_t *data, uint32_t sizeBytes)
{
    Command &c = beginPush();
    c.kind = Kind::UploadImage;
    c.data.assign(data, data + sizeBytes);
    endPush();
}

void GSThreadedBackend::Flush()
{
    Command &c = beginPush();
    c.kind = Kind::Flush;
    endPush();
}

void GSThreadedBackend::TextureFlush()
{
    Command &c = beginPush();
    c.kind = Kind::TextureFlush;
    endPush();
}

void GSThreadedBackend::Sync(GSSyncReason reason)
{
    if (reason == GSSyncReason::Finish)
    {
        Command &c = beginPush();
        c.kind = Kind::Sync;
        c.reason = reason;
        endPush();
        return;
    }
    drain();
    std::lock_guard<std::mutex> exec(m_exec);
    m_inner->Sync(reason);
}

PresentationFrame GSThreadedBackend::Present(const GSPresentationRequest &request)
{
    drain();
    std::lock_guard<std::mutex> exec(m_exec);
    return m_inner->Present(request);
}

bool GSThreadedBackend::ClearFramebuffer(const GSContext &context, uint32_t rgba)
{
    drain();
    std::lock_guard<std::mutex> exec(m_exec);
    return m_inner->ClearFramebuffer(context, rgba);
}

uint32_t GSThreadedBackend::ConsumeLocalToHostBytes(uint8_t *dst, uint32_t maxBytes)
{
    drain();
    std::lock_guard<std::mutex> exec(m_exec);
    return m_inner->ConsumeLocalToHostBytes(dst, maxBytes);
}

uint32_t GSThreadedBackend::ReadVram(uint32_t psm, uint32_t base, uint32_t bw, uint32_t x, uint32_t y) const
{
    drain();
    std::lock_guard<std::mutex> exec(m_exec);
    return m_inner->ReadVram(psm, base, bw, x, y);
}

void GSThreadedBackend::WriteVram(uint32_t psm, uint32_t base, uint32_t bw, uint32_t x, uint32_t y, uint32_t value)
{
    Command &c = beginPush();
    c.kind = Kind::WriteVram;
    c.psm = psm;
    c.base = base;
    c.bw = bw;
    c.x = x;
    c.y = y;
    c.value = value;
    endPush();
}

void GSThreadedBackend::SnapshotVram(std::vector<uint8_t> &out) const
{
    drain();
    std::lock_guard<std::mutex> exec(m_exec);
    m_inner->SnapshotVram(out);
}

GSTransferSnapshot GSThreadedBackend::GetTransferSnapshot() const
{
    drain();
    std::lock_guard<std::mutex> exec(m_exec);
    return m_inner->GetTransferSnapshot();
}
