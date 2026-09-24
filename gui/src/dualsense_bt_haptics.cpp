#include "dualsense_bt_haptics.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace
{

constexpr uint8_t SUBPKT_HAPTIC = 0x12;
constexpr size_t HAPTIC_FRAME_BYTES = 64; // 64 B = 10.667 ms @ 3 kHz estereo s8
constexpr int HAPTIC_RATE = 3000;
constexpr int HAPTIC_CHANNELS = 2;

struct ReportSize
{
    uint8_t id;
    size_t size;
};

// Escalera de reportes de salida BT (tamanos con ID, sin el 0xA2).
constexpr ReportSize kLadder[] = {
    {0x32, 142}, {0x33, 206}, {0x34, 270}, {0x35, 334},
    {0x36, 398}, {0x37, 462}, {0x38, 526}, {0x39, 547},
};

uint32_t g_crc_table[256];
bool g_crc_ready = false;

void ensure_crc_table()
{
    if (g_crc_ready)
        return;
    for (uint32_t i = 0; i < 256; ++i)
    {
        uint32_t c = i;
        for (int k = 0; k < 8; ++k)
            c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        g_crc_table[i] = c;
    }
    g_crc_ready = true;
}

// CRC-32 IEEE sobre [0xA2] + report_sin_crc.
uint32_t crc32_dualsense(const uint8_t *report_sin_crc, size_t len)
{
    ensure_crc_table();
    uint32_t crc = 0xFFFFFFFFu;
    auto feed = [&](uint8_t b) { crc = g_crc_table[(crc ^ b) & 0xFF] ^ (crc >> 8); };
    feed(0xA2);
    for (size_t i = 0; i < len; ++i)
        feed(report_sin_crc[i]);
    return crc ^ 0xFFFFFFFFu;
}

} // namespace

DualSenseBtHaptics::DualSenseBtHaptics(WriteReportCb cb, void *userdata, AllocSeqCb seq_cb)
    : cb_(cb), userdata_(userdata), seq_cb_(seq_cb)
{
}

void DualSenseBtHaptics::pushSamples(const int16_t *samples, size_t frame_count, int in_rate)
{
    if (!samples || frame_count == 0 || in_rate <= 0 || !cb_)
        return;
    const double step = static_cast<double>(in_rate) / HAPTIC_RATE;
    while (resample_pos_ < static_cast<double>(frame_count))
    {
        const size_t i0 = static_cast<size_t>(resample_pos_);
        const size_t i1 = std::min(i0 + 1, frame_count - 1);
        const double frac = resample_pos_ - static_cast<double>(i0);
        for (int ch = 0; ch < HAPTIC_CHANNELS; ++ch)
        {
            double v = samples[i0 * 2 + ch] * (1.0 - frac) + samples[i1 * 2 + ch] * frac;
            v *= gain_;
            v = std::clamp(v, -32768.0, 32767.0);
            int s8 = static_cast<int>(std::lround(v / 256.0));
            s8 = std::clamp(s8, -128, 127);
            pending_[pending_len_++] = static_cast<uint8_t>(s8 & 0xFF);
            if (pending_len_ == HAPTIC_FRAME_BYTES)
            {
                emitFrame(pending_);
                pending_len_ = 0;
            }
        }
        resample_pos_ += step;
    }
    resample_pos_ -= static_cast<double>(frame_count); // arrastra la fraccion
}

void DualSenseBtHaptics::flush()
{
    if (pending_len_ == 0)
        return;
    std::memset(pending_ + pending_len_, 0, HAPTIC_FRAME_BYTES - pending_len_);
    emitFrame(pending_);
    pending_len_ = 0;
}

void DualSenseBtHaptics::emitFrame(const uint8_t pcm[64])
{
    uint8_t sub[2 + HAPTIC_FRAME_BYTES];
    sub[0] = SUBPKT_HAPTIC | 0x80; // bit 'sized'
    sub[1] = HAPTIC_FRAME_BYTES;
    std::memcpy(sub + 2, pcm, HAPTIC_FRAME_BYTES);

    constexpr size_t need = 1 + 1 + sizeof(sub) + 4; // id + seq + sub + CRC
    const ReportSize *sel = nullptr;
    for (const auto &r : kLadder)
    {
        if (r.size >= need)
        {
            sel = &r;
            break;
        }
    }
    if (!sel)
        return; // imposible: un frame cabe en el 0x32

    std::vector<uint8_t> report(sel->size, 0); // resto = padding con ceros
    report[0] = sel->id;
    // Secuencia del transporte si hay callback (contador único thread-safe);
    // si no, contador interno (self-test).
    uint8_t seq;
    if (seq_cb_)
        seq = static_cast<uint8_t>(seq_cb_(userdata_) & 0x0F);
    else
    {
        seq = seq_;
        seq_ = static_cast<uint8_t>((seq_ + 1) & 0x0F);
    }
    report[1] = static_cast<uint8_t>(seq << 4);
    std::memcpy(report.data() + 2, sub, sizeof(sub));
    const uint32_t crc = crc32_dualsense(report.data(), sel->size - 4);
    report[sel->size - 4] = static_cast<uint8_t>(crc & 0xFF);
    report[sel->size - 3] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    report[sel->size - 2] = static_cast<uint8_t>((crc >> 16) & 0xFF);
    report[sel->size - 1] = static_cast<uint8_t>((crc >> 24) & 0xFF);

    cb_(report.data(), report.size(), userdata_);
}

#ifdef DUALSENSE_BT_HAPTICS_TEST
// Self-test sin hardware: verifica conversion, framing, tamanos, CRC y
// secuencia contra la especificacion documentada.
#include <cstdio>

namespace
{
struct Collector
{
    std::vector<std::vector<uint8_t>> reports;
    static bool cb(const uint8_t *data, size_t len, void *ud)
    {
        auto *self = static_cast<Collector *>(ud);
        self->reports.emplace_back(data, data + len);
        return true;
    }
};

uint32_t test_crc(const uint8_t *d, size_t n) { return crc32_dualsense(d, n); }

int failures = 0;
void check(bool ok, const char *msg)
{
    std::printf("[%s] %s\n", ok ? "OK" : "FAIL", msg);
    if (!ok)
        ++failures;
}
} // namespace

int main()
{
    std::printf("== self-test dualsense_bt_haptics (sin hardware) ==\n");
    Collector col;
    DualSenseBtHaptics haptics(&Collector::cb, &col);

    // 1 s de seno 100 Hz estereo int16 @ 48 kHz, en chunks como en streaming.
    const int in_rate = 48000, secs = 1;
    const size_t total_frames = static_cast<size_t>(in_rate) * secs;
    std::vector<int16_t> chunk(480 * 2);
    int max_abs = 0;
    for (size_t base = 0; base < total_frames; base += 480)
    {
        for (size_t n = 0; n < 480; ++n)
        {
            const double t = static_cast<double>(base + n) / in_rate;
            const int v = static_cast<int>(20000.0 * std::sin(2.0 * 3.141592653589793 * 100.0 * t));
            chunk[n * 2] = chunk[n * 2 + 1] = static_cast<int16_t>(v);
        }
        haptics.pushSamples(chunk.data(), 480, in_rate);
    }
    haptics.flush();

    check(!col.reports.empty(), "se emitieron reportes");
    size_t haptic_bytes = 0;
    bool seq_ok = true, crc_ok = true, size_ok = true, hdr_ok = true;
    for (size_t i = 0; i < col.reports.size(); ++i)
    {
        const auto &r = col.reports[i];
        const uint8_t id = r[0];
        size_t expected = 0;
        for (const auto &e : kLadder)
            if (e.id == id)
                expected = e.size;
        if (expected == 0 || r.size() != expected)
            size_ok = false;
        if ((r[1] >> 4) != (i % 16))
            seq_ok = false;
        uint32_t stored = static_cast<uint32_t>(r[r.size() - 4]) |
                          (static_cast<uint32_t>(r[r.size() - 3]) << 8) |
                          (static_cast<uint32_t>(r[r.size() - 2]) << 16) |
                          (static_cast<uint32_t>(r[r.size() - 1]) << 24);
        if (stored != test_crc(r.data(), r.size() - 4))
            crc_ok = false;
        if (r.size() > 4 && (r[2] != 0x92 || r[3] != 0x40))
            hdr_ok = false; // 0x12|sized, len 64
        for (size_t k = 4; k < 68 && k < r.size(); ++k)
        {
            const int8_t s = static_cast<int8_t>(r[k]);
            max_abs = std::max(max_abs, std::abs(static_cast<int>(s)));
        }
        haptic_bytes += 64;
    }
    check(size_ok, "tamanos de la escalera");
    check(seq_ok, "secuencia 0..15 con wrap");
    check(crc_ok, "CRC-32 de cada reporte");
    check(hdr_ok, "cabecera 0x92 0x40 en sub-paquetes");
    // 6000 bytes = 93.75 frames -> 93 completos + 1 parcial que flush() rellena
    check(haptic_bytes == 94u * 64, "94 frames (93 + 1 parcial rellenado por flush)");
    check(max_abs >= 70 && max_abs <= 90, "amplitud s8 coherente (seno 20000 -> ~78)");
    std::printf("reportes: %zu, primer id: 0x%02x\n", col.reports.size(), col.reports[0][0]);
    std::printf(failures == 0 ? "== superado ==\n" : "== %d FALLOS ==\n", failures);
    return failures == 0 ? 0 : 1;
}
#endif
