#include "dualsense_bt_transport.h"
#include "dualsense_bt_common.h"

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include <hidapi/hidapi.h>

namespace
{
// Bits calcados de hid-playstation.c
constexpr uint8_t VALID_FLAG0_MIC_VOLUME = 0x40;
constexpr uint8_t VALID_FLAG1_MIC_MUTE_LED = 0x01;
constexpr uint8_t VALID_FLAG1_POWER_SAVE = 0x02;
constexpr uint8_t POWER_SAVE_MIC_MUTE = 0x10; // BIT(4)
constexpr uint8_t MIC_VOLUME_MAX = 0x40;

bool PathLooksBluetooth(const char *path)
{
    if (!path)
        return false;
    std::string p(path);
    for (auto &c : p)
        c = static_cast<char>(tolower(c));
    return p.find("bth") != std::string::npos ||
           p.find("bthenum") != std::string::npos ||
           p.find("bluetooth") != std::string::npos;
}

int64_t NowMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// Confirma que un handle recién abierto es el control por Bluetooth leyendo
// un input report: por BT el reporte de estado es 0x31; por USB es 0x01.
// Es la señal real, no depende del texto del path hidapi.
bool ConfirmBluetooth(hid_device *h, uint8_t &seen_id)
{
    seen_id = 0;
    uint8_t buf[128];
    const int64_t deadline = NowMs() + 300; // el control emite ~250 Hz
    while (NowMs() < deadline)
    {
        int r = hid_read_timeout(h, buf, sizeof(buf), 50);
        if (r > 0)
        {
            seen_id = buf[0];
            if (seen_id == 0x31)
                return true;
            if (seen_id == 0x01)
                return false; // USB: no es lo que buscamos
            // Otro reporte: seguir esperando.
        }
        else if (r < 0)
        {
            return false;
        }
    }
    return false; // sin reportes: no confirmar
}
} // namespace

void DualSenseBtTransport::Log(const char *fmt, ...)
{
    if (!log_cb_)
        return;
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    log_cb_(buf);
}

DualSenseBtTransport::DualSenseBtTransport() : dev_(nullptr), seq_(0), mic_counter_(0)
{
}

DualSenseBtTransport::~DualSenseBtTransport()
{
    close();
}

bool DualSenseBtTransport::open()
{
    if (dev_)
        return true;
    if (hid_init() != 0)
        return false;

    hid_device_info *devs = hid_enumerate(DualSenseBt::kVidSony, 0);
    std::vector<std::pair<std::string, bool>> cands; // (path, parece BT por ruta)
    for (hid_device_info *d = devs; d; d = d->next)
    {
        if (d->product_id != DualSenseBt::kPidDualSense &&
            d->product_id != DualSenseBt::kPidDualSenseEdge)
            continue;
        if (!d->path)
            continue;
        bool bt_hint = PathLooksBluetooth(d->path);
        Log("dualsense-bt: candidato hidapi pid=0x%04x bt_hint=%d path=%s",
            d->product_id, bt_hint ? 1 : 0, d->path);
        cands.emplace_back(d->path, bt_hint);
    }
    hid_free_enumeration(devs);
    if (cands.empty())
    {
        Log("dualsense-bt: hidapi no enumeró ningún DualSense");
        hid_exit();
        return false;
    }
    // Dos pasadas: primero los que parecen BT por la ruta, luego el resto.
    // La confirmación definitiva es el input report (0x31 = BT, 0x01 = USB),
    // así un USB nunca se abre con protocolo BT aunque el path no matchee.
    for (int pass = 0; pass < 2 && !dev_; pass++)
    {
        for (auto &c : cands)
        {
            if ((pass == 0) != c.second)
                continue;
            hid_device *h = hid_open_path(c.first.c_str());
            if (!h)
            {
                Log("dualsense-bt: no se pudo abrir %s", c.first.c_str());
                continue;
            }
            uint8_t seen_id = 0;
            if (ConfirmBluetooth(h, seen_id))
            {
                dev_ = h;
                Log("dualsense-bt: control BT confirmado (input 0x31) en %s", c.first.c_str());
                break;
            }
            Log("dualsense-bt: %s no confirmó BT (input 0x%02x), se omite",
                c.first.c_str(), seen_id);
            hid_close(h);
        }
    }
    if (!dev_)
    {
        Log("dualsense-bt: ningún candidato confirmó Bluetooth");
        hid_exit();
        return false;
    }
    return true;
}

void DualSenseBtTransport::close()
{
    if (dev_)
    {
        hid_close(dev_);
        dev_ = nullptr;
    }
    hid_exit();
}

bool DualSenseBtTransport::writeOutputReport(const uint8_t *data, size_t len)
{
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (!dev_ || !data || len == 0)
        return false;
#ifdef _WIN32
    // En Windows el reporte BT viaja con el prefijo HIDP 0xA2
    // (precedente: DS4Windows).
    std::vector<uint8_t> buf(len + 1);
    buf[0] = DualSenseBt::kHidpOutputPrefix;
    std::memcpy(buf.data() + 1, data, len);
    return hid_write(dev_, buf.data(), buf.size()) == static_cast<int>(buf.size());
#else
    // En Linux/hidraw el kernel maneja la capa HIDP: reporte crudo.
    return hid_write(dev_, data, len) == static_cast<int>(len);
#endif
}

bool DualSenseBtTransport::sendMicControlReport(bool mic_muted)
{
    uint8_t r[DualSenseBt::kBtReport31Size];
    std::memset(r, 0, sizeof(r));
    r[0] = 0x31;
    r[1] = static_cast<uint8_t>(nextSeq() << 4);
    r[2] = 0x10; // tag (convención BT del kernel; validar en hardware)
    r[3] = VALID_FLAG0_MIC_VOLUME;
    r[4] = static_cast<uint8_t>(VALID_FLAG1_MIC_MUTE_LED | VALID_FLAG1_POWER_SAVE);
    r[9] = MIC_VOLUME_MAX;
    r[11] = mic_muted ? 1 : 0; // LED del botón: 1 = muteado (naranja)
    r[12] = mic_muted ? POWER_SAVE_MIC_MUTE : 0x00;
    const uint32_t crc = DualSenseBt::Crc32(r, sizeof(r) - 4);
    r[74] = static_cast<uint8_t>(crc & 0xFF);
    r[75] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    r[76] = static_cast<uint8_t>((crc >> 16) & 0xFF);
    r[77] = static_cast<uint8_t>((crc >> 24) & 0xFF);
    return writeOutputReport(r, sizeof(r));
}

bool DualSenseBtTransport::sendMicControlSubpacket(bool mic_on)
{
    // Reporte 0x32 (142 B) con un sub-paquete 0x11 de 9 B. Layout del SPEC §5.
    constexpr size_t kSize = 142;
    uint8_t r[kSize];
    std::memset(r, 0, sizeof(r));
    r[0] = 0x32;
    r[1] = static_cast<uint8_t>(nextSeq() << 4);
    r[2] = 0x91; // 0x11 | sized
    r[3] = 0x07; // longitud del payload
    r[4] = mic_on ? 0xFF : 0xFE;
    // r[5..8] = volume-ish (UNKNOWN, en cero; el SPEC desaconseja adivinar)
    r[9] = mic_counter_;
    // r[10] = resto del payload (cero)
    mic_counter_ = static_cast<uint8_t>(mic_counter_ + 2);
    const uint32_t crc = DualSenseBt::Crc32(r, kSize - 4);
    r[kSize - 4] = static_cast<uint8_t>(crc & 0xFF);
    r[kSize - 3] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    r[kSize - 2] = static_cast<uint8_t>((crc >> 16) & 0xFF);
    r[kSize - 1] = static_cast<uint8_t>((crc >> 24) & 0xFF);
    return writeOutputReport(r, sizeof(r));
}

int DualSenseBtTransport::readInputReport(uint8_t *buf, size_t len, int timeout_ms)
{
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (!dev_ || !buf || len == 0)
        return -1;
    return hid_read_timeout(dev_, buf, len, timeout_ms);
}
