#include "dualsense_bt_transport.h"
#include "dualsense_bt_common.h"

#include <cstring>
#include <string>
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
    return p.find("bth") != std::string::npos || p.find("bluetooth") != std::string::npos;
}
} // namespace

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
    hid_device_info *best = nullptr;
    bool best_is_bt = false;
    for (hid_device_info *d = devs; d; d = d->next)
    {
        if (d->product_id != DualSenseBt::kPidDualSense &&
            d->product_id != DualSenseBt::kPidDualSenseEdge)
            continue;
        bool is_bt = PathLooksBluetooth(d->path);
        if (!best || (is_bt && !best_is_bt))
        {
            best = d;
            best_is_bt = is_bt;
        }
    }
    std::string path;
    // Solo Bluetooth: si el mejor candidato no es BT, no abrir nada. Abrir el
    // control por USB con el protocolo BT rompería la coexistencia con SDL
    // (por USB la háptica y el mic los maneja chiaki-ng por audio USB).
    if (best && best_is_bt)
        path = best->path;
    hid_free_enumeration(devs);
    if (path.empty())
    {
        hid_exit();
        return false;
    }
    dev_ = hid_open_path(path.c_str());
    if (!dev_)
    {
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
