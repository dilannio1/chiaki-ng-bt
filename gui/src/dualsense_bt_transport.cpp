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
    startWriter();
    return true;
}

void DualSenseBtTransport::close()
{
    stopWriter();
    if (dev_)
    {
        hid_close(dev_);
        dev_ = nullptr;
    }
    hid_exit();
}

void DualSenseBtTransport::startWriter()
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (writer_running_)
        return;
    writer_stop_ = false;
    write_queue_.clear();
    writer_running_ = true;
    writer_thread_ = std::thread(&DualSenseBtTransport::writerLoop, this);
}

void DualSenseBtTransport::stopWriter()
{
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (!writer_running_)
            return;
        writer_stop_ = true;
        writer_running_ = false;
    }
    queue_cv_.notify_all();
    if (writer_thread_.joinable())
        writer_thread_.join();
    std::lock_guard<std::mutex> lock(queue_mutex_);
    write_queue_.clear(); // descarta lo que quedó sin enviar (apagado limpio)
}

void DualSenseBtTransport::writerLoop()
{
    while (true)
    {
        std::vector<uint8_t> pkt;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] { return writer_stop_ || !write_queue_.empty(); });
            if (!write_queue_.empty())
            {
                pkt = std::move(write_queue_.front());
                write_queue_.pop_front();
            }
            else if (writer_stop_)
            {
                return;
            }
            else
            {
                continue; // spurious wakeup sin trabajo
            }
        }
        // hid_write bloqueante FUERA del lock de la cola y con el lock de
        // I/O (serializado con los hid_read del poll del mic).
        std::lock_guard<std::mutex> io_lock(io_mutex_);
        if (dev_ && !pkt.empty())
            hid_write(dev_, pkt.data(), pkt.size());
    }
}

bool DualSenseBtTransport::writeOutputReport(const uint8_t *data, size_t len)
{
    if (!data || len == 0)
        return false;
    // El paquete se encola TAL CUAL (reporte crudo con su CRC final).
    //
    // OJO: en Windows NO se antepone 0xA2 al buffer. hidapi/WriteFile
    // interpreta el primer byte como report ID; 0xA2 no existe en el
    // descriptor y el reporte jamás llegaba válido al firmware. El 0xA2
    // es el encabezado HIDP que el stack Bluetooth pone en el cable, y
    // solo pertenece al cálculo del CRC (cfr. Crc32). Así lo hacen SDL,
    // DS4Windows y el kernel: 78 B empezando en 0x31 en ambas plataformas.
    std::vector<uint8_t> pkt;
    pkt.reserve(len);
    pkt.insert(pkt.end(), data, data + len);
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (!writer_running_)
            return false;
        if (write_queue_.size() >= kWriteQueueMax)
            write_queue_.pop_front(); // BT saturado: descarta lo más viejo
        write_queue_.push_back(std::move(pkt));
    }
    queue_cv_.notify_one();
    return true;
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
    // Layout verificado contra 5 implementaciones + captura real
    // (dualsense-neo, HeadsetPlayMusic): 0xFF en r[9], contador al final.
    r[9] = 0xFF;
    r[10] = mic_counter_;
    // Un sub-paquete por reporte -> el contador avanza de 1 en 1
    // (el += 2 es convención de reportes de 2 frames, cfr. DS5Dongle).
    mic_counter_ = static_cast<uint8_t>(mic_counter_ + 1);
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
