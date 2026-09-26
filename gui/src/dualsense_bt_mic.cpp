#include "dualsense_bt_mic.h"
#include "dualsense_bt_transport.h"
#include "dualsense_bt_common.h"

#include <opus/opus.h>

#include <chrono>

namespace
{
constexpr size_t kMicReportSize = 78;
constexpr size_t kOpusOffset = 3; // [0]=0x31 [1]=seq|flags [2]=UNKNOWN
constexpr size_t kCrcSize = 4;    // CRC-32 al final del input 0x31 por BT
constexpr int kSampleRate = 48000;
constexpr int kFrameSamples = 480; // 10 ms
} // namespace

DualSenseBtMic::DualSenseBtMic(DualSenseBtTransport *transport, PcmFrameCb cb, void *userdata)
    : transport_(transport), cb_(cb), userdata_(userdata), dec_(nullptr), muted_(true), poll_count_(0)
{
}

DualSenseBtMic::~DualSenseBtMic()
{
    stopPolling();
    if (dec_)
        opus_decoder_destroy(dec_);
}

bool DualSenseBtMic::begin()
{
    if (dec_)
        return true;
    int err = 0;
    dec_ = opus_decoder_create(kSampleRate, 1, &err);
    return err == OPUS_OK && dec_ != nullptr;
}

bool DualSenseBtMic::setMuted(bool muted)
{
    if (!transport_ || !transport_->isOpen())
        return false;
    // Best-effort: intentar ambas escrituras aunque una falle, para no dejar
    // el gate del mic y el LED a medias (mute honesto). El llamador decide con
    // el valor de retorno.
    bool ok = true;
    if (!transport_->sendMicControlSubpacket(!muted))
        ok = false;
    if (!transport_->sendMicControlReport(muted))
        ok = false;
    // Mute honesto: el estado interno solo refleja lo que el control
    // realmente aceptó. Si falla, el llamador lo ve en el retorno.
    if (ok)
    {
        muted_ = muted;
        poll_count_ = 0;
        mic_reports_seen_ = 0;
        mic_diag_warned_ = false;
        if (!muted)
            unmute_time_ = std::chrono::steady_clock::now();
    }
    return ok;
}

void DualSenseBtMic::startPolling()
{
    if (polling_.exchange(true))
        return;
    poll_thread_ = std::thread(&DualSenseBtMic::pollLoop, this);
}

void DualSenseBtMic::stopPolling()
{
    if (!polling_.exchange(false))
        return;
    if (poll_thread_.joinable())
        poll_thread_.join();
}

void DualSenseBtMic::pollLoop()
{
    uint8_t r[96];
    while (polling_.load())
    {
        if (!transport_ || !transport_->isOpen() || !dec_)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            continue;
        }
        // Diagnóstico del enable (una sola vez): si 5 s después del unmute
        // no llegó ninguna variante-mic, el 0x11 probablemente no hizo latch.
        if (!muted_.load() && !mic_diag_warned_.load() && mic_reports_seen_.load() == 0 &&
            std::chrono::steady_clock::now() - unmute_time_ > std::chrono::seconds(5))
        {
            mic_diag_warned_ = true;
            transport_->Log("dualsense-bt: sin variante-mic 5 s tras unmute (el enable 0x11 podria no hacer latch)");
        }
        // Drena sin bloquear: los input reports llegan a ~560/s.
        for (int i = 0; i < 16; i++)
        {
            int n = transport_->readInputReport(r, sizeof(r), 0);
            if (n <= 0)
                break;
            // Reenvío periódico del enable mientras el mic está abierto
            // (el enable viaja "en el stream"; validar en hardware).
            if (!muted_.load() && (++poll_count_ % 800) == 0)
                transport_->sendMicControlSubpacket(true);
            handleReport(r, static_cast<size_t>(n));
        }
        // Espera corta con timeout para no girar en vacío. Timeout corto (5 ms):
        // el read bloquea el lock de I/O del transporte y un timeout largo
        // retrasaría los reportes de háptica (frames de ~10.7 ms).
        int n = transport_->readInputReport(r, sizeof(r), 5);
        if (n > 0)
            handleReport(r, static_cast<size_t>(n));
    }
}

bool DualSenseBtMic::handleReport(const uint8_t *r, size_t len)
{
    if (len < kMicReportSize)
        return true; // reporte corto: ignorar
    if (r[0] != 0x31)
        return true; // no es 0x31
    if (!(r[1] & 0x02))
        return true; // variante gamepad-state, no mic
    // Diagnóstico del enable: contar variantes-mic aunque estemos muteados
    // (si el control streamea, el enable hizo latch).
    if (++mic_reports_seen_ == 1 && transport_)
        transport_->Log("dualsense-bt: variante-mic 0x31 recibida (enable con latch)");
    if (muted_)
        return true; // mute honesto: ni siquiera decodificamos
    if (!cb_)
        return true;

    // El frame Opus es auto-delimitado; opus_packet_parse nos da su longitud
    // exacta. Se excluyen los 4 B de CRC-32 al final del reporte BT
    // (cfr. hid-playstation.c: el input 0x31 BT cierra con CRC, seed 0xA1).
    const unsigned char *data = r + kOpusOffset;
    opus_int32 data_len = static_cast<opus_int32>(len - kOpusOffset - kCrcSize);
    unsigned char toc = 0;
    const unsigned char *frames[48];
    opus_int16 sizes[48];
    int payload_offset = 0;
    int nb_frames = opus_packet_parse(data, data_len, &toc, frames, sizes, &payload_offset);
    if (nb_frames != 1)
        return true; // no es un frame simple: descartar sin romper el stream
    opus_int32 packet_len = payload_offset + sizes[0];

    int16_t pcm[kFrameSamples];
    int decoded = opus_decode(dec_, data, packet_len, pcm, kFrameSamples, 0);
    if (decoded != kFrameSamples)
        return true; // frame corrupto: descartar
    cb_(pcm, static_cast<size_t>(decoded), userdata_);
    return true;
}
