#pragma once

// dualsense_bt_mic.h — captura del micrófono del DualSense por Bluetooth.
//
// El reporte de entrada 0x31 tiene DOS variantes (SPEC §7): estado del gamepad
// o audio del mic, distinguidas por el bit 1 del byte 1. La variante mic trae
// un frame Opus 48 kHz mono de 480 muestras (10 ms) desde el offset 3.
//
// MUTE HONESTO (requisito del proyecto): `setMuted()` es la ÚNICA fuente de
// verdad. En una sola operación hace las tres cosas:
//   1) sub-paquete 0x11 con la máscara del mic (on/off),
//   2) reporte 0x31 de control (gate MIC_MUTE + volumen + LED del botón),
//   3) flag interno que filtra los reportes de entrada.
// El LED nunca puede decir "muteado" mientras la captura sigue activa.
//
// Hilos: el poll corre en un hilo propio (los input reports llegan a
// ~560/s). `setMuted()` puede llamarse desde el hilo GUI; el transporte
// serializa los accesos hidapi con mutex.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <thread>

class DualSenseBtTransport;
struct OpusDecoder; // libopus (opaco)

class DualSenseBtMic
{
public:
    // PCM mono int16 @ 48 kHz, siempre 480 muestras (10 ms) por llamada.
    using PcmFrameCb = void (*)(const int16_t *pcm, size_t samples, void *userdata);

    DualSenseBtMic(DualSenseBtTransport *transport, PcmFrameCb cb, void *userdata);
    ~DualSenseBtMic();
    DualSenseBtMic(const DualSenseBtMic &) = delete;
    DualSenseBtMic &operator=(const DualSenseBtMic &) = delete;

    bool begin(); // crea el decodificador Opus; false si falla

    // Mute honesto: ver encabezado. Arranca muteado (default seguro).
    bool setMuted(bool muted);
    bool isMuted() const { return muted_.load(); }

    // Inicia/detiene el hilo de captura. Llamar después de begin().
    void startPolling();
    void stopPolling();

private:
    void pollLoop();
    bool handleReport(const uint8_t *r, size_t len);

    DualSenseBtTransport *transport_;
    PcmFrameCb cb_;
    void *userdata_;
    OpusDecoder *dec_;
    std::atomic<bool> muted_;
    std::atomic<bool> polling_;
    std::thread poll_thread_;
    unsigned poll_count_;
};
