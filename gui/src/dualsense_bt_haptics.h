#pragma once

#include <cstddef>
#include <cstdint>

// DualSenseBtHaptics: convierte frames de haptica (PCM estereo int16, como los
// entrega chiaki-ng desde el PS5) en output reports HID Bluetooth con
// sub-paquetes 0x11 (control) + 0x12 (haptico), segun la documentacion
// comunitaria verificada en hardware
// (dualsense-neo SPEC, SAxense, driver hid-playstation).
//
// Formato en el cable: [report_id][seq<<4][sub-paquetes][padding de ceros]
//   [CRC32-LE]. El CRC-32 se calcula sobre 0xA2 + bytes del reporte sin los
//   ultimos 4 (verificado contra captura real).
//
// El sub-paquete 0x11 (9 B: 91 07 FE 00 00 00 00 FF <ctr>) SIEMPRE acompaña
// al 0x12: es el que activa el pipeline de audio/hapticos en el firmware.
// La mascara 0xFE es mic-OFF (no habilita el microfono). Captura real de
// hardware (dualsense-neo, report.rs::matches_captured_wire_bytes):
//   39 00 91 07 FE 00 00 00 00 FF 00 92 40 <64 B> ...
//
// Diseno: sin dependencias de Qt/SDL/hidapi. El reporte sale por un
// callback; chiaki-ng lo conecta al transporte (hidapi) en la integracion.

class DualSenseBtHaptics
{
public:
    // data/len: un reporte completo SIN el prefijo 0xA2 de transporte.
    // len es siempre uno de los tamanos de la escalera (142..547).
    // Retorna true si el reporte fue aceptado.
    using WriteReportCb = bool (*)(const uint8_t *data, size_t len, void *userdata);

    // Asigna el siguiente valor de secuencia de 4 bits para un reporte.
    // Si se provee, la secuencia la lleva el transporte (contador único
    // thread-safe, como el driver del kernel); si no, el módulo usa su
    // contador interno (modo self-test / standalone).
    using AllocSeqCb = uint8_t (*)(void *userdata);

    explicit DualSenseBtHaptics(WriteReportCb cb, void *userdata, AllocSeqCb seq_cb = nullptr);

    // Muestras hapticas estereo int16 a in_rate Hz. Emite reportes por el
    // callback a medida que se completan frames de 64 bytes (10.667 ms).
    void pushSamples(const int16_t *samples, size_t frame_count, int in_rate);

    // Multiplicador de intensidad (1.0 = normal, segun la consola).
    void setGain(float gain) { gain_ = gain; }

    // Emite el frame parcial pendiente (rellenado con ceros).
    void flush();

private:
    void emitFrame(const uint8_t pcm[64]);

    WriteReportCb cb_;
    void *userdata_;
    AllocSeqCb seq_cb_ = nullptr;
    float gain_ = 1.0f;
    double resample_pos_ = 0.0; // posicion fraccional dentro del input actual
    uint8_t pending_[64];
    size_t pending_len_ = 0;
    uint8_t seq_ = 0;
    // Contador del sub-paquete de control 0x11: avanza de 1 en 1 por reporte
    // (SPEC dualsense-neo: "Its counter steps by 1 per report").
    uint8_t ctrl_counter_ = 0;
};
