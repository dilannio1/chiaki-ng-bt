#pragma once

#include <cstddef>
#include <cstdint>

// DualSenseBtHaptics: convierte frames de haptica (PCM estereo int16, como los
// entrega chiaki-ng desde el PS5) en output reports HID Bluetooth con
// sub-paquetes 0x12, segun la documentacion comunitaria
// (dualsense-neo SPEC, SAxense, driver hid-playstation).
//
// Formato en el cable (via HIDP): [0xA2][report_id][seq<<4][sub-paquetes]
//   [padding de ceros][CRC32-LE]. El CRC-32 se calcula sobre
//   0xA2 + bytes del reporte sin los ultimos 4.
//
// NOTA ABIERTA: la cobertura exacta del CRC (prefijo 0xA2 vs seed 0xA2 del
// kernel) se confirma con captura antes de probar en hardware. Ver
// docs del proyecto.
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
};
