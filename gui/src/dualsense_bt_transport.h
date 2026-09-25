#pragma once

// dualsense_bt_transport.h — transporte HID crudo (hidapi) para el DualSense
// por Bluetooth. SDL2 no expone escrituras de reportes arbitrarios, así que el
// parche abre el control por su cuenta con hidapi (precedente: DS4Windows,
// awalol/dualsense-bt-haptics).
//
// Layout del reporte 0x31 de salida (78 B) calcado de
// `struct dualsense_output_report_bt` en drivers/hid/hid-playstation.c:
//   [0]=0x31 [1]=seq_tag [2]=tag [3]=valid_flag0 [4]=valid_flag1
//   [5]=motor_right [6]=motor_left [7]=headphone_volume [8]=speaker_volume
//   [9]=mic_volume [10]=audio_control [11]=mute_button_led
//   [12]=power_save_control [13..39]=reserved [40]=audio_control2 ...
//   [74..77]=CRC32-LE

#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <utility>

struct hid_device_; // hidapi (opaco)
typedef struct hid_device_ hid_device;

class DualSenseBtTransport
{
public:
    DualSenseBtTransport();
    ~DualSenseBtTransport();
    DualSenseBtTransport(const DualSenseBtTransport &) = delete;
    DualSenseBtTransport &operator=(const DualSenseBtTransport &) = delete;

    // Busca un DualSense por Bluetooth y lo abre. false si no se encuentra.
    // Detección en dos niveles: primero heurística de ruta hidapi ("bth"),
    // luego confirmación real leyendo un input report (0x31 = Bluetooth,
    // 0x01 = USB). Así no depende del formato del path en cada Windows.
    bool open();
    void close();
    bool isOpen() const { return dev_ != nullptr; }

    // Callback opcional de diagnóstico (lo conecta StreamSession al log de
    // chiaki): informa qué candidatos vio hidapi y cuál se eligió.
    void setLogCallback(std::function<void(const std::string &)> cb) { log_cb_ = std::move(cb); }

    // Escribe un output report SIN el prefijo 0xA2 (lo agrega aquí según la
    // plataforma: Windows sí, Linux/hidraw no). data incluye su CRC final.
    bool writeOutputReport(const uint8_t *data, size_t len);

    // Reporte 0x31: gate del mic (power_save_control MIC_MUTE), volumen del
    // mic y LED del botón de mute. Parte del "mute honesto".
    bool sendMicControlReport(bool mic_muted);

    // Sub-paquete 0x11 (9 B: 0x91 0x07 + máscara + 4 B volume-ish + contador
    // + resto) dentro de un reporte 0x32. Máscara 0xFF = mic on, 0xFE = off
    // (SPEC §5: bit 6 de la máscara es mandatorio).
    bool sendMicControlSubpacket(bool mic_on);

    // Lee un input report. Retorna bytes leídos, 0 = timeout, <0 = error.
    int readInputReport(uint8_t *buf, size_t len, int timeout_ms);

    // Contador único de secuencia de 4 bits para TODOS los output reports
    // (como hace el driver hid-playstation del kernel). Protegido por el
    // mismo lock de I/O: sin data race entre háptica, poll del mic y GUI.
    uint8_t nextSeq()
    {
        std::lock_guard<std::mutex> lock(io_mutex_);
        uint8_t s = seq_;
        seq_ = static_cast<uint8_t>((seq_ + 1) & 0x0F);
        return s;
    }

private:
    hid_device *dev_;
    uint8_t seq_;
    uint8_t mic_counter_; // SPEC: += 2 por reporte con sub-paquete 0x11
    // El transporte se usa desde 3 hilos (háptica, poll del mic, GUI/mute):
    // hidapi no garantiza thread-safety, así que se serializa aquí.
    std::mutex io_mutex_;
    std::function<void(const std::string &)> log_cb_;
    void Log(const char *fmt, ...);
};
