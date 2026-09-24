#pragma once

// dualsense_bt_common.h — constantes y CRC compartidos por los módulos del
// parche dualsense-bt (transporte, háptica, micrófono).
//
// Formato documentado por la comunidad (dualsense-neo SPEC, SAxense,
// hid-playstation.c del kernel, DS5Dongle). Ver INTEGRATION.md.

#include <cstddef>
#include <cstdint>

namespace DualSenseBt
{

constexpr uint16_t kVidSony = 0x054C;
constexpr uint16_t kPidDualSense = 0x0CE6;
constexpr uint16_t kPidDualSenseEdge = 0x0DF2;

// Prefijo de transporte HIDP para reportes de salida por Bluetooth.
// En Windows (hidapi) se antepone al escribir; en Linux/hidraw el kernel
// maneja la capa HIDP y se escribe el reporte crudo. El CRC siempre se
// calcula sobre 0xA2 + reporte (es lo que verifica el control).
constexpr uint8_t kHidpOutputPrefix = 0xA2;

constexpr size_t kBtReport31Size = 78; // control plane / input state / mic

// CRC-32 IEEE sobre [0xA2] + data[0..len). `data` NO incluye el CRC.
uint32_t Crc32(const uint8_t *data, size_t len);

} // namespace DualSenseBt
