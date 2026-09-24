#include "dualsense_bt_common.h"

namespace DualSenseBt
{

namespace
{
uint32_t g_table[256];
bool g_ready = false;

void EnsureTable()
{
    if (g_ready)
        return;
    for (uint32_t i = 0; i < 256; ++i)
    {
        uint32_t c = i;
        for (int k = 0; k < 8; ++k)
            c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        g_table[i] = c;
    }
    g_ready = true;
}
} // namespace

uint32_t Crc32(const uint8_t *data, size_t len)
{
    EnsureTable();
    uint32_t crc = 0xFFFFFFFFu;
    auto feed = [&](uint8_t b) { crc = g_table[(crc ^ b) & 0xFF] ^ (crc >> 8); };
    feed(kHidpOutputPrefix);
    for (size_t i = 0; i < len; ++i)
        feed(data[i]);
    return crc ^ 0xFFFFFFFFu;
}

} // namespace DualSenseBt
