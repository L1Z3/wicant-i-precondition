/* OTA control on FEE1, before CAN protocol dispatch. */
#include "ble_ota.h"
#include <stddef.h>

const char *ble_ota_process_control(const uint8_t *data, uint16_t len)
{
    // Reserve only complete, standalone ATXOB writes. Never search inside a
    // binary frame or retain a partial prefix that might belong to GVRET.
    if (len > 0 && data[len - 1] == '\n') len--;
    if (len < 6 || data[len - 1] != '\r') return NULL;
    static const char prefix[] = "atxob";
    for (unsigned i = 0; i < sizeof(prefix) - 1; i++)
    {
        uint8_t ch = data[i];
        if (ch >= 'A' && ch <= 'Z') ch += 'a' - 'A';
        if (ch != prefix[i]) return NULL;
    }
    // A 32-bit image size takes at most ten digits. Leave room for malformed
    // arguments too, without accepting embedded CRs, NULs or binary payloads.
    char args[16];
    unsigned count = len - 6;
    if (count >= sizeof(args)) return NULL;
    for (unsigned i = 0; i < count; i++)
    {
        uint8_t ch = data[i + 5];
        if (ch < 0x20 || ch > 0x7e) return NULL;
        if (ch >= 'A' && ch <= 'Z') ch += 'a' - 'A';
        args[i] = ch;
    }
    args[count] = 0;
    const char *reply = ble_ota_command(args);
    return reply != NULL ? reply : "?";
}
