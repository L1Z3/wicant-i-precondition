// Exercise the control gate that runs before SavvyCAN/SLCAN/ELM327 dispatch.
#include <string.h>
#include "test_support.h"
#include "ble_ota_control.c"

static unsigned calls;
static char last_args[16];
const char *ble_ota_command(const char *args)
{
    calls++;
    strcpy(last_args, args);
    if (!strcmp(args, "")) return "XOB:1";
    if (!strcmp(args, "end")) return "XOB:DONE";
    if (!strcmp(args, "1048576")) return "XOB:READY";
    return NULL;
}

static void command(const char *input, const char *args, const char *reply)
{
    unsigned before = calls;
    const char *result = ble_ota_process_control((const uint8_t *)input, strlen(input));
    CHECK(result != NULL);
    if (result) CHECK(!strcmp(result, reply));
    CHECK(calls == before + 1);
    CHECK(!strcmp(last_args, args));
}

int main(void)
{
    command("ATXOB\r", "", "XOB:1");
    command("atxob1048576\r", "1048576", "XOB:READY");
    command("AtXoBeNd\r\n", "end", "XOB:DONE");
    command("ATXOBbad\r", "bad", "?");

    // Non-OTA traffic, incomplete commands and combined writes reach the
    // selected parser unchanged. An embedded ATXOB in binary data isn't OTA.
    const char *passthrough[] = {
        "", "A", "ATXOB", "ATXOB\rATZ\r", "ATZ\r", "ATXPC\r",
        "t1231AA\r", "S6\r", "ATXOB1234567890123456\r",
        "ATXOB\r\r", "\xf1\x00" "ATXOB\r"
    };
    unsigned before = calls;
    for (unsigned i = 0; i < sizeof(passthrough) / sizeof(passthrough[0]); i++)
        CHECK(ble_ota_process_control((const uint8_t *)passthrough[i], strlen(passthrough[i])) == NULL);
    const uint8_t binary[] = {'A','T','X','O','B',0,'\r'};
    CHECK(ble_ota_process_control(binary, sizeof(binary)) == NULL);
    CHECK(calls == before);
    command("ATXOB\r", "", "XOB:1");
    if (!test_failures) puts("BLE OTA control: all checks passed");
    return test_failures ? 1 : 0;
}
