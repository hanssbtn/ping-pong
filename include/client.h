#include <esp_now.h>
static const esp_now_peer_info_t server_info = {
    .peer_addr = {0xF8, 0xB3, 0xB7, 0x45, 0x4E, 0xAC},
    .channel = 1,
    .ifidx = WIFI_IF_STA,
    .encrypt = false,
};