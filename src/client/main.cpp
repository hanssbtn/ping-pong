#include <esp_wifi.h>
#include <esp_wifi_types.h>
#include <esp32/rom/gpio.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_now.h>
#include <nvs_flash.h>
#include <esp_system.h>
#include <esp_event_base.h>
#include <esp_mac.h>
#include <esp_netif.h>
#include <esp_wifi_netif.h>
#include <esp_event.h>
#include <esp_crc.h>
#include <cstring>

char buf[100] = {};

void recv_callback(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    std::memcpy(buf, reinterpret_cast<const char*>(data), len);
    ESP_LOGI("recv_callback", "got data %s\n", buf);
}

esp_now_peer_info_t peer_info = {
    .peer_addr = {0xF8, 0xB3,0xB7,0x45,0x4E,0xAC},
    .channel = 1,
    .ifidx = WIFI_IF_STA,
    .encrypt = false,
};

void app_main() {
    auto ret = ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_init());
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(recv_callback));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(wifi_interface_t::WIFI_IF_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
    while (true) {
        if (*buf) {
            ESP_LOGI("recv_data", "got data %s\n", buf);
            break;
        }
    }
}