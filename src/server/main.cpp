#include <esp_wifi.h>
#include <esp_wifi_types.h>
#include <esp32/rom/gpio.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <esp_system.h>
#include <esp_now.h>

static const char *HELLO = "Hello world"; 

wifi_ap_config_t apcfg = {};
wifi_sta_config_t stacfg = {};

void inline wifi_init() {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
}

void inline wifi_set_config() {
    wifi_config_t cfg = {};
    ESP_ERROR_CHECK(esp_wifi_set_mode(wifi_mode_t::WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_get_config(wifi_interface_t::WIFI_IF_AP, &cfg));
    apcfg = cfg.ap;
    ESP_ERROR_CHECK(esp_wifi_get_config(wifi_interface_t::WIFI_IF_STA, &cfg));
    stacfg = cfg.sta;
}

esp_now_peer_info_t peer_info = {
    .peer_addr = {0xF8, 0xB3, 0xB7, 0x3F, 0x0B, 0xAC},
    .channel = 1,
    .ifidx = WIFI_IF_STA,
    .encrypt = false
};

void send_callback(const uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGI("send_callback", "got status %s\n", status == esp_now_send_status_t::ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAILURE");
}

extern "C" void app_main(void) {
    auto ret = ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_init());
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    wifi_init();
    wifi_set_config();
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(send_callback));
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
    while (true) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_now_send(nullptr, (uint8_t*)HELLO, sizeof HELLO));
        vTaskDelay(2000); 
    }
    return;
}