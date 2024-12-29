#include <Arduino.h>
#include <esp_now.h>
#include <esp_mac.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_random.h>
#include <esp_pm.h>
#include <cstring>

char buf[100] = {};

esp_now_peer_info_t client_infos[2] = {
    {
        .peer_addr = {0xF8, 0xB3,0xB7,0x45,0x6A,0xDC},
        .channel = 1,
        .ifidx = WIFI_IF_STA,
        .encrypt = false
    },
    {
        
    }
};
uint32_t chosen_goal = esp_random() % (sizeof client_infos/sizeof client_infos[0]);
bool sent = false;

void send_callback(const uint8_t *mac_addr, esp_now_send_status_t status) {
    sent = status == esp_now_send_status_t::ESP_NOW_SEND_SUCCESS;
    #ifndef ENABLE
    esp_deep_sleep_start();
    #endif
}

void recv_callback(const uint8_t *mac_addr, const uint8_t *data, int len) {
    chosen_goal = esp_random() % (sizeof client_infos/sizeof client_infos[0]);
    std::memcpy(buf, reinterpret_cast<const char*>(data), len);
    Serial.printf("got data %s\n", buf);
    #ifdef ENABLE
    sent = false;
    #endif
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_add_peer(&client_infos[0]);
    // esp_now_add_peer(&client_infos[1]);
    #ifdef ENABLE
        esp_pm_config_esp32_t pm_cfg;
        esp_pm_get_configuration(&pm_cfg);
        pm_cfg.light_sleep_enable = true;
        esp_pm_configure(&pm_cfg);
        esp_sleep_enable_wifi_wakeup();
    #endif
    esp_now_register_send_cb(esp_now_send_cb_t(send_callback));
    esp_now_register_recv_cb(esp_now_recv_cb_t(recv_callback));
}

void loop() {
    if (!sent) {
        Serial.print("Trying to send data...\n");
        esp_now_send(client_infos[chosen_goal].peer_addr, reinterpret_cast<const uint8_t*>("Hello from ESP32"), strlen("Hello from ESP32"));
        delay(2000);
        return;
    }
    Serial.print("Data sent!!\n");
    #ifndef ENABLE
    esp_deep_sleep_start();
    #endif
}
