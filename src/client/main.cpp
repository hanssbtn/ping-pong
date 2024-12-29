#include <Arduino.h>
#include <Wire.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <WiFi.h>
#include <cstring>
#include <client.h>
#include <esp_pm.h>

char buf[100] = {};

bool should_update_server = false;
bool goal = false;

void send_callback(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == esp_now_send_status_t::ESP_NOW_SEND_SUCCESS) {
        should_update_server = false;
    }
}

void recv_callback(const uint8_t *mac_addr, const uint8_t *data, int len) {
    std::memcpy(buf, reinterpret_cast<const char*>(data), len);
    digitalWrite(18, HIGH);
    Serial.printf("got data %s\n", buf);
}

void listen_goal(bool should) {
    if (should) {
        // Turn on sensor 
        return;
    }
    
}

void setup() {
    Serial.begin(115200);   
    WiFi.mode(WIFI_STA);
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_add_peer(&server_info);
    esp_now_register_send_cb(esp_now_send_cb_t(send_callback));
    esp_now_register_recv_cb(esp_now_recv_cb_t(recv_callback));
    #ifdef ENABLE
    esp_sleep_enable_wifi_wakeup();
    esp_pm_config_esp32_t pm_cfg;
    esp_pm_get_configuration(&pm_cfg);
    pm_cfg.light_sleep_enable = true;
    esp_pm_configure(&pm_cfg);
    #endif
    listen_goal(true);
}

void loop() {
    if (goal) {
        should_update_server = true;
        listen_goal(false);
    }
    while (should_update_server) {
        esp_now_send(nullptr, reinterpret_cast<const uint8_t*>("G"), 1ULL);
        delayMicroseconds(200);
    }
    Serial.printf("data: %s\n", buf);
    delay(1000);
    //
}