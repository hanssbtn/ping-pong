#include <esp_event.h>
#include <freertos/task.h>
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <Arduino.h>
#include <esp_now.h>
#include <esp_mac.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_random.h>
#include <esp_pm.h>
#include <cstring>

#ifdef CONFIG_ESP_SYSTEM_PANIC_HALT
#warning "using CONFIG_ESP_SYSTEM_PANIC_HALT option"
#endif
#ifdef CONFIG_ESP_SYSTEM_PANIC_REBOOT
#warning "using CONFIG_ESP_SYSTEM_PANIC_REBOOT option"
#endif

#define LCD_PIN 1

char buf[100] = {};

esp_now_peer_info_t client_infos[2] = {
    {
        .peer_addr = {0xF8, 0xB3,0xB7,0x45,0x6A,0xDC},
        .channel = 1,
        .ifidx = WIFI_IF_STA,
        .encrypt = false
    },
    {
        .peer_addr = {0xF8, 0xB3,0xB7,0x45,0x6A,0xDC},
        .channel = 1,
        .ifidx = WIFI_IF_STA,
        .encrypt = false
    }
};

uint64_t goals = 0; 
uint32_t chosen_goal = esp_random() % (sizeof client_infos/sizeof client_infos[0]);
bool sent = false;

void notify_client(void *param);

void send_callback(const uint8_t *mac_addr, esp_now_send_status_t status) {
    sent = status == esp_now_send_status_t::ESP_NOW_SEND_SUCCESS;
    #ifdef ENABLE_SLEEP
    esp_deep_sleep_start();
    #endif
}

void recv_callback(const uint8_t *mac_addr, const uint8_t *data, int len) {
    chosen_goal = esp_random() % (sizeof client_infos/sizeof client_infos[0]);
    goals++;
    std::memcpy(buf, reinterpret_cast<const char*>(data), len);
    log_i("got data %s\n", buf);
    #ifndef ENABLE_SLEEP
    sent = false;
    #endif
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_add_peer(&client_infos[0]);
    // esp_now_add_peer(&client_infos[1]);
    #ifdef ENABLE_SLEEP
    esp_pm_config_esp32_t pm_cfg;
    esp_pm_get_configuration(&pm_cfg);
    pm_cfg.light_sleep_enable = true;
    esp_pm_configure(&pm_cfg);
    esp_sleep_enable_wifi_wakeup();
    #endif
    pinMode(12, INPUT);
    esp_now_register_send_cb(esp_now_send_cb_t(send_callback));
    esp_now_register_recv_cb(esp_now_recv_cb_t(recv_callback));
}

void notify_client(void *param) {
    while (1) {
        log_i("stack bytes remaining: %u bytes\n", uxTaskGetStackHighWaterMark(nullptr));
        esp_now_send(client_infos[0].peer_addr, reinterpret_cast<const uint8_t*>("G"), 1);
        vTaskDelay(1);
    }
}

void loop() {
    if (!sent) {
        log_i("Trying to send data...\n");
        esp_now_send(client_infos[chosen_goal].peer_addr, reinterpret_cast<const uint8_t*>("Hello from ESP32"), strlen("Hello from ESP32"));
        vTaskDelay(1);
        return;
    }
    
    log_i("Data sent!!\n");
}