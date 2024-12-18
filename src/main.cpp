#include <Arduino.h>
#include <Wire.h>
#include <esp_wifi.h>
#include <esp_tls_errors.h>

#define SENSOR_PIN 0 // Change to actual pin number

uint32_t count = 0;
wifi_ap_config_t ap_conf = {};
wifi_sta_config_t sta_conf = {};

const inline uint32_t read_sensor() noexcept {
    const uint32_t register timeout_cycles = microsecondsToClockCycles(10000000);
    const uint32_t input_timeout = microsecondsToClockCycles(200000);
    const uint32_t start_cycle_count = cpu_ll_get_cycle_count();
    while (digitalRead(SENSOR_PIN) == HIGH) {
        if (cpu_ll_get_cycle_count() - start_cycle_count > input_timeout) {
            return -1;
        }
    }
    while (digitalRead(SENSOR_PIN) == LOW) {
        if (cpu_ll_get_cycle_count() - start_cycle_count > timeout_cycles) {
            return -1;
        }
    }
    const uint32_t pulse_start_cycle_count = cpu_ll_get_cycle_count();
    while (digitalRead(SENSOR_PIN) == HIGH) {
        if (cpu_ll_get_cycle_count() - start_cycle_count > timeout_cycles) {
            return -1;
        }
    }
    return clockCyclesToMicroseconds(cpu_ll_get_cycle_count() - pulse_start_cycle_count);
}

void setup() {
    wifi_init_config_t init_conf = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t retcode;
    if ((retcode = esp_wifi_init(&init_conf)) != ESP_OK) {
        Serial.print("Failed to initialize wifi with return code: ");
        Serial.println(retcode);
    }
    if ((retcode = esp_wifi_set_mode(wifi_mode_t::WIFI_MODE_APSTA)) != ESP_OK) {
        Serial.print("Failed to set wifi mode with return code: ");
        Serial.println(retcode);
    }
    wifi_config_t conf = {};
    if ((retcode = esp_wifi_get_config(wifi_interface_t::WIFI_IF_AP, &conf))) {
        Serial.print("Failed to get wifi AP config with return code: ");
        Serial.println(retcode);
    }
    ap_conf = conf.ap;
    conf = {};
    if ((retcode = esp_wifi_get_config(wifi_interface_t::WIFI_IF_STA, &conf))) {
        Serial.print("Failed to get wifi STA config with return code: ");
        Serial.println(retcode);
    }
    sta_conf = conf.sta;
    if ((retcode = esp_wifi_start()) != ESP_OK) {
        Serial.print("Failed to start wifi with return code: ");
        Serial.println(retcode);
    }
    if ((retcode = esp_wifi_connect()) != ESP_OK) {
        Serial.print("Failed to connect to ");
        // Serial.print(ap_conf.);
        Serial.print(" with retcode: ");
        Serial.println(retcode);
    }

    // Set sensor pin mode to input
    pinMode(SENSOR_PIN, INPUT);

}


void loop() {
    const uint32_t dur = read_sensor();
    Serial.print("duration: ");
    Serial.println(dur);
    if (dur != -1) {
        count++;
    }

}

