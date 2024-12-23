#include <Arduino.h>
#include <Wire.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_tls_errors.h>
#include <nvs_flash.h>
#include <nvs.h>

#define TRIGGER_PIN 2
#define ECHO_PIN 4

#if defined(ARDUHAL_LOG_LEVEL)
    #undef ARDUHAL_LOG_LEVEL
#endif
#define ARDUHAL_LOG_LEVEL ARDUHAL_LOG_LEVEL_INFO

#if !defined(SSID)
    #define SSID "esp2_ctrl_jfw"
    #define SSID_LENGTH sizeof(SSID) - 1
#endif
#if !defined(PASSWORD)
    #define PASSWORD "esp2_connect_1cke"
    #define PASSWORD_LENGTH sizeof(PASSWORD) - 1
#endif

uint32_t count = 0;
wifi_ap_config_t ap_conf = {};
wifi_sta_config_t sta_conf = {};

const inline uint32_t read_sensor() noexcept {
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    const uint32_t register timeout_cycles = microsecondsToClockCycles(1000);
    const uint32_t input_timeout = microsecondsToClockCycles(1000);
    const uint32_t start_cycle_count = cpu_ll_get_cycle_count();
    while (digitalRead(ECHO_PIN) == LOW) {
        if (cpu_ll_get_cycle_count() - start_cycle_count > timeout_cycles) {
            return -1;
        }
    }
    while (digitalRead(ECHO_PIN) == HIGH) {
        if (cpu_ll_get_cycle_count() - start_cycle_count > timeout_cycles) {
            return -1;
        }
    }
    const uint32_t pulse_start_cycle_count = cpu_ll_get_cycle_count();
    while (digitalRead(ECHO_PIN) == LOW) {
        if (cpu_ll_get_cycle_count() - start_cycle_count > timeout_cycles) {
            return -1;
        }
    }
    return clockCyclesToMicroseconds(cpu_ll_get_cycle_count() - pulse_start_cycle_count);
}

void wifi_event_handler(void *arg, esp_event_base_t eb, int32_t eid, void *data) {
    if (eid == WIFI_EVENT_STA_START) {
        esp_err_t retcode = ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
        if (retcode != ESP_OK) {
            Serial.print("Failed to connect to ");
            for (int i = 0; i < ap_conf.ssid_len; i++)
                Serial.write(ap_conf.ssid[i]);
            Serial.print(" with retcode: ");
            Serial.println(retcode);
        }
    }
}

void ip_event_handler(void *arg, esp_event_base_t eb, int32_t eid, void *data) {
    if (eid == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) data;
        ESP_LOGI("ESP", "Got IP: %s", ip4addr_ntoa(&(event->ip_info.ip)));
    }
}

void setup() {
    esp_err_t retcode = nvs_flash_init();
    if (retcode == ESP_ERR_NVS_NO_FREE_PAGES || retcode == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        retcode = nvs_flash_init();
    }
    ESP_ERROR_CHECK(retcode);
    Serial.begin(9600);
    while(!Serial) {}
    wifi_init_config_t init_conf = WIFI_INIT_CONFIG_DEFAULT();
    retcode = ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_init(&init_conf));
    if (retcode != ESP_OK) {
        Serial.print("Failed to initialize wifi with return code: ");
        Serial.println(retcode);
    }
    retcode = ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_mode(wifi_mode_t::WIFI_MODE_APSTA));
    if (retcode != ESP_OK) {
        Serial.print("Failed to set wifi mode with return code: ");
        Serial.println(retcode);
    }
    wifi_config_t conf = {};
    retcode = ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_get_config(wifi_interface_t::WIFI_IF_AP, &conf));
    if (retcode != ESP_OK) {
        Serial.print("Failed to get wifi AP config with return code: ");
        Serial.println(retcode);
    }
    ap_conf = conf.ap;
    conf = {};
    retcode = esp_wifi_get_config(wifi_interface_t::WIFI_IF_STA, &conf);
    if (retcode != ESP_OK) {
        Serial.print("Failed to get wifi STA config with return code: ");
        Serial.println(retcode);
    }
    sta_conf = conf.sta;
    retcode = ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_start());
    if (retcode != ESP_OK) {
        Serial.print("Failed to start wifi with return code: ");
        Serial.println(retcode);
    }
    memcpy(ap_conf.ssid, SSID, strlen(SSID));
    memcpy(ap_conf.password, PASSWORD, strlen(PASSWORD));

    EventGroupHandle_t wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, wifi_event_group));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, wifi_event_group));
    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIGGER_PIN, OUTPUT);
}

bool detected = false;
long start_time = 0, end_time = 0;

void loop() {
    if (!detected) {
        if (read_sensor() != -1) {
            detected = true;
            start_time = cpu_ll_get_cycle_count();
        }
    } else {
        if (read_sensor() == -1) {
            detected = false;
            end_time = cpu_ll_get_cycle_count();
            long duration = end_time - start_time;
            Serial.print("Duration: ");
            Serial.println(duration);
        }
    }

}

