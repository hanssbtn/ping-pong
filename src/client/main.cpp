#include <esp_event.h>
#include <freertos/task.h>
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <Arduino.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <cstring>
#include <esp_pm.h>
#include <esp32-hal-log.h>
#include <esp32-hal-adc.h>

#ifdef DEBUG
#include <esp_gdbstub.h>
#endif

#ifdef CONFIG_ESP_SYSTEM_PANIC_HALT
#pragma message("using CONFIG_ESP_SYSTEM_PANIC_HALT option")
#endif
#ifdef CONFIG_ESP_SYSTEM_PANIC_REBOOT
#pragma message("using CONFIG_ESP_SYSTEM_PANIC_REBOOT option")
#endif

#define TEST
#pragma message("Building TEST mode")

// TODO: add actual pin number
#define LED_PIN 2
#define VIBRATION_SENSOR_PIN 3
#define IF_SENSOR_PIN 4
#define QUEUE_LEN 64

char buf[100] = {};
const uint8_t *data = (const uint8_t*)"123";

const esp_now_peer_info_t server_info = {
    .peer_addr = {0xF8, 0xB3, 0xB7, 0x45, 0x4E, 0xAC},
    .lmk = {},
    .channel = 1,
    .ifidx = WIFI_IF_STA,
    .encrypt = false,
    .priv = NULL
};

bool should_update_server = false;
bool goal = false;
QueueHandle_t table_queue = NULL, goal_queue = NULL;

void send_callback(const uint8_t *mac_addr, esp_now_send_status_t status) {
    should_update_server = status != esp_now_send_status_t::ESP_NOW_SEND_SUCCESS;
    log_i("should_update_server [%p]: %d\n", &should_update_server, should_update_server);
}

void recv_callback(const uint8_t *mac_addr, const uint8_t *data, int len) {
    std::memcpy(buf, reinterpret_cast<const char*>(data), len);
    digitalWrite(18, HIGH);
    log_i("got data %s\n", buf);
}

void listen_goal(void *param) {
    uint8_t res = 0;
    uint8_t a = 0;
    while (1) {
        log_i("queue: %p\n", table_queue);
        auto k = xQueueReceive(table_queue, &res, portMAX_DELAY);
        if (k) {
            log_i("received %c\n", res);
            auto start_cycle = cpu_ll_get_cycle_count();
            log_i("a: %p\n", &a);
            log_i("res: %p\n", &res);
            res = 0;
            // Try to read IF sensor for 100 microseconds.
            while (cpu_ll_get_cycle_count() - start_cycle < microsecondsToClockCycles(100)) {
                log_i("%u\n", cpu_ll_get_cycle_count() - start_cycle);
                if (digitalRead(IF_SENSOR_PIN)) {
                    res = 1;
                    break;
                }
            }
            #ifdef TEST
            auto l = esp_random();
            res = (uint8_t)l & 1;
            log_i("received %d\n", res);
            #endif
            if (res) {
                char msg = 'G';
                auto x = xQueueSend(goal_queue, &msg, portMAX_DELAY);
                log_i("got %d\n", x);
            }
            log_i("Finished sending.\n");
        }
        log_i("Finished.\n");
        log_i("stack bytes remaining: %u bytes\n", uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(1);
    }
}

void listen_table(void *param) {
    const char msg = 'T';
    uint8_t res;
    while (1) {
        res = 0;
        // Try to read vibration sensor for 100 microseconds.
        auto start_cycle = cpu_ll_get_cycle_count();
        while (cpu_ll_get_cycle_count() - start_cycle < microsecondsToClockCycles(100)) {
            log_i("%u\n", cpu_ll_get_cycle_count() - start_cycle);
            if (digitalRead(VIBRATION_SENSOR_PIN)) {
                res = 1;
                break;
            }
        }
        #ifdef TEST
        auto k = esp_random();
        res = (uint8_t)k & 1;
        #endif
        if (res) {
            log_i("queue: %p\n", table_queue);
            auto x = xQueueSend(table_queue, &msg, portMAX_DELAY);
            auto u = uxQueueSpacesAvailable(table_queue);
            log_i("got %d, space remaining: %u\n", x, u);
        }
        auto u =  uxQueueMessagesWaiting(table_queue);
        log_i("table: %u messages waiting.\n", u);
        u = uxQueueMessagesWaiting(goal_queue);
        log_i("goal: %u messages waiting.\n", u);
        log_i("stack bytes remaining: %u bytes\n", uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(1);
    }
}

void send_packet(void *param) {
    uint8_t res = 0;
    while (1) {
        uint8_t a = 0;
        log_i("%u messages waiting.\n", uxQueueMessagesWaiting(goal_queue));
        log_i("goal_queue: %p\n", goal_queue);
        if ((a = xQueuePeek(goal_queue, &res, portMAX_DELAY))) {
            esp_now_send(server_info.peer_addr, data, strlen("123"));
            #ifdef TEST
            log_i("a: %p\n", &a);
            log_i("res: %p\n", &res);
            log_i("data: %p\n", data);
            log_i("got %c\n", res);
            log_i("goal_queue: %p", goal_queue);
            auto u = uxQueueSpacesAvailable(goal_queue);
            log_i("%u goal queue bytes remaining\n", u);
            auto x = xQueueReceive(goal_queue, &res, 0);
            log_i("received data: %d\n", x);
            #endif
            if (!should_update_server) {
                auto x = xQueueReceive(goal_queue, &res, 0);
            }
            log_i("Finished sending.\n");
        }
        log_i("a: %d, Finished.\n", a);
        log_i("stack bytes remaining: %u bytes\n", uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(1);
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    #ifdef DEBUG
    esp_gdbstub_init();
    #endif
    pinMode(IF_SENSOR_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(VIBRATION_SENSOR_PIN, INPUT);
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_add_peer(&server_info));
    ESP_ERROR_CHECK(esp_now_register_send_cb(esp_now_send_cb_t(send_callback)));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb_t(recv_callback)));
    #ifdef ENABLE_SLEEP
    ESP_ERROR_CHECK(esp_sleep_enable_wifi_wakeup());
    esp_pm_config_esp32_t pm_cfg;
    ESP_ERROR_CHECK(esp_pm_get_configuration(&pm_cfg));
    pm_cfg.light_sleep_enable = true;
    ESP_ERROR_CHECK(esp_pm_configure(&pm_cfg));
    #endif

    table_queue = xQueueCreate(QUEUE_LEN, (sizeof(char)));
    if (!table_queue) {
        log_e("Failed to allocate memory for table_queue.\n");
        abort();
    }
    goal_queue = xQueueCreate(QUEUE_LEN, (sizeof(char)));
    if (!goal_queue) {
        log_e("Failed to allocate memory for goal_queue.\n");
        abort();
    }
    if (xTaskCreatePinnedToCore(
        listen_table,
        "listen_table",
        4096,
        NULL,
        0,
        NULL,
        1
    ) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) {
        log_e("Failed to allocate memory for %s task.\n", "listen_table");
        abort();
    };
    if (xTaskCreatePinnedToCore(
        listen_goal,
        "listen_goal",
        4096,
        NULL,
        0,
        NULL,
        0
    ) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) {
        log_e("Failed to allocate memory for %s task.\n", "listen_goal");
        abort();
    };
    if (xTaskCreatePinnedToCore(
        send_packet,
        "send_packet",
        4096,
        NULL,
        0,
        NULL,
        1
    ) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) {
        log_e("Failed to allocate memory for %s task.\n", "send_packet");
        abort();
    };
}

void loop() {
    // Delete unused loop task
    vTaskDelete(NULL);
}