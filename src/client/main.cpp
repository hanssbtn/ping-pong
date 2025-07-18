#include <esp_event.h>
#include <esp32-hal-adc.h>
#include <freertos/task.h>
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <Arduino.h>
#include <esp_now.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <cstring>
#include <Wire.h>
#include <esp_pm.h>
#include <FastLED.h>
#include <esp32-hal-log.h>
#include <LiquidCrystal_I2C.h>

#ifdef DEBUG
#include <esp_gdbstub.h>
#endif

#ifdef CONFIG_ESP_SYSTEM_PANIC_HALT
#pragma message("using CONFIG_ESP_SYSTEM_PANIC_HALT option")
#endif
#ifdef CONFIG_ESP_SYSTEM_PANIC_REBOOT
#pragma message("using CONFIG_ESP_SYSTEM_PANIC_REBOOT option")
#endif

#define SDA_PIN 21
#define SCL_PIN 22
#define LED_PIN 2 // Example pin for visual feedback
#define SOUND_SENSOR_PIN 35 // Update with actual pin
#define IF_SENSOR_PIN 25 // Update with actual pin
#define NUM_LEDS 56

// Define how close in time sensor events must be (milliseconds)
#define STALE_DATA_THRESHOLD_MS 500U // e.g., events must be within 0.5 seconds
#define SOUND_THRESHOLD 10 
#define ACCELEROMETER_THRESHOLD 10 

#define SEND_TAG "SEND"
#define RECV_TAG "RECV"
#define LOOP_TAG "LOOP"
#define SETUP_TAG "SETUP"
#define SERVER_TAG "SERVER"
#define SENSOR_TAG "SENSOR"
#define PACKET_TAG "PACKET"

// Data to send back to server on success
const uint8_t RESPONSE_DATA[] = {'A'};
const size_t RESPONSE_DATA_LEN = sizeof(RESPONSE_DATA);

// Server Info (Update MAC Address if needed)
const esp_now_peer_info_t server_info = {
    .peer_addr = {0xF8, 0xB3, 0xB7, 0x45, 0x4E, 0xAC}, 
    .channel = 1,
    .ifidx = WIFI_IF_STA,
    .encrypt = false,
};

// --- Global Variables for Sensor State ---
volatile uint8_t vibration_count = 0;
volatile uint32_t last_vibration_time = 0;

volatile bool ir_triggered = false;
volatile uint32_t last_ir_time = 0;

// Flag to signal the send task
volatile bool ready_to_send_response = false;

// Buffer for receiving messages from server
char recv_buf[100] = {};

CRGB leds[NUM_LEDS];

LiquidCrystal_I2C lcd(0x27, 20, 4); 

xSemaphoreHandle led_mux = NULL, game_start_mux = NULL;

TaskHandle_t sensor_logic_handle = NULL, read_sensor_handle = NULL, send_packet_handle = NULL;

void set_led_color(int red, int green, int blue);
void game_start(void);

void read_sensor_task(void *param) {
    ESP_LOGI("READ", "Sensor Reading Task Started.");
    int adc_value;
    while (1) {
		xSemaphoreTake(game_start_mux, portMAX_DELAY);
		ESP_LOGI("MIC", "free heap size: %u bytes\n", esp_get_free_heap_size());
		ESP_LOGI("MIC", "memory used: %u bytes\n", uxTaskGetStackHighWaterMark(NULL));
        // --- Read ADC ---
        adc_value = analogRead(SOUND_SENSOR_PIN);
        ESP_LOGI("MIC", "ADC Value (Pin %d): %d", SOUND_SENSOR_PIN, adc_value); // Uncomment for detailed logging
		if (adc_value >= SOUND_THRESHOLD) {
			last_vibration_time = millis();
			vibration_count++;
		}
        // Delay before next reading cycle
		xSemaphoreGive(game_start_mux);
		vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void IRAM_ATTR ir_isr(void) {
    last_ir_time = millis();
    ir_triggered = true;
}

// --- ESP-NOW Callbacks ---
void send_callback(const uint8_t *mac_addr, esp_now_send_status_t status) {
	vTaskSuspend(sensor_logic_handle);
	vTaskSuspend(read_sensor_handle);
	cli();
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(SEND_TAG, "--> Response Sent OK to Server");
		lcd.clear();
		vTaskDelay(pdMS_TO_TICKS(1000));
		xSemaphoreTake(led_mux, portMAX_DELAY);
		set_led_color(0, 0, 0);
		xSemaphoreGive(led_mux);
		vTaskSuspend(send_packet_handle);
    } else {
        ESP_LOGW(SEND_TAG, "--> Response Send FAILED to Server");
		ready_to_send_response = true;
    }
	vTaskDelay(pdMS_TO_TICKS(100));
}

void recv_callback(const uint8_t *mac_addr, const uint8_t *data, int len) {
    // Safely copy received data
    size_t len_to_copy = (len < sizeof(recv_buf)) ? len : sizeof(recv_buf) - 1;
    std::memcpy(recv_buf, reinterpret_cast<const char*>(data), len_to_copy);
    recv_buf[len_to_copy] = '\0'; // Null-terminate

    ESP_LOGI(RECV_TAG, "<-- Recv %d bytes from Server: '%s'", len, recv_buf);
	xSemaphoreTake(led_mux, portMAX_DELAY);
	set_led_color(200, 0, 200);
	xSemaphoreGive(led_mux);
	game_start();
	vTaskDelay(pdMS_TO_TICKS(100)); // Keep LED on shortly
}

// --- FreeRTOS Tasks ---

// Task to check sensor flags and timestamps
void sensor_logic_task(void *param) {

    ESP_LOGI(SENSOR_TAG, "Sensor Logic Task Started. Waiting for sensor events...");

    while (1) {
		xSemaphoreTake(game_start_mux, portMAX_DELAY);
		cli();
		ESP_LOGI(SENSOR_TAG, "free heap size: %u bytes\n", esp_get_free_heap_size());
		ESP_LOGI(SENSOR_TAG, "memory used: %u bytes\n", uxTaskGetStackHighWaterMark(NULL));
		// Atomically read volatile variables (often safe for bool/ulong on ESP32,
        // but critical sections are safer if strict atomicity is required)

		ESP_LOGI(SENSOR_TAG, "vibration_count: %d", vibration_count);
		ESP_LOGI(SENSOR_TAG, "ir_triggered: %d", ir_triggered);
		ESP_LOGI(SENSOR_TAG, "last_vibration_time: %d ms", last_vibration_time);
		ESP_LOGI(SENSOR_TAG, "last_ir_time: %d ms", last_ir_time);
		
        if (vibration_count && ir_triggered) {
            // Both sensors have been triggered *at some point*

            // Check if timestamps are within the allowed threshold (not stale)
            int64_t time_diff;
			time_diff = last_vibration_time - last_ir_time;

			lcd.clear();
            if (time_diff > STALE_DATA_THRESHOLD_MS) {
                // Stale Data: Events happened too far apart
                ESP_LOGW(SENSOR_TAG, "STALE EVENT: Vibration and IR detected but too far apart (Diff: %lld ms > %u ms)",
				time_diff, STALE_DATA_THRESHOLD_MS);
				xSemaphoreTake(led_mux, portMAX_DELAY);
				set_led_color(255, 0, 0);
				xSemaphoreGive(led_mux);
				lcd.print("Timeout!");
				lcd.setCursor(1, 0);
				lcd.printf("Time: %lld ms", time_diff);
				vTaskDelay(pdMS_TO_TICKS(1000));
				lcd.clear();
				lcd.print("Waiting...");
            } else if (vibration_count > 1 || time_diff <= 0) {
				ESP_LOGI(SENSOR_TAG, "VALID EVENT (fault): Vibration and IR detected within %u ms (Diff: %lld ms)",
					STALE_DATA_THRESHOLD_MS, time_diff);
				xSemaphoreTake(led_mux, portMAX_DELAY);
				set_led_color(255, 0, 0);
				xSemaphoreGive(led_mux);
				lcd.print("Fault!");
				lcd.setCursor(1, 0);
				lcd.printf("Time: %lld ms", time_diff);
				vTaskDelay(pdMS_TO_TICKS(1000));
				lcd.clear();
				lcd.print("Waiting...");
			} else {
				// **** Valid Detection: Both sensors triggered recently ****
                ESP_LOGI(SENSOR_TAG, "VALID EVENT: Vibration and IR detected within %u ms (Diff: %lld ms)",
					STALE_DATA_THRESHOLD_MS, time_diff);
				lcd.print("Goal!");
				ready_to_send_response = true;
				xSemaphoreTake(led_mux, portMAX_DELAY);
				set_led_color(0, 255, 0);
				xSemaphoreGive(led_mux);
				lcd.setCursor(1, 0);
				lcd.printf("Time: %lld ms", time_diff);
            }

			vibration_count = 0;
			ir_triggered = false;
        }
        // If only one or none triggered, do nothing and wait
		// Delay to prevent busy-waiting and allow other tasks
		sei();
		xSemaphoreGive(game_start_mux);
        vTaskDelay(pdMS_TO_TICKS(20)); 
        // vTaskDelay(pdMS_TO_TICKS(2000)); 
    }
}

// Task to send ESP-NOW packet when signaled
void send_packet_task(void *param) {
    ESP_LOGI(PACKET_TAG, "Send Packet Task Started.");
    while (1) {
		ESP_LOGI(PACKET_TAG, "free heap size: %u bytes\n", esp_get_free_heap_size());
		ESP_LOGI(PACKET_TAG, "memory used: %u bytes\n", uxTaskGetStackHighWaterMark(NULL));
		ESP_LOGI(PACKET_TAG, "detected signal to send response to server.");
        if (ready_to_send_response) {

            // Reset the flag immediately before sending
            ready_to_send_response = false;

            // Send the predefined response data
            esp_err_t result = esp_now_send(server_info.peer_addr, RESPONSE_DATA, RESPONSE_DATA_LEN);

            if (result == ESP_OK) {
                ESP_LOGI(PACKET_TAG, "ESP-NOW send initiated successfully.");
            } else {
                ESP_LOGE(PACKET_TAG, "ESP-NOW send initiation failed: %s", esp_err_to_name(result));
                // Optional: Handle failure (e.g., set ready_to_send_response = true to retry?)
				ready_to_send_response = true;
            }
             // Add a delay after sending to prevent immediate re-triggering if needed
            // vTaskDelay(pdMS_TO_TICKS(100));
            vTaskDelay(pdMS_TO_TICKS(1000));

        } else {
            // Nothing to send, wait politely
            vTaskDelay(pdMS_TO_TICKS(50));
            // vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
}

// void dummy_send_task(void *param) {
// 	ESP_LOGI("DUMMY", "send data task started.");
// 	while (1) {
// 		esp_err_t result = esp_now_send(server_info.peer_addr, RESPONSE_DATA, RESPONSE_DATA_LEN);
// 		ESP_LOGI(PACKET_TAG, "free heap size: %u bytes\n", esp_get_free_heap_size());
// 		ESP_LOGI(PACKET_TAG, "memory used: %u bytes\n", uxTaskGetStackHighWaterMark(NULL));
// 		ESP_LOGI(PACKET_TAG, ">>> Detected signal to send response to server.");
// 		xSemaphoreTake(mux, portMAX_DELAY);
// 		if (result == ESP_OK) {
// 			ESP_LOGI(PACKET_TAG, "ESP-NOW send initiated successfully.");
// 			set_led_color(0, 255, 0);
// 		} else {
// 			log_e("ESP-NOW send initiation failed: %s", esp_err_to_name(result));
// 			// Optional: Handle failure (e.g., set ready_to_send_response = true to retry?)
// 			ready_to_send_response = true;
// 			set_led_color(255, 0, 0);
// 		}
// 		set_led_color(0, 0, 255);
// 		vTaskDelay(pdMS_TO_TICKS(1000));
// 		set_led_color(0, 255, 0);
// 		vTaskDelay(pdMS_TO_TICKS(1000));
// 		set_led_color(255, 0, 0);
// 		vTaskDelay(pdMS_TO_TICKS(1000));
// 		xSemaphoreGive(mux);
// 		// Add a delay after sending to prevent immediate re-triggering if needed
// 		// vTaskDelay(pdMS_TO_TICKS(100));
// 		vTaskDelay(pdMS_TO_TICKS(2000));
// 	}
// }

void set_led_color(int red, int green, int blue) {
	for (int i = 0; i < NUM_LEDS; i++) {
		leds[i] = CRGB(red, green, blue);
	}
	FastLED.show();
}

void init_system() {
	ESP_LOGI(SETUP_TAG, "LED Setup Started...");
	FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
	FastLED.setBrightness(64);
	FastLED.clear();
	set_led_color(200, 0, 200); // Set all LEDs to yellow for idling
	FastLED.show();
	ESP_LOGI(SETUP_TAG, "LED Setup Finished\n");
}

void game_start() {
	for (int i = 3; i > 0; i--) {
		lcd.clear();
		lcd.print("Game starts in ");
		lcd.setCursor(0, 1);
		lcd.print(i);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	
	lcd.clear();
	lcd.print("Game Started!");
	vTaskDelay(pdMS_TO_TICKS(1000));
	lcd.clear();
	sei();
	vTaskResume(sensor_logic_handle);
	vTaskResume(read_sensor_handle);
	vTaskResume(send_packet_handle);

	lcd.print("Waiting...");
}

void setup() {
    Serial.begin(115200);
	led_mux = xSemaphoreCreateMutex();
	if (!led_mux) {
		ESP_LOGE(SETUP_TAG, "Failed to initialize mutex\n");
		while(1) {
			vTaskDelay(1);
		};
	}
	game_start_mux = xSemaphoreCreateMutex();
	if (!game_start_mux) {
		ESP_LOGE(SETUP_TAG, "Failed to initialize mutex\n");
		while(1) {
			vTaskDelay(1);
		};
	}
	init_system();

	ESP_LOGI(SETUP_TAG, "Client Setup Started...");
    WiFi.mode(wifi_mode_t::WIFI_MODE_STA);
    ESP_LOGI(SETUP_TAG, "Client WiFi MAC: %s", WiFi.macAddress().c_str()); // Log client MAC

	// Initialize ESP-NOW
	ESP_ERROR_CHECK(esp_now_init());
	ESP_ERROR_CHECK(esp_now_register_send_cb(send_callback));
	ESP_ERROR_CHECK(esp_now_register_recv_cb(recv_callback));
	
	// Add server peer
	esp_err_t addStatus = esp_now_add_peer(&server_info);
	if (addStatus == ESP_OK) {
		ESP_LOGI(SETUP_TAG, "Server peer added successfully");
	} else {
		ESP_LOGE(SETUP_TAG, "Failed to add server peer, error: %s", esp_err_to_name(addStatus));
	}

	Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C bus (SDA, SCL)
	// Initialize LCD
	lcd.init();
	lcd.backlight();
	lcd.clear();
	ESP_LOGI(SETUP_TAG, "LCD Initialized.");
	
    // --- Configure Sensor Pins and Interrupts ---
    // IMPORTANT: Adjust INPUT_PULLUP/INPUT_PULLDOWN/INPUT based on your sensor wiring.
    // If your sensor outputs HIGH when active and has no pull-down, use INPUT_PULLDOWN.
    // If it outputs LOW when active and has no pull-up, use INPUT_PULLUP.
    pinMode(SOUND_SENSOR_PIN, INPUT); // Example: Assume sensor pulls LOW when active
    pinMode(IF_SENSOR_PIN, INPUT); // Example: Assume sensor pulls LOW when active
	
    // Attach Interrupts
    attachInterrupt(digitalPinToInterrupt(IF_SENSOR_PIN), ir_isr, RISING);
	
    ESP_LOGI(SETUP_TAG, "Interrupts attached to pins %d and %d", SOUND_SENSOR_PIN, IF_SENSOR_PIN);
    // --- End Sensor Config ---
	
    // Optional: Power Management / Sleep Config (if needed for client)
    #ifdef ENABLE_SLEEP
    // ESP_ERROR_CHECK(esp_sleep_enable_wifi_wakeup()); // If server wakes client
    // esp_pm_config_esp32_t pm_cfg;
    // ... configure light sleep ...
    #endif
	// xTaskCreatePinnedToCore(dummy_send_task, "dummy_task", 2048, NULL, 1, NULL, 0);

	ESP_LOGI(SETUP_TAG, "Creating tasks...");
    if (xTaskCreatePinnedToCore(sensor_logic_task, "sensor_logic", 2048, NULL, 1, &sensor_logic_handle, 0)) { // Core 0
        ESP_LOGE(SETUP_TAG, "Failed to create Sensor Logic Task");
        while(1) {
			vTaskDelay(1);
		};
    }
	if (xTaskCreatePinnedToCore(read_sensor_task, "read_sensor", 2048, NULL, 1, &read_sensor_handle, 0)) { // Core 0
		ESP_LOGE(SETUP_TAG, "Failed to create Read Sensor Task");
		while(1) {
			vTaskDelay(1);
		};
	}
	if (xTaskCreatePinnedToCore(send_packet_task, "send_packet", 2048, NULL, 1, &send_packet_handle, 1)) { // Core 1
        ESP_LOGE(SETUP_TAG, "Failed to create Send Packet Task");
        while(1) {
			vTaskDelay(1);
		};
    }

	game_start();
	ESP_LOGI(SETUP_TAG,"Client Setup Finished.");
}

void loop() {
    // Delete the default Arduino loopTask as we are using FreeRTOS tasks
    vTaskDelete(NULL);
}