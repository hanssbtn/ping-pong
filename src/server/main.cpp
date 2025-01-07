#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

const char* ssid = "your_wifi_ssid";       // Replace with Wi-Fi SSID
const char* password = "your_wifi_password"; // Replace with Wi-Fi password
const char* wled_ip = "192.168.1.100";    // Replace with the WLED device's IP address

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");
}

// Function to send commands to WLED
void triggerWLED(bool power, int effect = -1, int brightness = -1, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0) {
  HTTPClient http;
  String url = "http://" + String(wled_ip) + "/json/state";

  // Create a JSON object
  StaticJsonDocument<256> doc;

  // Add common properties
  doc["on"] = power; // Power state (true = on, false = off)

  // Optional: Add effect, if provided
  if (effect >= 0) {
    JsonArray seg = doc.createNestedArray("seg");
    JsonObject segment = seg.createNestedObject();
    segment["id"] = 0;   // Segment ID
    segment["fx"] = effect; // Effect ID
  }

  // Optional: Add brightness, if provided
  if (brightness >= 0) {
    doc["bri"] = brightness; // Brightness value (0-255)
  }

  // Optional: Add color, if provided
  if (r > 0 || g > 0 || b > 0) {
    JsonArray seg = doc["seg"]; // Access existing segment
    if (seg.size() == 0) {
      JsonObject segment = seg.createNestedObject();
      segment["id"] = 0; // Ensure a segment exists
    }
    JsonArray color = seg[0].createNestedArray("col");
    JsonArray primaryColor = color.createNestedArray();
    primaryColor.add(r); // Red
    primaryColor.add(g); // Green
    primaryColor.add(b); // Blue
  }

  // Serialize JSON to a string
  String json_payload;
  serializeJson(doc, json_payload);

  // Send the HTTP POST request
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int httpResponseCode = http.POST(json_payload);

  // Handle the response
  if (httpResponseCode > 0) {
    Serial.printf("HTTP Response code: %d\n", httpResponseCode);
  } else {
    Serial.printf("Error code: %s\n", http.errorToString(httpResponseCode).c_str());
  }

  http.end();
}

void loop() {
  // Example: Turn on the LED strip with a red color
  triggerWLED(true, -1, 128, 255, 0, 0);
  delay(5000);

  // Example: Set a rainbow effect
  triggerWLED(true, 3); // Effect 3 = Rainbow
  delay(5000);

  // Example: Turn off the LED strip
  triggerWLED(false);
  delay(5000);
}




// #include <Arduino.h>
// #include <esp_now.h>
// #include <esp_mac.h>
// #include <WiFi.h>
// #include <Wire.h>
// #include <esp_random.h>
// #include <esp_pm.h>
// #include <cstring>

// #define LED_PIN 2

// char buf[100] = {};

// esp_now_peer_info_t client_infos[2] = {
//     {
//         .peer_addr = {0xF8, 0xB3, 0xB7, 0x45, 0x6A, 0xDC},
//         .channel = 1,
//         .ifidx = WIFI_IF_STA,
//         .encrypt = false
//     },
//     {
//         .peer_addr = {0xF8, 0xB3, 0xB7, 0x3F, 0x0B, 0xAC},
//         .channel = 1,
//         .ifidx = WIFI_IF_STA,
//         .encrypt = false
//     }
// };
// uint32_t chosen_goal = esp_random() % (sizeof client_infos/sizeof client_infos[0]);
// bool sent = false;

// void send_callback(const uint8_t *mac_addr, esp_now_send_status_t status) {
//     sent = status == esp_now_send_status_t::ESP_NOW_SEND_SUCCESS;
//     #ifndef ENABLE
//     esp_deep_sleep_start();
//     #endif
// }

// void recv_callback(const uint8_t *mac_addr, const uint8_t *data, int len) {
//     chosen_goal = esp_random() % (sizeof client_infos/sizeof client_infos[0]);
//     std::memcpy(buf, reinterpret_cast<const char*>(data), len);
//     Serial.printf("got data %s\n", buf);
//     #ifdef ENABLE
//     sent = false;
//     #endif
// }

// void setup() {
//     Serial.begin(115200);
//     WiFi.mode(WIFI_STA);
//     // ESP_ERROR_CHECK(esp_now_init());
//     // esp_now_add_peer(&client_infos[0]);
//     // // esp_now_add_peer(&client_infos[1]);
//     // #ifdef ENABLE
//     //     esp_pm_config_esp32_t pm_cfg;
//     //     esp_pm_get_configuration(&pm_cfg);
//     //     pm_cfg.light_sleep_enable = true;
//     //     esp_pm_configure(&pm_cfg);
//     //     esp_sleep_enable_wifi_wakeup();
//     // #endif
//     // esp_now_register_send_cb(esp_now_send_cb_t(send_callback));
//     // esp_now_register_recv_cb(esp_now_recv_cb_t(recv_callback));

//     pinMode(LED_PIN, OUTPUT);
//     digitalWrite(LED_PIN, HIGH);
// }

// void loop() {
//     // if (!sent) {
//     //     Serial.print("Trying to send data...\n");
//     //     esp_now_send(client_infos[chosen_goal].peer_addr, reinterpret_cast<const uint8_t*>("Hello from ESP32"), strlen("Hello from ESP32"));
//     //     delay(2000);
//     //     return;
//     // }
//     // Serial.print("Data sent!!\n");
//     // #ifndef ENABLE
//     // esp_deep_sleep_start();
//     // #endif
// }
