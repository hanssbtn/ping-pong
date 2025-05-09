#include <esp_event.h>
#include <freertos/task.h>
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <LiquidCrystal_I2C.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <esp_now.h>
#include <esp_mac.h>
#include <WiFi.h>
#include <esp_random.h>
#include <esp_pm.h>
#include <cstring>
#include <esp_log.h>
#include <esp_task_wdt.h>


// --- Configuration ---
const char *WAKEUP_MESSAGE = "W", // Simple message to send to client
		   *EXPECTED_CLIENT_RESPONSE = "1", // What client sends on success
		   *SERVER_TAG = "SERVER",
		   *SETUP_TAG = "SETUP",
		   *SEND_TAG = "SEND",
		   *RECV_TAG = "RECV",
		   *LOOP_TAG = "LOOP";
const char *AP_SSID = "ESP32_Goal_Server";
// const char *AP_PASS = "ESP32_PASSWORD";
		   
char recv_buf[100] = {}; // Renamed buffer for clarity

// *** FIX: Use UNIQUE MAC addresses for clients ***
// Replace these with the actual MAC addresses of your client ESP32s
esp_now_peer_info_t client_infos[] = {
    {
        // MAC from your client code example
        .peer_addr = {0xF8, 0xB3, 0xB7, 0x45, 0x4E, 0xAC},
        .channel = 1,
        .ifidx = WIFI_IF_STA,
        .encrypt = false
    },
    // Add second client if applicable
    {
        .peer_addr = {0xF8, 0xB3, 0xB7, 0x3F, 0x0B, 0xAC}, // Example Client 2 MAC
        .channel = 1,
        .ifidx = WIFI_IF_STA,
        .encrypt = false
    }
};

#define NUM_CLIENTS (sizeof(client_infos) / sizeof(client_infos[0]))

uint64_t goals_confirmed = 0; // Renamed for clarity
uint32_t current_target_client = 0; // Renamed for clarity, start with client 0
bool trigger_sent_ok = false; // Renamed: True if the last WAKEUP_MESSAGE send was successful

AsyncWebServer server(80);
AsyncEventSource events("/events");

// Callback when ESP-NOW send completes (for the WAKEUP_MESSAGE)
void send_callback(const uint8_t *mac_addr, esp_now_send_status_t status) {
    trigger_sent_ok = (status == ESP_NOW_SEND_SUCCESS);
    if (trigger_sent_ok) {
        ESP_LOGI(SEND_TAG, "--> Sent '%s' OK to %02X:%02X:%02X:%02X:%02X:%02X", WAKEUP_MESSAGE,
              mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    } else {
        ESP_LOGW(SEND_TAG, "--> Sent '%s' FAILED to %02X:%02X:%02X:%02X:%02X:%02X", WAKEUP_MESSAGE,
              mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        // Loop will retry sending to the same current_target_client because trigger_sent_ok is false
    }
}

// Callback when ESP-NOW data is received
void recv_callback(const uint8_t *mac_addr, const uint8_t *data, int len) {
    // *** FIX: Buffer overflow protection ***
    size_t len_to_copy = (len < sizeof(recv_buf)) ? len : sizeof(recv_buf) - 1;
    std::memcpy(recv_buf, reinterpret_cast<const char*>(data), len_to_copy);
    recv_buf[len_to_copy] = '\0'; // Null-terminate

    ESP_LOGI(RECV_TAG, "<-- Recv %d bytes from %02X:%02X:%02X:%02X:%02X:%02X: '%s'",
          len, mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], recv_buf);

    // Only process if the trigger message was successfully sent previously
    // AND the response is from the client we are currently targeting
    if (trigger_sent_ok &&
        memcmp(mac_addr, client_infos[current_target_client].peer_addr, ESP_NOW_ETH_ALEN) == 0)
    {
        // Check if the received data is the expected confirmation
        if (strcmp(recv_buf, EXPECTED_CLIENT_RESPONSE) == 0) {
            goals_confirmed++;
            ESP_LOGI(RECV_TAG, "!!! Goal confirmed by client %d! Total goals: %llu", current_target_client, goals_confirmed);

            // *** Choose the next target client RANDOMLY ***
            current_target_client = esp_random() % NUM_CLIENTS;
            // *** OR Choose next target client SEQUENTIALLY (Round Robin) ***
            // current_target_client = (current_target_client + 1) % num_clients;

            ESP_LOGI(RECV_TAG, ">>> Selecting next target client: %d", current_target_client);

            // Allow loop() to send the trigger to the *new* target client
            trigger_sent_ok = false;

        } else {
            ESP_LOGW(RECV_TAG, "Received unexpected data ('%s') from target client %d.", recv_buf, current_target_client);
        }
    } else if (trigger_sent_ok) {
         // Log if data received from unexpected MAC while waiting
		ESP_LOGW(RECV_TAG, "Received data from non-target MAC %02X:%02X:%02X:%02X:%02X:%02X while waiting for client %d",
              mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], current_target_client);
    }
    // Ignore data received if we haven't successfully sent the trigger message yet (!trigger_sent_ok)
}

void web_server_task(void *pvParameters) {
    ESP_LOGI(SERVER_TAG, "Web Server Task started");

    // Configure Access Point
    ESP_LOGI(SERVER_TAG, "Setting up AP: %s", AP_SSID);
    #ifdef AP_PASS
        WiFi.softAP(AP_SSID, AP_PASS);
        ESP_LOGI(SERVER_TAG, "AP Password: %s", AP_PASS);
    #else
        WiFi.softAP(AP_SSID);
        ESP_LOGI(SERVER_TAG, "AP Password: <None>");
    #endif

	events.onConnect([](AsyncEventSourceClient *client){
        if(client->lastId()){
            ESP_LOGI(SERVER_TAG, "SSE Client reconnected! Last message ID: %u", client->lastId());
        }
        // send converted goals_confirmed as initial data
        client->send(String(goals_confirmed).c_str(), "goal_update", millis());
        ESP_LOGI(SERVER_TAG, "SSE: Initial goal_update sent to new client.");
    });
    server.addHandler(&events);

    IPAddress ap_ip = WiFi.softAPIP();
    ESP_LOGI(SERVER_TAG, "AP IP address: %s", ap_ip.toString().c_str());
	
    // Define Web Server Routes (Handlers)
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        ESP_LOGI(SERVER_TAG, "Received request for /");
        // Create a simple HTML response showing the goal count
        String html = "<!DOCTYPE html><html><head><title>ESP32 Goal Server</title>"
              // Add meta tag for auto-refresh every 5 seconds
              "<meta http-equiv='refresh' content='5'>"
              "<style>body { font-family: sans-serif; text-align: center; background-color: #f0f0f0; }"
              "h1 { color: #333; } .goals { font-size: 3em; color: #0066cc; margin: 20px; }"
              ".mac { font-size: 0.8em; color: #666; margin-top: 30px; }</style>"
              "</head><body>"
              "<h1>ESP32 Goal Server Status</h1>"
              // Display the current goal count (accessing the global variable)
              "<div class='goals'>Goals Confirmed: " + String((unsigned long)goals_confirmed) + "</div>"
              "<div class='mac'>Server MAC: " + WiFi.macAddress() + "</div>"
              "<div class='mac'>AP IP: " + WiFi.softAPIP().toString() + "</div>"
              "</body></html>"
			  "<script>"
			  "var eventSource = new EventSource('/events');" // Connect to the SSE endpoint
			  "eventSource.addEventListener('open', function(e) {"
			  "  console.log('SSE connection opened');"
			  "}, false);"
		  	  "eventSource.addEventListener('error', function(e) {"
			  "  if (e.readyState == EventSource.CLOSED) {"
			  "    console.log('SSE connection closed');"
			  "  } else {"
			  "    console.error('SSE error:', e);"
			  "  }"
			  "}, false);"
			  "eventSource.addEventListener('goal_update', function(e) {"
			  "  console.log('Received goal_update:', e.data);"
			  "  // Update the content of the span with ID 'goalCount'"
			  "  document.getElementById('goalCount').innerText = e.data;"
			  "}, false);"
			  "</script>";
        request->send(200, "text/html", html);
    });

    server.onNotFound([](AsyncWebServerRequest *request){
        ESP_LOGW(SERVER_TAG, "Not found: %s", request->url().c_str());
        request->send(404, "text/plain", "Not found");
    });
	
    // Start the server
    server.begin();
    ESP_LOGI(SERVER_TAG, "HTTP server started");

    // Keep task alive
    while (true) {
		vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 milliseconds
    }
}

void setup() {
    Serial.begin(115200);
    ESP_LOGI(SETUP_TAG, "Server Setup Started...");
    WiFi.mode(wifi_mode_t::WIFI_MODE_STA);
    ESP_LOGI(SETUP_TAG, "Server WiFi MAC: %s", WiFi.macAddress().c_str());

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(send_callback));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(recv_callback));
	WiFi.setAutoReconnect(true);

    // *** FIX: Add all defined peers ***	
    ESP_LOGI(SETUP_TAG, "Adding %d client peers...", NUM_CLIENTS);
    for (int i = 0; i < NUM_CLIENTS; ++i) {
        esp_err_t add_status = esp_now_add_peer(&client_infos[i]);
        if (add_status == ESP_OK) {
            ESP_LOGI(SETUP_TAG, "Peer %d (%02X:%02X:%02X:%02X:%02X:%02X) added successfully", i,
                  client_infos[i].peer_addr[0], client_infos[i].peer_addr[1], client_infos[i].peer_addr[2],
                  client_infos[i].peer_addr[3], client_infos[i].peer_addr[4], client_infos[i].peer_addr[5]);
        } else {
            ESP_LOGE(SETUP_TAG,"Failed to add peer %d, error: %s", i, esp_err_to_name(add_status));
            // Consider halting setup if peers are essential
			while (1) {
				vTaskDelay(1);
			};
        }
		vTaskDelay(pdMS_TO_TICKS(5000));
    }

    // Initialize first target (can be random or fixed)
	current_target_client = esp_random() % NUM_CLIENTS;
    ESP_LOGI(SETUP_TAG,"Initial target client index: %d", current_target_client);
	ESP_LOGI(SETUP_TAG, "Creating Web Server Task...");
    trigger_sent_ok = false; // Ensure we start by sending the trigger
	xTaskCreatePinnedToCore(
        web_server_task,          // Function that implements the task.
        "Web_server_task",        // Text name for the task.
        8192,                   // Stack size in words, not bytes (adjust as needed, web servers can use more RAM)
        NULL,                   // Parameter passed into the task.
        1,                      // Priority at which the task is created.
        NULL,                    // Used to pass out the created task's handle.
		1
    );
	ESP_LOGI(SETUP_TAG, "web server task created.");
    ESP_LOGI(SETUP_TAG,"Server Setup Finished.");
}

// loop() function now handles sending the trigger message
void loop() {
	// vTaskDelete(NULL);
    if (!trigger_sent_ok) {
        // Need to send the trigger/wakeup message
        ESP_LOGI(LOOP_TAG, ">>> Attempting to send trigger ('%s') to client %d", WAKEUP_MESSAGE, current_target_client);
        esp_err_t result = esp_now_send(client_infos[current_target_client].peer_addr,
                                         reinterpret_cast<const uint8_t*>(WAKEUP_MESSAGE),
                                         strlen(WAKEUP_MESSAGE));

        if (result != ESP_OK) {
			ESP_LOGE(LOOP_TAG, "Send initiation error for trigger to client %d: %s", current_target_client, esp_err_to_name(result));
			// Keep trigger_sent_ok = false, will retry next loop iteration
			vTaskDelay(pdMS_TO_TICKS(5000)); // Delay before retry on immediate failure
        } else {
			// Send initiated, waiting for send_callback confirmation.
			// trigger_sent_ok will be updated in the callback.
			// Add a small delay to allow callback time to execute if send was very fast.
			vTaskDelay(pdMS_TO_TICKS(200)); // Short delay after initiating send
        }
    } else {
        // Trigger message was successfully sent (confirmed by send_callback).
        // Now waiting for the expected response via recv_callback.
        ESP_LOGD(LOOP_TAG, "Waiting for confirmation ('%s') from client %d...", EXPECTED_CLIENT_RESPONSE, current_target_client);
        vTaskDelay(pdMS_TO_TICKS(5000)); // Yield/wait to avoid busy loop
    }
}