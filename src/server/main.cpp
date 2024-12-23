#include <esp_wifi.h>
#include <esp32/rom/gpio.h>
#include <driver/gpio.h>

extern "C" void app_main(void) {
    gpio_pad_select_gpio(GPIO_NUM_2);
    return;
}