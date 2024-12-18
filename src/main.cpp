#include <Arduino.h>
#include <Wire.h>

#define SENSOR_PIN 0 // Change to actual pin number

uint32_t count = 0;

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
    // put your setup code here, to run once:
    pinMode(SENSOR_PIN, INPUT);
    
}


void loop() {
    const uint32_t dur = read_sensor();
    Serial.print("duration: ");
    Serial.println(dur);
    if (dur != -1) {
        count++;
    }

    // put your main code here, to run repeatedly:
}

