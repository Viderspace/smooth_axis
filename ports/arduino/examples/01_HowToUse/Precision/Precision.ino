/*
 * 02_Advanced/PrecisionLiveMode
 * * Demonstrates how to manually drive the filter with 'Delta Time'.
 * * WHY USE THIS?
 * By default, the library assumes a steady loop speed. 
 * However, if your program runs at inconsistent speeds (e.g. heavy LED effects, 
 * delays, or WiFi tasks), the filter needs to know exactly how much time 
 * has passed to maintain consistent physics.
 * * Providing 'deltaTime' ensures the smoothing feels exactly the same 
 * whether your loop runs at 1000Hz or lags down to 50Hz.
 */

#include <SmoothAxis.h>

// --- Configuration ---
const int   SENSOR_PIN  = A0;
const int   SENSOR_MAX  = 1023;
const float SETTLE_TIME = 0.3;

// Initialize with the LIVE flag.
// This tells the library: "I will calculate the time myself."
SmoothAxis axis(SENSOR_MAX, SETTLE_TIME, SmoothAxis::LIVE);

// Track the exact moment of the previous frame
unsigned long lastMicros = 0;

void setup() {
    Serial.begin(9600);
    while (!Serial) { delay(10); }
    
    lastMicros = micros();
    Serial.println("Precision Mode initialized.");
}

void loop() {
    // 1. Calculate 'Delta Time'
    // The exact amount of time (in seconds) that passed since the last update.
    unsigned long now = micros();
    
    // Convert microseconds to seconds (1,000,000 us = 1.0 sec)
    float deltaTime = (now - lastMicros) / 1000000.0;
    
    // Reset timer for the next frame
    lastMicros = now;
    
    
    
    // 2. Update with the exact time step
    // Even if this loop lagged for 100ms, the filter will use 'deltaTime'
    // to calculate the correct position, preventing "slow motion" drag.
    int rawValue = analogRead(SENSOR_PIN);
    axis.update(rawValue, deltaTime);
    
    // 3. Print result
    if (axis.hasChanged()) {
        Serial.print("Precise Value: ");
        Serial.println(axis.read());
    }
}