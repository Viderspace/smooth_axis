/*
 * 01_Basic/SimplePotentiometer
 * * The simplest way to get a stable, non-flickering reading.
 * Uses 'Settle Time' to smooth the signal without adding lag.
 */

#include <SmoothAxis.h>

// --- Configuration ---
const int SENSOR_PIN = A0;

// The highest value your sensor can read.
// Standard Arduino (Uno/Nano) = 1023
// ESP32 / RP2040 = 4095
const int SENSOR_MAX = 1023;

// How long (in seconds) for the reading to stabilize.
// Lower (0.10) = Very snappy, fast response.
// Higher (0.30) = Very smooth, slow "cinematic" feel.
const float SETTLE_TIME = 0.15;

// Initialize the filter
SmoothAxis axis(SENSOR_MAX, SETTLE_TIME);

void setup() {
    Serial.begin(9600);
    
    // Wait for Serial Monitor (useful for Leonardo/Pro Micro)
    while (!Serial) { delay(10); }
    
    Serial.println("SmoothAxis initialized.");
}

void loop() {
    // 1. Read the sensor
    int rawValue = analogRead(SENSOR_PIN);
    
    // 2. Update the filter
    axis.update(rawValue);
    
    // 3. Print ONLY if the value has effectively changed
    if (axis.hasChanged()) {
        Serial.print("Smoothed Value: ");
        Serial.println(axis.read());
    }
}