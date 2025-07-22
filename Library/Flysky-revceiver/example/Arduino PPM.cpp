#include <Arduino.h>

#define PPM_PIN 2  // PPM signal input pin
#define NUM_CHANNELS 6  // Number of channels

volatile uint16_t ppmValues[NUM_CHANNELS];
volatile uint8_t currentChannel = 0;
volatile uint32_t lastTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterrupt, RISING);
}

void loop() {
  // Print the PPM values
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    Serial.print("Channel ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(ppmValues[i]);
    Serial.print(" us\t");
  }
  Serial.println();
  delay(1000);  // Print every second
}

void ppmInterrupt() {
  uint32_t currentTime = micros();
  uint32_t pulseLength = currentTime - lastTime;
  lastTime = currentTime;

  if (pulseLength >= 3000) {
    // Sync pulse detected, reset channel counter
    currentChannel = 0;
  } else if (currentChannel < NUM_CHANNELS) {
    // Store pulse length in the current channel
    ppmValues[currentChannel] = pulseLength;
    currentChannel++;
  }
}