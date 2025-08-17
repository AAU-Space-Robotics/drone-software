// Code which has been flashed on the Arduino Nano every

#include <Adafruit_NeoPixel.h>

#define LED_PIN 6      // Pin where the WS2812 data line is connected
#define LED_COUNT 2    // Number of LEDs

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; } // Wait for Serial to initialize (useful for some boards)
  Serial.println("Setup: Initializing LEDs to Yellow");
  
  strip.begin();
  strip.setBrightness(50); // Set brightness to 50% to reduce power demand
  setColor(255, 255, 0); // Yellow standby by default
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read entire string until newline
    input.trim(); // Remove any whitespace
    Serial.print("Received command: ");
    Serial.println(input);

    int cmd = input.toInt();
    switch (cmd) {
      case -1:
        Serial.println("Setting LEDs to Red");
        setColor(255, 0, 0);   // Red
        break;
      case 0:
        Serial.println("Setting LEDs to Yellow (Standby)");
        setColor(255, 255, 0); // Yellow (Standby)
        break;
      case 1:
        Serial.println("Setting LEDs to Blue (Manual)");
        setColor(0, 0, 255);   // Blue (Manual)
        break;
      case 2:
        Serial.println("Setting LEDs to Green (Autonomous)");
        setColor(0, 255, 0);   // Green (Autonomous)
        break;
      default:
        Serial.println("Unknown command, ignoring");
        break;
    }
  }
}

// Helper function to set both LEDs to a color
void setColor(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
    Serial.print("Setting LED ");
    Serial.print(i);
    Serial.print(" to RGB(");
    Serial.print(r);
    Serial.print(",");
    Serial.print(g);
    Serial.print(",");
    Serial.print(b);
    Serial.println(")");
  }
  strip.show();
}