#include <Adafruit_NeoPixel.h>

// Define the PWM pins for the linear actuator
#define linAcc1 6
#define linAcc2 5

#define NUM_PIXELS 1 // Status LED

Adafruit_NeoPixel pixel(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800); // Status LED

void retract() // Retract actuator to reset hydrogen reaction 
{
  analogWrite(linAcc2, 0);     // 100% power
  digitalWrite(linAcc1, HIGH); // For slow decay
  delay(16660);                // Wait to finish extending
  analogWrite(linAcc2, 255);   // Stop extending
}

void setup() // Setup (executes once)
{
  // Initialize the stir motor pins as outputs
  pinMode(linAcc1, OUTPUT);
  pinMode(linAcc2, OUTPUT);
  
  // Indicate status to be initialized
  pixel.begin();
  pixel.setBrightness(255);
  pixel.show();
  pixel.setPixelColor(0, 255, 0, 0);
  pixel.show();
   
  retract(); // Extend syringe to reset
  
  // Indicate status to be finished
  pixel.setPixelColor(0, 0, 0, 255);
  pixel.show();
}

void loop() // Loop (main loop)
{
	// Do nothing
}