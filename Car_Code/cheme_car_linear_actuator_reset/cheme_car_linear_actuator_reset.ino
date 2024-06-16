#define LED 8 // Status LED

// Define the PWM pins for the linear actuator
#define linAcc1 6
#define linAcc2 5

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
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  
  retract(); // Extend syringe to reset
  
  // Indicate status to be finished
  digitalWrite(LED, LOW);
}

void loop() // Loop (main loop)
{
	// Do nothing
}