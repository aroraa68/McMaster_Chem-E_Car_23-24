// TODO:
// Serial Interface - Needs to be initialized, look into plotting data with the data streamer add-on in Excel.
// Rida -In Progress- Temperature Sensor - Needs to be initialized, periodically poll sensor, store temperature in a variable, print out to serial monitor, look into Kalman filters for reducing sensor noise.
// IMU - Needs to be initialized, periodically poll sensor, store Z-axis angle in variable, take initial value as 0 deg print out to serial monitor, look into kalman filters for reducing sensor noise.
// Stir Bar Motor - Needs to be initialized, just constantly turn at 80% speed for now, we'll tune speed later.
// Servo Motor - Needs to be initialized, turn 180 deg clockwise once at start to dump reactants, wait 1 sec to finish dumping, turn back 180 deg counter-clockwise to return to upright position, set speed to 100% for now, tune later.
// MicroSD Card - Store all variable data to MicroSD Card on microcontroller, allow for each run to be saved under a different filename (potentially in CSV format) so we can keep track of everything.

#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS A1 // pin for the DS18B20 data line
OneWire oneWire(ONE_WIRE_BUS); // create a OneWire instance to communicate with the senso
DallasTemperature sensors(&oneWire); // pass oneWire reference to Dallas Temperature sensor

// Define the PWM pins for the stir bar motor
const int stirPin1 = 5;
const int stirPin2 = 6;

// variable to store temperature
float temperatureC;
// KALMAN FILTER variables
float x_k; // State estimate: current best estimate of true temp
float p_k; // Error covariance: reps the uncertainity in the state estimate
// Process noise and measurement noise
float q; // Process noise covariance
float r; // Measurement noise covariance

void setup() {
  Serial.begin(9600);  // start serial communication (adjust baud rate as needed)
  sensors.begin();   // initialize the DS18B20 sensor

  // Initialize Kalman filter parameters
  x_k = 0.0; // Initial state estimate
  p_k = 1.0; // Initial error covariance
  q = 0.01;  // Process noise covariance 
  r = 0.1;   // Measurement noise covariance 
  
  // Initialize the motor pins as outputs
  pinMode(stirPin1, OUTPUT);
  pinMode(stirPin2, OUTPUT);

  // Set the initial speed to 80%
  analogWrite(stirPin1, 204); // 80% of 255
  digitalWrite(stirPin2, LOW); //for fast decay
}

void loop() {
  sensors.requestTemperatures();   // request temperature from all devices on the bus
  temperatureC = sensors.getTempCByIndex(0); // get temperature in Celsius


  // Kalman filter prediction
  float x_k_minus = x_k;      // Predicted next state estimate
  float p_k_minus = p_k + q;  // Predicted error covariance for the next state

  // Kalman filter update

  /* Kalman gain: calculated based on the predicted error covariance 
  and the measurement noise covariance,,, used to update the 
  state estimate (x_k) and error covariance (p_k) */ 
  float k = p_k_minus / (p_k_minus + r); // kalman gain
  //comparison with actual temp reading 
  x_k = x_k_minus + k * (temperatureC - x_k_minus); // Updated state estimate
  p_k = (1 - k) * p_k_minus; // Updated error covariance

  
  // Print the filtered temperature
  Serial.print("Filtered Temperature: ");
  Serial.println(x_k);
  // Print raw temperature to Serial monitor
  Serial.print("Raw Temperature: ");
  Serial.println(temperatureC);

  delay(1000); // Add delay before the next reading (change as needed)
}
