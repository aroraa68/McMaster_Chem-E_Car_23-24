// TODO:
// IMU - Needs to be initialized, periodically poll sensor, store Z-axis angle in variable, take initial value as 0 deg, if veering off by more than 5 deg, look into kalman filters for reducing sensor noise.
// Drive Motors - Need need to add PID with IMU.
// Servo Motor - Needs to be initialized, turn 180 deg clockwise once at start to dump reactants, wait 1 sec to finish dumping, turn back 180 deg counter-clockwise to return to upright position, set speed to 100% for now, tune later.

// Included libraries
#include <OneWire.h>
#include <DallasTemperature.h>

// Define drive motor pins
#define left_pwm1 9
#define left_pwm2 10
#define right_pwm1 11
#define right_pwm2 12

// Define the PWM pins for the stir bar motor
#define stirPin1 5
#define stirPin2 6

#define ONE_WIRE_BUS A1 // pin for the DS18B20 data line

OneWire oneWire(ONE_WIRE_BUS);       // create a OneWire instance to communicate with the senso
DallasTemperature sensors(&oneWire); // pass oneWire reference to Dallas Temperature sensor

// Temperature threshold
const float tempDiff = 3;

// Time limit in milliseconds
const unsigned long tLim = 12000;

// Define accelerometer variables
float zAngle;         // z-axis angle
float zAngleFiltered; // Filtered z-axis angle

// variables to store temperature
float temperatureC;
float initTemp;

// KALMAN FILTER variables
float k;         // kalman gain
float x_k;       // Filtered temperature
float p_k;       // Initial error covariance
float x_k_minus; // Predicted next state estimate
float p_k_minus; // Predicted error covariance for the next state

// Process noise and measurement noise
float q; // Process noise covariance
float r; // Measurement noise covariance

// Keeping track of time
unsigned long currTime;
unsigned long startTime;

void drive_forward(int speed) // Drive function
{
  // left wheel
  digitalWrite(left_pwm1, HIGH);
  analogWrite(left_pwm2, speed);

  // right wheel
  digitalWrite(right_pwm1, HIGH);
  analogWrite(right_pwm2, speed);
}

void stop_driving() // Stop function
{
  // left wheel
  digitalWrite(left_pwm1, HIGH);
  analogWrite(left_pwm2, 0);

  // right wheel
  digitalWrite(right_pwm1, HIGH);
  analogWrite(right_pwm2, 0);
}

void setup() // Setup (executes once)
{
  // Get time at start
  startTime = millis();

  // Initialize Kalman filter parameters
  x_k = 0.0; // Initial state estimate
  p_k = 1.0; // Initial error covariance
  q = 0.01;  // Process noise covariance
  r = 0.1;   // Measurement noise covariance

  // Initialize the stir motor pins as outputs
  pinMode(stirPin1, OUTPUT);
  pinMode(stirPin2, OUTPUT);

  // Set the stir initial speed to 80%
  analogWrite(stirPin1, 204);  // 80% of 255
  digitalWrite(stirPin2, LOW); // for fast decay

  sensors.begin();                       // initialize the DS18B20 sensor
  sensors.requestTemperatures();         // request temperature from all devices on the bus
  initTemp = sensors.getTempCByIndex(0); // get temperature in Celsius

  // Servo acctuation goes here

  // Setting to drive motors output mode
  pinMode(left_pwm1, OUTPUT);
  pinMode(left_pwm2, OUTPUT);
  pinMode(right_pwm1, OUTPUT);
  pinMode(right_pwm2, OUTPUT);

  // Start drive motors completely stopped
  analogWrite(left_pwm1, 0);
  digitalWrite(left_pwm2, LOW);
  analogWrite(right_pwm1, 0);
  digitalWrite(right_pwm2, LOW);
}

void loop() // Loop (main loop)
{
  sensors.requestTemperatures();             // request temperature from all devices on the bus
  temperatureC = sensors.getTempCByIndex(0); // get temperature in Celsius

  // Kalman filter prediction
  x_k_minus = x_k;     // Predicted next state estimate
  p_k_minus = p_k + q; // Predicted error covariance for the next state

  // Kalman filter update

  /* Kalman gain: calculated based on the predicted error covariance
  and the measurement noise covariance,,, used to update the
  state estimate (x_k) and error covariance (p_k) */
  k = p_k_minus / (p_k_minus + r); // kalman gain

  // comparison with actual temp reading
  x_k = x_k_minus + k * (temperatureC - x_k_minus); // Updated state estimate
  p_k = (1 - k) * p_k_minus;                        // Updated error covariance

  drive_forward(204); // 80% speed is 204

  if (((x_k - initTemp) > tempDiff) || ((currTime - startTime) > tLim))
  {
    stop_driving();
  }
}
