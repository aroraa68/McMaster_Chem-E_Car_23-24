// TODO:
// Serial Interface - Needs to be initialized, look into plotting data with the data streamer add-on in Excel.
// Temeperature Sensor - Needs to be initialized, periodically poll sensor, store temperature in variable, print out to serial monitor, look into kalman filters for reducing sensor noise.
// IMU - Needs to be initialized, periodically poll sensor, store Z-axis angle in variable, take initial value as 0 deg print out to serial monitor, look into kalman filters for reducing sensor noise.
// Stir Bar Motor - Needs to be initialized, just constantly turn at 80% speed for now, we'll tune speed later.
// Servo Motor - Needs to be initialized, turn 180 deg clockwise once at start to dump reactants, wait 1 sec to finish dumping, turn back 180 deg counter-clockwise to return to upright position, set speed to 100% for now, tune later.
// MicroSD Card - Store all variable data to MicroSD Card on microcontroller, allow for each run to be saved under a different filename (potentially in CSV format) so we can keep track of everything.

// Define the PWM pins for the stir bar motor
const int motorPin1 = 5;
const int motorPin2 = 6;

void setup() {
  // Initialize the motor pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);

  // Set the initial speed to 80%
  analogWrite(motorPin1, 204); // 80% of 255
  analogWrite(motorPin2, 204);
}

void loop() {

}
