// TODO:
// Serial Interface - Needs to be initialized, look into plotting data with the data streamer add-on in Excel.
// IMU - Needs to be initialized, periodically poll sensor, store Z-axis angle in variable, take initial value as 0 deg print out to serial monitor, look into kalman filters for reducing sensor noise.
// Servo Motor - Needs to be initialized, turn 180 deg clockwise once at start to dump reactants, wait 1 sec to finish dumping, turn back 180 deg counter-clockwise to return to upright position, set speed to 100% for now, tune later.

#include <OneWire.h>
#include <DallasTemperature.h>
#include <SD.h>
#include <SPI.h>

#define ONE_WIRE_BUS A1 // pin for the DS18B20 data line

OneWire oneWire(ONE_WIRE_BUS);       // create a OneWire instance to communicate with the senso
DallasTemperature sensors(&oneWire); // pass oneWire reference to Dallas Temperature sensor

// Define pin numbers
const int chipSelect = 4;

// Initialize run count for SD card file
int runCount;
int checkRun;

// Define the PWM pins for the stir bar motor
const int stirPin1 = 5;
const int stirPin2 = 6;

// Define files
File root;
File nextFile;
File dataFile;
String fileName;
bool isFileNew = false; // Checks for new file

// Define accelerometer variables
float zAngle;         // z-axis angle
float zAngleFiltered; // Filtered z-axis angle

// variable to store temperature
float temperatureC;

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
unsigned long prevTime;

void setup()
{
  Serial.begin(9600); // start serial communication (adjust baud rate as needed)
  sensors.begin();    // initialize the DS18B20 sensor

  // Initialize Kalman filter parameters
  x_k = 0.0; // Initial state estimate
  p_k = 1.0; // Initial error covariance
  q = 0.01;  // Process noise covariance
  r = 0.1;   // Measurement noise covariance

  // Initialize the motor pins as outputs
  pinMode(stirPin1, OUTPUT);
  pinMode(stirPin2, OUTPUT);

  // Set the initial speed to 80%
  analogWrite(stirPin1, 204);  // 80% of 255
  digitalWrite(stirPin2, LOW); // for fast decay

  // Initialize SD Card
  Serial.println("SD card is initializing...");

  if (!SD.begin(chipSelect))
  {
    Serial.println("SD card initialization failed.");
    while (1)
      ; // Halts and waits if failure
  }

  root = SD.open("/"); // Open SD root directory
  checkRun = 0;

  while (true)
  {
    nextFile = root.openNextFile();

    if (nextFile)
    { // Increment with each existing file
      checkRun++;
    }
    else
    {
      nextFile.close();
      break;
    }
  }

  root.close();

  runCount = checkRun + 1;

  Serial.println("Success! SD card initialized.");
}

void loop()
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

  fileName = "Run_" + String(runCount++) + ".csv";
  dataFile = SD.open(fileName, FILE_WRITE);

  if (dataFile)
  {
    // Writes header if it's a new file
    if (!isFileNew)
    {
      dataFile.println("Time, Temperature, Filtered Temperature, z-angle, Filtered z-angle");
      isFileNew = true;
    }

    // Obtain current time in seconds
    currTime = millis();

    // Write variable data to the file in CSV format
    dataFile.print(currTime);
    dataFile.print(", ");
    dataFile.print(temperatureC);
    dataFile.print(", ");
    dataFile.print(x_k);
    dataFile.print(", ");
    dataFile.print(zAngle);
    dataFile.print(", ");
    dataFile.println(zAngleFiltered);

    dataFile.close();
    isFileNew = false; // Reset for next file
  }
  else
  {
    Serial.println("Error! Cannot open file for writing.");
  }

  // Print the filtered temperature
  Serial.print("Current Time: ");
  Serial.println(currTime);

  // Print the filtered temperature
  Serial.print("Filtered Temperature: ");
  Serial.println(x_k);

  // Print raw temperature to Serial monitor
  Serial.print("Raw Temperature: ");
  Serial.println(temperatureC);
}
