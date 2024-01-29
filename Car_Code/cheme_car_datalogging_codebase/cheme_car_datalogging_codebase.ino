// TODO:
// Serial Interface - Needs to be initialized, look into plotting data with the data streamer add-on in Excel.
// Temeperature Sensor - Needs to be initialized, periodically poll sensor, store temperature in variable, print out to serial monitor, look into kalman filters for reducing sensor noise.
// IMU - Needs to be initialized, periodically poll sensor, store Z-axis angle in variable, take initial value as 0 deg print out to serial monitor, look into kalman filters for reducing sensor noise.
// Stir Bar Motor - Needs to be initialized, just constantly turn at 80% speed for now, we'll tune speed later.
// Servo Motor - Needs to be initialized, turn 180 deg clockwise once at start to dump reactants, wait 1 sec to finish dumping, turn back 180 deg counter-clockwise to return to upright position, set speed to 100% for now, tune later.
// Ashviya: MicroSD Card - Store all variable data to MicroSD Card on microcontroller, allow for each run to be saved under a different filename (potentially in CSV format) so we can keep track of everything.

#include <SD.h>
#include <SPI.h>

// Define pin numbers
const int chipSelect = 4;

// Define files
File dataFile;
String fileName;
bool isFileNew = false; // Checks for new file

// Define variables
float temperatureC; // Raw temperature in Celcius
float x_k; // Filtered temperature
float zAngle; // z-axis angle
float zAngleFiltered; // Filtered z-axis angle
int runCount = 0; 

void setup() {
  // Begin serial communication
  Serial.begin(9600);

  // Initialize SD Card
  Serial.print("SD card is initializing...");

  if (!SD.begin(chipSelect)){
    Serial.println("SD card initialization failed.");
    while(1); // Halts and waits if failure
  }

  File root = SD.open("/"); // Open SD root directory
  int checkRun = 0;

  while (true){
    File nextFile = root.openNextFile();
    
    if (!nextFile){ // Increment with each existing file
      checkRun++;
    }

    else {
      nextFile.close();
    }
  }
  
  root.close();

  runCount = checkRun + 1;

  Serial.println("Success! SD card initialized.");
}

void loop() {
  fileName = "Run_" +  String(runCount++) + ".csv";
  dataFile = SD.open(fileName, FILE_WRITE);

  if (dataFile){
    // Writes header if it's a new file
    if (!isFileNew) {
      dataFile.println("Time, Temperature, Filtered Temperature, z-angle, Filtered z-angle");
      isFileNew = true; 
    }

    // Obtain current time in seconds
    unsigned long currTime = millis() / 1000;

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

  else{
    Serial.println("Error! Cannot open file for writing.");
  }

  delay(1000);
}