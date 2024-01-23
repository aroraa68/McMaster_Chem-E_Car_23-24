// TODO:
// Temeperature Sensor - Needs to be initialized, periodically poll sensor, store temperature in variable, if reaches threshold of 3 deg increase from what it first read, stop the drive motors, look into kalman filters for reducing sensor noise.
// IMU - Needs to be initialized, periodically poll sensor, store Z-axis angle in variable, take initial value as 0 deg, if veering off by more than 5 deg, adjust using a PID loop to get back on track, look into kalman filters for reducing sensor noise.
// Drive Motors - Need to be initialized, take input from the IMU as necessary and move, otherwise start off moving straight, and keep moving straight, move at about 80% max speed for now, we will change as needed.
// Stir Bar Motor - Needs to be initialized, just constantly turn at 80% speed for now, we'll tune speed later.
// Servo Motor - Needs to be initialized, turn 180 deg clockwise once at start to dump reactants, wait 1 sec to finish dumping, turn back 180 deg counter-clockwise to return to upright position, set speed to 100% for now, tune later.

#define left_pwm1 9
#define left_pwm2 10
#define right_pwm1 11
#define right_pwm2 12

void setup() {

  //Setting to output mode
  pinMode(left_pwm1, OUTPUT);
  pinMode(left_pwm2, OUTPUT);
  pinMode(right_pwm1, OUTPUT);
  pinMode(right_pwm2, OUTPUT);

  //start motors completely stopped
  analogWrite(left_pwm1, 0);
  digitalWrite(left_pwm2, LOW);
  analogWrite(right_pwm1, 0);
  digitalWrite(right_pwm2, LOW);

}

void loop() {

    drive_forward(204); //80% speed is 204
    //stop_driving();  //uncomment to stop

}

void drive_forward(int speed){
  //left wheel
  analogWrite(left_pwm1, speed);
  digitalWrite(left_pwm2, LOW);

  //right wheel
  analogWrite(right_pwm1, speed);
  digitalWrite(right_pwm2, LOW);

}

void stop_driving(){
  //left wheel
  analogWrite(left_pwm1, 0);
  digitalWrite(left_pwm2, LOW);

  //right wheel
  analogWrite(right_pwm1, 0);
  digitalWrite(right_pwm2, LOW);
}

