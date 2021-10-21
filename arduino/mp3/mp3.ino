
#include <Shell.h>
#include <Adafruit_MotorShield.h>
#include "pinout.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *leftMotor = AFMS.getMotor(MOTOR_LEFT);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(MOTOR_RIGHT);

int forwardSpeed = 30;
int turnSpeedForward = 30;
int turnSpeedReverse = -10;

uint16_t sensor_threshold = 500;

void setup() {
  // Prepare serial communication
  Serial.begin(9600);
  
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  // Wait after reset or power on...
  delay(1000);

  // Initialize command line interface (CLI)
  // We pass the function pointers to the read and write functions that we implement below
  // We can also pass a char pointer to display a custom start message
  shell_init(shell_reader, shell_writer, 0);
  shell_register(command_getsensors, PSTR("getsensors"));
  shell_register(command_setthreshold, PSTR("setthreshold"));
  shell_register(command_setmotors, PSTR("setmotors"));

  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);
}

int command_getsensors(int argc, char** argv)
{
  SensorData data = readSensors();
  
  shell_println("Analog Sensor Readings:");
  char buff[80];
  sprintf(buff, "Left:\t%i\tRight:\t%i", data.rawLeft, data.rawRight);
  shell_println(buff);

  shell_println("Processed Sensor Readings:");
  sprintf(buff, "Left:\t%i\tRight:\t%i", data.left, data.right);
  shell_println(buff);
  return SHELL_RET_SUCCESS;
}

int command_setthreshold(int argc, char** argv)
{
  if (argc == 2) {
    sensor_threshold = atoi(argv[1]);

    char buff[80];
    sprintf(buff, "New sensor threshold:\t%i", sensor_threshold);
    shell_println(buff);
    return SHELL_RET_SUCCESS;
  } else {
    return SHELL_RET_FAILURE;
  }
}


int command_setmotors(int argc, char** argv)
{
  int leftCommand;
  int rightCommand;
  if (argc == 3) {
    leftCommand = atoi(argv[1]);
    rightCommand = atoi(argv[2]);
  } else if (argc == 1) {
    leftCommand = 0;
    rightCommand = 0;
  } else {
    return SHELL_RET_FAILURE;
  }
  shell_println("Setting motors to:");
  char buff[80];
  sprintf(buff, "Left:\t%i\tRight:\t%i", leftCommand, rightCommand);
  shell_println(buff);

  setMotors(leftCommand, rightCommand);
  return SHELL_RET_SUCCESS;
}

int shell_reader(char * data)
{
  // Wrapper for Serial.read() method
  if (Serial.available()) {
    *data = Serial.read();
    return 1;
  }
  return 0;
}

void shell_writer(char data)
{
  // Wrapper for Serial.write() method
  Serial.write(data);
}

SensorData readSensors() {
  uint16_t leftRaw = analogRead(SENSOR_LEFT);
  uint16_t rightRaw = analogRead(SENSOR_RIGHT);

  bool left = leftRaw > sensor_threshold;
  bool right = rightRaw > sensor_threshold;
  
  return {left, right, leftRaw, rightRaw};
}

void setMotors(int left, int right) {
  if (left > 0) {
    leftMotor->run(FORWARD);
  } else if (left < 0) {
    leftMotor->run(BACKWARD);
  } else {
    leftMotor->run(RELEASE);
  }

  if (right > 0) {
    rightMotor->run(FORWARD);
  } else if (right < 0) {
    rightMotor->run(BACKWARD);
  } else {
    rightMotor->run(RELEASE);
  }
 

  leftMotor->setSpeed(abs(left));
  rightMotor->setSpeed(abs(right));
}

void loop() {
  // put your main code here, to run repeatedly:

  //shell_task();  // Comment this out when running the final code

  SensorData sensors = readSensors();

  int leftCommand = 0;
  int rightCommand = 0;
  if (sensors.left && sensors.right) {
    // go left
    leftCommand = turnSpeedReverse;
    rightCommand = turnSpeedForward;
  } else if (!sensors.left && sensors.right) {
    // go straight
    leftCommand = forwardSpeed;
    rightCommand = forwardSpeed;
  } else if (sensors.left && !sensors.right) {
    // turn left
    leftCommand = turnSpeedReverse;
    rightCommand = turnSpeedForward;
  } else if (!sensors.left && !sensors.right) {
    // turn right
    leftCommand = turnSpeedForward;
    rightCommand = turnSpeedReverse;
  }

  setMotors(leftCommand, rightCommand);

  // Gather data
  char buff[80];
  sprintf(buff, "%i,%i,%i,%i", sensors.rawLeft, sensors.rawRight, leftCommand, rightCommand);
  Serial.println(buff);
}
