

/*This code moves a stepper motor with relative number of steps to its current position.
  Input can be positive or negative integers. The sign of the integer determines direction of movement. 
  The number of steps is provided via serial communication. This can be done via Arduino
  terminal or using the python script provided in zivid_turntable.
  Hardware required: arduino uno and motorshield rev3*/

// Include the AccelStepper and serial library:
#include <AccelStepper.h>
#include <SoftwareSerial.h>

// Define number of steps per revolution:
const int stepsPerRevolution = 200;

// Give the motor control pins names:
#define pwmA 3
#define pwmB 11
#define brakeA 9
#define brakeB 8
#define dirA 12
#define dirB 13

// Define the AccelStepper interface type:
#define MotorInterfaceType 2

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(MotorInterfaceType, dirA, dirB);

void setup() {
  //Baud rate set to match baud rate in python
  Serial.begin(9600);
  // Set the PWM and brake pins so that the direction pins can be used to control the motor:
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(brakeA, OUTPUT);
  pinMode(brakeB, OUTPUT);
  digitalWrite(pwmA, HIGH);
  digitalWrite(pwmB, HIGH);
  digitalWrite(brakeA, LOW);
  digitalWrite(brakeB, LOW);
  // Set the maximum steps per second:
  stepper.setMaxSpeed(600);
  // Set the maximum acceleration in steps per second^2:
  stepper.setAcceleration(60);
}


void loop() {
  //sets steps to zero, will not move from current position before user input
  int steps = 0; 
  //Wait for user input of integer amount of steps to move
  while(true){
    if(Serial.available() > 0){
      String input = Serial.readString();
      steps = input.toInt();
      break;
    }
  }
  //Sets relative movement in regards to current position
  stepper.move(steps);
  // Run to position with set speed and acceleration:
  stepper.runToPosition();
  delay(100);
  while(stepper.isRunning()){
    delay(10);  
  }
  Serial.write(1);
}
