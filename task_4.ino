int switch_1=13;
int switch_2=12;
int motor_1=7;
int motor_2=8;
int potentiometer_1=A1;
int potentiometer_2=A5;

void setup()
{
  pinMode(switch_1,0);
  pinMode(switch_2,0);
  pinMode(motor_1,1);
  pinMode(motor_2,1);
  Serial.begin(9600);
}

void loop()
{
  int high1=digitalRead(switch_1);
  int high2=digitalRead(switch_2);
  int readingpt_1=map(analogRead(potentiometer_1),0,1023,0,255);
  int readingpt_2=map(analogRead(potentiometer_2),0,1023,0,255);
  
                      
  if (high1==1 && high2==0)
  {
    analogWrite(motor_1,readingpt_1);
    analogWrite(motor_2,0);
  }
  else if(high2==1 && high1==0)
  {
    analogWrite(motor_2,readingpt_2);
    analogWrite(motor_1,0);
  }
  else if(high1==1 && high2==1)
  {
    analogWrite(motor_1,readingpt_1);
    analogWrite(motor_2,readingpt_2);
  }
  else if(high1==0 && high2==0)
  {
    analogWrite(motor_1,0);
    analogWrite(motor_2,0);
  }
  if(Serial.available())
  {
    char reading=Serial.read();
    if(reading=='R'){
      analogWrite(motor_1,readingpt_1);
    }
    else if(reading=='L'){
      analogWrite(motor_2,readingpt_2);
    }
    else if(reading=='S'){
      analogWrite(motor_1,0);
      analogWrite(motor_2,0);
    }
  }                                       
}





//// ( chat gpt )
Pin definitions
//const int leftMotorPin = 9; // Pin connected to left motor input on motor driver
//const int rightMotorPin = 10; // Pin connected to right motor input on motor driver
//const int leftPotPin = A0; // Analog pin for left motor speed control
//const int rightPotPin = A1; // Analog pin for right motor speed control
//const int leftSwitchPin = 2; // Digital pin for left motor enable/disable switch
//const int rightSwitchPin = 3; // Digital pin for right motor enable/disable switch
//
//// Variables
//int leftSpeed = 0; // Speed for left motor (0-255)
//int rightSpeed = 0; // Speed for right motor (0-255)
//
//void setup() {
//  // Setup motor pins as outputs
//  pinMode(leftMotorPin, OUTPUT);
//  pinMode(rightMotorPin, OUTPUT);
//
//  // Setup switch pins as inputs
//  pinMode(leftSwitchPin, INPUT_PULLUP);
//  pinMode(rightSwitchPin, INPUT_PULLUP);
//
//  // Initialize serial communication
//  Serial.begin(9600);
//}
//
//void loop() {
//  // Read potentiometer values
//  leftSpeed = map(analogRead(leftPotPin), 0, 1023, 0, 255);
//  rightSpeed = map(analogRead(rightPotPin), 0, 1023, 0, 255);
//
//  // Read switch states
//  bool leftSwitchState = digitalRead(leftSwitchPin);
//  bool rightSwitchState = digitalRead(rightSwitchPin);
//
//  // Control left motor
//  if (leftSwitchState == LOW) { // If switch is closed
//    analogWrite(leftMotorPin, leftSpeed);
//  } else {
//    analogWrite(leftMotorPin, 0); // Stop motor if switch is open
//  }
//
//  // Control right motor
//  if (rightSwitchState == LOW) { // If switch is closed
//    analogWrite(rightMotorPin, rightSpeed);
//  } else {
//    analogWrite(rightMotorPin, 0); // Stop motor if switch is open
//  }
//
//  // Serial communication for remote control
//  if (Serial.available() > 0) {
//    char command = Serial.read();
//    if (command == 'L') {
//      analogWrite(leftMotorPin, leftSpeed);
//    } else if (command == 'R') {
//      analogWrite(rightMotorPin, rightSpeed);
//    } else if (command == 'S') {
//      analogWrite(leftMotorPin, 0);
//      analogWrite(rightMotorPin, 0);
//    }
//  }
//}
