#include <Servo.h>  

Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29

// defines variables
long duration;  // variable for the duration of sound wave travel
int distance;   // variable for the distance measurement
#define echoPin A4  // attach pin A4 Arduino to pin Echo of HC-SR04
#define trigPin A5  //attach pin A5 Arduino to pin Trig of HC-SR04

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;




void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);  // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);   // Sets the echoPin as an INPUT

    // defines variables




  pinMode(left_front, OUTPUT);
  pinMode(left_rear, OUTPUT);
  pinMode(right_rear, OUTPUT);
  pinMode(right_front, OUTPUT);


}

void forward()
{
  left_font_motor.writeMicroseconds(1500 + 200);
  left_rear_motor.writeMicroseconds(1500 + 200);
  right_rear_motor.writeMicroseconds(1500 - 200);
  right_font_motor.writeMicroseconds(1500 - 200);
}



int measuredistanceUltrasonic(void) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);  
  digitalWrite(trigPin, HIGH); // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);         
  duration = pulseIn(echoPin, HIGH);  // Reads the echoPin, returns the sound wave travel time in microseconds
  distance = duration * 0.034 / 2; // Calculating the distance and dividing by 2 (distance to and from)
  return distance;
}


void enable_motors()
{
  left_font_motor.attach(left_front);  // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}


void driveDistanceSonar(int target){
  int Kp = 20;
  int error = 2; //in cm
  int currentDist = measuredistanceUltrasonic();

  while(error > 1){
    currentDist = measuredistanceUltrasonic();


    
    error = -  target  + currentDist;

    Serial.println(error);
      left_font_motor.writeMicroseconds(1500 + Kp*error);
      left_rear_motor.writeMicroseconds(1500 + Kp*error);
      right_rear_motor.writeMicroseconds(1500 - Kp*error);
      right_font_motor.writeMicroseconds(1500 - Kp*error);
  }
      //shut down motors once complete
      
      left_font_motor.writeMicroseconds(1500);
      left_rear_motor.writeMicroseconds(1500);
      right_rear_motor.writeMicroseconds(1500);
      right_font_motor.writeMicroseconds(1500);
      
}


void loop() {
  // put your main code here, to run repeatedly:
  enable_motors();
  
  driveDistanceSonar(20);

 
  //int retard = measuredistanceUltrasonic();
  
 // Serial.println(retard);
}
