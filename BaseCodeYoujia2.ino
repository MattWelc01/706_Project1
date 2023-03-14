#include <SoftwareSerial.h>
// defines variables
long duration;  // variable for the duration of sound wave travel
int distance;   // variable for the distance measurement
volatile int32_t Counter = 1;

//infrared sensor variables
int irsensorR = A8;   //sensor is attached on pinA8 (right side sensor)
int irsensorL = A9;   //sensor is attached on pinA8 (left side sensor)
byte serialRead = 0;  //for control serial communication
int signalADC = 0;    // the read out signal in 0-1023 corresponding to 0-5v

#define INTERNAL_LED 13
#define BLUETOOTH_RX 10   // Serial Data input pin
#define BLUETOOTH_TX 11   // Serial Data output pin
#define STARTUP_DELAY 10  // Seconds
#define LOOP_DELAY 2      // miliseconds
#define SAMPLE_DELAY 10   // miliseconds

#define OUTPUTMONITOR 0  // USB Serial Port
#define OUTPUTPLOTTER 0

#define OUTPUTBLUETOOTHMONITOR 1  // Bluetooth Serial Port

#define echoPin A4  // attach pin A4 Arduino to pin Echo of HC-SR04
#define trigPin A5  //attach pin A5 Arduino to pin Trig of HC-SR04

SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

void setup() {

  // defines variables
  long duration;  // variable for the duration of sound wave travel
  int distance;   // variable for the distance measurement

  pinMode(trigPin, OUTPUT);  // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);   // Sets the echoPin as an INPUT

  pinMode(INTERNAL_LED, OUTPUT);

  Serial.begin(115200);

  BluetoothSerial.begin(115200);

  Serial.print("Ready, waiting for ");
  Serial.print(STARTUP_DELAY, DEC);
  Serial.println(" seconds");

  delaySeconds(STARTUP_DELAY);
}

////////////////////////////////////////////*************NON FUNCTIONAL CODE********/////////////////////////////////////////////////////////////////

void delaySeconds(int TimedDelaySeconds) { //10 second start up delay for set up 
  for (int i = 0; i < TimedDelaySeconds; i++) {
    delay(1000);
  }
}

void serialOutputMonitor(int32_t Value1, int32_t Value2, int32_t Value3) {
  String Delimiter = ", ";

  Serial.print(Value1, DEC);
  Serial.print(Delimiter);
  Serial.print(Value2, DEC);
  Serial.print(Delimiter);
  Serial.println(Value3, DEC);
}

void serialOutput(int32_t Value1, int32_t Value2, int32_t Value3) { //only for plotting - to turn on change OUTPUTMONITOR = 1
  if (OUTPUTMONITOR) {
    serialOutputMonitor(Value1, Value2, Value3);
  }

  if (OUTPUTPLOTTER) {
    serialOutputPlotter(Value1, Value2, Value3);
  }

  if (OUTPUTBLUETOOTHMONITOR) {
    bluetoothSerialOutputMonitor(Value1, Value2, Value3);
    ;
  }
}

void serialOutputPlotter(int32_t Value1, int32_t Value2, int32_t Value3) {
  String Delimiter = ", ";

  Serial.print(Value1, DEC);
  Serial.print(Delimiter);
  Serial.print(Value2, DEC);
  Serial.print(Delimiter);
  Serial.println(Value3, DEC);
}

void bluetoothSerialOutputMonitor(int32_t Value1, int32_t Value2, int32_t Value3) {
  String Delimiter = ", ";

  BluetoothSerial.print(Value1, DEC);
  BluetoothSerial.print(Delimiter);
  BluetoothSerial.print(Value2, DEC);
  BluetoothSerial.print(Delimiter);
  BluetoothSerial.println(Value3, DEC);
}

/////////////////////////////////////////////*************NON FUNCTIONAL CODE********/////////////////////////////////////////////////////////////////

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

int measuredistanceRightIR() {
  signalADC = analogRead(irsensorR);  // the read out is a signal from 0-1023 corresponding to 0-5v
  int distance1 = 17948 * pow(signalADC, -1.22);  // calculate the distance using the datasheet graph
  //  int distancec = 46161 * pow(signalADC, -1.302);  // calculate the distance using the calibrated graph

  return distance1;  //not sure which distance we should use
} 

void loop() {

  // serialOutput(measuredistanceIR("R"), 1, 999);
  Serial.print(measuredistanceRightIR());
  Serial.println("");
  delay(100);
  
}
