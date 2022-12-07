#include <Servo.h>

// Only Comms if On
bool on = false;

// PID Coefficients
double kP = 1;
double kI = 0;
double kD = 0;

// Tracks Time for PID
unsigned long currentTime = 0;
unsigned long previousTime = 0;
double elapsedTime = 0;

// Tracks Error for PID
double error = 0;
double lastError = 0;

// Tracks IO + setPoint
double input = 0;
double output = 0;
int servoOutput = 0;
double setPoint = 3;

// Tracks Error
double cumError = 0;
double rateError = 0;

// For Formatting Results
String formatOutput = "";
int outputSize = 200;

// Serial Port for Comms
int serialPort = 9600;

// Port for IR Sensor
int sensorPort = A0;

// Delays
int lowDelay = 2;
int highDelay = 10;

// Variables for Ultrasonic
long dur = 0;
long cm = 0;

// Ports for Servo
int servoPort = 9;
Servo myServo;

void setup() {
  // Starts Serial
  Serial.begin(serialPort);

  // Reserves Memory for Output Formatting
  formatOutput.reserve(outputSize);

  // Attaches Servo
  myServo.attach(servoPort);

  // Connects to IR Sensor
  pinMode(sensorPort, INPUT);
}

void loop() {
  // Only Gets Data and Writes Results if Ready
  if(on) {
    // Calculates cm from sensors (obtained from sensorCalib.xslx)
    input = 1851.6 * pow(analogRead(A0), -0.634) - 32;

    // Computes PID For Output
    output = computePID(input);

    // Formats Output to a Range of 0 to 180 Degrees
    servoOutput = min(max(0, int(output)), 180);

    Serial.println("I " + String(input) + " O " + String(servoOutput) + " T " +  String(elapsedTime));

    // Sets Servo Degrees
    myServo.write(servoOutput);
  }
}

void serialEvent() {
  char key = char(Serial.read());
  switch (key) {
    case 'f': // Turns off functionality
      on = false;
      Serial.println('f');
      break;
    case 'o': // Turns on functionality
      on = true;
      Serial.println('o');
      break;
    case 'u': // Updates PID values
      kP = Serial.parseFloat();
      kI = Serial.parseFloat();
      kD = Serial.parseFloat();
      setPoint = Serial.parseFloat();
      Serial.println('u');
      break;
  }
}

// Computes PID
double computePID(double inp) {
  currentTime = millis(); // Time
  elapsedTime = double(currentTime - previousTime); // Elapsed Time

  error = setPoint - inp; // Error
  cumError += error * elapsedTime; // Integral
  rateError = (error - lastError) / elapsedTime; // Derivative

  double out = kP * error + kI * cumError + kD * rateError; // PID Equation

  // For Next Iteration
  lastError = error;
  previousTime = currentTime;

  return out;
}
