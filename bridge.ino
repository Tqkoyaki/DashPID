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

// Ports for Ultrasonic
int ultraIn = 8; // ECHO
int ultraOut = 9; // TRIG

// Delays
int lowDelay = 2;
int highDelay = 10;

// Variables for Ultrasonic
long dur = 0;
long cm = 0;

// Ports for Servo
int servoPort = 6;
Servo myServo;

void setup() {
  // Starts Serial
  Serial.begin(serialPort);

  // Reserves Memory for Output Formatting
  formatOutput.reserve(outputSize);

  // Attaches Servo
  myServo.attach(servoPort);

  // Connects to Ultrasonic
  pinMode(ultraIn, INPUT);
  pinMode(ultraOut, OUTPUT);
}

void loop() {
  // Only Gets Data and Writes Results if Ready
  if(on) {
    // Sends Signals
    digitalWrite(ultraOut, LOW);
    delayMicroseconds(lowDelay);
    digitalWrite(ultraOut, HIGH);
    delayMicroseconds(highDelay);
    digitalWrite(ultraOut, LOW);

    // Calculates Distance
    dur = pulseIn(ultraIn, HIGH);
    // Speed of Sound is 29 microseconds per cm
    // Sound Waves Going and Reflecting Back Takes Twice The Distance
    cm = dur / 29 / 2;

    // Input
    input = double(cm);

    // Reports Input
    reportResult("Distance", input);

    // Computes PID For Output
    output = computePID(input);

    // Formats Output to a Range of 0 to 180 Degrees
    servoOutput = min(max(0, int(output)), 180);

    // Sets Servo Degrees
    myServo.write(servoOutput);
    
    // Reports Output
    reportResult("Output", servoOutput);
  }
}

void serialEvent() {
  char key = char(Serial.read());
    switch (key) {
      case 'f': // Turns off functionality
        on = false;
        Serial.println("Turned Off");
        break;
      case 'o': // Turns on functionality
        on = true;
        Serial.println("Turned On");
        break;
      case 'u': // Updates PID values
        kP = Serial.parseFloat();
        kI = Serial.parseFloat();
        kD = Serial.parseFloat();
        setPoint = Serial.parseFloat();
        reportResult("Updated Successfully!");
        break;
    }
  }
}

// Prints Results Back to Python
void reportResult(String msg, double val) {
  formatOutput = msg;
  formatOutput.concat(": ");
  formatOutput.concat(val);
  Serial.println(formatOutput);
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
