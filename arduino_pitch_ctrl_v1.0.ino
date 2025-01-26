#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <LiquidCrystal.h>

// Pin Definitions
#define JOYSTICK_Y A1        // Joystick Y-axis for manual pitch control
#define JOYSTICK_BUTTON 7    // Joystick button for autopilot toggle
#define SERVO_PIN 9          // Servo control pin
#define TRIG_PIN 8           // Ultrasonic sensor trigger pin
#define ECHO_PIN 6           // Ultrasonic sensor echo pin
#define POTENTIOMETER_PIN A2 // Potentiometer for target altitude adjustment

// LCD Pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2); // RS, E, D4, D5, D6, D7

// Sensors
Adafruit_BMP280 bmp;

// Servo
Servo elevator;

// Variables
bool autopilot = false;        // Autopilot toggle state
double altitude = 0.0;         // Current altitude from BMP280
double targetAltitude = 0.0;  // Default target altitude
double obstacleDistance = 0.0; // Distance from ultrasonic sensor
int servoPosition = 90;        // Target servo position
int currentServoPosition = 90; // Gradual transition tracking for autopilot mode

// PID Variables
double Kp = 2.5, Ki = 0.1, Kd = 0.3;  // PID tuning values for altitude correction
double prevError = 0.0, integral = 0.0; // PID terms

// Kalman Filter Variables
double kalmanEstimate = 0.0, kalmanGain = 0.1;
const double processNoise = 0.1;       // Kalman tuning
const double measurementNoise = 1.0;   // Kalman tuning

// LCD Update Variables
unsigned long lastLCDUpdate = 0;
const unsigned long LCD_UPDATE_INTERVAL = 500; // Update every 500ms

// Function Prototypes
double measureDistance();
double calculatePID(double setpoint, double measured, double &prevError, double &integral);
double kalmanFilter(double measurement, double &estimate, double processNoise, double measurementNoise);
void updateLCD();

void setup() {
    Serial.begin(9600);

    // Initialize ultrasonic sensor pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Initialize joystick button
    pinMode(JOYSTICK_BUTTON, INPUT_PULLUP);

    // Initialize BMP280 sensor
    if (!bmp.begin(0x76)) {
        Serial.println("BMP280 initialization failed!");
        while (1);
    }

    // Initialize servo
    elevator.attach(SERVO_PIN);
    elevator.write(servoPosition);

    // Initialize LCD
    lcd.begin(16, 2); // 16x2 LCD
    lcd.setCursor(0, 0);
    lcd.print("System Ready!"); // Display initial message
    delay(2000);
    lcd.clear();
}

void loop() {
    unsigned long currentTime = millis();

    // Measure distance using ultrasonic sensor
    obstacleDistance = measureDistance();

    // Read altitude using BMP280 and apply Kalman filter
    double rawAltitude = bmp.readAltitude(1032.84); 
    altitude = kalmanFilter(rawAltitude, kalmanEstimate, processNoise, measurementNoise);

    // Read joystick button for autopilot toggle
    static bool lastButtonState = HIGH;
    bool currentButtonState = digitalRead(JOYSTICK_BUTTON);
    if (lastButtonState == HIGH && currentButtonState == LOW) {
        autopilot = !autopilot;
        Serial.println(autopilot ? "Autopilot ON" : "Autopilot OFF");
    }
    lastButtonState = currentButtonState;

    // Adjust target altitude using potentiometer in autopilot
    if (autopilot) {
        int potValue = analogRead(POTENTIOMETER_PIN);
        targetAltitude = map(potValue, 0, 1023, 50, 200); // Map to 50–200 meters
    }

    // Servo Control Logic
    if (obstacleDistance > 0 && obstacleDistance < 15) { // Obstacle detected
        servoPosition = map(obstacleDistance, 6, 15, 135, 90); // Full pull-up within 6 cm
        servoPosition = constrain(servoPosition, 90, 135); // Ensure within servo range
        elevator.write(servoPosition);  // Immediate servo response
    } else {
        if (autopilot) { // Autopilot mode
            double error = targetAltitude - altitude;

            // Deadband: Force servo to neutral when within ±2 meters of target altitude
            if (abs(error) <= 2.0) { // ±2 meters tolerance
                servoPosition = 90; // Neutral position
                integral = 0;       // Reset integral term
                prevError = 0;      // Reset previous error
            } else {
                // Use PID for smooth corrections
                double adjustment = calculatePID(targetAltitude, altitude, prevError, integral);
                servoPosition = constrain(90 + adjustment, 45, 135); // Apply PID adjustment
            }

            // Gradual transition for smooth movement in autopilot
            if (currentServoPosition < servoPosition) {
                currentServoPosition++;
            } else if (currentServoPosition > servoPosition) {
                currentServoPosition--;
            }
            elevator.write(currentServoPosition); // Gradual servo response
        } else { // Manual joystick control
            int joystickY = analogRead(JOYSTICK_Y);
            servoPosition = map(joystickY, 0, 1023, 135, 45); // Map joystick input to servo range
            elevator.write(servoPosition); // Immediate servo response for joystick
        }
    }

    // Update the LCD at set intervals
    if (currentTime - lastLCDUpdate >= LCD_UPDATE_INTERVAL) {
        updateLCD();
        lastLCDUpdate = currentTime;
    }

    // Debugging Output
    Serial.print("Distance: "); Serial.print(obstacleDistance);
    Serial.print(" cm, Altitude: "); Serial.print(altitude);
    Serial.print(" m, Target Altitude: "); Serial.print(targetAltitude);
    Serial.print(" m, Servo Pos: "); Serial.println(servoPosition);

    delay(10); // Reduce delay to improve responsiveness
}

// Function to measure distance using ultrasonic sensor
double measureDistance() {
    static double previousDistance = 15.0; // Initial baseline distance (safe default)
    double threshold = 4.0;               // Max allowed change between consecutive readings

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout at 30ms
    double distance = (duration * 0.0343) / 2; // Convert to cm

    // Ensure distance is within valid range
    if (distance < 2 || distance > 400) {
        return previousDistance; // Ignore invalid values and use the last valid distance
    }

    // Apply noise filtering: Reject abrupt changes
    if (abs(distance - previousDistance) > threshold && distance < 15) {
        return previousDistance; // Ignore sudden spikes within the critical range
    }

    // Update baseline distance and return valid measurement
    previousDistance = distance;
    return distance;
}


// PID Controller Function
double calculatePID(double setpoint, double measured, double &prevError, double &integral) {
    double error = setpoint - measured;

    if (abs(error) <= 0.05) {
        integral = 0;
        return 0;
    }

    integral += error * 0.05;
    double derivative = (error - prevError) / 0.05;
    prevError = error;

    return Kp * error + Ki * integral + Kd * derivative;
}

// Kalman Filter Function
double kalmanFilter(double measurement, double &estimate, double processNoise, double measurementNoise) {
    double predictedEstimate = estimate;
    double predictedError = processNoise;
    kalmanGain = predictedError / (predictedError + measurementNoise);
    estimate = predictedEstimate + kalmanGain * (measurement - predictedEstimate);
    return estimate;
}

// LCD Update Function
void updateLCD() {
    lcd.setCursor(0, 0);
    lcd.print("AP: ");
    lcd.print(autopilot ? "ON " : "OFF");

    lcd.setCursor(0, 1);
    lcd.print("TGT ALT: ");
    lcd.print(targetAltitude, 1); // Display target altitude with 1 decimal point
    lcd.print(" m  ");           // Padding to clear extra characters
}
