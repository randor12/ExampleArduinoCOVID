/**
 * AUTHOR: Ryan Nicholas
 * DATE: April 3, 2020
 * DESCRIPTION: The purpose of this code is to have the arduino communicate with the 
 *              RC SERVER in order to regulate pumps for the ventilator. 
 *              If something is wrong, the ventilator should activate an emergency noise 
 */

//#include <Servo.h>
#include "Arduino.h"
#include <Stepper.h>

const int POT = A1; // Add the POTENTIOMETER to Port ANALOG 1 (A1)
bool isOn; // check if the ventilator should be on and pumping 
const int FSR_PIN = A0; // PRESSURE SENSOR in Port ANALOG 0 (A0)
const float VCC = 4.98; // Measure VOLTS of Arduino 5V line
const float R_DIV = 3230.0; // Average RESISTANCE of Arduino (3.3k resistor)

const int buzzer = 14; // BUZZER on Port 14
//
//Servo servoMotor; // Add the SERVO MOTOR that will pump the ventilator
const int stepsPerRevolution = 200; // Might be 360 steps but claims 200 on website 
Stepper stepMotor(stepsPerRevolution, 10, 11, 12, 13); // Initialize STEPPER motor on Ports 10, 11, 12, 13

// THESE VALUES WILL NEED TO BE TUNED -> by tuning them, this will regulate the pump
float kP = 0.08; // Currently estimated to start approximately at 25 pumps per minute 
float kI = 0.0;
float kD = 0.08;

double errorPump = 0.0;
double preErrorPump = 0.0;
double derivativePump = 0.0;
double totalError = 0.0;

const float desiredForce = 20; // Desired force is 20 cmH2O
bool problem = false;

/**
 * Set up function 
 * Used to run this before anything starts up
 */
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(POT, INPUT);
  pinMode(FSR_PIN, INPUT);
//  servoMotor.attach(9); // attach to port 9
  stepMotor.setSpeed(25); // Set at 25 rpm
  isOn = false;
  pinMode(buzzer, OUTPUT);
}

/**
 * pump with the stepper - set the rpm to set the rotations per minute 
 * @arg rpm rotation per minute
 * @return returns false if there are no problems 
 */
bool pumpWithStepper(int rpm) {
  // Use for loop and move both 1 at a time to move at "same time"

  float force = getForce();

  if (force > 30 || force <= 0) {
    return true;
  }
  
  errorPump = desiredForce - force;
  totalError += errorPump;
  derivativePump = errorPump - preErrorPump;
  preErrorPump = errorPump;

  float rpmChange = (errorPump * kP) + (totalError * kI) + (derivativePump * kD);

  int newRPM = rpm + rpmChange;
  // Check RPM is within range of 10 to 30 RPM
  if (newRPM > 30) {
    newRPM = 30;
  }
  if (newRPM < 10) {
    newRPM = 10;
  }

  stepMotor.setSpeed(newRPM);
  stepMotor.step(stepsPerRevolution); // Rotate clockwise 1 rotation 
  delay(300);

  return false;
  // stepMotor.step(-stepsPerRevolution); // rotate counterclockwise 1 rotation 
}
//
///**
// * Pump the ventilator with servo
// * @arg tDelay this is the time delay in seconds per pump
// */
//void pump(double tDelay) {
//  
//  // NOTE: tDelay should be on average 6 seconds in order to do 25 pumps per minute
//
//  tDelay = ((tDelay * 1000) - 200) / 360.0; // Calculate time delay in milliseconds
//  
//  // Loop though the angle that the servo motor is -> steps by 5 degrees each time
//  for (int i = 0; i < 180; i++) {
//    // Rotate servo motor 180 degrees
//    servoMotor.write(i);
//    delay(tDelay);
//  }
//
//  delay(100);
//  // Loop through the angle of servo motor -> go back
//  for (int i = 180; i > 0; i--) {
//    servoMotor.write(i);
//    delay(tDelay);
//  }
//  delay(100);
//}

/**
 * Get the force of the sensor
 * @return return the pressure, if no pressure is detected, return -1 
 */
float getForce() {
  int fsrADC = analogRead(FSR_PIN);

  if (fsrADC != 0) {
    // Use ADC reading to calculate voltage:
    float fsrV = fsrADC * VCC / 1023.0;
    // Use voltage and static resistor value to 
    // calculate FSR resistance:
    float fsrR = R_DIV * (VCC / fsrV - 1.0);
    // Guesstimate force based on slopes in figure 3 of
    // FSR datasheet:
    float force;
    float fsrG = 1.0 / fsrR; // Calculate conductance
    // Break parabolic curve down into two linear slopes:
    if (fsrR <= 600) 
      force = (fsrG - 0.00075) / 0.00000032639;
    else
      force =  fsrG / 0.000000642857;
    delay(100);
    return force; // Force in cmH2O
  }
  else {
    // If no force is detected
    delay(100);
    return -1.0;
  }
}

///**
// * Pump the ventilator code 
// * @arg desiredValue desired force value in cmH20 
// */
//void pumpVentilator(float desiredValue) {
//  // Pump the ventilator code here
//  float force = getForce();
//
//  if (force != -1)
//  {
//    errorPump = desiredValue - force;
//    totalError += errorPump;
//    derivativePump = errorPump - preErrorPump;
//
//    // type cast the pumps per seconds 
//    double pumpsPerSecond = (double)((kP * errorPump) + (totalError * kI) + (derivativePump * kD));
//    preErrorPump = errorPump;
//    // Ensure it is not over 30 pumps per minute
//    if (pumpsPerSecond > .5) {
//      pumpsPerSecond = .5;
//    }
//  
//    // Ensure it is not less than 10 pumps per minute 
//    if (pumpsPerSecond < .17) {
//      pumpsPerSecond = .17;
//    }
//  
//    double secsPerPump = (1.0 / pumpsPerSecond);
//  
//    pump(secsPerPump);
//  }
//  delay(100);
//}


/**
 * Loop continuously 
 */
void loop() {
  // put your main code here, to run repeatedly:
  int potOn = analogRead(POT);

  float pumpsPerMin = potOn * (30.0 / 1028.0); // Average speed of pumping 
   
  // Turn the ventilator pump on
  if (pumpsPerMin >= 10) {
    isOn = true; // Change if the ventilator is on or off
    
  }
  else {
    isOn = false;
  }

  // Pump the ventilator 
  if (isOn) {
    // Attempt to pump venitilator so the pressure sensor reads approximately a pressure of 20 cmH20
    // pumpVentilator(pumpsPerMin);
    problem = pumpWithStepper((int)pumpsPerMin);
  }
  else {
    // Reset PID values
    errorPump = 0.0;
    totalError = 0.0;
    derivativePump = 0.0;
    preErrorPump = 0.0;
  }

  // Check if there is a problem 
  if (problem) {
    tone(buzzer, 1000); // Send 1 KHZ sound signal 
  }
  else {
    noTone(buzzer); // Stop the sound 
  }

  delay(100); // delay to not hog the CPU
}
