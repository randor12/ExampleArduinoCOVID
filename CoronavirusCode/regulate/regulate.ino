/**
 * AUTHOR: Ryan Nicholas
 * DATE: April 3, 2020
 * DESCRIPTION: The purpose of this code is to have the arduino communicate with the 
 *              RC SERVER in order to regulate pumps for the ventilator. 
 *              If something is wrong, the ventilator should activate an emergency noise 
 */

#include <Servo.h>

const int ON_BUTTON = 4; // Add the ON BUTTON to Port 4
const int OFF_BUTTON = 5; // Add the OFF BUTTON to Port 5
bool isOn; // check if the ventilator should be on and pumping 
const int FSR_PIN = A0; // PRESSURE SENSOR in Port ANALOG 0 (A0)
const float VCC = 4.98; // Measure VOLTS of Arduino 5V line
const float R_DIV = 3230.0; // Average RESISTANCE of Arduino (3.3k resistor)

Servo servoMotor; // Add the SERVO MOTOR that will pump the ventilator

// THESE VALUES WILL NEED TO BE TUNED -> by tuning them, this will regulate the pump
float kP = 0.006; // Currently estimated to start approximately at 25 pumps per minute 
float kI = 0.0;
float kD = 0.006;

int errorPump = 0;
int preErrorPump = 0;
int derivativePump = 0;
int totalError = 0;

/**
 * Set up function 
 * Used to run this before anything starts up
 */
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ON_BUTTON, INPUT);
  pinMode(OFF_BUTTON, INPUT);
  pinMode(FSR_PIN, INPUT);
  servoMotor.attach(9); // attach to port 9
  isOn = false;
  
}

/**
 * Pump the ventilator 
 * @arg tDelay this is the time delay in milliseconds for the pump
 */
void pump(double tDelay) {
  
  // NOTE: tDelay should be on average 6 seconds in order to do 25 pumps per minute

  tDelay = ((tDelay * 1000) - 200) / 360.0; // Calculate time delay in milliseconds
  
  // Loop though the angle that the servo motor is -> steps by 5 degrees each time
  for (int i = 0; i < 180; i++) {
    // Rotate servo motor 180 degrees
    servoMotor.write(i);
    delay(tDelay);
  }

  delay(100);
  // Loop through the angle of servo motor -> go back
  for (int i = 180; i > 0; i--) {
    servoMotor.write(i);
    delay(tDelay);
  }
  delay(100);
}

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
    return force;
  }
  else {
    // If no force is detected
    delay(100);
    return -1.0;
  }
}

/**
 * Pump the ventilator code 
 */
void pumpVentilator(float desiredValue) {
  // Pump the ventilator code here
  float force = getForce();

  if (force != -1)
  {
    errorPump = desiredValue - force;
    totalError += errorPump;
    derivativePump = errorPump - preErrorPump;
  
    double pumpsPerSecond = (kP * errorPump) + (totalError * kI) + (derivativePump * kD);
    preErrorPump = errorPump;
    // Ensure it is not over 30 pumps per minute
    if (pumpsPerSecond > .5) {
      pumpsPerSecond = .5;
    }
  
    // Ensure it is not less than 10 pumps per minute 
    if (pumpsPerSecond < .17) {
      pumpsPerSecond = .17;
    }
  
    double secsPerPump = (1.0 / pumpsPerSecond);
  
    pump(secsPerPump);
  }
  delay(100);
}


/**
 * Loop continuously 
 */
void loop() {
  // put your main code here, to run repeatedly:
  int onButton = digitalRead(ON_BUTTON);
  int offButton = digitalRead(OFF_BUTTON);
   
  // Turn the ventilator pump on
  if (onButton == HIGH) {
    isOn = true;
  }

  // Turn the ventialor pump off
  if (offButton == HIGH) {
    isOn = false;
  }

  // Pump the ventilator 
  if (isOn) {
    // Attempt to pump venitilator so the pressure sensor reads a pressure of 25 cmH20
    pumpVentilator(25.0);
  }
  else {
    // Reset PID values
    errorPump = 0;
    totalError = 0;
    derivativePump = 0;
    preErrorPump = 0;
  }

  delay(100); // delay to not hog the CPU
}
