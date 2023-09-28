/* Inverted Pendulum on a Cart
 *  by Benjamin Kelm - benjamin.kelm@rwth-aachen.de
 *  
 *  Small project of a self-righting inverted pendulum, to showcase a classical control problem.
 *  Implements a LQR (Full) State Feedback Controller for attitude control.
 *  SImulink Model in repository
 *  
 *  Necessary Hardware:
 *  Teensy 4.1
 *  MPU6050 IMU Sensor
 *  L298 Dual H-Bridge
 *  Brushed Motors with Wheels
 * License: MIT
 */

// Libaries
#include "Wire.h"
#include <MPU6050_light.h>

// Sensor Object
MPU6050 mpu(Wire);

// Timer
unsigned long timer = 0;

// Setpoints
float theta_0 = 2; // Slightly tilted for stationary pendulum because of weight distribution
float omega_0 = 0; // No angular speed

// States
float theta = 0;
float omega = 0;

// Errors
float e_Theta = 0;
float e_Omega = 0;

% Controller 


// Control Law - Full State Feedback
float  k_theta = 48*PI/180*3.8; // gain for angular feedback
float  k_omega = 8*PI/180*3.8; // gain for rate feedback

float  k_I_theta = 0.3; // I Integrator Gain (for stationary precision)


// Accumulated control gains
float I_theta = 0;
float P_theta = 0;
float P_omega = 0;

// Control Input (for both wheels)
float u_C = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // Function Check IMU
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050

  // IMU Calibration
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

  // Set Inputs to 0 
  analogWrite(2, 0);
  analogWrite(3, 0);

}

void loop() {
  elapsedMillis loopTime;
  mpu.update();

 // print data every 10ms to Serial Port
  if((millis()-timer)>10){ 
	Serial.print("theta : ");
	Serial.print(mpu.getAngleX());
	Serial.print("\t omega : ");
	Serial.print(mpu.getGyroX());
  Serial.print("\t LoopTime : ");
  Serial.print(loopTime);
  Serial.print("\t u_C : ");
  Serial.println(u_C);
	timer = millis();  
  }

  // Get State from Sensor
  theta = mpu.getAngleX(); // Angle
  omega = mpu.getGyroX(); // Angular Velocity

  // Calculate Control Errror
  e_Theta = (theta - theta_0 - 90); // Error of theta
  e_Omega = omega - omega_0;

  // Compute Control Input
  if(abs(e_Theta) < 20) { // Stop wheels if >20 deg - Dead Band
    I_theta = I_theta + e_Theta * k_I_theta * loopTime; // Integrator
    P_theta = k_theta * e_Theta; // Proportional
    P_omega = k_omega * e_Omega; // Dampening

    // Compute total control input
    u_C =   P_omega + P_theta + I_theta; 

    // Mapping to reverse input in H-Bridge
    if(u_C > 0){
      u_C = map(u_C, 0, 255, 0, 255);
      }
    else{
      u_C = map(u_C, -255, 0, -255, -0);
      }
    }
   else { // Stop Controller
      I_theta = 0; // reset Integrator 
      u_C = 0;
      }

  // Set Control Input to pins (Voltage, Analog)
  analogWrite(2, +u_C);
  analogWrite(3, -u_C);
  
  analogWrite(4, -u_C);
  analogWrite(5, +u_C);
  
}
