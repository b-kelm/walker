/* Get tilt angles on X and Y, and rotation angle on Z
 * Angles are given in degrees
 * 
 * License: MIT
 */

#include "Wire.h"
#include <MPU6050_light.h>



MPU6050 mpu(Wire);
unsigned long timer = 0;
float e_Theta = 0;
float e_Omega = 0;
float theta = 0;
float omega = 0;

float theta_0 = 0;
float omega_0 = 0;

float k_theta = 0;
float k_omega = 0;

float k_I_theta = 0;
float I_theta = 0;
float P_theta = 0;
float P_omega = 0;

float u_C = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

  analogWrite(2, 0);
  analogWrite(3, 0);

}

void loop() {
  elapsedMillis loopTime;
  mpu.update();
  
  if((millis()-timer)>10){ // print data every 10ms
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

  theta = mpu.getAngleX(); // Angle
  omega = mpu.getGyroX(); // Angular Velocity

// Control Law - Full State Feedback
// Gains
  k_theta = 48*PI/180*3.8; // 12
  k_omega = 8*PI/180*3.8; // 8
  // k_theta_V = 13; //  P Gain
  k_I_theta = 0.2; // I Integrator Gain 0.02

  theta_0 = 14;
  omega_0 = 0;
  
  e_Theta = (theta - theta_0 - 90); // Error of theta
  e_Omega = omega - omega_0;


  // Add Dead Band to it
  if(abs(e_Theta) < 20) {
    I_theta = I_theta + e_Theta * k_I_theta * loopTime; // Integrator
    P_theta = k_theta * e_Theta; // Proportional
    P_omega = k_omega * e_Omega; // Dampening
    
    u_C =   P_omega + P_theta + I_theta;

    
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

  analogWrite(2, +u_C);
  analogWrite(3, -u_C);
  
  analogWrite(4, -u_C);
  analogWrite(5, +u_C);


  
}
