/*
    MPU6050 Triple Axis Gyroscope & Accelerometer. Pitch & Roll & Yaw Gyroscope Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html
    GIT: https://github.com/jarzebski/Arduino-MPU6050
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>



Servo myServo1;
Servo myServo2;
Servo myServo3;

MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;
int angle;
int aSer;
int bSer;
int cSer;

int t = 0;

void setup() 
{
  myServo1.attach(9);
  myServo2.attach(10);
  myServo3.attach(11);
  
  Serial.begin(115200);

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();
  Serial.println("Finished Calibration: ");
    delay(2000);
  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(1);
  Serial.print("Threshold Set at: ");
  Serial.println(t);
    delay(2000);
  Serial.println("Guidance and Location Entered");
    delay(2000); 
}

void loop()
{
  timer = millis();

  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;

  aSer = (roll) + yaw + 90;
  myServo1.write(aSer);
  cSer = (0.866)*(pitch)-(0.500)*(roll) + yaw + 90;
  myServo3.write(bSer);
  bSer = (-0.866)*(pitch)-(0.500)*(roll) + yaw + 90;
  myServo2.write(cSer);
  
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.print(roll);  
  Serial.print(" Yaw = ");
  Serial.println(yaw);
  
  Serial.print(" aSer = ");
  Serial.print(aSer);
  Serial.print(" bSer = ");
  Serial.print(bSer);
  Serial.print(" cSer = ");
  Serial.println(cSer);
  // Wait to full timeStep period
  delay((timeStep*1000) - (millis() - timer));
  
  
}
