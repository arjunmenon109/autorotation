#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h> //Arduino Servo Library 

MPU6050 mpu;
Servo ESC;
const float GYRO_SCALE = 16.4; // Sensitivity scale factor for gyro
float gyro_x_offset = 0.0; // Gyro z-axis offset for calibration
float wheelmass = 0.01322; //Mass of reaction Wheel in kg
float bodymass = 0.150; //Mass of body in kg
float bodyradius = (0.083/2); //Radius of Body
const float moment = 0.5*0.15*0.0415*0.0415;
const float moment2 = (0.5*0.01322*0.04*0.04);
float neededrpm;
float torque;
float percentage;
int pwm;
void setup() {
  Wire.begin();
  Serial.begin(115200);
  ESC.attach(15);
  delay(100);
  while (!Serial);
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  mpu.setFullScaleGyroRange(2000);
  Serial.println(mpu.getFullScaleGyroRange());
  Serial.println("Calibrating gyro...");
  for (int i = 0; i < 100; i++) {
    int16_t gyro_x, gyro_y, gyro_z;
    mpu.getRotation(&gyro_x, &gyro_y, &gyro_z);
    gyro_x_offset += gyro_x;
    delay(10);
  }
  gyro_x_offset /= 100.0;
  Serial.print("Gyro x-axis offset: ");
  Serial.println(gyro_x_offset);
  
  
}

void loop() {
  int16_t gyro_x, gyro_y, gyro_z;

  mpu.getRotation(&gyro_x, &gyro_y, &gyro_z);

  // Convert gyro readings to radians/sec
  float gyro_x_rad = (gyro_x - gyro_x_offset) / GYRO_SCALE * M_PI / 180.0;

  // Calculate angular acceleration
  float accel_x_rad = gyro_x_rad;

  float torque = moment*gyro_x_rad;
  float neededrpm = ((torque / moment2)*60)/(2* M_PI);
  //Serial.print("bodytorque: ");
  //Serial.print(torque, 10);
  //Serial.print(" RPM: ");
  //Serial.println(neededrpm, 10);
  float percentage = (neededrpm/7300)*100;
  if (percentage<0) {
    int pwm = map(percentage, 0, -100, 130, 180);
    Serial.println(pwm, 10);
    ESC.write(pwm);
  }
  else if (percentage>0){
    int pwm = map(percentage, 0, 100, 130 , 92);
    Serial.println(pwm, 10);
    ESC.write(pwm);
  }


  
  delay(100);
}
