/*
         ___           __          ____          __          __              
        /   |  __  __ / /_ ____   / __ \ ____   / /_ ____ _ / /_ ____   _____
       / /| | / / / // __// __ \ / /_/ // __ \ / __// __ `// __// __ \ / ___/
      / ___ |/ /_/ // /_ / /_/ // _, _// /_/ // /_ / /_/ // /_ / /_/ // /    
     /_/  |_|\__,_/ \__/ \____//_/ |_| \____/ \__/ \__,_/ \__/ \____//_/     
                                                                            
*/
// Data include
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Adafruit_BMP085.h>
#include <HMC5883L_Simple.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <Wire.h>
#include <LittleFS.h>

// Defining

const int MPU_addr=0x68;
MPU6050 accelgyro;
Adafruit_BMP085 bmp;
HMC5883L_Simple Compass;
Servo paraservo;
#define FILENAME "/logged.txt"
File dataFile;

//gy87
float temp;
float pressure;
float altitude;
float realaltitude;
float heading;
int i = 0;
char command;


//Init
int16_t ax, ay, az;
int16_t gx, gy, gz;



#define LED_PIN 16
bool blinkState = false;

void setup(){
  Serial.begin(9600);
  paraservo.attach(2);
  paraservo.write(130);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(500); 
  Wire.begin(4,5);
  Serial.println("Initializing I2C devices...");
  delay(500);
  Serial.println("Initializing BMP");
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
  delay(500);
  Serial.println("Initializing MPU6050");
  accelgyro.initialize();
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  accelgyro.setI2CBypassEnabled(true); // set bypass mode for gateway to hmc5883L
  delay(500);
  Serial.println("Initializing COMPASS");
  Compass.SetDeclination(23, 35, 'E');
  Compass.SetSamplingMode(COMPASS_SINGLE);
  Compass.SetScale(COMPASS_SCALE_130);
  Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
  Serial.println("Compass Working!");
  delay(500);
  Serial.println("LittleFS Configuring");
  LittleFS.begin();
  dataFile = LittleFS.open(FILENAME, "a+");
  if (!dataFile) {
    Serial.println("Failed to open file for writing");
  }
  Serial.println("FS Initialised");
  
  Serial.println("Initialization Done");
  Serial.println("This is Arjun Menons E19MEC012's self built DAS");
  Serial.println("▄▀█ █░█ ▀█▀ █▀█ █▀█ █▀█ ▀█▀ ▄▀█ ▀█▀ █▀█ █▀█ ® ");
  Serial.println("█▀█ █▄█ ░█░ █▄█ █▀▄ █▄█ ░█░ █▀█ ░█░ █▄█ █▀▄ ");                                    
  Serial.println("Ready, awaiting commands sire!");
  Serial.println("D = Delete, R = Read, L =Start Logging, x to restart (case sensitive)");
  digitalWrite(LED_PIN, HIGH);
}
void loop()
  {
    if(Serial.available())
    {
      char command = Serial.read();
      Serial.println(command);
      if (command == 'R')
      {
        readfile();
        return;
      }
      else if (command == 'L')
      {
        Serial.print("Starting logging in 3 Seconds...");
        delay(3000);
        while (Serial.read() != 'S')
        {
          logging();
          delay(250);
        }
        return;
      }
      else if (command == 'D')
      {
        deletefile();
        return;
      }
      else if (command == 'x') {
      Serial.println("Restarting ESP");
      ESP.restart();
    }

    }
  }
void logging(){   
  digitalWrite(LED_PIN, HIGH);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float temp = bmp.readTemperature();
  float pressure = bmp.readPressure();
  float altitude = bmp.readAltitude();
  float realaltitude = bmp.readAltitude(101500);
  float heading = Compass.GetHeadingDegrees();
  if (dataFile) {
    dataFile.print(altitude);
    dataFile.print(",");
    dataFile.print(ax);
    dataFile.print(",");
    dataFile.print(ay);
    dataFile.print(",");
    dataFile.print(az);
    dataFile.print(",");
    dataFile.print(gx);
    dataFile.print(",");
    dataFile.print(gy);
    dataFile.print(",");
    dataFile.println(gy);
    dataFile.flush();
    Serial.print("data logged succesfully current altitude: ");
    Serial.println(altitude);
  }
  digitalWrite(LED_PIN, LOW);
}
 void deletefile(){
  LittleFS.remove(FILENAME);
  Serial.println("Data File Removed");
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW); 
 }

void readfile(){
  if (!dataFile){
    Serial.print("Error - No Saved DATA");
    return;
  }
    while (dataFile.available()) {
        char c = dataFile.read();

        if (c == '\n') {
          Serial.println();
        } 
        else {
          Serial.write(c);
        }
      }
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
}
 
