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
#include <EEPROM.h>

// Defining

const int MPU_addr=0x68;
MPU6050 mpu;
Adafruit_BMP085 bmp;
HMC5883L_Simple Compass;
Servo paraservo;
#define FILENAME "/logged.txt"
File dataFile;
const int outputPin = 10;
#define PARAM_START_ADDR 0

//Initializing Variables
float temp;
float pressure;
float altitude;
float realaltitude;
float heading;
int i = 0;
char command;
char input;
bool wheelState;
int p = 0;
bool paratest = 0;
const int caliamount = 10; //EPPROM DEFAULT VALUE Amount Of Values for Level
float calialtiude[caliamount]; //EPPROM DEFAULT VALUE
float calidelay = 1000; //EPPROM DEFAULT VALUE
float calisum = 0.0; 
float SurfAlt = 0; //EPPROM DEFAULT VALUE
bool chadmode = false;
String chadmodepasskey = ("ArjunMenon123");
int AccelScale = 2; 
int GyroScale = 250;
int DLPFScale = 5;
int stored_Accel;
int stored_Gyro;
int stored_DLPF;
int stored_caliamount;
int stored_calidelay;
int stored_SurfAlt;
int value;


/*
EPROM ASSIGNMENT: 512bytes

Accel = 2 PARAM_START_ADDR + 1
Gyro = 4 PARAM_START_ADDR + 5
DLPF = 3 PARAM_START_ADDR + 11

Calibration Settings
caliamount = 3 PARAM_START_ADDR + 16
calidelay = 4 PARAM_START_ADDR + 21
SurfAlt = 5 PARAM_START_ADDR + 27

Accelerometer full-scale ranges:

MPU6050_ACCEL_FS_2: ±2g (default)
MPU6050_ACCEL_FS_4: ±4g
MPU6050_ACCEL_FS_8: ±8g
MPU6050_ACCEL_FS_16: ±16g
Gyro full-scale ranges:

MPU6050_GYRO_FS_250: ±250 °/s (default)
MPU6050_GYRO_FS_500: ±500 °/s
MPU6050_GYRO_FS_1000: ±1000 °/s
MPU6050_GYRO_FS_2000: ±2000 °/s

The MPU6050 offers a range of DLPF modes with different bandwidths (i.e., cutoff frequencies) and filter response times. The available modes are:

MPU6050_DLPF_BW_256: bandwidth = 256 Hz, delay = 0.98 ms
MPU6050_DLPF_BW_188: bandwidth = 188 Hz, delay = 1.9 ms
MPU6050_DLPF_BW_98: bandwidth = 98 Hz, delay = 2.8 ms
MPU6050_DLPF_BW_42: bandwidth = 42 Hz, delay = 4.9 ms
MPU6050_DLPF_BW_20: bandwidth = 20 Hz, delay = 8.5 ms
MPU6050_DLPF_BW_10: bandwidth = 10 Hz, delay = 13.8 ms
MPU6050_DLPF_BW_5: bandwidth = 5 Hz, delay = 19.0 ms
*/

//Init
int16_t ax, ay, az;
int16_t gx, gy, gz;



#define LED_PIN 16
bool blinkState = false;

void setup(){
  Serial.begin(115200);
  paraservo.attach(2);
  paraservo.write(130);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(outputPin, OUTPUT);
  digitalWrite(outputPin, LOW);
  bool wheelState = LOW;
  delay(500); 

  Serial.println("Initializing EEPROM...");
  EEPROM.begin(512);
  EEPROM.get(PARAM_START_ADDR + 1, stored_Accel);
  EEPROM.get(PARAM_START_ADDR + 5, stored_Gyro);
  EEPROM.get(PARAM_START_ADDR + 11, stored_DLPF);
  EEPROM.get(PARAM_START_ADDR + 16, stored_caliamount);
  EEPROM.get(PARAM_START_ADDR + 21, stored_calidelay);
  EEPROM.get(PARAM_START_ADDR + 27, stored_SurfAlt);
  //Serial.println(stored_SurfAlt);
  // Initialize variables with retrieved data if it's not zero
  int AccelScale = (stored_Accel != 0) ? stored_Accel : 2;
  int GyroScale = (stored_Gyro != 0) ? stored_Gyro : 250;
  int DLPFScale = (stored_DLPF != 0) ? stored_DLPF : 5;
  int caliamount = (stored_caliamount != 0) ? stored_caliamount : 0;
  int calidelay = (stored_calidelay != 0) ? stored_calidelay : 0;
  int SurfAlt = (stored_SurfAlt != 0) ? stored_SurfAlt : 0;

  Serial.println("Initializing I2C devices...");
  Wire.begin(4,5);
  delay(500); 
  
  Serial.println("Initializing BMP");
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
  delay(500);
  
  Serial.println("Initializing 6050");
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  mpu.setI2CBypassEnabled(true); // set bypass mode for gateway to hmc5883L
  switch (AccelScale)
  {
  case 2:
     mpu.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_2);
     Serial.println("Set Accel 2");
    break;
  case 4:
     mpu.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_4);
     Serial.println("Set Accel 4");
    break;
  case 8:
     mpu.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_8);
     Serial.println("Set Accel 8");
    break;
  case 16:
     mpu.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_16);
     Serial.println("Set Accel 16");
    break;
  default:
    mpu.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_2);
    Serial.println("Something Wrong Setting Default Accel");
  }
  switch (GyroScale)
  {
  case 250:
     mpu.setFullScaleGyroRange(MPU6050_IMU::MPU6050_GYRO_FS_250);
     Serial.println("Set Gyro 250");
    break;
  case 500:
     mpu.setFullScaleGyroRange(MPU6050_IMU::MPU6050_GYRO_FS_500);
     Serial.println("Set Gyro 500");
    break;
  case 1000:
     mpu.setFullScaleGyroRange(MPU6050_IMU::MPU6050_GYRO_FS_1000);
     Serial.println("Set Gyro 1000");
    break;
  case 2000:
     mpu.setFullScaleGyroRange(MPU6050_IMU::MPU6050_GYRO_FS_2000);
     Serial.println("Set Gyro 2000");
    break;
  default:
    mpu.setFullScaleGyroRange(MPU6050_IMU::MPU6050_GYRO_FS_250);
    Serial.println("Something Wrong Setting Default Gyro 250");
  }
  switch (DLPFScale)
  {
  case 5:
     mpu.setDLPFMode(MPU6050_IMU::MPU6050_DLPF_BW_5);
     Serial.println("Set DLPF 5");
    break;
  case 10:
     mpu.setDLPFMode(MPU6050_IMU::MPU6050_DLPF_BW_10);
     Serial.println("Set DLPF 10");
    break;
  case 20:
     mpu.setDLPFMode(MPU6050_IMU::MPU6050_DLPF_BW_20);
     Serial.println("Set DLPF 20");
    break;
  case 42:
     mpu.setDLPFMode(MPU6050_IMU::MPU6050_DLPF_BW_42);
     Serial.println("Set DLPF 42");
    break;
  case 98:
     mpu.setDLPFMode(MPU6050_IMU::MPU6050_DLPF_BW_98);
     Serial.println("Set DLPF 98");
    break;
  case 188:
     mpu.setDLPFMode(MPU6050_IMU::MPU6050_DLPF_BW_188);
     Serial.println("Set DLPF 188");
    break;
  case 256:
     mpu.setDLPFMode(MPU6050_IMU::MPU6050_DLPF_BW_256);
     Serial.println("Set DLPF 256");
    break;
  default:
    mpu.setDLPFMode(MPU6050_IMU::MPU6050_DLPF_BW_256);
    Serial.println("Something Wrong Setting Default DLPF 256");
  }

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
  Serial.println("Send Command 'I' for list of commands");                                 
  Serial.println("Ready, awaiting commands sire!");
  
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
      else if (command == 'I')
      {
        Serial.println("This system runs at a baud rate of 115200 and uses ASCII letters as commands.");
        Serial.println("List Of Commands: ");
        Serial.println("L: Start Logging & S: Stop Logging");
        Serial.println("R: Read Previous Log & D: Delete Log");
        Serial.println("P: Open/Close Parachute Latch (Toggle)");
        Serial.println("W: Start/Stop Reaction Wheel");
        Serial.println("%: Sudo Mode");
        Serial.println("@: Sudo Settings");
        Serial.println("z: Lower Case, to Restart System");        
        return;
      }
      else if (command == 'L')
      {
        Serial.println("Starting logging in 3 Seconds...");
        delay(3000);
        if (SurfAlt != 0)
        {
          while (Serial.read() != 'S')
          {
            logging();
            delay(10);
          }
          return;
        }
        else 
        {
          while (Serial.read() != 'S')
          {
            logging();
            delay(10);
          }
          return;
        }
        
      }
      else if (command == 'D')
      {
        deletefile();
        return;
      }
      else if (command == 'P')
      {
        if (paratest == 0)
        {
          Serial.println("Opening Parachute Latch");
          paraservo.write(50);
           paratest = 1;
        return;
        }
        else 
        {
          Serial.println("Closing Parahute Latch");
          paraservo.write(130);
          paratest = 0;
        return;
        }
      }
      else if (command == 'W') 
      {
        if (wheelState == LOW){
          Serial.print("Starting Reaction Wheel Systems");
          digitalWrite(outputPin, HIGH);
          wheelState = HIGH;
          delay(10);
          return;
        }
        else if (wheelState == HIGH)
        {
          Serial.println("Stopping Reaction Wheel Systems");
          digitalWrite(outputPin, LOW);
          wheelState = LOW;
          delay(10);
          return;
        }
        return;
      }
      else if (command == 'C') 
      {
        calibrate();
        return;
      }
      else if (command == '@') 
      {
        if (chadmode == true)
        {
          Serial.println("You have accessed Development Settings");
          Serial.println("Pick One Of The Below");
          Serial.println("A - Accel Scale");
          Serial.println("G - Gyro Scale");
          Serial.println("D - DLPF");
          Serial.println("C - Calibration Settings");
          Serial.println("# - Leave");
          while (chadmode == true)
          {
            char command = Serial.read();
            if (command == '#')
            {
              return;
            }
            switch (command)
            {
              case 'A':
                Serial.println("Accelarometer Scale Pick One Of The Below: ");
                Serial.println("1 - ±2g (default)");
                Serial.println("2 - ±4g");
                Serial.println("3 - ±8g");
                Serial.println("4 - ±16g");
                Serial.println("L - Leave");
                while (chadmode == true)
                {
                  char command = Serial.read();
                  if (command == 'L')
                  {
                    break;
                  }
                  switch (command)
                  {
                    case '1':
                      mpu.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_2);
                      Serial.println("Set Accel 2");
                      AccelScale = 2;
                      EEPROM.put(PARAM_START_ADDR + 1, AccelScale);
                      EEPROM.commit();
                      Serial.println("EEPROM Commited");
                      break;
                    case '2':
                      mpu.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_4);
                      Serial.println("Set Accel 4");
                      AccelScale = 4;
                      EEPROM.put(PARAM_START_ADDR + 1, AccelScale);
                      EEPROM.commit();
                      Serial.println("EEPROM Commited");
                      break;  
                    case '3':
                      mpu.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_8);
                      Serial.println("Set Accel 8");
                      AccelScale = 8;
                      EEPROM.put(PARAM_START_ADDR + 1, AccelScale);
                      EEPROM.commit();
                      Serial.println("EEPROM Commited");
                      break;
                    case '4':
                      mpu.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_16);
                      Serial.println("Set Accel 16");
                      AccelScale = 16;
                      EEPROM.put(PARAM_START_ADDR + 1, AccelScale);
                      EEPROM.commit();
                      Serial.println("EEPROM Commited");
                      break;
                    default:
                      Serial.print(" L to Leave ");
                      delay(500);
                      break;
                  }
                  
                }
                break;
            

              case 'G':
                Serial.println("Gyro Scale Pick One Of The Below: ");
                Serial.println("1 - 250 (default)");
                Serial.println("2 - 500");
                Serial.println("3 - 1000");
                Serial.println("4 - 2000");
                Serial.println("L - Leave");
                while (chadmode == true)
                {
                  char command = Serial.read();
                  if (command == 'L')
                  {
                    break;
                  }
                  switch (command)
                  {
                    case '1':
                      mpu.setFullScaleGyroRange(MPU6050_IMU::MPU6050_GYRO_FS_250);
                      Serial.println("Set Gyro 250");
                      GyroScale = 250;
                      EEPROM.put(PARAM_START_ADDR + 5, GyroScale);
                      EEPROM.commit();
                      Serial.println("EEPROM Commited");
                      break;
                    case '2':
                      mpu.setFullScaleGyroRange(MPU6050_IMU::MPU6050_GYRO_FS_500);
                      Serial.println("Set Gyro 500");
                      GyroScale = 500;
                      EEPROM.put(PARAM_START_ADDR + 5, GyroScale);
                      EEPROM.commit();
                      Serial.println("EEPROM Commited");
                      break;  
                    case '3':
                      mpu.setFullScaleGyroRange(MPU6050_IMU::MPU6050_GYRO_FS_1000);
                      Serial.println("Set Gyro 1000");
                      GyroScale = 1000;
                      EEPROM.put(PARAM_START_ADDR + 5, GyroScale);
                      EEPROM.commit();
                      Serial.println("EEPROM Commited");
                      break;
                    case '4':
                      mpu.setFullScaleGyroRange(MPU6050_IMU::MPU6050_GYRO_FS_2000);
                      Serial.println("Set Gyro 2000");
                      GyroScale = 2000;
                      EEPROM.put(PARAM_START_ADDR + 5, GyroScale);
                      EEPROM.commit();
                      Serial.println("EEPROM Commited");
                      break;
                    default:
                      Serial.print(" L to Leave ");
                      delay(500);
                      break;
                  } 
                }
                break;
              case 'D':
                Serial.println("DLPF Pick One Of The Below: ");
                Serial.println("1 - 5 (default)");
                Serial.println("2 - 10");
                Serial.println("3 - 20");
                Serial.println("4 - 42");
                Serial.println("5 - 98");
                Serial.println("6 - 188");
                Serial.println("7 - 256");
                Serial.println("L - Leave");
                while (chadmode == true)
                {
                  char command = Serial.read();
                  if (command == 'L')
                  {
                    break;
                  }
                  switch (command)
                  {
                    case '1':
                      mpu.setDLPFMode(MPU6050_IMU::MPU6050_DLPF_BW_5);
                      Serial.println("Set DLPF 5");
                      DLPFScale = 5;
                      EEPROM.put(PARAM_START_ADDR + 11, DLPFScale);
                      EEPROM.commit();
                      Serial.println("EEPROM Commited");
                      break;
                    case '2':
                      mpu.setDLPFMode(MPU6050_IMU::MPU6050_DLPF_BW_10);
                      Serial.println("Set DLPF 10");
                      DLPFScale = 10;
                      EEPROM.put(PARAM_START_ADDR + 11, DLPFScale);
                      EEPROM.commit();
                      Serial.println("EEPROM Commited");
                      break;  
                    case '3':
                      mpu.setDLPFMode(MPU6050_IMU::MPU6050_DLPF_BW_20);
                      Serial.println("Set DLPF 20");
                      DLPFScale = 20;
                      EEPROM.put(PARAM_START_ADDR + 11, DLPFScale);
                      EEPROM.commit();
                      Serial.println("EEPROM Commited");
                      break;
                    case '4':
                      mpu.setDLPFMode(MPU6050_IMU::MPU6050_DLPF_BW_42);
                      Serial.println("Set DLPF 42");
                      DLPFScale = 42;
                      EEPROM.put(PARAM_START_ADDR + 11, DLPFScale);
                      EEPROM.commit();
                      Serial.println("EEPROM Commited");
                      break;
                    case '5':
                      mpu.setDLPFMode(MPU6050_IMU::MPU6050_DLPF_BW_98);
                      Serial.println("Set DLPF 98");
                      DLPFScale = 98;
                      EEPROM.put(PARAM_START_ADDR + 11, DLPFScale);
                      EEPROM.commit();
                      Serial.println("EEPROM Commited");
                      break;
                    case '6':
                      mpu.setDLPFMode(MPU6050_IMU::MPU6050_DLPF_BW_188);
                      Serial.println("Set DLPF 188");
                      DLPFScale = 188;
                      EEPROM.put(PARAM_START_ADDR + 11, DLPFScale);
                      EEPROM.commit();
                      Serial.println("EEPROM Commited");
                      break;
                    case '7':
                      mpu.setDLPFMode(MPU6050_IMU::MPU6050_DLPF_BW_256);
                      Serial.println("Set DLPF 256");
                      DLPFScale = 256;
                      EEPROM.put(PARAM_START_ADDR + 11, DLPFScale);
                      EEPROM.commit();
                      Serial.println("EEPROM Commited");
                      break;
                    default:
                      Serial.print(" L to Leave ");
                      delay(500);
                      break;
                  }
                  
                }
                break;
              case 'C':
                Serial.println("Calibration Settings: Pick What You Would Like To Change");
                Serial.print("Last Surface Offset Value");
                Serial.println(SurfAlt);
                Serial.println("1 - Calib Delay");
                Serial.println("2 - Calib Amount");
                Serial.println("L - Leave");
                while (chadmode == true)
                {
                  char command = Serial.read();
                  if (command == 'L')
                  {
                    break;
                  }
                  switch (command)
                  {
                    case '1':
                      
                      Serial.println("100 ms Delay is taken between Values, Please look into code to change this");
                      return;
                    case '2':
                      
                      Serial.println("10 Values are taken for Avg, Please look into code to change this");     
                      return;                
                    default:
                      Serial.print(" L to Leave ");
                      delay(500);
                      break;
                  }
                }
                
                  
                
                break;
              
              default:
                Serial.print(" # to Leave ");
                delay(500);
                break;
            }            
          }
            
        }
      
        
        else 
        {
          Serial.print("Turn On Dev Mode");
          return;
        }
      }
    
  

      
      else if (command == '%')
      {
        if (chadmode == false)
        {
          Serial.println("Activating Dev Mode");
          chadmode = true;
          return;
        }
        else 
        {
          Serial.println("Deactivating Dev Mode");
          chadmode = false;
          return;
        }
      }
      else if (command == '*')
      {
        if (p == 0)
        {
          Serial.println("Parachute ARMED");
          p = p+1;
          return;
        }
        else if (p == 1)
        {
          Serial.println("Parachute DISARMED");
          p = p-1;
          return;
        }
      }
      else if (command == 'z') {
      Serial.println("Restarting ESP");
      ESP.restart();
      }
    }
  }
  
void calibrate()
{
  digitalWrite(LED_PIN, LOW);
  Serial.println("Calibration Started (Please Keep On Level Surface)");
  Serial.print("Mean Values : ");
  Serial.print(caliamount);
  Serial.print(" Delay: ");
  Serial.print(calidelay);
  Serial.println();
  float calialtiude[caliamount];

  for (int o = 0; o < caliamount; o++) {
  calialtiude[o] = bmp.readAltitude();
  Serial.print("Altitude Recorded: ");
  Serial.println(calialtiude[o]);
  delay(calidelay);
}
  for (int o = 0; o < caliamount; o++) 
  {
  calisum += calialtiude[o];
  }
  SurfAlt = calisum / caliamount;

  Serial.print("SurfaceOffset: ");
  Serial.println(SurfAlt);
  Serial.println("Writing Offcet to EEPROM ");
  EEPROM.put(PARAM_START_ADDR + 27, int(SurfAlt));
  EEPROM.commit();
  mpu.CalibrateGyro(6);
  Serial.println();
  mpu.CalibrateAccel(6);
  Serial.println();
  mpu.PrintActiveOffsets();
  Serial.println();
  Serial.println("Calibration End");
  digitalWrite(LED_PIN, HIGH);
  return;
}
void logging(){   
  digitalWrite(LED_PIN, HIGH);
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float temp = bmp.readTemperature();
  float pressure = bmp.readPressure();
  float altitude = bmp.readAltitude();
  float realaltitude = bmp.readAltitude(101500);
  float heading = Compass.GetHeadingDegrees();
  if (dataFile) {
    if (altitude <= 20 && p == 1)
    {
      paraservo.write(50);
      p=p+1;
    }
    dataFile.print(altitude - SurfAlt);
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
    Serial.print("data logged altitude: ");
    Serial.println(altitude - SurfAlt);
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
 
