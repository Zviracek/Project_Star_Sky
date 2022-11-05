/*
  For anyone insane enough to read this.
  This code is huge mess. Don't judge me. Or you know what, do.
  There are many unused parts commented out. Despite being unused, they serve a purpouse. At least sometimes.
  Many of the commented lines are bits, that have worked in past, and are replaced with new, not properly tested, code.
  There will most certainly be many bugs. I will be glad if you decide to adopt parts of my code, but keep in mind,
  that this is delicate piece of software in a very unstable enviroment. I do not guarantee that anything will work outside 
  this instance of software, and even inside of it. So if you crash precious payload, crew or entire space station because of me,
  don't blame me, you have been warned.
  Happy flying
*/

/*
    LIBRARIES
*/
// Servo control inclusion
#include <Servo.h>

// Comunication protocol inclusion
#include "Wire.h"
#include <SPI.h>
#include <SoftwareSerial.h>

// FLASH chip library inclusion
// #include <SPIMemory.h>

// SD card library inclusion
// #include <SD.h>

/*
    MACROS
*/
// Macros for pins
#define LED_R 23
#define LED_G 22
#define LED_B 19

#define PYRO_1 3
#define PYRO_2 9
#define PYRO_3 21
#define PYRO_4 16

#define CS_FLASH 39
#define CS_IMU 10
#define CS_BME 38
#define CS_BREAKOUT 0

#define GPS_RX 29
#define GPS_TX 28

#define BL_RX 35
#define BL_TX 34

#define SERVO_X 36
#define SERVO_Z 33

// definiton of flight states
#define GroundIdle 1
#define Countdown 2
#define Launch 3
#define Flight 4
#define Return 5
#define Landed 6
#define LaunchAbort 7
#define FlightAbort 8

// MPU6050 macros
#define MPU6050_Addr 0x68
#define G 9.80665

#define GYRO_FULL_SCALE_250_DPS  0x00
#define GYRO_FULL_SCALE_500_DPS  0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define ACC_FULL_SCALE_2G  0x00
#define ACC_FULL_SCALE_4G  0x08
#define ACC_FULL_SCALE_8G  0x10
#define ACC_FULL_SCALE_16G 0x18

// Activating certain parts of code
#define IMUI2C

/*
    VARIABLES
*/
// FlightState global int
// Should be 1 (ground idle) at default, now is set to 4(flight)
int flightState = 4;

// Base angle for servo
int targetAngleX = 90;
int targetAngleZ = 90;

// PID setup for servos
float ServoP = 10.0, ServoI = 0.5, ServoD = 0.5;

float ServoXError, ServoZError;
float ServoXPID, ServoZPID;

// Time references for countdown
int countdownTimer = 120; // in seconds
int countdownEndTime;
int lastCountdownTime;
int countdownTime;
int countdownHoldTime;
bool countdownPause = false;

int lastIMUTime;
int launchTime;

// Initialization states of certain systems
bool flashInitSuccesfull = false;
bool IMUI2CInitSuccesfull = false;

// Macros and object for IMU I2C bus if external sensor is used, unactive when using onboard sensor!
// *****IDK IN WHAT STATE THIS IS*****
#ifdef IMUI2C
  /*#define I2Cclock 400000
  #define I2Cport Wire
  #define MPU9250_ADDRESS MPU9250_ADDRESS_AD0 // I2C addres AD1 can also be used if needed*/

#else
// Macros and object for IMU SPI bus if onboard sensor is used
  // ***Block for SPI IMU***
#endif

// SPIFlash flash(CS_FLASH);

float roll_IMU, pitch_IMU, yaw_IMU;

int16_t acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z;
float norm_acc_x, norm_acc_y, norm_acc_z, norm_gyro_x, norm_gyro_y, norm_gyro_z, norm_mag_x, norm_mag_y, norm_mag_z;
int16_t tempRaw;
//  In future, I can add automatic calibration, but that would need better filters on the output to have some effect
// float acc_x_err, acc_y_err, acc_z_err, gyro_x_err, gyro_y_err, gyro_z_err, mag_x_err, mag_y_err, mag_z_err;

/*
    OBJECTS
*/
// Instances of servo objects
Servo servoX;
Servo servoZ;

// Instances of object using UART communication
SoftwareSerial bluetooth(BL_TX, BL_RX);
SoftwareSerial gps(GPS_TX, GPS_RX);

void setup() {
  Serial.begin(9600);
  
  // pin I/O setups
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  pinMode(PYRO_1, OUTPUT);
  pinMode(PYRO_2, OUTPUT);
  pinMode(PYRO_3, OUTPUT);
  pinMode(PYRO_4, OUTPUT);

  // servo setups
  servoX.attach(SERVO_X); 
  servoZ.attach(SERVO_Z); 

  // If using external IMU sensor inicilize I2C library
  #ifdef IMUI2C
    Wire.begin();
    Wire.setClock(400000);
    Wire.beginTransmission(MPU6050_Addr);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Data Output Rate = 1000 / (1 + SRD)
    // Output rate must be at least 2x DLPF rate
    I2CwriteByte(MPU6050_Addr, 25, 0x01); //  Set the SRD to 1
    // Set the DLPF to 184HZ by default
    I2CwriteByte(MPU6050_Addr, 26, 0x01);
    // Configure gyroscope and accelerometer scale
    I2CwriteByte(MPU6050_Addr, 27, GYRO_FULL_SCALE_1000_DPS);
    I2CwriteByte(MPU6050_Addr, 28, ACC_FULL_SCALE_2G);
    // Enable interrupt pin for raw data
    I2CwriteByte(MPU6050_Addr, 56, 0x01);
    
    IMUI2CInitSuccesfull = true; // this here will be overrided by false if unsuccesfull, better than adding else :)
  #else
  // If using onboard IMU sensor inicilize SPI library
    // TODO: ***Block for SPI IMU***
  #endif
  // flash.begin();
 
  bluetooth.begin(9600);
  bluetooth.println("We have a connection, POG");
  
  // gps.begin();

  // Set servos at default angle
  servoX.write(targetAngleX);
  servoZ.write(targetAngleZ);

}

void loop()
{
  // Check for current state: GroundIdle, Countdown, Launch, Flight, Return, Landed, LaunchAbort, FlightAbort
  switch(flightState)
  {
    case 1: // Ground Idle - sensor logging directly to SD, wating for launch signal
      GroundIdleState();
      break;

    case 2: // Countdown - Countdown, SD card logging suspended, using FLASH instead
      CountdownState();
      break;

    case 3: // Launch - first few seconds of the flight checking for flight nominality, if launch is not detected, switch to LaunchAbort
      LaunchState();
      break;

    case 4: // Flight - stabilization online, complete loging, chechks for flight nominality or return state conditions, if flight is not nominal, switch to FlightAbort
      FlightState();
      break;

    case 5: // Return - MECO, deploy chutes at safe atitude, wait for landing
      ReturnState();
      break;

    case 6: // Landed - landing detected, transfer data from flash to SD
      LandedState();
      break;

    case 7: // LaunchAbort - error at countdown -> stop countdown, transfer data from flash to SD, add error log to SD
      LaunchAbortState();
      break;

    case 8: // FlightAbort - jiggle engine vector to bleed as much power as possible, extended logging to flash, special chute conditions, wait for landing
      FlightAbortState();
      break;  
  }
 
  readIMU();
  
  // Servos PID
  // ServoXError = targetAngleX - SERVO_X.read();
  // ServoZError = targetAngleZ - SERVO_Z.read();

  targetAngleX = ServoXError;
  // Set servos ***I dont know where I should put it yet***
  writeServos(targetAngleX, targetAngleZ);
}

// Functions for each state
void GroundIdleState()
{
  // read bluetooth for launch signal
  byte DATA;
  if(bluetooth.available() > 0)
  {
    DATA = bluetooth.read();

    switch (DATA) 
    {
      case '1': // Start countdown
        flightState = Countdown;
        countdownEndTime = millis() + countdownTimer * 1000; 
        bluetooth.println("Countdown started");
        break;

      case '2': // Get status
      
        break;
    }
  }
}

void CountdownState()
{
  if(countdownTime/1000 < 10)
  // time reference update
  countdownTime = countdownEndTime - millis();

  // TODO: send remaining time troughout bluetooth
  if(lastCountdownTime/1000 != countdownTime/1000)
  {
    lastCountdownTime = countdownTime;
    bluetooth.println(countdownTime/1000);
  }
    

  if(millis() >= countdownEndTime) // countdown over, proceed to launch
  {
    flightState = Launch;
    // TODO: log this info to flash memory
  }

  // read bluetooth for launch abort signal
  // TODO: if bluetooth disconnects, we want to hold countdown
  byte DATA;
  if(bluetooth.available() > 0)
  {
    DATA = bluetooth.read();

    switch (DATA) 
    {
      case '0': // stop countdown, abort launch
        flightState = LaunchAbort;
        break;

      case '2': // skip countdown
        flightState = Launch;
        break;

      case '3': // pause countdown
        if(!countdownPause)
        {
          countdownHoldTime = countdownEndTime - millis();
          countdownPause = true;
        }
        countdownEndTime = millis() + 20000000; // Around 5 and half hour, should be enought :)
        break;

      case '4': // resume countdown
        if(countdownPause)
        {
          countdownEndTime = millis() + countdownHoldTime;
          countdownPause = false;
        }
        break;
    }
  
  }
}

void LaunchState()
{
  digitalWrite(PYRO_2, HIGH); // pyro_1 should always be first stage rocket motor NOT TRUE ANYMORE LOL
  bluetooth.println("And off we go");
  // TODO: check if launch is nominal
  flightState = Flight;
  launchTime = millis();
}

void FlightState()
{
  digitalWrite(PYRO_2, LOW); // despite pyro charge should no longer condct, for safety reasons we want to switch pyro off
  if(millis() > launchTime + 7000) // TODO: apogee condition
  {
    // flightState = Return;
  }
  readIMU();
  // printIMUData();
}
 
void ReturnState()
{
  if(true) // TODO: altitude condition
  {
    digitalWrite(PYRO_1, HIGH);
  }

  if(false) // TODO: backup charge condition
  {
    digitalWrite(PYRO_4, HIGH);
  }
  
}

void LandedState()
{

}

void LaunchAbortState()
{

}

void FlightAbortState()
{

}

// Pasive loops functions
void readIMU()
{
  #ifdef IMUI2C
    uint8_t buff[14];

    if(millis() > lastIMUTime + 10)
    {
      // Read output registers:
      // [59-64] Accelerometer
      // [65-66] Temperature
      // [67-72] Gyroscope
      I2Cread(MPU6050_Addr, 59, 14, buff);
      
      acc_x = (buff[0] << 8 | buff[1]);
      acc_y = (buff[2] << 8 | buff[3]);
      acc_z = (buff[4] << 8 | buff[5]);
      tempRaw = (buff[6] << 8 | buff[7]);
      gyro_x = (buff[8] << 8 | buff[9]);
      gyro_y = (buff[10] << 8 | buff[11]);
      gyro_z = (buff[12] << 8 | buff[13]);

      // Normalization, not shure if numbers are correct
      // Switched y for -z axis because of vertical orientation of the sensor
      norm_gyro_x = gyro_x / 32.8;
      norm_gyro_y = -gyro_z / 32.8;
      norm_gyro_z  = gyro_y / 32.8;

      norm_acc_x = acc_x * G / 16384;
      norm_acc_y = -acc_z * G/ 16384;
      norm_acc_z = acc_y * G/ 16384;
      
      float a_x = atan(norm_acc_y / sqrt(sq(norm_acc_x) + sq(norm_acc_z)));
      float a_y = atan(-1 * norm_acc_x / sqrt(sq(norm_acc_y) + sq(norm_acc_z)));
      float a_z = atan2(a_y, a_x);

      float dt = (millis() - lastIMUTime);
      float g_x = norm_gyro_x * dt / 100000;
      float g_y = norm_gyro_y * dt / 100000;
      float g_z = norm_gyro_z * dt / 100000;

      pitch_IMU = 0.98 * (pitch_IMU + degrees(g_x)) + 0.02 * degrees(a_x);
      roll_IMU = 0.98 * (roll_IMU + degrees(g_y)) + 0.02 * degrees(a_y);
      
      // Serial.println(yaw_IMU);
      Serial.print(pitch_IMU);
      Serial.print(" ");
      Serial.print(roll_IMU);
      Serial.println();

      lastIMUTime = millis();
    }
  #endif
}

void printIMUData()
{
  // This should be done better
  // Raw data for further analysis
  
  Serial.print(acc_x,6);
  Serial.print("\t");
  Serial.print(acc_y,6);
  Serial.print("\t");
  Serial.print(acc_z,6);
  Serial.print("\t");
  Serial.print(gyro_x,6);
  Serial.print("\t");
  Serial.print(gyro_y,6);
  Serial.print("\t");
  Serial.print(gyro_z,6);
  Serial.print("\t");

  /*
  bluetooth.print(acc_x,6);
  bluetooth.print("\t");
  bluetooth.print(acc_y,6);
  bluetooth.print("\t");
  bluetooth.print(acc_z,6);
  bluetooth.print("\t");
  bluetooth.print(gyro_x,6);
  bluetooth.print("\t");
  bluetooth.print(gyro_y,6);
  bluetooth.print("\t");
  bluetooth.print(gyro_z,6);
  bluetooth.print("\t");
  */
}

void blink(int lenght, int collor)
{

}

void writeServos(int angleX, int angleZ)
{
  /*oldAngleX = servoX.read();
  oldAngleZ = servoZ.read();

  if(oldAngleX != angleX)
  {
    servoX.write(angleX);
  }

  if(oldAngleZ != angleZ)
  {
    servoZ.write(angleZ);
  }*/
}
// I wonder if anyone is able to read this mess :P
// Cant even believed someone reched the end
