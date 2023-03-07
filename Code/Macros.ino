/*
    File for macros
    Separated for cleaner code
    This is possible due to way how arduino manages .ino files
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