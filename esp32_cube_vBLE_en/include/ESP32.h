#pragma once
#include <Arduino.h>

#define BUZZER      27
#define VBAT        34
#define INT_LED     39

#define BRAKE       26

#define DIR1        4
#define ENC1_1      35
#define ENC1_2      33
#define PWM1        32
#define PWM1_CH     1

#define DIR2        15
#define ENC2_1      13
#define ENC2_2      14
#define PWM2        25
#define PWM2_CH     0

#define DIR3        5
#define ENC3_1      16
#define ENC3_2      17
#define PWM3        18
#define PWM3_CH     2

#define TIMER_BIT   8
#define BASE_FREQ   20000

#define MPU6050       0x68   // Device address
#define ACCEL_CONFIG  0x1C   // Accelerometer configuration address
#define GYRO_CONFIG   0x1B   // Gyro configuration address
#define PWR_MGMT_1    0x6B
#define PWR_MGMT_2    0x6C

#define accSens 0            // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 0           // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s

#define EEPROM_SIZE   128

#define LED_PIN       23    // Pin that connects to WS2812B
#define NUM_PIXELS    3      // The number of LEDs (pixels) on WS2812B

extern float K1 ; 
extern float K2;
extern float K3;
extern float K4;
extern float zK2; 
extern float zK3; 
extern float eK1; 
extern float eK2;
extern float eK3; 
extern float eK4;

extern float Gyro_amount;

extern bool vertical_vertex;
extern bool vertical_edge;
extern bool calibrating;
extern bool vertex_calibrated;
extern bool calibrated;
extern bool calibrated_leds;

extern byte loop_time;

struct OffsetsObj {
  byte ID;
  float acXv; float acYv; float acZv; float acXe; float acYe; float acZe; 
  float K1;   float K2;   float K3;   float K4;   float zK2;  float zK3;
  float eK1;  float eK2;  float eK3;  float eK4;
  byte Red;   byte Green; byte Blue;
  byte vertexRed; byte vertexGreen; byte vertexBlue;
  byte edgeRed; byte edgeGreen; byte edgeBlue;
};
extern OffsetsObj offsets;

extern float alpha;

extern int16_t AcX;
extern int16_t AcY;
extern int16_t AcZ;
extern int16_t AcXc;
extern int16_t AcYc;
extern int16_t AcZc;
extern int16_t GyX;
extern int16_t GyY;
extern int16_t GyZ;
extern float gyroX;
extern float gyroY;
extern float gyroZ;
extern float gyroXfilt;
extern float gyroYfilt;
extern float gyroZfilt;
extern float speed_X;
extern float speed_Y;

extern int16_t GyZ_offset;
extern int16_t GyY_offset; 
extern int16_t GyX_offset;
extern int32_t GyZ_offset_sum;
extern int32_t GyY_offset_sum;
extern int32_t GyX_offset_sum;

extern float robot_angleX;
extern float robot_angleY;
extern float Acc_angleX; 
extern float Acc_angleY;      
extern int32_t motors_speed_X; 
extern int32_t motors_speed_Y; 
extern int32_t motors_speed_Z;   

extern long currentT;
extern long previousT_1;
extern long previousT_2;
extern long previousT_3;

extern volatile int  enc_count1;
extern volatile int enc_count2;
extern volatile int enc_count3;
extern int16_t motor1_speed;
extern int16_t motor2_speed;
extern int16_t motor3_speed; 

extern byte f;
extern word i;
extern byte j;
extern byte k;
extern bool lock;
extern byte Mode;   
extern bool Push;
extern bool LedRGB;
extern byte RGB_val;
extern bool ChooseColor;

extern byte Red;
extern byte Green;
extern byte Blue; 

extern byte vertexRed;
extern byte vertexGreen;
extern byte vertexBlue; 

extern byte edgeRed;
extern byte edgeGreen;
extern byte edgeBlue; 
 
extern String s1;

extern bool ConnectBT;        
extern word DelayBT;

extern char RGB_Color[18][3];