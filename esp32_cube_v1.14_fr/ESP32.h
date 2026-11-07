#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <BluetoothSerial.h>
#include <FastLED.h>

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

#define EEPROM_SIZE    128

#define LED_PIN         23      // Pin that connects to WS2812B
#define NUM_PIXELS      12      // The total number of LEDs (pixels) WS2812B 
#define NUM_PIX_MOT      4      // The numbers of LEDS (pixels) per motors
#define LED_LUX         50      // Brightness of LEDS (10 to 250)

float K1 = 180.00, K2 = 30.00, K3 = 1.60, K4 = 0.008, zK2 = 8.00, zK3 = 0.30; 
float eK1 = 190.00, eK2 = 31.00, eK3 = 2.50, eK4 = 0.014;

float Gyro_amount = 0.996;

bool vertical_vertex = false;
bool vertical_edge = false;

const bool VERTEX_ROTATION_ENABLED = true; // true pour faire tourner le cube automatiquement en vertex

byte edge_motor = 0;
byte edge_candidate = 0;
uint32_t edge_candidate_since = 0;
struct EdgeAxis { float x; float y; };
const EdgeAxis EDGE_AXES[4] = {
  {0.0f, 0.0f}, {0.464f, 0.886f}, {0.556f, -0.831f}, {-1.0f, 0.0f}
};
const uint32_t EDGE_CONFIRM_MS = 200;
const float EDGE_ARM_DEG = 0.3f; 
const float EDGE_CUTOFF_DEG = 7.0f; 

bool calibrating = false;
bool vertex_calibrated = false;
bool calibrated = false;
bool calibrated_leds = false;

byte loop_time = 15;

struct OffsetsObj {
  byte ID;
  float acXv; float acYv; float acZv; float acXe; float acYe; float acZe; 
  float K1;   float K2;   float K3;   float K4;   float zK2;  float zK3;
  float eK1;  float eK2;  float eK3;  float eK4;
  byte Red;   byte Green; byte Blue;  byte RGB_val;
};
OffsetsObj offsets;

float alpha = 0.7;

int16_t  AcX, AcY, AcZ, AcXc, AcYc, AcZc, GyX, GyY, GyZ;
float gyroX, gyroY, gyroZ, gyroXfilt, gyroYfilt, gyroZfilt;
float speed_X, speed_Y;

int16_t  GyZ_offset = 0; int16_t  GyY_offset = 0; int16_t  GyX_offset = 0;
int32_t  GyZ_offset_sum = 0; int32_t  GyY_offset_sum = 0; int32_t  GyX_offset_sum = 0;

float robot_angleX, robot_angleY;
float Acc_angleX, Acc_angleY;      
int32_t motors_speed_X; int32_t motors_speed_Y; int32_t motors_speed_Z;   

long currentT, previousT_1, previousT_2, previousT_3;

volatile int  enc_count1 = 0, enc_count2 = 0, enc_count3 = 0; 
int16_t motor1_speed; int16_t motor2_speed; int16_t motor3_speed; 

byte f; word i; byte j; byte k; bool lock; byte Mode;   
bool Push; bool LedRGB; byte RGB_val;  bool ChooseColor; 
byte Red = LED_LUX, Green = LED_LUX, Blue = 0;

String s1 = "";

bool ConnectBT = false;        
word DelayBT = 1500;

char RGB_Color[18][3] = {
  {LED_LUX, LED_LUX,   0}, {LED_LUX*0.7, LED_LUX,   0}, 
  { LED_LUX*0.3, LED_LUX,   0}, {  0, LED_LUX,   0}, 
  {  0, LED_LUX,  LED_LUX*0.3}, {  0, LED_LUX, LED_LUX*0.7}, 
  {  0, LED_LUX, LED_LUX}, {  0, LED_LUX*0.7, LED_LUX}, 
  {  0,  LED_LUX*0.3, LED_LUX}, {  0,   0, LED_LUX}, 
  { LED_LUX*0.3,   0, LED_LUX}, {LED_LUX*0.7,   0, LED_LUX}, 
  {LED_LUX,   0, LED_LUX}, {LED_LUX,   0, LED_LUX*0.7}, 
  {LED_LUX,   0,  LED_LUX*0.3}, {LED_LUX,   0,   0},
  {LED_LUX,  LED_LUX*0.3,   0}, {LED_LUX, LED_LUX*0.7,   0}
};

BluetoothSerial SerialBT;
CRGB leds[NUM_PIXELS];
