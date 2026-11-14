

// SIMPLE USER SETTINGS

// true: French messages; false: English messages (also changes the app language)
const bool SERIAL_FRENCH = false; 

// true: play a short LED animation when the cube starts
const bool START_ANIM_LEDS = true; 

// true: rotation enabled by default; false: rotation disabled by default
// It will be possible to reverse this setting when the cube is turned on by holding the Del button to the beep.
const bool VERTEX_AUTO_SPIN = true;

// ------------------------------------------------------------

// Libraries used by the program.
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <BluetoothSerial.h>
#include <FastLED.h>

// Pin assignments for the buzzer, battery, and button.
#define BUZZER      27
#define VBAT        34
#define INT_LED     39

// Motor brake control.
#define BRAKE       26

// Motor 1 connections: direction, encoder, and power.
#define DIR1        4
#define ENC1_1      35
#define ENC1_2      33
#define PWM1        32
#define PWM1_CH     1

// Motor 2 connections.
#define DIR2        15
#define ENC2_1      13
#define ENC2_2      14
#define PWM2        25
#define PWM2_CH     0

// Motor 3 connections.
#define DIR3        5
#define ENC3_1      16
#define ENC3_2      17
#define PWM3        18
#define PWM3_CH     2

// Motor control signal settings.
#define TIMER_BIT   8
#define BASE_FREQ   20000

// MPU6050 motion sensor addresses and settings.
#define MPU6050       0x68   // Sensor address
#define ACCEL_CONFIG  0x1C   // Accelerometer configuration
#define GYRO_CONFIG   0x1B   // Gyroscope configuration
#define PWR_MGMT_1    0x6B

#define accSens 0            // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 0           // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s

// Reserved space for saved settings.
#define EEPROM_SIZE    128

// Settings for 12 LEDs arranged in three groups of four.
#define LED_PIN         23      // LED pin
#define NUM_PIXELS      12      // Total LED count 
#define NUM_PIX_MOT      4      // LEDs per motor
#define LED_LUX         50      // LED brightness (10 to 250)

// Objects used to manage Bluetooth and LEDs.
BluetoothSerial SerialBT;
CRGB leds[NUM_PIXELS];

// Balance settings for a corner (Vertex).
// These values are already tuned for this cube.
float K1 = 180.00, K2 = 30.00, K3 = 1.60, K4 = 0.008, zK2 = 8.00, zK3 = 0.30; 
// Balance settings for an edge (Edge).
float eK1 = 190.00, eK2 = 31.00, eK3 = 2.50, eK4 = 0.014;

// Automatic Vertex rotation: speed, gradual start, and safety conditions.
const float VERTEX_SPIN_SPEED = 5.0f;
const float VERTEX_SPIN_RAMP = 0.03f;
const float VERTEX_SPIN_MAX_TILT = 2.5f;
const float VERTEX_SPIN_MAX_GYRO = 12.0f;
const unsigned long VERTEX_SPIN_DELAY = 5000;
unsigned long vertexSpinStableSince = 0;
float vertexSpinTarget = 0.0f;
bool vertexAutoSpinEnabled = VERTEX_AUTO_SPIN;

// Corrections specific to automatic rotation.
// They are independent of the usual balance settings.
const float VERTEX_SPIN_KP = 8.0f;       // Automatic rotation speed correction
const float VERTEX_SPIN_KI = 0.4f;       // Integral correction to maintain rotation
const float VERTEX_SPIN_I_MAX = 20.0f;   // Limit for this correction
float vertexSpinIntegral = 0.0f;

// Sensor-based tilt calculation setting.
float Gyro_amount = 0.996;

// Indicates whether the cube is balancing on a corner or an edge.
bool vertical_vertex = false;
bool vertical_edge = false;

// Identifies the supporting edge and confirms its position before activation.
byte edge_motor = 0;
byte edge_candidate = 0;
uint32_t edge_candidate_since = 0;
struct EdgeAxis { float x; float y; };
const EdgeAxis EDGE_AXES[4] = {
  {0.0f, 0.0f}, {0.464f, 0.886f}, {0.556f, -0.831f}, {-1.0f, 0.0f}
};
const char EDGE_LABELS[4] = {'?', 'B', 'C', 'A'};
const uint32_t EDGE_CONFIRM_MS = 200;
const float EDGE_ARM_DEG = 0.3f; 
const float EDGE_CUTOFF_DEG = 7.0f; 

// Tracks balance position calibration.
bool calibrating = false;
bool vertex_calibrated = false;
bool calibrated = false;
bool calibrated_leds = false;

// Interval between balance calculations (milliseconds).
byte loop_time = 15;

// Groups settings and measurements to retain in memory.
struct OffsetsObj {
  byte ID;
  float acXv; float acYv; float acZv; float acXe; float acYe; float acZe; 
  float K1;   float K2;   float K3;   float K4;   float zK2;  float zK3;
  float eK1;  float eK2;  float eK3;  float eK4;
  byte Red;   byte Green; byte Blue;  byte RGB_val;
};
OffsetsObj offsets;

// Variables for sensor readings and balance calculations.
float alpha = 0.7;

int16_t  AcX, AcY, AcZ, AcXc, AcYc, AcZc, GyX, GyY, GyZ;
float gyroX, gyroY, gyroZ, gyroXfilt, gyroYfilt;
float speed_X, speed_Y;

int16_t  GyZ_offset = 0; int16_t  GyY_offset = 0; int16_t  GyX_offset = 0;
int32_t  GyZ_offset_sum = 0; int32_t  GyY_offset_sum = 0; int32_t  GyX_offset_sum = 0;

float robot_angleX, robot_angleY;
float Acc_angleX, Acc_angleY;      
int32_t motors_speed_X; int32_t motors_speed_Y; int32_t motors_speed_Z;   

// Tracks timing for program actions.
long currentT, previousT_1, previousT_2, previousT_3;

// Encoder counters for measuring wheel movement.
volatile int  enc_count1 = 0, enc_count2 = 0, enc_count3 = 0; 
int16_t motor1_speed; int16_t motor2_speed; int16_t motor3_speed; 

// Variables used for modes, buttons, and LEDs.
byte f; word i; byte j; word k; bool lock; byte Mode;   
bool Push; bool LedRGB; byte RGB_val;  bool ChooseColor; 
byte Red = LED_LUX, Green = LED_LUX, Blue = 0;

// Temporary text used for Bluetooth messages.
String s1 = "";

// Controls Bluetooth connection and commands.
bool ConnectBT = false;        
word DelayBT = 1500;

// List of available LED colors.
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
