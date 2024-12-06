#include <Arduino.h>
#include "ESP32.h"

float K1 = 140.00, K2 = 15.00, K3 = 1.10, K4 = 0.012, zK2 = 8.00, zK3 = 0.30; 
float eK1 = 150.00, eK2 = 16.00, eK3 = 2.60, eK4 = 0.004;

float Gyro_amount = 0.996;

bool vertical_vertex = false;
bool vertical_edge = false;
bool calibrating = false;
bool vertex_calibrated = false;
bool calibrated = false;
bool calibrated_leds = false;

int motors[3][5] = {
  4,35,33,32,1,
  15,13,14,25,0, 
  5,16,17,18,2
};

byte loop_time = 15;

float alpha = 0.7;


int16_t  GyZ_offset = 0; 
int16_t  GyY_offset = 0; 
int16_t  GyX_offset = 0;
int32_t  GyZ_offset_sum = 0; 
int32_t  GyY_offset_sum = 0; 
int32_t  GyX_offset_sum = 0;

volatile int enc_count1 = 0;
volatile int enc_count2 = 0;
volatile int enc_count3 = 0; 

byte Red = 255, Green = 255, Blue = 0; 
byte edgeRed = 255, edgeGreen = 255, edgeBlue = 255; 
byte vertexRed = 255, vertexGreen = 255, vertexBlue = 255; 
 
String s1 = "";

bool ConnectBT = false;        
word DelayBT = 1500;

char RGB_Color[18][3] = {
  {255, 255,   0}, {170, 255,   0}, 
  { 85, 255,   0}, {  0, 255,   0}, 
  {  0, 255,  85}, {  0, 255, 170}, 
  {  0, 255, 255}, {  0, 170, 255}, 
  {  0,  85, 255}, {  0,   0, 255}, 
  { 85,   0, 255}, {170,   0, 255}, 
  {255,   0, 255}, {255,   0, 170}, 
  {255,   0,  85}, {255,   0,   0},
  {255,  85,   0}, {255, 170,   0}
};
