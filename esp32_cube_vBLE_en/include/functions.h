
#pragma once
#include "ESP32.h"
#include "pitches.h"
#include <FastLED.h>
#include "BluetoothSerial.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SETTINGS_K1 '1'
#define SETTINGS_K2 '2'
#define SETTINGS_K3 '3'
#define SETTINGS_K4 '4'
#define SETTINGS_ZK2 '5'
#define SETTINGS_ZK3 '6'
#define SETTINGS_EK1 '7'
#define SETTINGS_EK2 '8'
#define SETTINGS_EK3 '9'
#define SETTINGS_EK4 '0'
#define SETTINGS_PLUS '+'
#define SETTINGS_MINUS '-'
#define SETTINGS_QUERY 'q'
#define SETTINGS_SAVE_VERTEX 'v'
#define SETTINGS_SAVE_EDGE 'e'

extern BLECharacteristic* pSensorCharacteristic;
extern BLECharacteristic* pLedCharacteristic; 
extern BluetoothSerial SerialBT;
extern CRGB leds[NUM_PIXELS];
extern bool play;


extern long lasttime;
extern bool deviceConnected;
extern bool oldDeviceConnected;
extern void colorLed(int Red, int Green, int Blue , int firstled, int lastled, int wait);
extern void save();
extern void angle_setup();
extern void angle_calc();
extern void XYZ_to_threeWay(float pwm_X, float pwm_Y, float pwm_Z);
extern void threeWay_to_XY(int in_speed1, int in_speed2, int in_speed3);
extern void battVoltage(double voltage);
extern void pwmSet(uint8_t channel, uint32_t value);
extern void Motor1_control(int sp);
extern void Motor2_control(int sp);
extern void Motor3_control(int sp);
extern void ENC1_READ();
extern void ENC2_READ();
extern void ENC3_READ();
extern void RGB_Process();
extern void beep();
extern int Tuning();
extern void init_BT();
extern void testMotors();
extern void calibrate_edge();
extern void calibrate_vertex();
extern void getSettings();
extern void setSettings(char s, char c);
extern void saveSettings(char t);
extern void balancing(String type);
extern void notBalancing(); 
extern void setLeds(char t, int r, int g, int b);
extern void musicLoop(void* pvParameters);
extern void saveMotors(int m1, int m2, int m3);