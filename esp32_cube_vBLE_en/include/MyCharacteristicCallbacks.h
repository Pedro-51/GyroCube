
#pragma once
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <FastLED.h>
#include "BluetoothSerial.h"
#include "ESP32.h"

#define TEST_MOTORS  '1'
#define CALIBRATE_VERTEX  '2'
#define CALIBRATE_EDGE  '3'
#define GET_SETTINGS  '4'
#define SET_SETTINGS '5'
#define SAVE_SETTINGS '6'
#define SET_LED '7'

extern BLECharacteristic* pSensorCharacteristic;
extern BLECharacteristic* pLedCharacteristic;


class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
    using m_cb = void (*)(); //alias function pointer
    using m_cb1 = void (*)(char); //alias function pointer 1 int param
    using m_cb2 = void (*)(char, char); //alias function pointer 2 int param
    using m_cb3 = void (*)( char, int, int, int ); //alias function pointer 3 byte param
  private:
    m_cb tm_cb;
    m_cb ce_cb;
    m_cb cv_cb;
    m_cb s_cb;
    m_cb2 ss_cb;
    m_cb1 sv_cb;
    m_cb3 sl_cb;
    int v1, v2, v3;
  public:
    MyCharacteristicCallbacks(m_cb test_motor_callback, m_cb calibrate_edge, m_cb calibrate_vertex, m_cb get_settings, m_cb2 set_setting, m_cb1 save_settings, m_cb3 set_leds){
      tm_cb = test_motor_callback;
      ce_cb = calibrate_edge;
      cv_cb = calibrate_vertex;
      s_cb = get_settings;
      ss_cb = set_setting;
      sv_cb = save_settings;
      sl_cb = set_leds;
    }
  int ahex2int(char a, char b){

    a = (a <= '9') ? a - '0' : (a & 0x7) + 9;
    b = (b <= '9') ? b - '0' : (b & 0x7) + 9;

    return (a << 4) + b;
}
    void onWrite(BLECharacteristic* pLedCharacteristic) {
      std::__cxx11::string value = pLedCharacteristic->getValue();
      if (value.length() > 0) {
        uint16_t receivedValue = static_cast<uint16_t>(value[0]);
        uint16_t receivedValue1 = static_cast<uint16_t>(value[1]);
        Serial.println("DA");
        Serial.println(char(receivedValue));
        Serial.println(char(receivedValue1));
        switch (char(receivedValue))
        {
        case TEST_MOTORS:
          tm_cb();
          break;
        case CALIBRATE_EDGE:
          ce_cb();
          break;
        case CALIBRATE_VERTEX:
          cv_cb();
          break;
        case GET_SETTINGS:
          s_cb();
        Serial.println("GS");
          break;
        case SET_SETTINGS:
            Serial.println("setSetting");
              v2 = value[1];
              v3 = value[2];
              ss_cb(char(v2),char(v3));
          break;
        case SAVE_SETTINGS:
            Serial.println("saveSetting");
          sv_cb(char(value[1]));
          break;
        case SET_LED:
          int t = value[1];
          int g = ahex2int(value[2], value[3]);
          int r = ahex2int(value[4], value[5]);
          int b = ahex2int(value[6], value[7]);
          sl_cb(t,r,g,b);
          break;
        }
      }
    }
  };