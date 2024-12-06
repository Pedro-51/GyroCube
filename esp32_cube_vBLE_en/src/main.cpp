#include <MyCharacteristicCallbacks.h>
#include <MyServerCallbacks.h>
#include <ESP32.h>
#include <functions.h>
#include "BluetoothSerial.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <EEPROM.h>

/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp32-web-bluetooth/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

BLEServer* pServer = NULL;
uint32_t value = 0;
TaskHandle_t Task1;
TaskHandle_t Task2;
BLECharacteristic* pSensorCharacteristic;
BLECharacteristic* pLedCharacteristic; 
float gyroX;
float gyroY;
float gyroZ;

float gyroYfilt;
float gyroZfilt;
float gyroXfilt;
word i;
int ledPin = 2; // Use the appropriate GPIO pin for your setup

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define COM_CHARACTERISTIC_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

bool play=false;

void setup() {
  Serial.begin(115200);    
  EEPROM.begin(EEPROM_SIZE);
  pinMode(ledPin, OUTPUT);
  // Create the BLE Device
  BLEDevice::init("Balance Cube");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pSensorCharacteristic = pService->createCharacteristic(
                      SENSOR_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // Create the ON button Characteristic
  pLedCharacteristic = pService->createCharacteristic(
                      COM_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );
//   // Register the callback for the ON button characteristic
  pLedCharacteristic->setCallbacks(new MyCharacteristicCallbacks(testMotors, calibrate_edge, calibrate_vertex, getSettings, setSettings, saveSettings, setLeds, saveMotors));
  
  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pSensorCharacteristic->addDescriptor(new BLE2902());
  pLedCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");

  FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, NUM_PIXELS);  // GRB ordering is typical
  
  pinMode(BUZZER, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);
  pinMode(INT_LED, INPUT);
  for (i=0;i<250;i+=10){ colorLed(i, i, 0, 1, 3, 5);}
  for (RGB_val=0;RGB_val<18;RGB_val++){
    Red = RGB_Color[RGB_val][0]; Green = RGB_Color[RGB_val][1]; Blue  = RGB_Color[RGB_val][2];
    colorLed(Red, Green, Blue, 1, 3, 80); //Red, Green, Blue, FirstPixel, LastPixel, Delay
  }
  for (i=250;i>0;i-=10){colorLed(i, i*0.68, 0, 1, 3, 5);}
  colorLed(0, 0, 0, 1, 3, 5);

  EEPROM.get(0, offsets);

  int m1=offsets.motor1;
  int m2=offsets.motor2;
  int m3=offsets.motor3;
  if(m1 < 0 || m1 > 3){
    m1=0;
    m2=1;
    m3=2;
    offsets.motor1 = 0;
    offsets.motor2 = 1;
    offsets.motor3 = 2;
    Serial.println("reseting pins");
    save();
  }
    Serial.println(m1);
    Serial.println(m2);
    Serial.println(m3);
  DIR1      = motors[m1][0];
  ENC1_1    = motors[m1][1];
  ENC1_2    = motors[m1][2];
  PWM1      = motors[m1][3];
  PWM1_CH   = motors[m1][4];

  DIR2      = motors[m2][0];
  ENC2_1    = motors[m2][1];
  ENC2_2    = motors[m2][2];
  PWM2      = motors[m2][3];
  PWM2_CH   = motors[m2][4];

  DIR3      = motors[m3][0];
  ENC3_1    = motors[m3][1];
  ENC3_2    = motors[m3][2];
  PWM3      = motors[m3][3];
  PWM3_CH   = motors[m3][4];

  pinMode(DIR1, OUTPUT);
  pinMode(ENC1_1, INPUT);
  pinMode(ENC1_2, INPUT);
  attachInterrupt(ENC1_1, ENC1_READ, CHANGE);
  attachInterrupt(ENC1_2, ENC1_READ, CHANGE);
  ledcSetup(PWM1_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM1, PWM1_CH);
  Motor1_control(0);
  
  pinMode(DIR2, OUTPUT);
  pinMode(ENC2_1, INPUT);
  pinMode(ENC2_2, INPUT);
  attachInterrupt(ENC2_1, ENC2_READ, CHANGE);
  attachInterrupt(ENC2_2, ENC2_READ, CHANGE);
  ledcSetup(PWM2_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM2, PWM2_CH);
  Motor2_control(0);
  
  pinMode(DIR3, OUTPUT);
  pinMode(ENC3_1, INPUT);
  pinMode(ENC3_2, INPUT);
  attachInterrupt(ENC3_1, ENC3_READ, CHANGE);
  attachInterrupt(ENC3_2, ENC3_READ, CHANGE);
  ledcSetup(PWM3_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM3, PWM3_CH);
  Motor3_control(0);

  //offsets.ID = 0; // for debug
  if(offsets.ID == 255){
    offsets.ID = 0;
    offsets.Red = Red; offsets.Green = Green; offsets.Blue = Blue;
    offsets.edgeRed = edgeRed; offsets.edgeGreen = edgeGreen; offsets.edgeBlue = edgeBlue;
    offsets.vertexRed = vertexRed; offsets.vertexGreen = vertexGreen; offsets.vertexBlue = vertexBlue;
    offsets.K1 = K1; offsets.K2 = K2; offsets.K3 = K3; offsets.K4 = K4; offsets.zK2 = zK2; offsets.zK3 = zK3;
    offsets.eK1 = eK1; offsets.eK2 = eK2; offsets.eK3 = eK3; offsets.eK4 = eK4;
    save();
  }else if (offsets.ID == 96){
    Red = offsets.Red; Green = offsets.Green; Blue = offsets.Blue;
    vertexRed = offsets.vertexRed; vertexGreen = offsets.vertexGreen; vertexBlue = offsets.vertexBlue;
    edgeRed = offsets.edgeRed; edgeGreen = offsets.edgeGreen; edgeBlue = offsets.edgeBlue;
    K1 = offsets.K1; K2 = offsets.K2; K3 = offsets.K3; K4 = offsets.K4; zK2 = offsets.zK2; zK3 = offsets.zK3;
    eK1 = offsets.eK1; eK2 = offsets.eK2; eK3 = offsets.eK3; eK4 = offsets.eK4;
    calibrated = true;
  }
play=false;
  angle_setup();
  // xTaskCreatePinnedToCore (
  //   musicLoop,     // Function to implement the task
  //   "musicLoop",   // Name of the task
  //   10000,      // Stack size in bytes
  //   NULL,      // Task input parameter
  //   0,         // Priority of the task
  //   &Task1,      // Task handle.
  //   0          // Core where the task should run
  // );
}

void loop() {
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    Serial.println("Device disconnected.");
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
            pSensorCharacteristic->setValue("");
            pSensorCharacteristic->notify();
            lasttime = millis();
    Serial.println("Device Connected");
  }
  currentT = millis();
  if (currentT - previousT_1 >= loop_time) {
    Tuning();
    angle_calc();
        
    motor1_speed = enc_count1;
    enc_count1 = 0;
    motor2_speed = enc_count2;
    enc_count2 = 0;
    motor3_speed = enc_count3;
    enc_count3 = 0;
    

  if (Mode == 0){

    threeWay_to_XY(motor1_speed, motor2_speed, motor3_speed);
    motors_speed_Z = motor1_speed + motor2_speed + motor3_speed;
    
    if (vertical_vertex && calibrated && !calibrating) {    
      digitalWrite(BRAKE, HIGH);
      gyroX = GyX / 131.0;
      gyroY = GyY / 131.0;
      gyroZ = GyZ / 131.0;
      gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;
      gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt;
      
      int pwm_X = constrain(K1 * robot_angleX + K2 * gyroXfilt + K3 * speed_X + K4 * motors_speed_X, -255, 255);
      int pwm_Y = constrain(K1 * robot_angleY + K2 * gyroYfilt + K3 * speed_Y + K4 * motors_speed_Y, -255, 255);
      int pwm_Z = constrain(zK2 * gyroZ + zK3 * motors_speed_Z, -255, 255);

      motors_speed_X += speed_X / 5; 
      motors_speed_Y += speed_Y / 5;
      XYZ_to_threeWay(-pwm_X, pwm_Y, -pwm_Z);
      balancing("Vertex");
      colorLed(vertexRed, vertexGreen, vertexBlue, 1, 3, 0); 
    } else if (vertical_edge && calibrated && !calibrating) {
      digitalWrite(BRAKE, HIGH);
      gyroX = GyX / 131.0;
      gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;
      
      int pwm_X = constrain(eK1 * robot_angleX + eK2 * gyroXfilt + eK3 * motor3_speed + eK4 * motors_speed_X, -255, 255);
      
      motors_speed_X += motor3_speed / 5;
      Motor3_control(pwm_X);
      balancing("Edge");
      colorLed(edgeRed, edgeGreen, edgeBlue, 1, 3, 0); 
    } else {
      XYZ_to_threeWay(0, 0, 0);
      digitalWrite(BRAKE, LOW);
      motors_speed_X = 0;
      motors_speed_Y = 0; 
      colorLed(0, 0, 0, 1, 3, 0); 
      if  (!calibrating and calibrated) {
        RGB_Process();
      }
    }
  }

  if (Mode == 1 ){
      switch (f) {
        case 1:
          digitalWrite(BRAKE, HIGH);
          if(lock){
            Serial.println("Rotating motor 1.");
            SerialBT.println("46");
            pSensorCharacteristic->setValue(String("*m1*Rotating").c_str());
            pSensorCharacteristic->notify();
          }
          Motor1_control(50);
          break;
        case 4:
          digitalWrite(BRAKE, LOW);
          if(lock){
            Serial.println("Stop.");
            SerialBT.println("41");
            pSensorCharacteristic->setValue(String("*m1*Stop").c_str());
            pSensorCharacteristic->notify();
          }
          Motor1_control(0);
          break;      
        case 7:
          digitalWrite(BRAKE, HIGH);
          if(lock){
            Serial.println("Change direction..."); SerialBT.println("42");
            pSensorCharacteristic->setValue(String("*m1*Reversing.").c_str());
            pSensorCharacteristic->notify();
          }
          Motor1_control(-50);
          break;
        case 10:
          digitalWrite(BRAKE, LOW); 
          if(lock){
            Serial.println("Stop."); 
            SerialBT.println("41");
            pSensorCharacteristic->setValue(String("*m1*Stop").c_str());
            pSensorCharacteristic->notify();
          }
          Motor1_control(0);
          break; 
        case 13:
          digitalWrite(BRAKE, HIGH);
          if(lock){
            Serial.println("Checking encoder...");
            SerialBT.println("43");
            pSensorCharacteristic->setValue(String("*m1*Checking encoder.").c_str());
            pSensorCharacteristic->notify();
          }
          Motor1_control(70);
          break; 
        case 16:
          digitalWrite(BRAKE, LOW);
          if (lock && motor1_speed > 300) {
            Serial.println("Encoder OK."); 
            Serial.print("Speed: "); Serial.println(motor1_speed);
            char str[22];
            sprintf(str, "*m1*Speed: %u", motor1_speed);
            pSensorCharacteristic->setValue(str);
            pSensorCharacteristic->notify();
          Motor1_control(70);

            s1.concat(motor1_speed);
            SerialBT.println("44"  + s1);
            Serial.println("Stop.");
          } else if (lock && motor1_speed <= 0) {
            Serial.println("Encoder FAIL.");
            Serial.print("Speed: "); Serial.println(motor1_speed);
              char str[22];
              pSensorCharacteristic->setValue(String("*m1*Encoder Fail").c_str());
              pSensorCharacteristic->notify();
            s1.concat(motor1_speed);
            SerialBT.println("45"  + s1);
            Serial.println("Stop.");
          }
          s1 = "";
          Motor1_control(0);
          break;      
  
        case 19:
          digitalWrite(BRAKE, HIGH);
          if(lock){
            Serial.println("Rotating motor 2."); SerialBT.println("47");
            pSensorCharacteristic->setValue(String("*m2*Rotating.").c_str());
            pSensorCharacteristic->notify();
          }
          Motor2_control(50);
          break;
        case 22:
          digitalWrite(BRAKE, LOW);
          if(lock){
            Serial.println("Stop."); 
            SerialBT.println("41");
            pSensorCharacteristic->setValue(String("*m2*Stop").c_str());
            pSensorCharacteristic->notify();
          }
          Motor2_control(0);
          break;      
        case 25:
          digitalWrite(BRAKE, HIGH);
          if (lock) {
            Serial.println("Change direction..."); 
            SerialBT.println("42");
            pSensorCharacteristic->setValue(String("*m2*Reversing.").c_str());
            pSensorCharacteristic->notify();
          }
          Motor2_control(-50);
          break;
        case 28:
          digitalWrite(BRAKE, LOW);
          if (lock) {
            Serial.println("Stop."); 
            SerialBT.println("41");
            pSensorCharacteristic->setValue(String("*m2*Stop").c_str());
            pSensorCharacteristic->notify();
          }
          Motor2_control(0);
          break; 
        case 31:
          digitalWrite(BRAKE, HIGH);
          if (lock) {
            Serial.println("Checking encoder..."); 
            SerialBT.println("43");
            pSensorCharacteristic->setValue(String("*m2*Checking encoder.").c_str());
            pSensorCharacteristic->notify();
          }
        Motor2_control(70);
          break; 
        case 34:
          digitalWrite(BRAKE, LOW);
          if (lock && motor2_speed > 300) {
            Serial.println("Encoder OK.");
            Serial.print("Speed: "); Serial.println(motor2_speed);
            s1.concat(motor2_speed);

            char str[22];
            sprintf(str, "*m2*Speed: %u", motor2_speed);
            pSensorCharacteristic->setValue(str);
            pSensorCharacteristic->notify();
            SerialBT.println("44"  + s1);
            Serial.println("Stop.");
          } else if (lock && motor2_speed <= 0) {
            Serial.println("Encoder FAIL.");
            Serial.print("Speed: "); Serial.println(motor2_speed);
            pSensorCharacteristic->setValue("*m2*Encoder Fail");
            pSensorCharacteristic->notify();
            s1.concat(motor2_speed);
            SerialBT.println("45"  + s1);
            Serial.println("Stop.");
          }
          s1 = "";
          Motor2_control(0);
          break;        
  
        case 37:
          digitalWrite(BRAKE, HIGH);
          if (lock) {
            Serial.println("Rotating motor 3."); 
            SerialBT.println("48");
            pSensorCharacteristic->setValue(String("*m3*Rotating.").c_str());
            pSensorCharacteristic->notify();
          }
          Motor3_control(50);
          break;
        case 40:
          digitalWrite(BRAKE, LOW);
          if (lock) {
            Serial.println("Stop."); 
            SerialBT.println("41");
            pSensorCharacteristic->setValue(String("*m3*Stop").c_str());
            pSensorCharacteristic->notify();
          }
          lock = 0;
          Motor3_control(0);
          break;      
        case 43:
          digitalWrite(BRAKE, HIGH);
          if (lock) {
            Serial.println("Change direction..."); 
            SerialBT.println("42");
            pSensorCharacteristic->setValue(String("*m3*Reversing.").c_str());
            pSensorCharacteristic->notify();
          }
          Motor3_control(-50);
          break;
        case 46:
          digitalWrite(BRAKE, LOW);
          if (lock) {
            Serial.println("Stop."); 
            SerialBT.println("41");
            pSensorCharacteristic->setValue(String("*m3*Stop").c_str());
            pSensorCharacteristic->notify();
          }
          Motor3_control(0);
          break; 
        case 49:
          digitalWrite(BRAKE, HIGH);
          if (lock) {
            Serial.println("Checking encoder..."); 
            SerialBT.println("43");
            pSensorCharacteristic->setValue(String("*m3*Checking Encoder").c_str());
            pSensorCharacteristic->notify();
          }
          Motor3_control(70);
          break; 
        case 52:
          digitalWrite(BRAKE, LOW);
          if (lock && motor3_speed > 300) {
            Serial.println("Encoder OK.");
            Serial.print("Speed: "); Serial.println(motor3_speed);
            s1.concat(motor3_speed);
            char str[22];
            sprintf(str, "*m3*Speed: %u", motor3_speed);
            pSensorCharacteristic->setValue(str);
            pSensorCharacteristic->notify();
            SerialBT.println("44"  + s1);
            Serial.println("Stop.");
          } else if (lock && motor3_speed <= 0) {
            Serial.println("Encoder FAIL.");
            Serial.print("Speed: "); 
            Serial.println(motor3_speed);
            pSensorCharacteristic->setValue("*m3*Encoder Fail");
            pSensorCharacteristic->notify();
            s1.concat(motor3_speed);
            SerialBT.println("45"  + s1);
            Serial.println("Stop.");
          }
          s1 = "";
          Motor3_control(0);
          break;        
        }
        lock = 0;
  }
    
  previousT_1 = currentT;
}    
  if (currentT - previousT_2 >= 1000) {

    battVoltage((double)analogRead(VBAT) / 204); // value 204 must be selected by measuring battery voltage!
    
    if (!calibrated && !calibrating) {
      Serial.println("first you need to calibrate the balancing points...");
      SerialBT.println("95");

      pSensorCharacteristic->setValue("*a*First you need to calibrate the balancing points...");
      pSensorCharacteristic->notify();
      if (!calibrated_leds) {
        colorLed(0, 50, 0, 1, 3, 0); 
        calibrated_leds = true; 
      } else {
        colorLed(0, 0, 0, 1, 3, 0); 
        calibrated_leds = false; 
      }
    }
    
    if (Mode == 1){
      f++; 
      if (f == 53){ beep(); Serial.println("End of Engine Tests."); SerialBT.println("96"); f = 1; Mode = 0;} 
    lock = 1;
    }
    previousT_2 = currentT;
  }
  
  if (ConnectBT == true && (currentT - previousT_3) > DelayBT)
      {
        init_BT();
        ConnectBT = false;
      }
}
