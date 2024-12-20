#include <Arduino.h>
#include "functions.h"
#include <Wire.h>
#include <EEPROM.h>
BluetoothSerial SerialBT;
CRGB leds[NUM_PIXELS];
byte Mode, j, RGB_val;
bool LedRGB, Push, ChooseColor;
float speed_X, speed_Y;
OffsetsObj offsets;
int16_t motor1_speed, motor2_speed, motor3_speed;
int16_t AcX;
int16_t AcY;
int16_t AcZ;
int16_t AcXc;
int16_t AcYc;
int16_t AcZc;
int16_t GyX;
int16_t GyY;
int16_t GyZ;
float robot_angleX;
float robot_angleY;
float Acc_angleX; 
float Acc_angleY;  
long currentT;
long previousT_1;
long previousT_2;
long previousT_3;
byte f;
bool lock;
long lasttime;
long batterylasttime = millis();
int32_t motors_speed_X; 
int32_t motors_speed_Y; 
int32_t motors_speed_Z;
bool balanceNotify = true;
int DIR1;
int ENC1_1;
int ENC1_2;
int PWM1;
int PWM1_CH;

int DIR2;
int ENC2_1;
int ENC2_2;
int PWM2;
int PWM2_CH;

int DIR3;
int ENC3_1;
int ENC3_2;
int PWM3;
int PWM3_CH;
int melody[] = {
    NOTE_G5,4,
    NOTE_C6,4,
    NOTE_B5,4,
    NOTE_AS5,8,
    NOTE_B5,8,
    NOTE_AS5,8,
    // ----
    NOTE_A5,8,
    NOTE_GS5,4,
    NOTE_G5,4,
    NOTE_FS5,4,
    // ----
    NOTE_G5,4,
    NOTE_A5,4,
    NOTE_GS5,4,
    NOTE_G5,8,
    NOTE_GS5,8,
    NOTE_G5,8,
    // ----
    NOTE_FS5,8,
    NOTE_F5,4,
    NOTE_E5,4,
    NOTE_DS5,4,
    // ----
    NOTE_E5,4,
    NOTE_G5,8,
    NOTE_F5,8,
    NOTE_F5,4,
    NOTE_CS5,4,
    // ----
    NOTE_D5,4,
    NOTE_G5,8,
    NOTE_F5,8,
    NOTE_F5,4,
    NOTE_CS5,4,
    // ----
    NOTE_D5,4,
    NOTE_B4,8,
    NOTE_C5,8,
    NOTE_CS5,8,
    NOTE_D5,8,
    NOTE_DS5,8,
    NOTE_E5,8,
    NOTE_F5,8,
    // -----
    NOTE_FS5,8,
    NOTE_G5,8,
    NOTE_GS5,8,
    NOTE_A5,8,
    NOTE_B5,8,
    NOTE_A5,4,
    NOTE_G5,4,
    // -----
  };
void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void beep() {
    digitalWrite(BUZZER, HIGH);
    delay(2);
    digitalWrite(BUZZER, LOW);
    delay(80);
}
    
//***********************************************************************
// Initialise Bluetooth
//***********************************************************************
void init_BT()
{
        Mode = 0; loop_time = 15;
        calibrating = false;
        colorLed(0, 0, 0, 1, 3, 0);
        LedRGB = false;
}

//***********************************************************************
// Rgb Led Command
//***********************************************************************
void colorLed(int Red, int Green, int Blue , int firstled, int lastled, int wait) {
    for(int i = firstled-1; i < lastled; i++) {    // For each pixel in strip...
    leds[i] = CRGB(Red, Green, Blue);
    FastLED.show();                          //  Update strip to match
  }
    delay(wait);                           //  Pause for a moment
}

//***********************************************************************
// Rgb Led Process
//***********************************************************************
void RGB_Process(){
  j = 0;
  while (digitalRead(INT_LED) == LOW){
    if (Push == false){Push = true;}
    j++; delay (5);
    if (j >= 250){break;}
  }
  if (!ChooseColor){
    if (Push == true){
      if (j < 250){
        if (LedRGB == true){
          colorLed(0, 0, 0, 1, 3, 0);
          LedRGB = false;
        }else{
          colorLed(Red, Green, Blue, 1, 3, 0);
          LedRGB = true;
        }
        beep();
      }else{
        j = 0; Push = false; LedRGB = true; 
        colorLed(Red, Green, Blue, 1, 3, 80);
        colorLed(0, 0, 0, 1, 3, 80);
        colorLed(Red, Green, Blue, 1, 3, 0); beep ();
        while (digitalRead(INT_LED) == LOW){}
        delay (5); ChooseColor = true;
      }
    }
  }else{
    if (Push == true){
      if (j < 250){
        if (RGB_val < 18){RGB_val++;}else{RGB_val = 0;}
        Red = RGB_Color[RGB_val][0]; Green = RGB_Color[RGB_val][1]; Blue = RGB_Color[RGB_val][2];
        colorLed(Red, Green, Blue, 1, 3, 0);
        Push = false; j = 0;
        beep();
      }else{
        offsets.Red = Red; offsets.Green = Green; offsets.Blue = Blue; offsets.ID = 96;
        save();
        colorLed(0, 0, 0, 1, 3, 0); 
        beep (); colorLed(Red, Green, Blue, 1, 3, 0); 
        beep ();
        Push = false; ChooseColor = false;
      }
    }
  }
  while (digitalRead(INT_LED) == LOW){}
  if (Push == true){Push = false;}
  delay (5);
}

void save() {
    EEPROM.put(0, offsets);
    EEPROM.commit();
    beep();
}

void angle_setup() {
  Wire.begin();
  delay (100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);
  
  beep();
  colorLed(50, 0, 0, 3, 3, 0);
  for (int i = 0; i < 512; i++) {
    angle_calc();
    GyZ_offset_sum += GyZ;
    delay(5);
  }
  GyZ_offset = GyZ_offset_sum >> 9;
  Serial.print("GyZ offset value = "); Serial.println(GyZ_offset);
  beep();
  colorLed(0, 0, 0, 3, 3, 0);

  colorLed(50, 0, 0, 2, 2, 0);
  for (int i = 0; i < 512; i++) {
    angle_calc();
    GyY_offset_sum += GyY;
    delay(5);
  }
  GyY_offset = GyY_offset_sum >> 9;
  Serial.print("GyY offset value = "); Serial.println(GyY_offset);
  beep();
  colorLed(0, 0, 0, 2, 2, 0);

  colorLed(50, 0, 0, 1, 1, 0);
  for (int i = 0; i < 512; i++) {
    angle_calc();
    GyX_offset_sum += GyX;
    delay(5);
  }
  GyX_offset = GyX_offset_sum >> 9;
  Serial.print("GyX offset value = "); Serial.println(GyX_offset);
  colorLed(50, 0, 0, 1, 1, 150);
  beep();
  beep();
  colorLed(50, 0, 0, 1, 3, 300);
  colorLed(0, 0, 0, 1, 3, 150);
  colorLed(50, 0, 0, 1, 3, 300);
  colorLed(0, 0, 0, 1, 3, 0);
}

void angle_calc() {
  
  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);  
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true); 
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  if (abs(AcX) < 2000) {
    AcXc = AcX - offsets.acXv;
    AcYc = AcY - offsets.acYv;
    AcZc = AcZ - offsets.acZv;
  } else {
    AcXc = AcX - offsets.acXe;
    AcYc = AcY - offsets.acYe;
    AcZc = AcZ - offsets.acZe;
  }
  GyZ -= GyZ_offset;
  GyY -= GyY_offset;
  GyX -= GyX_offset;

  robot_angleY += GyY * loop_time / 1000 / 65.536;
  Acc_angleY = atan2(AcXc, -AcZc) * 57.2958;
  robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);

  robot_angleX += GyX * loop_time / 1000 / 65.536;
  Acc_angleX = -atan2(AcYc, -AcZc) * 57.2958;
  robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount);


  if (abs(AcX) < 2000 && abs(Acc_angleX) < 0.4 && abs(Acc_angleY) < 0.4 && !vertical_vertex && !vertical_edge) {
    robot_angleX = Acc_angleX;
    robot_angleY = Acc_angleY;
    vertical_vertex = true;
  } else if (abs(AcX) > 7000 && abs(AcX) < 10000 && abs(Acc_angleX) < 0.3 && !vertical_vertex && !vertical_edge) {
    robot_angleX = Acc_angleX;
    robot_angleY = Acc_angleY;
    vertical_edge = true;
  } else if ((abs(robot_angleX) > 7 || abs(robot_angleY) > 7) && vertical_vertex) {
    vertical_vertex = false;
  } else if ((abs(robot_angleX) > 7 || abs(robot_angleY) > 7) && vertical_edge) {
    vertical_edge = false;
  }
  if(!vertical_vertex && !vertical_edge){
    notBalancing();
  }
}

void XYZ_to_threeWay(float pwm_X, float pwm_Y, float pwm_Z) {
  int16_t m1 = round((0.5 * pwm_X - 0.866 * pwm_Y) / 1.37 + pwm_Z);  
  int16_t m2 = round((0.5 * pwm_X + 0.866 * pwm_Y) / 1.37 + pwm_Z);
  int16_t m3 = -pwm_X / 1.37 + pwm_Z;  
  Motor1_control(m1);
  Motor2_control(m2);
  Motor3_control(m3);
}

void threeWay_to_XY(int in_speed1, int in_speed2, int in_speed3) {
  speed_X = ((in_speed3 - (in_speed2 + in_speed1) * 0.5) * 0.5) * 1.81;
  speed_Y = -(-0.866 * (in_speed2 - in_speed1)) / 1.1;
}

void battVoltage(double voltage) {
  if(deviceConnected && millis() - batterylasttime > 10) {
    batterylasttime = millis();
    char str[10];
    sprintf(str, "*bv*%.2f", voltage);
    pSensorCharacteristic->setValue(str);
    pSensorCharacteristic->notify();
  }
  if (voltage > 7 && voltage <= 9.6) {
    if (digitalRead(BUZZER) == LOW){
      digitalWrite(BUZZER, HIGH); 
      colorLed(0, 50, 0, 1, 3, 0);
      } else {
      digitalWrite(BUZZER, LOW); 
      colorLed(0, 0, 0, 1, 3, 0);
    }
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

void pwmSet(uint8_t channel, uint32_t value) {
  ledcWrite(channel, value);
}

void Motor1_control(int sp) {
  if (Mode == 0){  
    sp = sp + motor1_speed;
  }
  if (sp < 0) 
    digitalWrite(DIR1, LOW);
  else 
    digitalWrite(DIR1, HIGH);
  pwmSet(PWM1_CH, 255 - abs(sp));
}

void Motor2_control(int sp) {
  if (Mode == 0){
    sp = sp + motor2_speed;
  }
  if (sp < 0) 
    digitalWrite(DIR2, LOW);
  else 
    digitalWrite(DIR2, HIGH);
  pwmSet(PWM2_CH, 255 - abs(sp));
}

void Motor3_control(int sp) {
  if (Mode == 0){
    sp = sp + motor3_speed;
  }
  if (sp < 0) 
    digitalWrite(DIR3, LOW);
  else 
    digitalWrite(DIR3, HIGH);
  pwmSet(PWM3_CH, 255 - abs(sp));
}

void ENC1_READ() {
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC1_1) << 1) | digitalRead(ENC1_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count1++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count1--;
  }
}

void ENC2_READ() {
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC2_1) << 1) | digitalRead(ENC2_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count2++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count2--;
  }
}

void ENC3_READ() {
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC3_1) << 1) | digitalRead(ENC3_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count3++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count3--;
  }
}
void testMotors() {
      f = 1; lock = 1; Mode = 1; loop_time = 100;
      
      XYZ_to_threeWay(0, 0, 0);
      digitalWrite(BRAKE, LOW);
      motors_speed_X = 0;
      motors_speed_Y = 0;
}
void getSettings() {
  char str[64];

  sprintf(str, "*sv*%f|%f|%f|%f|%f|%f", K1, K2, K3, K4, zK2, zK3);
  pSensorCharacteristic->setValue(str);
  pSensorCharacteristic->notify();


  sprintf(str, "*se*%f|%f|%f|%f", eK1, eK2, eK3, eK4);
  pSensorCharacteristic->setValue(str);
  pSensorCharacteristic->notify();

  sprintf(str, "*fv*%.02f", FIRMWARE_VERSION);
  pSensorCharacteristic->setValue(str);
  pSensorCharacteristic->notify();

  sprintf(str, "*sm*%d|%d|%d", offsets.motor1, offsets.motor2, offsets.motor3);
  pSensorCharacteristic->setValue(str);
  pSensorCharacteristic->notify();

  sprintf(str, "*sl*%d|%d|%d|%d|%d|%d", offsets.vertexRed, offsets.vertexGreen, offsets.vertexBlue, offsets.edgeRed, offsets.edgeGreen, offsets.edgeBlue);
  pSensorCharacteristic->setValue(str);
  pSensorCharacteristic->notify();
}

void setSettings(char s, char c) {
  switch(s){
    case SETTINGS_K1:
      if(c == SETTINGS_PLUS) {
        K1 += 1;
      } else if(c == SETTINGS_MINUS) {
        K1 -= 1;
      }
      getSettings();
      break;
    case SETTINGS_K2:
      if(c == SETTINGS_PLUS) {
        K2 += .5;
      } else if(c == SETTINGS_MINUS) {
        K2 -= .5;
      }
      getSettings();
      break;
    case SETTINGS_K3:
      if(c == SETTINGS_PLUS) {
        K3 += .05;
      } else if(c == SETTINGS_MINUS) {
        K3 -= .05;
      }
      getSettings();
      break;
    case SETTINGS_K4:
      if(c == SETTINGS_PLUS) {
        K4 += 0.001;
      } else if(c == SETTINGS_MINUS) {
        K4 -= 0.001;
      }
      getSettings();
      break;
    case SETTINGS_ZK2:
      if(c == SETTINGS_PLUS) {
        zK2 += 0.1;
      } else if(c == SETTINGS_MINUS) {
        zK2 -= 0.1;
      }
      getSettings();
      break;
    case SETTINGS_ZK3:
      if(c == SETTINGS_PLUS) {
        zK3 += 0.05;
      } else if(c == SETTINGS_MINUS) {
        zK3 -= 0.01;
      }
      getSettings();
      break;
    case SETTINGS_EK1:
      if(c == SETTINGS_PLUS) {
        eK1 += 1;
      } else if(c == SETTINGS_MINUS) {
        eK1 -= 1;
      }
      getSettings();
      break;
    case SETTINGS_EK2:
      if(c == SETTINGS_PLUS) {
        eK2 += .5;
      } else if(c == SETTINGS_MINUS) {
        eK2 -= .5;
      }
      getSettings();
      break;
    case SETTINGS_EK3:
      if(c == SETTINGS_PLUS) {
        eK3 += .05;
      } else if(c == SETTINGS_MINUS) {
        eK3 -= .05;
      }
      getSettings();
      break;
    case SETTINGS_EK4:
      if(c == SETTINGS_PLUS) {
        eK4 += 0.001;
      } else if(c == SETTINGS_MINUS) {
        eK4 -= 0.001;
      }
      getSettings();
      break;
  }
}
void saveSettings(char t) {
      if (t == SETTINGS_SAVE_VERTEX){

        offsets.K1 = K1; offsets.K2 = K2; offsets.K3 = K3; offsets.K4 = K4; offsets.zK2 = zK2; offsets.zK3 = zK3;
        save();
        pSensorCharacteristic->setValue("*a*Vertex Settings Saved");
        pSensorCharacteristic->notify();
      }
      if (t == SETTINGS_SAVE_EDGE){
        offsets.eK1 = eK1; offsets.eK2 = eK2; offsets.eK3 = eK3; offsets.eK4 = eK4;
        save();
        pSensorCharacteristic->setValue("*a*Edge Settings Saved");
        pSensorCharacteristic->notify();
      }
}
int Tuning() {

  if (!SerialBT.available()) return 0;
  char param = SerialBT.read();               // get parameter byte
  if (!SerialBT.available()) return 0;
  char cmd = SerialBT.read();                 // get command byte
  switch (param) {

    case 'T': 
      if (cmd = 'x'){
        if (ConnectBT == false){init_BT(); ConnectBT = true;}
        previousT_3 = currentT;
      }
      break;

    case 'm': 
      if (cmd == SETTINGS_PLUS){
        Mode = 0; loop_time = 15;
      }
      if (cmd == '1'){
        f = 1; lock = 0; Mode = 1; loop_time = 100;
      }
      XYZ_to_threeWay(0, 0, 0);
      digitalWrite(BRAKE, LOW);
      motors_speed_X = 0;
      motors_speed_Y = 0;
    break;

    case 'o': 
      if (cmd == '1'){
        offsets.K1 = K1; offsets.K2 = K2; offsets.K3 = K3; offsets.K4 = K4; offsets.zK2 = zK2; offsets.zK3 = zK3;
        save();
        SerialBT.println("91");
      }
      if (cmd == '2'){
        offsets.eK1 = eK1; offsets.eK2 = eK2; offsets.eK3 = eK3; offsets.eK4 = eK4;
        save();
        SerialBT.println("92");
      }
    break;

    case 'p': 
      if (cmd == '1'){
        K1 = offsets.K1; K2 = offsets.K2; K3 = offsets.K3; K4 = offsets.K4; zK2 = offsets.zK2; zK3 = offsets.zK3;
        SerialBT.println("93");
      }
      if (cmd == '2'){
        eK1 = offsets.eK1; eK2 = offsets.eK2; eK3 = offsets.eK3; eK4 = offsets.eK4;
        SerialBT.println("94");
      }
    break;

      case 'q': 
      if (cmd == '+')    K1 += 1;
      if (cmd == '-')    K1 -= 1;
      if (cmd == '?')    {}
      s1.concat(K1);
      SerialBT.println("21" + s1);
      s1 = "";
      break;
    case 'r':
      if (cmd == '+')    K2 += 0.5;
      if (cmd == '-')    K2 -= 0.5;
      if (cmd == '?')    {}
      s1.concat(K2);
      SerialBT.println("22" + s1);
      s1 = "";
      break;
    case 's':
      if (cmd == '+')    K3 += 0.05;
      if (cmd == '-')    K3 -= 0.05;
      if (cmd == '?')    {}
      s1.concat(K3);
      SerialBT.println("23"  + s1);
      s1 = "";
      break;  
    case 't':
      if (cmd == '+')    K4 += 0.001;
      if (cmd == '-')    K4 -= 0.001;
      if (cmd == '?')    {}
      s1.concat(K4*1000);
      SerialBT.println("24"  + s1);
      s1 = "";
      break; 
    case 'u':
      if (cmd == '+')    zK2 += 0.1;
      if (cmd == '-')    zK2 -= 0.1;
      if (cmd == '?')    {}
      s1.concat(zK2);
      SerialBT.println("25"  + s1);
      s1 = "";
      break;
    case 'v':
      if (cmd == '+')    zK3 += 0.05;
      if (cmd == '-')    zK3 -= 0.05;
      if (cmd == '?')    {}
      s1.concat(zK3);
      SerialBT.println("26"  + s1);
      s1 = "";
      break;
    
    case 'w':
      if (cmd == '+')    eK1 += 1;
      if (cmd == '-')    eK1 -= 1;
      if (cmd == '?')    {}
      s1.concat(eK1);
      SerialBT.println("31"  + s1);
      s1 = "";
      break;
    case 'x':
      if (cmd == '+')    eK2 += 0.5;
      if (cmd == '-')    eK2 -= 0.5;
      if (cmd == '?')    {}
      s1.concat(eK2);
      SerialBT.println("32"  + s1);
      s1 = "";
      break;
    case 'y':
      if (cmd == '+')    eK3 += 0.05;
      if (cmd == '-')    eK3 -= 0.05;
      if (cmd == '?')    {}
      s1.concat(eK3);
      SerialBT.println("33"  + s1);
      s1 = "";
      break;  
    case 'z':
      if (cmd == '+')    eK4 += 0.001;
      if (cmd == '-')    eK4 -= 0.001;
      if (cmd == '?')    {}
      s1.concat(eK4*1000);
      SerialBT.println("34"  + s1);
      s1 = "";
      break; 
 
    case 'c':
      if (cmd == '+' && !calibrating) {
        calibrating = true;
        SerialBT.println("11");
        Serial.println("Calibrating on.");
        Serial.println("Set the cube on vertex...");
        colorLed(50, 50, 0, 1, 3, 0);
        beep(); 
      }
      
      if (cmd == '-' && calibrating)  {
        if (abs(AcX) < 2000 && abs(AcY) < 2000) {
          //Serial.print("X: "); SerialBT.print(AcX); SerialBT.print(" Y: "); SerialBT.print(AcY); SerialBT.print(" Z: "); SerialBT.println(AcZ + 16384);
          offsets.ID = 96;
          offsets.acXv = AcX;
          offsets.acYv = AcY;
          offsets.acZv = AcZ + 16384;
          SerialBT.println("12");
          Serial.println("Vertex OK.");
          Serial.println("Set the cube on Edge...");
          vertex_calibrated = true;
          colorLed(0, 50, 50, 1, 3, 0);
          beep();
        } else if (abs(AcX) > 7000 && abs(AcX) < 10000 && abs(AcY) < 2000 && vertex_calibrated) {
          //Serial.print("X: "); SerialBT.print(AcX); SerialBT.print(" Y: "); SerialBT.print(AcY); SerialBT.print(" Z: "); SerialBT.println(AcZ + 16384);
          SerialBT.println("13");
          Serial.println("Edge OK.");
          offsets.acXe = AcX;
          offsets.acYe = AcY;
          offsets.acZe = AcZ + 16384;
          colorLed(0, 0, 0, 1, 3, 0);
          save();
          calibrated = true;
          calibrating = false;
          Serial.println("Calibrating off.");
          beep();
        } else {
          SerialBT.println("14");
          Serial.println("The angles are wrong!!!");
          beep();
          beep();
        }
      }
      
      break;              
   }
   return 1;
}
void calibrate_edge() {
      if(!vertex_calibrated){
        pSensorCharacteristic->setValue("*a*You must calibrate the vertex first");
        pSensorCharacteristic->notify();
        return;
      }
  if (abs(AcX) > 7000 && abs(AcX) < 10000 && abs(AcY) < 2000) {
      offsets.acXe = AcX;
      offsets.acYe = AcY;
      offsets.acZe = AcZ + 16384;
      colorLed(0, 0, 0, 1, 3, 0);
      save();
      beep();

      calibrated = true;
      calibrating = false;
      pSensorCharacteristic->setValue("*a*Edge Accepted");
      pSensorCharacteristic->notify();
      return;
    } 


    pSensorCharacteristic->setValue("*a*The angles are wrong!!!");
    pSensorCharacteristic->notify();
    beep();
    beep();
}
void calibrate_vertex() {

  if (abs(AcX) < 2000 && abs(AcY) < 2000) {
    offsets.ID = 96;
    offsets.acXv = AcX;
    offsets.acYv = AcY;
    offsets.acZv = AcZ + 16384;

    pSensorCharacteristic->setValue("*a*Vertex Accepted, now calibrate edge");
    pSensorCharacteristic->notify();
    vertex_calibrated = true;
    colorLed(0, 50, 50, 1, 3, 0);
    beep();
    return;
  }
  pSensorCharacteristic->setValue("*a*The angles are wrong!!!");
  pSensorCharacteristic->notify();
}


void balancing(String type){
  if(deviceConnected && millis() - lasttime > 2000){
    play=true;
    lasttime = millis();
    balanceNotify = true;
    char str[10];
    sprintf(str, "*b*%s", type);
    pSensorCharacteristic->setValue(str);
    pSensorCharacteristic->notify();
  }
}
void notBalancing(){
  if(deviceConnected && millis() - lasttime > 2000 && balanceNotify) {
    play=false;
    lasttime = millis();
    balanceNotify= false;
    pSensorCharacteristic->setValue("*nb*0");
    pSensorCharacteristic->notify();
  }
}
void setLeds(char t, int r, int g, int b) {
  offsets.ID = 96;
  if(t == 'e') {
    edgeRed = r;
    edgeGreen = g;
    edgeBlue = b;
    colorLed(r, g, b, 1, 3, 0);
    offsets.edgeRed = r; offsets.edgeGreen = g; offsets.edgeBlue = b; 
  }
  if(t == 'v') {
    vertexRed = r;
    vertexGreen = g;
    vertexBlue = b;
    colorLed(r, g, b, 1, 3, 0);
    offsets.vertexRed = r; offsets.vertexGreen = g; offsets.vertexBlue = b;

  } else {
    Red = r;
    Green = g;
    Blue = b;
    colorLed(r, g, b, 1, 3, 0);
    offsets.Red = r; offsets.Green = g; offsets.Blue = b;
  }
  save();
}
void musicLoop(void* pvParameters){
  for(;;) {
        for (int thisNote = 0; thisNote <= 88; thisNote+=2) {
          int noteDuration = 1000 / melody[thisNote+1];
          tone(BUZZER, melody[thisNote], noteDuration);

          int pauseBetweenNotes = noteDuration * 1.30;
          //delay(pauseBetweenNotes);
          noTone(BUZZER);
        }
  }
        
  }
void saveMotors(int mot1, int mot2, int mot3) {
  offsets.motor1 = mot1;
  offsets.motor2 = mot2;
  offsets.motor3 = mot3;
  save();

  DIR1      = motors[mot1][0];
  ENC1_1    = motors[mot1][1];
  ENC1_2    = motors[mot1][2];
  PWM1      = motors[mot1][3];
  PWM1_CH   = motors[mot1][4];

  DIR2      = motors[mot2][0];
  ENC2_1    = motors[mot2][1];
  ENC2_2    = motors[mot2][2];
  PWM2      = motors[mot2][3];
  PWM2_CH   = motors[mot2][4];

  DIR3      = motors[mot3][0];
  ENC3_1    = motors[mot3][1];
  ENC3_2    = motors[mot3][2];
  PWM3      = motors[mot3][3];
  PWM3_CH   = motors[mot3][4];


  attachInterrupt(ENC1_1, ENC1_READ, CHANGE);
  attachInterrupt(ENC1_2, ENC1_READ, CHANGE);

  attachInterrupt(ENC2_1, ENC2_READ, CHANGE);
  attachInterrupt(ENC2_2, ENC2_READ, CHANGE);

  attachInterrupt(ENC3_1, ENC3_READ, CHANGE);
  attachInterrupt(ENC3_2, ENC3_READ, CHANGE);
  


}
