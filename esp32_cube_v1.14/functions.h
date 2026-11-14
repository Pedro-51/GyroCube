#ifndef GYROCUBE_FUNCTIONS_H
#define GYROCUBE_FUNCTIONS_H

byte detect_edge_motor();
void edge_sensor_projection(byte motor);
void regulate_edge();
void writeTo(byte device, byte address, byte value);
void beep();
void init_BT();
void colorLed(int Red, int Green, int Blue , int firstled, int lastled, int wait);
void RGB_Process();
void save();
void angle_setup();
void angle_calc();
void XYZ_to_threeWay(float pwm_X, float pwm_Y, float pwm_Z);
void threeWay_to_XY(int in_speed1, int in_speed2, int in_speed3);
void battVoltage(double voltage);
void pwmSet(uint8_t channel, uint32_t value);
void Motor1_control(int sp);
void Motor2_control(int sp);
void Motor3_control(int sp);
void ENC1_READ();
void ENC2_READ();
void ENC3_READ();
int Tuning();

// Detects which of the three edges the cube is resting on.
// Returns 0 if no edge can be identified.
byte detect_edge_motor() {
  if (AcX < -7000 && AcX > -10000) return 3;
  const float ax = (float)AcX, ay = (float)AcY, az = (float)AcZ;
  const float xy = sqrtf(ax * ax + ay * ay);
  if (xy <= 7000.0f || xy >= 12500.0f ||
      az >= -10500.0f || az <= -15500.0f) return 0;
  const float bproj = (ax * EDGE_AXES[1].x + ay * EDGE_AXES[1].y) / xy;
  const float cproj = (ax * EDGE_AXES[2].x + ay * EDGE_AXES[2].y) / xy;
  if (bproj > 0.95f && bproj >= cproj) return 1;
  if (cproj > 0.95f) return 2;
  return 0;
}

// Adjusts sensor readings for the edge supporting the cube.
void edge_sensor_projection(byte motor) {
  const float rx = EDGE_AXES[motor].x;
  const float ry = EDGE_AXES[motor].y;

  const float ax = (float)AcX, ay = (float)AcY;
  const float radialX = -(rx * ax + ry * ay);
  const float tangentX = -ry * ax + rx * ay;
  const float projectedGyX = -(rx * GyX + ry * GyY);
  const float projectedGyY = ry * GyX - rx * GyY;

  const float tangentBias = (motor == 3) ? offsets.acYe : 0.0f;
  const float zBiasX = (motor == 3) ? offsets.acZe : 0.0f;
  Acc_angleX = atan2f(tangentX + tangentBias, -(AcZ - zBiasX)) * 57.2958f;
  Acc_angleY = atan2f(radialX - offsets.acXe, -(AcZ - offsets.acZe)) * 57.2958f;

  robot_angleY += projectedGyY * loop_time / 1000 / 131.0;
  robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0f - Gyro_amount);
  robot_angleX += projectedGyX * loop_time / 1000 / 131.0;
  robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0f - Gyro_amount);
  gyroX = projectedGyX / 131.0f;
}

// Balances the cube on an edge using the corresponding motor.
void regulate_edge() {
  const int16_t wheel_speed = edge_motor == 1 ? motor1_speed :
                              (edge_motor == 2 ? motor2_speed : motor3_speed);
  digitalWrite(BRAKE, HIGH);
  gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;
  int pwm_X = constrain(eK1 * robot_angleX + eK2 * gyroXfilt +
                        eK3 * wheel_speed + eK4 * motors_speed_X, -255, 255);
  motors_speed_X += wheel_speed / 5;

  Motor1_control(edge_motor == 1 ? pwm_X : 0);
  Motor2_control(edge_motor == 2 ? pwm_X : 0);
  Motor3_control(edge_motor == 3 ? pwm_X : 0);
}


// Writes a setting to the MPU6050 sensor.
void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

// Emits a short beep to indicate an action.
void beep() {
    digitalWrite(BUZZER, HIGH);
    delay(2);
    digitalWrite(BUZZER, LOW);
    delay(80);
}
    
// Restores normal cube operation during Bluetooth handling.
//***********************************************************************
// Initializes Bluetooth
//***********************************************************************
void init_BT()
{
        Mode = 0; loop_time = 15;
        calibrating = false;
        colorLed(0, 0, 0, 1, NUM_PIXELS, 0);
        LedRGB = false;
}

// Sets the selected LEDs to a given color.
// LED indices in calls start at 1; wait adds a delay.
//***********************************************************************
// Rgb Led Command
//***********************************************************************
void colorLed(int Red, int Green, int Blue , int firstled, int lastled, int wait) {
    for(int i = firstled-1; i < lastled; i++) {    // Iterates over the selected LEDs
    leds[i] = CRGB(Red, Green, Blue);
    FastLED.show();                          // Updates the displayed color
  }
    delay(wait);                           
}

// Handles the LED button: short press toggles LEDs,
// long press selects and saves a color.
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
          colorLed(0, 0, 0, 1, NUM_PIXELS, 0);
          LedRGB = false;
        }else{
          colorLed(Red, Green, Blue, 1, NUM_PIXELS, 0);
          LedRGB = true;
        }
        beep();
      }else{
        Push = false; LedRGB = true; 
        colorLed(Red, Green, Blue, 1, NUM_PIXELS, 80);
        colorLed(0, 0, 0, 1, 12, 80);
        colorLed(Red, Green, Blue, 1, NUM_PIXELS, 0); beep ();
        while (digitalRead(INT_LED) == LOW){}
        delay (5); ChooseColor = true;
      }
    }
  }else{
    if (Push == true){
      if (j < 250){
        if (RGB_val < 17){RGB_val++;}else{RGB_val = 0;}
        Red = RGB_Color[RGB_val][0]; Green = RGB_Color[RGB_val][1]; Blue = RGB_Color[RGB_val][2];
        colorLed(Red, Green, Blue, 1, NUM_PIXELS, 0);
        Push = false;
        beep();
      }else{
        offsets.Red = Red; offsets.Green = Green; offsets.Blue = Blue; offsets.RGB_val = RGB_val;
        save();
        colorLed(0, 0, 0, 1, NUM_PIXELS, 0); 
        beep (); colorLed(Red, Green, Blue, 1, NUM_PIXELS, 0); 
        beep ();
        Push = false; ChooseColor = false;
      }
    }
  }
  while (digitalRead(INT_LED) == LOW){}
  if (Push == true){Push = false;}
  delay (5);
}

// Saves settings to the cube memory.
void save() {
    EEPROM.put(0, offsets);
    EEPROM.commit();
    beep();
}

// Starts the motion sensor and measures its stationary offsets.
// LEDs and beeps indicate setup progress.
void angle_setup() {
  Wire.begin();
  delay (100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Sets the accelerometer measurement range
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Sets the gyroscope measurement range
  delay (100);
  
  beep();
  colorLed(LED_LUX, 0, 0, NUM_PIX_MOT*2+1, NUM_PIXELS, 0);
  for (int i = 0; i < 512; i++) {
    angle_calc();
    GyZ_offset_sum += GyZ;
    delay(5);
  }
  GyZ_offset = GyZ_offset_sum >> 9;
  //Serial.print((SERIAL_FRENCH ? "Decalage GyZ = " : "GyZ offset value = ")); Serial.println(GyZ_offset);
  beep();
  colorLed(0, 0, 0, NUM_PIX_MOT*2+1, NUM_PIXELS, 0);

  colorLed(LED_LUX, 0, 0, NUM_PIX_MOT+1, NUM_PIX_MOT*2, 0);
  for (int i = 0; i < 512; i++) {
    angle_calc();
    GyY_offset_sum += GyY;
    delay(5);
  }
  GyY_offset = GyY_offset_sum >> 9;
  //Serial.print((SERIAL_FRENCH ? "Decalage GyY = " : "GyY offset value = ")); Serial.println(GyY_offset);
  beep();
  colorLed(0, 0, 0, NUM_PIX_MOT+1, NUM_PIX_MOT*2, 0);

  colorLed(LED_LUX, 0, 0, 1, NUM_PIX_MOT, 0);
  for (int i = 0; i < 512; i++) {
    angle_calc();
    GyX_offset_sum += GyX;
    delay(5);
  }
  GyX_offset = GyX_offset_sum >> 9;
  //Serial.print((SERIAL_FRENCH ? "Decalage GyX = " : "GyX offset value = ")); Serial.println(GyX_offset);
  colorLed(0, 0, 0, 1, NUM_PIX_MOT, 150);
  beep();
  beep();
  colorLed(LED_LUX, 0, 0, 1, NUM_PIXELS, 300);
  colorLed(0, 0, 0, 1, NUM_PIXELS, 150);
  colorLed(LED_LUX, 0, 0, 1, NUM_PIXELS, 300);
  colorLed(0, 0, 0, 1, NUM_PIXELS, 0);
}

// Reads the sensor and estimates the cube tilt.
// Also detects Vertex and Edge positions.
void angle_calc() {
  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(static_cast<uint8_t>(MPU6050), static_cast<size_t>(6), true);
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(static_cast<uint8_t>(MPU6050), static_cast<size_t>(6), true);
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

  // Checks for a possible edge and waits for reliable detection.
  const byte observed = detect_edge_motor();
  if (observed != edge_candidate) {
    edge_candidate = observed;
    edge_candidate_since = millis();
  }

  // Calculates angles for the current cube position.
  if (vertical_edge || (!vertical_vertex && observed != 0)) {
    edge_sensor_projection(vertical_edge ? edge_motor : observed);
  } else {
    robot_angleY += GyY * loop_time / 1000 / 131.0;
    Acc_angleY = atan2(AcXc, -AcZc) * 57.2958;
    robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);

    robot_angleX += GyX * loop_time / 1000 / 131.0;
    Acc_angleX = -atan2(AcYc, -AcZc) * 57.2958;
    robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount);
  }

  // Enables Vertex when the cube is properly resting on a corner.
  if (abs(AcX) < 2000 && abs(Acc_angleX) < 0.4 &&
      abs(Acc_angleY) < 0.4 && !vertical_vertex && !vertical_edge) {
    robot_angleX = Acc_angleX;
    robot_angleY = Acc_angleY;
    vertical_vertex = true;
    edge_candidate = 0;
  }

  // Enables Edge after confirming a stable edge position.
  if (!vertical_vertex && !vertical_edge && Mode == 0 &&
      calibrated && !calibrating && observed != 0 &&
      (uint32_t)(millis() - edge_candidate_since) >= EDGE_CONFIRM_MS &&
      fabsf(Acc_angleX) < EDGE_ARM_DEG) {
    robot_angleX = Acc_angleX;
    robot_angleY = Acc_angleY;
    edge_motor = observed;
    vertical_edge = true;
  }

  // Exits Vertex if the cube tilts too far.
  if (vertical_vertex && (abs(robot_angleX) > 7 || abs(robot_angleY) > 7)) {
    vertical_vertex = false;
  }
  // Exits Edge if the cube falls or the mode can no longer maintain balance.
  if (vertical_edge &&
      (Mode != 0 || !calibrated || calibrating ||
       !isfinite(robot_angleX) || !isfinite(robot_angleY) ||
       abs(robot_angleX) > EDGE_CUTOFF_DEG || abs(robot_angleY) > EDGE_CUTOFF_DEG)) {
    vertical_edge = false;
    edge_motor = 0;
    edge_candidate = 0;
  }
}

// Converts balance corrections into commands for the three motors.
void XYZ_to_threeWay(float pwm_X, float pwm_Y, float pwm_Z) {
  int16_t m1 = round((0.5 * pwm_X - 0.866 * pwm_Y) / 1.37 + pwm_Z);  
  int16_t m2 = round((0.5 * pwm_X + 0.866 * pwm_Y) / 1.37 + pwm_Z);
  int16_t m3 = -pwm_X / 1.37 + pwm_Z;  
  Motor1_control(m1);
  Motor2_control(m2);
  Motor3_control(m3);
}

// Uses all three wheel speeds to estimate cube motion.
void threeWay_to_XY(int in_speed1, int in_speed2, int in_speed3) {
  speed_X = ((in_speed3 - (in_speed2 + in_speed1) * 0.5) * 0.5) * 1.81;
  speed_Y = -(-0.866 * (in_speed2 - in_speed1)) / 1.1;
}

// Warns of low battery voltage using the buzzer and LEDs.
void battVoltage(double voltage) {
  if (voltage > 7 && voltage <= 9.6) {
    if (digitalRead(BUZZER) == LOW){
      digitalWrite(BUZZER, HIGH); 
      colorLed(0, LED_LUX, 0, 1, NUM_PIXELS, 0);
      } else {
      digitalWrite(BUZZER, LOW); 
      colorLed(0, 0, 0, 1, NUM_PIXELS, 0);
    }
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

// Sends the requested power to a motor driver.
void pwmSet(uint8_t channel, uint32_t value) {
  ledcWrite(channel, value);
}

// Sets motor 1 direction and power.
void Motor1_control(int sp) {
  if (Mode == 0){  
    sp = sp + motor1_speed;
  }
  if (sp < 0) 
    digitalWrite(DIR1, LOW);
  else 
    digitalWrite(DIR1, HIGH);
  sp = constrain(sp, -255, 255);
  pwmSet(PWM1_CH, 255 - abs(sp));
}

// Sets motor 2 direction and power.
void Motor2_control(int sp) {
  if (Mode == 0){
    sp = sp + motor2_speed;
  }
  if (sp < 0) 
    digitalWrite(DIR2, LOW);
  else 
    digitalWrite(DIR2, HIGH);
  sp = constrain(sp, -255, 255);
  pwmSet(PWM2_CH, 255 - abs(sp));
}

// Sets motor 3 direction and power.
void Motor3_control(int sp) {
  if (Mode == 0){
    sp = sp + motor3_speed;
  }
  if (sp < 0) 
    digitalWrite(DIR3, LOW);
  else 
    digitalWrite(DIR3, HIGH);
  sp = constrain(sp, -255, 255);
  pwmSet(PWM3_CH, 255 - abs(sp));
}

// Counts motor 1 encoder pulses to measure speed.
void ENC1_READ() {
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC1_1) << 1) | digitalRead(ENC1_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count1++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count1--;
  }
}

// Counts motor 2 encoder pulses.
void ENC2_READ() {
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC2_1) << 1) | digitalRead(ENC2_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count2++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count2--;
  }
}

// Counts motor 3 encoder pulses.
void ENC3_READ() {
  static int state = 0;
  state = (state << 2 | (digitalRead(ENC3_1) << 1) | digitalRead(ENC3_2)) & 0x0f;
  if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b) {
    enc_count3++;
  } else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07) {
    enc_count3--;
  }
}

// Receives commands from the Bluetooth application:
// modes, calibration, balance settings, and saving.
int TuningCommand(char param, char cmd) {
  switch (param) {

    // Requests the language used for messages.
    case 'L':
      if (cmd == '?') {
        SerialBT.println(SERIAL_FRENCH ? "LANG:FR" : "LANG:EN");
      }
      break;

    // Handles communication with the application.
    case 'T': 
      if (cmd == 'x'){
        if (ConnectBT == false){init_BT(); ConnectBT = true;}
        previousT_3 = currentT;
      }
      break;

    // Selects normal mode or motor tests.
    case 'm': 
      if (cmd == '0'){
        Mode = 0; loop_time = 15;
      }
      if (cmd == '1'){
        f = 1; lock = 0; Mode = 1; loop_time = 100;
      }
      colorLed(0, 0, 0, 1, NUM_PIXELS, 0);
      XYZ_to_threeWay(0, 0, 0);
      digitalWrite(BRAKE, LOW);
      motors_speed_X = 0;
      motors_speed_Y = 0;
    break;

    // Saves balance settings.
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

    // Reloads saved balance settings.
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

    // Updates or reads Vertex and Z rotation settings.
      // Each letter corresponds to a coefficient shown in the application.
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
    
    // Updates or reads Edge settings.
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
 
    // Starts calibration and records Vertex, then Edge positions.
    case 'c':
      if (cmd == '+' && !calibrating) {
        calibrating = true;
        SerialBT.println("11");
        Serial.println(SERIAL_FRENCH ? "\nCalibration en cours." : "\nCalibration in progress.");
        Serial.println(SERIAL_FRENCH ? "Placez le cube en position Vertex..." : "Place the cube in vertex position...");
        colorLed(LED_LUX, LED_LUX, 0, 1, NUM_PIXELS, 0);
        beep(); 
      }
      
      if (cmd == '-' && calibrating)  {
        if (abs(AcX) < 2000 && abs(AcY) < 2000) {
          offsets.ID = 96;
          offsets.acXv = AcX;
          offsets.acYv = AcY;
          offsets.acZv = AcZ + 16384;
          SerialBT.println("12");
          Serial.println(SERIAL_FRENCH ? "Vertex OK." : "Vertex OK.");
          Serial.println(SERIAL_FRENCH ? "Placez le cube en position Edge..." : "Place the cube in edge position...");
          vertex_calibrated = true;
          colorLed(0, 0, 0, 1, NUM_PIXELS, 0);
          colorLed(0, LED_LUX, LED_LUX, NUM_PIX_MOT*2+1, NUM_PIXELS, 0);
          beep();
        } else if (abs(AcX) > 7000 && abs(AcX) < 10000 && abs(AcY) < 2000 && vertex_calibrated) {
          SerialBT.println("13");
          Serial.println(SERIAL_FRENCH ? "Edge OK." : "Edge OK.");
          offsets.acXe = AcX;
          offsets.acYe = AcY;
          offsets.acZe = AcZ + 16384;
          colorLed(0, 0, 0, 1, NUM_PIXELS, 0);
          save();
          calibrated = true;
          calibrating = false;
          Serial.println(SERIAL_FRENCH ? "Calibration terminee." : "Calibration OK.");
          beep();
        } else {
          SerialBT.println("14");
          Serial.println(SERIAL_FRENCH ? "Les angles sont incorrects !" : "The angles are wrong!!!!!!");
          beep();
          beep();
        }
      }
      
      break;              
   }
   return 1;
}

// Original Bluetooth protocol unchanged.
int Tuning() {
  if (SerialBT.available() < 2) return 0;
  char param = SerialBT.read();
  char cmd = SerialBT.read();
  return TuningCommand(param, cmd);
}

// Non-blocking USB commands: c+, c-, m1, and m0 only.
void serialUSBCommands() {
  static char first = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r' || ch == '\n' || ch == ' ' || ch == '\t') continue;
    if (!first) {
      if (ch == 'c' || ch == 'm') first = ch;
      else Serial.println(SERIAL_FRENCH ? "Commande inconnue." : "Unknown command.");
    } else {
      char param = first;
      first = 0;
      if ((param == 'c' && (ch == '+' || ch == '-')) ||
          (param == 'm' && (ch == '0' || ch == '1'))) {
        if (param == 'm')
          Serial.println(SERIAL_FRENCH
            ? (ch == '1' ? "Demarrage des tests moteurs (cube immobilise)." : "Retour au mode normal.")
            : (ch == '1' ? "Starting motor tests (secure the cube)." : "Returning to normal mode."));
        TuningCommand(param, ch);
      } else Serial.println(SERIAL_FRENCH ? "Commande invalide : c+, c-, m1 ou m0." : "Invalid command: c+, c-, m1 or m0.");
    }
  }
}
#endif 
