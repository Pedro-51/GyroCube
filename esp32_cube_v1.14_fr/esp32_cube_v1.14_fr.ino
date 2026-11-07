#include "ESP32.h"
#include "functions.h"

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32-Cube");
  EEPROM.begin(EEPROM_SIZE);
  
  FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, NUM_PIXELS);  // GRB ordering is typical
  
  pinMode(BUZZER, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);
  pinMode(INT_LED, INPUT);

  for (i=0;i<LED_LUX;i+=LED_LUX/5){ colorLed(i, i, 0, 1, NUM_PIXELS, 0);}
  for (RGB_val=0;RGB_val<18;RGB_val++){
    Red = RGB_Color[RGB_val][0]; Green = RGB_Color[RGB_val][1]; Blue  = RGB_Color[RGB_val][2];
    colorLed(Red, Green, Blue, 1, NUM_PIXELS, 80); //Red, Green, Blue, FirstPixel, LastPixel, Delay
  }
  for (i=LED_LUX;i>0;i-=LED_LUX/5){colorLed(i, i*0.7, 0, 1, NUM_PIXELS, 0);}
  colorLed(0, 0, 0, 1, NUM_PIXELS, 0);
  
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

  EEPROM.get(0, offsets);
  //offsets.ID = 0; // for debug
  if(offsets.ID == 255){
    offsets.ID = 0;
    offsets.Red = Red; offsets.Green = Green; offsets.Blue = Blue; offsets.RGB_val = RGB_val;
    offsets.K1 = K1; offsets.K2 = K2; offsets.K3 = K3; offsets.K4 = K4; offsets.zK2 = zK2; offsets.zK3 = zK3;
    offsets.eK1 = eK1; offsets.eK2 = eK2; offsets.eK3 = eK3; offsets.eK4 = eK4;
    save();
  }else if (offsets.ID == 96){
    Red = offsets.Red; Green = offsets.Green; Blue = offsets.Blue;
    K1 = offsets.K1; K2 = offsets.K2; K3 = offsets.K3; K4 = offsets.K4; zK2 = offsets.zK2; zK3 = offsets.zK3;
    eK1 = offsets.eK1; eK2 = offsets.eK2; eK3 = offsets.eK3; eK4 = offsets.eK4;
    calibrated = true;
  }

  angle_setup();
  delay(1000);
}

void loop() {
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
      bool spinStable = fabsf(robot_angleX) < VERTEX_SPIN_MAX_TILT &&
                        fabsf(robot_angleY) < VERTEX_SPIN_MAX_TILT &&
                        fabsf(gyroXfilt) < VERTEX_SPIN_MAX_GYRO &&
                        fabsf(gyroYfilt) < VERTEX_SPIN_MAX_GYRO;
      if (VERTEX_AUTO_SPIN && spinStable) {
        if (vertexSpinStableSince == 0) vertexSpinStableSince = currentT;
        if (currentT - vertexSpinStableSince >= VERTEX_SPIN_DELAY) {
          vertexSpinTarget = fminf(VERTEX_SPIN_SPEED, vertexSpinTarget + VERTEX_SPIN_RAMP);
        }
      } else {
        vertexSpinStableSince = 0;
        vertexSpinTarget = 0.0f;
      }
      int pwm_Z = constrain(zK2 * (gyroZ - vertexSpinTarget) + zK3 * motors_speed_Z, -255, 255);

      motors_speed_X += speed_X / 5; 
      motors_speed_Y += speed_Y / 5;
      XYZ_to_threeWay(-pwm_X, pwm_Y, -pwm_Z);
      
    } else if (vertical_edge && calibrated && !calibrating) {
      vertexSpinStableSince = 0;
      vertexSpinTarget = 0.0f;
      regulate_edge();
    } else {
      vertexSpinStableSince = 0;
      vertexSpinTarget = 0.0f;
      XYZ_to_threeWay(0, 0, 0);
      digitalWrite(BRAKE, LOW);
      motors_speed_X = 0;
      motors_speed_Y = 0; 
      
      if  (calibrated && !calibrating) {
        RGB_Process();
      }
    }
  }

  if (Mode == 1 ){
      switch (f) {
        case 1:
          colorLed(LED_LUX, LED_LUX, LED_LUX, 1, NUM_PIX_MOT, 0);
          digitalWrite(BRAKE, HIGH);
          if (lock) Serial.println("Rotation du moteur 1."); SerialBT.println("46");
          Motor1_control(50);
          break;
        case 4:
          digitalWrite(BRAKE, LOW);
          if (lock) Serial.println("Arret."); SerialBT.println("41");
          Motor1_control(0);
          break;      
        case 7:
          digitalWrite(BRAKE, HIGH);
          if (lock) Serial.println("Changement de sens..."); SerialBT.println("42");
          Motor1_control(-50);
          break;
        case 10:
          digitalWrite(BRAKE, LOW);
          if (lock) Serial.println("Arret."); SerialBT.println("41");
          Motor1_control(0);
          break; 
        case 13:
          digitalWrite(BRAKE, HIGH);
          if (lock) Serial.println("Verification de l'encodeur..."); SerialBT.println("43");
          Motor1_control(70);
          break; 
        case 16:
          digitalWrite(BRAKE, LOW);
          if (lock && motor1_speed > 300) {
            Serial.println("Encodeur OK."); 
            Serial.print("Vitesse : "); Serial.println(motor1_speed);
            s1.concat(motor1_speed);
            SerialBT.println("44"  + s1);
            Serial.println("Arret.");
          } else if (lock && motor1_speed <= 0) {
            Serial.println("Echec de l'encodeur.");
            Serial.print("Vitesse : "); Serial.println(motor1_speed);
            s1.concat(motor1_speed);
            SerialBT.println("45"  + s1);
            Serial.println("Arret.");
          }
          s1 = "";
          Motor1_control(0);
          colorLed(0, 0, 0, 1, NUM_PIX_MOT, 0);
          break;      
  
        case 19:
          colorLed(LED_LUX, LED_LUX, LED_LUX, NUM_PIX_MOT+1, NUM_PIX_MOT*2, 0);
          digitalWrite(BRAKE, HIGH);
          if (lock) Serial.println("Rotation du moteur 2."); SerialBT.println("47");
          Motor2_control(50);
          break;
        case 22:
          digitalWrite(BRAKE, LOW);
          if (lock) Serial.println("Arret."); SerialBT.println("41");
          Motor2_control(0);
          break;      
        case 25:
          digitalWrite(BRAKE, HIGH);
          if (lock) Serial.println("Changement de sens..."); SerialBT.println("42");
          Motor2_control(-50);
          break;
        case 28:
          digitalWrite(BRAKE, LOW);
          if (lock) Serial.println("Arret."); SerialBT.println("41");
          Motor2_control(0);
          break; 
        case 31:
          digitalWrite(BRAKE, HIGH);
          if (lock) Serial.println("Verification de l'encodeur..."); SerialBT.println("43");
          Motor2_control(70);
          break; 
        case 34:
          digitalWrite(BRAKE, LOW);
          if (lock && motor2_speed > 300) {
            Serial.println("Encodeur OK.");
            Serial.print("Vitesse : "); Serial.println(motor2_speed);
            s1.concat(motor2_speed);
            SerialBT.println("44"  + s1);
            Serial.println("Arret.");
          } else if (lock && motor2_speed <= 0) {
            Serial.println("Echec de l'encodeur.");
            Serial.print("Vitesse : "); Serial.println(motor2_speed);
            s1.concat(motor2_speed);
            SerialBT.println("45"  + s1);
            Serial.println("Arret.");
          }
          s1 = "";
          Motor2_control(0);
          colorLed(0, 0, 0, NUM_PIX_MOT+1, NUM_PIX_MOT*2, 0);
          break;        
  
        case 37:
          colorLed(LED_LUX, LED_LUX, LED_LUX, NUM_PIX_MOT*2+1, NUM_PIXELS, 0);
          digitalWrite(BRAKE, HIGH);
          if (lock) Serial.println("Rotation du moteur 3."); SerialBT.println("48");
          Motor3_control(50);
          break;
        case 40:
          digitalWrite(BRAKE, LOW);
          if (lock) Serial.println("Arret."); SerialBT.println("41");
          Motor3_control(0);
          break;      
        case 43:
          digitalWrite(BRAKE, HIGH);
          if (lock) Serial.println("Changement de sens..."); SerialBT.println("42");
          Motor3_control(-50);
          break;
        case 46:
          digitalWrite(BRAKE, LOW);
          if (lock) Serial.println("Arret."); SerialBT.println("41");
          Motor3_control(0);
          break; 
        case 49:
          digitalWrite(BRAKE, HIGH);
          if (lock) Serial.println("Verification de l'encodeur..."); SerialBT.println("43");
          Motor3_control(70);
          break; 
        case 52:
          digitalWrite(BRAKE, LOW);
          if (lock && motor3_speed > 300) {
            Serial.println("Encodeur OK.");
            Serial.print("Vitesse : "); Serial.println(motor3_speed);
            s1.concat(motor3_speed);
            SerialBT.println("44"  + s1);
            Serial.println("Arret.");
          } else if (lock && motor3_speed <= 0) {
            Serial.println("Echec de l'encodeur.");
            Serial.print("Vitesse : "); Serial.println(motor3_speed);
            s1.concat(motor3_speed);
            SerialBT.println("45"  + s1);
            Serial.println("Arret.");
          }
          s1 = "";
          Motor3_control(0);
          colorLed(0, 0, 0, NUM_PIX_MOT*2+1, NUM_PIXELS, 0);
          break;        
        }
        lock = 0;
  }
    
  previousT_1 = currentT;
}    
  if (currentT - previousT_2 >= 1000) {

    battVoltage((double)analogRead(VBAT) / 204); // value 204 must be selected by measuring battery voltage!
    
    if (Mode == 0 && !calibrated && !calibrating) {
      Serial.println("first you need to calibrate the balancing points (over bluetooth)...");
      SerialBT.println("95");
      if (!calibrated_leds) {
        colorLed(0, LED_LUX, 0, 1, NUM_PIXELS, 0); 
        calibrated_leds = true; 
      } else {
        colorLed(0, 0, 0, 1, NUM_PIXELS, 0); 
        calibrated_leds = false; 
      }
    }
    
    if (Mode == 1){
      f++; 
      if (f == 53){ beep(); Serial.println("Fin des tests des moteurs."); SerialBT.println("96"); f = 1; Mode = 0;} 
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
