#include "ESP32.h"
#include "functions.h"

// Prepares communications, LEDs, motors, and sensor at startup.
void setup() {
  // Starts communication with the computer and Bluetooth application.
  Serial.begin(115200);
  SerialBT.begin("ESP32-Cube");
  EEPROM.begin(EEPROM_SIZE);
  
  // Initializes the LEDs and plays a short startup animation.
  FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, NUM_PIXELS);  // Initializes the LEDs
  
  pinMode(BUZZER, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);
  pinMode(INT_LED, INPUT);

  // Holding the LED button during power-up inverts the default Vertex
  vertexAutoSpinEnabled = VERTEX_AUTO_SPIN;
  if (digitalRead(INT_LED) == LOW) {
    delay(20);  // Confirm the initial button level.
    if (digitalRead(INT_LED) == LOW) {
      vertexAutoSpinEnabled = !VERTEX_AUTO_SPIN;
      beep();
    }
  }
  Serial.println(vertexAutoSpinEnabled
      ? (SERIAL_FRENCH ? "Rotation Vertex auto : activee." : "Vertex auto spin: enabled.")
      : (SERIAL_FRENCH ? "Rotation Vertex auto : desactivee." : "Vertex auto spin: disabled."));

  if (START_ANIM_LEDS){
    for (i=0;i<LED_LUX;i+=LED_LUX/5){ colorLed(i, i, 0, 1, NUM_PIXELS, 0);}
    for (RGB_val=0;RGB_val<18;RGB_val++){
      Red = RGB_Color[RGB_val][0]; Green = RGB_Color[RGB_val][1]; Blue  = RGB_Color[RGB_val][2];
      colorLed(Red, Green, Blue, 1, NUM_PIXELS, 80); // Color, first and last LEDs, delay
    }
    for (i=LED_LUX;i>0;i-=LED_LUX/5){colorLed(i, i*0.7, 0, 1, NUM_PIXELS, 0);}
    colorLed(0, 0, 0, 1, NUM_PIXELS, 0);
  }

  // Initializes motor 1 and its encoder (rotation measurement).
  pinMode(DIR1, OUTPUT);
  pinMode(ENC1_1, INPUT);
  pinMode(ENC1_2, INPUT);
  attachInterrupt(ENC1_1, ENC1_READ, CHANGE);
  attachInterrupt(ENC1_2, ENC1_READ, CHANGE);
  ledcSetup(PWM1_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM1, PWM1_CH);
  Motor1_control(0);
  
  // Initializes motor 2 and its encoder.
  pinMode(DIR2, OUTPUT);
  pinMode(ENC2_1, INPUT);
  pinMode(ENC2_2, INPUT);
  attachInterrupt(ENC2_1, ENC2_READ, CHANGE);
  attachInterrupt(ENC2_2, ENC2_READ, CHANGE);
  ledcSetup(PWM2_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM2, PWM2_CH);
  Motor2_control(0);
  
  // Initializes motor 3 and its encoder.
  pinMode(DIR3, OUTPUT);
  pinMode(ENC3_1, INPUT);
  pinMode(ENC3_2, INPUT);
  attachInterrupt(ENC3_1, ENC3_READ, CHANGE);
  attachInterrupt(ENC3_2, ENC3_READ, CHANGE);
  ledcSetup(PWM3_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM3, PWM3_CH);
  Motor3_control(0);

  // Loads saved settings or uses default values.
  EEPROM.get(0, offsets);
  //offsets.ID = 0; // Debug only: Uncomment to force EEPROM initialization
  if(offsets.ID != 96){
    offsets.Red = Red; offsets.Green = Green; offsets.Blue = Blue; offsets.RGB_val = RGB_val;
    offsets.K1 = K1; offsets.K2 = K2; offsets.K3 = K3; offsets.K4 = K4; offsets.zK2 = zK2; offsets.zK3 = zK3;
    offsets.eK1 = eK1; offsets.eK2 = eK2; offsets.eK3 = eK3; offsets.eK4 = eK4;
    save();
  }else{
    Red = offsets.Red; Green = offsets.Green; Blue = offsets.Blue;
    K1 = offsets.K1; K2 = offsets.K2; K3 = offsets.K3; K4 = offsets.K4; zK2 = offsets.zK2; zK3 = offsets.zK3;
    eK1 = offsets.eK1; eK2 = offsets.eK2; eK3 = offsets.eK3; eK4 = offsets.eK4;
    calibrated = true;
  }

  // Initializes the motion sensor before operation.
  angle_setup();
  delay(1000);
}

// Runs continuously while the cube is powered on.
void loop() {
  currentT = millis();
  // Reads sensors and adjusts motors at regular intervals.
  if (currentT - previousT_1 >= loop_time) {
    serialUSBCommands();
    Tuning();
    angle_calc();
        
    // Reads wheel movement since the previous calculation.
    motor1_speed = enc_count1;
    enc_count1 = 0;
    motor2_speed = enc_count2;
    enc_count2 = 0;
    motor3_speed = enc_count3;
    enc_count3 = 0;
    

  // MODE 0: normal operation and balance control.
  if (Mode == 0){

    threeWay_to_XY(motor1_speed, motor2_speed, motor3_speed);
    motors_speed_Z = motor1_speed + motor2_speed + motor3_speed;
    
    // In Vertex, all three motors keep the cube upright.
    if (vertical_vertex && calibrated && !calibrating) {    
      digitalWrite(BRAKE, HIGH);
      gyroX = GyX / 131.0;
      gyroY = GyY / 131.0;
      gyroZ = GyZ / 131.0;
      gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;
      gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt;
      
      int pwm_X = constrain(K1 * robot_angleX + K2 * gyroXfilt + K3 * speed_X + K4 * motors_speed_X, -255, 255);
      int pwm_Y = constrain(K1 * robot_angleY + K2 * gyroYfilt + K3 * speed_Y + K4 * motors_speed_Y, -255, 255);
      // Checks whether the cube is stable enough to start rotating.
      bool spinStable = fabsf(robot_angleX) < VERTEX_SPIN_MAX_TILT &&
                        fabsf(robot_angleY) < VERTEX_SPIN_MAX_TILT &&
                        fabsf(gyroXfilt) < VERTEX_SPIN_MAX_GYRO &&
                        fabsf(gyroYfilt) < VERTEX_SPIN_MAX_GYRO;
      // If automatic rotation is enabled, gradually increases its speed.
      if (vertexAutoSpinEnabled && spinStable) {
        if (vertexSpinStableSince == 0) vertexSpinStableSince = currentT;
        if (currentT - vertexSpinStableSince >= VERTEX_SPIN_DELAY) {
          vertexSpinTarget = fminf(VERTEX_SPIN_SPEED, vertexSpinTarget + VERTEX_SPIN_RAMP);
        }
      } else {
        vertexSpinStableSince = 0;
        vertexSpinTarget = 0.0f;
      }
      // Without automatic rotation, retains the usual Z-axis correction.
      // Original Z regulation retained whenever auto-spin is inactive.
      const float baseZ = zK2 * gyroZ + zK3 * motors_speed_Z;

      // With automatic rotation, a separate correction maintains the target speed.
      // It does not change the usual Z-axis settings.
      // Independent AUTO rotation loop: does not use zK2 / zK3.
      float spinP = 0.0f;
      float spinOutput = baseZ;
      const bool spinActive = vertexAutoSpinEnabled && spinStable && vertexSpinTarget > 0.0f;
      if (spinActive) {
        const float dtSpin = fminf((currentT - previousT_1) * 0.001f, 0.05f);
        const float spinError = gyroZ - vertexSpinTarget;
        spinP = VERTEX_SPIN_KP * spinError;
        const float deltaI = VERTEX_SPIN_KI * spinError * dtSpin;
        const float candidateI = constrain(vertexSpinIntegral + deltaI,
                                           -VERTEX_SPIN_I_MAX, VERTEX_SPIN_I_MAX);
        const float candidateOutput = spinP + candidateI;
        if (!((candidateOutput < -255.0f && deltaI < 0.0f) ||
              (candidateOutput > 255.0f && deltaI > 0.0f))) {
          vertexSpinIntegral = candidateI;
        }
        spinOutput = spinP + vertexSpinIntegral;
      } else {
        vertexSpinIntegral = 0.0f;
      }
      
      // Distributes corrections across the three motors.
      int pwm_Z = constrain(spinOutput, -255.0f, 255.0f);
      motors_speed_X += speed_X / 5; 
      motors_speed_Y += speed_Y / 5;
      XYZ_to_threeWay(-pwm_X, pwm_Y, -pwm_Z);
      
    // On an edge (Edge), only the corresponding motor maintains balance.
    } else if (vertical_edge && calibrated && !calibrating) {
      vertexSpinStableSince = 0;
      vertexSpinTarget = 0.0f;
      vertexSpinIntegral = 0.0f;
      regulate_edge();
    } else {
      vertexSpinStableSince = 0;
      vertexSpinTarget = 0.0f;
      vertexSpinIntegral = 0.0f;
      XYZ_to_threeWay(0, 0, 0);
      digitalWrite(BRAKE, LOW);
      motors_speed_X = 0;
      motors_speed_Y = 0; 
      
      if  (calibrated && !calibrating) {
        RGB_Process();
      }
    }
  }

  // MODE 1: test sequence for the three motors and encoders.
  if (Mode == 1 ){
      vertexSpinStableSince = 0;
      vertexSpinTarget = 0.0f;
      vertexSpinIntegral = 0.0f;
      // Each motor is tested in both directions, then its encoder is checked.
      switch (f) {
        // Motor 1 test.
        case 1:
          colorLed(LED_LUX, LED_LUX, LED_LUX, 1, NUM_PIX_MOT, 0);
          digitalWrite(BRAKE, HIGH);
          if (lock) Serial.println(SERIAL_FRENCH ? "Rotation du moteur 1." : "Rotating motor 1."); SerialBT.println("46");
          Motor1_control(50);
          break;
        case 4:
          digitalWrite(BRAKE, LOW);
          if (lock) Serial.println(SERIAL_FRENCH ? "Arret." : "Stop."); SerialBT.println("41");
          Motor1_control(0);
          break;      
        case 7:
          digitalWrite(BRAKE, HIGH);
          if (lock) Serial.println(SERIAL_FRENCH ? "Changement de sens..." : "Change direction..."); SerialBT.println("42");
          Motor1_control(-50);
          break;
        case 10:
          digitalWrite(BRAKE, LOW);
          if (lock) Serial.println(SERIAL_FRENCH ? "Arret." : "Stop."); SerialBT.println("41");
          Motor1_control(0);
          break; 
        case 13:
          digitalWrite(BRAKE, HIGH);
          if (lock) Serial.println(SERIAL_FRENCH ? "Verification de l'encodeur..." : "Checking encoder..."); SerialBT.println("43");
          Motor1_control(70);
          break; 
        case 16:
          digitalWrite(BRAKE, LOW);
          if (lock && motor1_speed > 300) {
            Serial.println(SERIAL_FRENCH ? "Encodeur OK." : "Encoder OK."); 
            Serial.print(SERIAL_FRENCH ? "Vitesse: " : "Speed"); Serial.println(motor1_speed);
            s1.concat(motor1_speed);
            SerialBT.println("44"  + s1);
            Serial.println(SERIAL_FRENCH ? "Arret." : "Stop.");
          } else if (lock && motor1_speed <= 0) {
            Serial.println(SERIAL_FRENCH ? "Echec de l'encodeur." : "Encoder FAIL.");
            Serial.print(SERIAL_FRENCH ? "Vitesse: " : "Speed"); Serial.println(motor1_speed);
            s1.concat(motor1_speed);
            SerialBT.println("45"  + s1);
            Serial.println(SERIAL_FRENCH ? "Arret." : "Stop.");
          }
          s1 = "";
          Motor1_control(0);
          colorLed(0, 0, 0, 1, NUM_PIX_MOT, 0);
          break;      
  
        // Motor 2 test.
        case 19:
          colorLed(LED_LUX, LED_LUX, LED_LUX, NUM_PIX_MOT+1, NUM_PIX_MOT*2, 0);
          digitalWrite(BRAKE, HIGH);
          if (lock) Serial.println(SERIAL_FRENCH ? "Rotation du moteur 2." : "Rotating motor 2."); SerialBT.println("47");
          Motor2_control(50);
          break;
        case 22:
          digitalWrite(BRAKE, LOW);
          if (lock) Serial.println(SERIAL_FRENCH ? "Arret." : "Stop."); SerialBT.println("41");
          Motor2_control(0);
          break;      
        case 25:
          digitalWrite(BRAKE, HIGH);
          if (lock) Serial.println(SERIAL_FRENCH ? "Changement de sens..." : "Change direction..."); SerialBT.println("42");
          Motor2_control(-50);
          break;
        case 28:
          digitalWrite(BRAKE, LOW);
          if (lock) Serial.println(SERIAL_FRENCH ? "Arret." : "Stop."); SerialBT.println("41");
          Motor2_control(0);
          break; 
        case 31:
          digitalWrite(BRAKE, HIGH);
          if (lock) Serial.println(SERIAL_FRENCH ? "Verification de l'encodeur..." : "Checking encoder..."); SerialBT.println("43");
          Motor2_control(70);
          break; 
        case 34:
          digitalWrite(BRAKE, LOW);
          if (lock && motor2_speed > 300) {
            Serial.println(SERIAL_FRENCH ? "Encodeur OK." : "Encoder OK.");
            Serial.print(SERIAL_FRENCH ? "Vitesse: " : "Speed"); Serial.println(motor2_speed);
            s1.concat(motor2_speed);
            SerialBT.println("44"  + s1);
            Serial.println(SERIAL_FRENCH ? "Arret." : "Stop.");
          } else if (lock && motor2_speed <= 0) {
            Serial.println(SERIAL_FRENCH ? "Echec de l'encodeur." : "Encoder FAIL.");
            Serial.print(SERIAL_FRENCH ? "Vitesse: " : "Speed"); Serial.println(motor2_speed);
            s1.concat(motor2_speed);
            SerialBT.println("45"  + s1);
            Serial.println(SERIAL_FRENCH ? "Arret." : "Stop.");
          }
          s1 = "";
          Motor2_control(0);
          colorLed(0, 0, 0, NUM_PIX_MOT+1, NUM_PIX_MOT*2, 0);
          break;        
  
        // Motor 3 test.
        case 37:
          colorLed(LED_LUX, LED_LUX, LED_LUX, NUM_PIX_MOT*2+1, NUM_PIXELS, 0);
          digitalWrite(BRAKE, HIGH);
          if (lock) Serial.println(SERIAL_FRENCH ? "Rotation du moteur 3." : "Rotating motor 3."); SerialBT.println("48");
          Motor3_control(50);
          break;
        case 40:
          digitalWrite(BRAKE, LOW);
          if (lock) Serial.println(SERIAL_FRENCH ? "Arret." : "Stop."); SerialBT.println("41");
          Motor3_control(0);
          break;      
        case 43:
          digitalWrite(BRAKE, HIGH);
          if (lock) Serial.println(SERIAL_FRENCH ? "Changement de sens..." : "Change direction..."); SerialBT.println("42");
          Motor3_control(-50);
          break;
        case 46:
          digitalWrite(BRAKE, LOW);
          if (lock) Serial.println(SERIAL_FRENCH ? "Arret." : "Stop."); SerialBT.println("41");
          Motor3_control(0);
          break; 
        case 49:
          digitalWrite(BRAKE, HIGH);
          if (lock) Serial.println(SERIAL_FRENCH ? "Verification de l'encodeur..." : "Checking encoder..."); SerialBT.println("43");
          Motor3_control(70);
          break; 
        case 52:
          digitalWrite(BRAKE, LOW);
          if (lock && motor3_speed > 300) {
            Serial.println(SERIAL_FRENCH ? "Encodeur OK." : "Encoder OK.");
            Serial.print(SERIAL_FRENCH ? "Vitesse: " : "Speed"); Serial.println(motor3_speed);
            s1.concat(motor3_speed);
            SerialBT.println("44"  + s1);
            Serial.println(SERIAL_FRENCH ? "Arret." : "Stop.");
          } else if (lock && motor3_speed <= 0) {
            Serial.println(SERIAL_FRENCH ? "Echec de l'encodeur." : "Encoder FAIL.");
            Serial.print(SERIAL_FRENCH ? "Vitesse: " : "Speed"); Serial.println(motor3_speed);
            s1.concat(motor3_speed);
            SerialBT.println("45"  + s1);
            Serial.println(SERIAL_FRENCH ? "Arret." : "Stop.");
          }
          s1 = "";
          Motor3_control(0);
          colorLed(0, 0, 0, NUM_PIX_MOT*2+1, NUM_PIXELS, 0);
          break;        
        }
        lock = false;
  }
    
  previousT_1 = currentT;
}    
  // Every second: monitors the battery and advances motor tests.
  if (currentT - previousT_2 >= 1000) {

    battVoltage((double)analogRead(VBAT) / 204); // The value 204 must be calibrated against measured battery voltage!
    
    // Prompts for calibration if no balance settings are saved.
    if (Mode == 0 && !calibrated && !calibrating) {
      SerialBT.println("95");
      if (!calibrated_leds) {
        colorLed(0, LED_LUX, 0, 1, NUM_PIXELS, 0); 
        calibrated_leds = true; 
      } else {
        colorLed(0, 0, 0, 1, NUM_PIXELS, 0); 
        calibrated_leds = false; 
      }
      if (k-- <= 0){
        k = 4;
      Serial.println(SERIAL_FRENCH ? "\nCalibration requise." : "\nCalibration required.");
      Serial.println(SERIAL_FRENCH ? "Ne touchez pas au cube et attendez la fin du test (leds vertes)." : "Do not touch the cube and wait for the test to finish (green LEDs).");
      Serial.println(SERIAL_FRENCH ? "\nm1 : tester les moteurs et encodeurs (cube immobilise)." : "\nm1 : test motors and encoders (secure the cube).");
      Serial.println(SERIAL_FRENCH ? "m0 : arret du test des moteurs." : "m0 : stop motors test.");
      Serial.println(SERIAL_FRENCH ? "\nc+ : commencer la calibration." : "\nc+ : start calibration.");
      Serial.println(SERIAL_FRENCH ? "c- : valider Vertex, puis Edge." : "c- : confirm Vertex, then Edge.");
      }
    }
    
    // Advances to the next motor test step.c-
    if (Mode == 1){ 
      if (f++ == 53){ beep(); Serial.println(SERIAL_FRENCH ? "Fin des tests des moteurs." : "End of motor tests."); SerialBT.println("96"); f = 1; Mode = 0;} 
    lock = true;
    }
    previousT_2 = currentT;
  }
  
  // Ends the Bluetooth connection phase after the specified delay.
  if (ConnectBT == true && (currentT - previousT_3) > DelayBT)
      {
        init_BT();
        ConnectBT = false;
      }
}
