#include "Motor.h"
#include <Arduino.h>
#include <ESP32Encoder.h>

ESP32Encoder pendEnc, motorEnc;

Motor* m;

hw_timer_t *Timer0_Cfg = NULL;
hw_timer_t *Timer1_Cfg = NULL;


const double belt_pitch = 0.2; // (cm/tooth)
const double num_pulley_teeth = 60.0; 
const double pi_val = 3.1415926535; 
const double num_encoder_ticks = 4096.0; // (ticks/rev)
const double tick_to_cm = 0.0014715;//belt_pitch * num_pulley_teeth / (pi_val * num_encoder_ticks); // (cm)

void write_cart_position() {
  int ticks = m->get_ticks();
  double cart_pos = static_cast<double>(ticks) * tick_to_cm;
  Serial.printf("cm: %f \n", cart_pos);
  Serial.printf("ticks: %d \n", ticks);
}


void IRAM_ATTR updatePID() {
    m->update_input();
    m->update_PID();
}

u_long start;
void setup() {
  
  pinMode(16, INPUT);
  pinMode(17, INPUT);

  pinMode(34, INPUT);
  pinMode(35, INPUT);

  pendEnc.attachFullQuad(16,17);
  pendEnc.clearCount();

  motorEnc.attachFullQuad(34, 35);
  motorEnc.clearCount();

  m = new Motor(33, 25, &motorEnc);

  Serial.begin(115200);

  m->set_setpoint(10.0/tick_to_cm);
  m->config_PIDF(0.1, 0.0003, 0, 0);
  m->set_PID_enabled(true);

  Serial.println(10.0/tick_to_cm);

  delay(1000);

  // Sets timer to update pid on 10ms loop time
  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &updatePID, true);
  timerAlarmWrite(Timer0_Cfg, 10*1000, true);
  timerAlarmEnable(Timer0_Cfg);

  start = millis();
}

void loop() {
    m->write_output();
    //Run logging at a slower speed so it doesnt overwhelm the serial monitor
    u_long elapsed = millis() - start;
    if (elapsed > 750){
      m->log_data();
      write_cart_position();
      start = millis();
    }
    
}

