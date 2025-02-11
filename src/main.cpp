#include "Motor.h"
#include <Arduino.h>
#include <ESP32Encoder.h>

ESP32Encoder pendEnc, motorEnc;

Motor* m;

hw_timer_t *Timer0_Cfg = NULL;
hw_timer_t *Timer1_Cfg = NULL;

u_long start;
u_long t_count = 0;
int count = 0;

const double tick_to_cm = 0.0014715; //belt_pitch * num_pulley_teeth / (pi_val * num_encoder_ticks); // (cm)
const double tick_to_deg = 360.0/4096.0;

void write_cart_position() {
  int ticks = m->get_ticks();
  double cart_pos = static_cast<double>(ticks) * tick_to_cm;

  double pend_pos = static_cast<double>(ticks % 360) * tick_to_deg;
  Serial.printf("time (s): %f ", (float)t_count);
  Serial.printf("cart_pos (cm): %f ", cart_pos);
  Serial.printf("pend_pos (deg): %f ", pend_pos);
  Serial.printf("cart_vel (cm/s): %f ", m->velocity * tick_to_cm);
  Serial.printf("pend_vel (deg/s): %f \r\n", m->velocity * tick_to_deg); // This needs to be a different name, cant have velocity of pendulum and motor the same
  Serial.printf("ticks: %d \r\n", ticks);
}


void IRAM_ATTR updatePID() {
    m->update_input();
    m->update_PID();
}

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
  m->config_PIDF(0.08, 0.003, 0.002, 35);
  m->set_PID_enabled(true);

  Serial.println(10.0/tick_to_cm);

  delay(1000);

  // Sets timer to update pid on 10ms loop time
  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &updatePID, true);
  timerAlarmWrite(Timer0_Cfg, 10*1000, true);
  timerAlarmEnable(Timer0_Cfg);

  start = micros();
}

void loop() {
    m->write_output();
    //Run logging at a slower speed so it doesnt overwhelm the serial monitor
    u_long t = micros();
    u_long elapsed = t - start;
    if (elapsed > 20000){ //outputs cart position every 20ms
      start = micros();
      count += 1;
      t_count += 20;
      write_cart_position();
    }


    if (count == 300) {
      m->set_setpoint(25.0/tick_to_cm);
      Serial.printf("setpoint: %f", 25.0/tick_to_cm);
    }else if (count == 600) {
      m->set_setpoint(0/tick_to_cm);
      Serial.printf("setpoint: %f", 0.0/tick_to_cm);
    }else if (count == 900) {
      m->set_setpoint(-25/tick_to_cm);
      Serial.printf("setpoint: %f", -25.0/tick_to_cm);
    } else if (count >= 1200){
      m->set_setpoint(10.0);
      Serial.printf("setpoint: %f", 10.0/tick_to_cm);
      count = 0;
    }


    
}

