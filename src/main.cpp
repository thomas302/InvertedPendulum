#include "Motor.h"
#include <Arduino.h>
#include <ESP32Encoder.h>

ESP32Encoder pendEnc, motorEnc;

Motor* m;

hw_timer_t *Timer0_Cfg = NULL;

u_long start;
u_long t_count = 0;
int count = 0;

const double tick_to_cm = 2.0*60.0/(10*4095); //belt_pitch (mm/th) * num_pulley_teeth (th/rev) / (10(mm/cm)*num_encoder_ticks(/rev) // (cm)
const double tick_to_deg = 360.0/8192.0;
double setpoint = 10;

void write_cart_position() {
  int m_ticks = m->get_motor_count();
  int p_ticks = m->get_pend_count();

  double cart_pos = static_cast<double>(m_ticks) * tick_to_cm; //paranoid conversion to double
  double pend_pos = static_cast<double>((p_ticks)) * tick_to_deg;

  Serial.printf("time (s): %f ", (float) t_count); //cast to float to ensure proper formatting
  Serial.printf("cart_pos (cm): %f ", cart_pos);
  Serial.printf("pend_pos (deg): %f ", pend_pos);
  Serial.printf("cart_vel (cm/s): %f ", m->cart_vel * tick_to_cm);
  Serial.printf("pend_vel (deg/s): %f ", m->pend_vel * tick_to_deg);
  Serial.printf("setpoint (cm): %f ", setpoint);
  Serial.printf("output: %f ", m->output);
  Serial.printf("ticks: %d \r\n", m_ticks); // output ticks as signed number
  // time (s): XX cart_pos (cm): XX pend_pos (deg): XX cart_vel (cm/s): XX ... ticks: XX
}


void IRAM_ATTR updatePID() {
    m->update_input();
    m->update_PID();
    m->update_LQR();
    m->write_output();
}

void setup() {
  pinMode(16, INPUT);
  pinMode(17, INPUT);
  pendEnc.attachFullQuad(34,35);
  pendEnc.clearCount();

  pinMode(34, INPUT);
  pinMode(35, INPUT);
  motorEnc.attachFullQuad(16, 17);
  motorEnc.clearCount();

  m = new Motor(33, &motorEnc, &pendEnc);

  Serial.begin(115200);
  m->set_setpoint(10.0/tick_to_cm);
  m->config_PIDF(0.065, 0.0065, 0.004, 5);

  m->set_PID_enabled(false);
  m->set_LQR_enable(true);
  delay(1000);

  // Sets timer to update pid on 10ms loop time
  Timer0_Cfg = timerBegin((uint32_t) 1000000); // 1MHz frequency or 1 us per tick
  timerAttachInterrupt(Timer0_Cfg, &updatePID);
  timerAlarm(Timer0_Cfg, 20*1000, true, 0);//20,000 ticks/us or 20ms
  timerStart(Timer0_Cfg);
  
  start = micros();
}

void loop() {
    u_long t = micros();
    u_long elapsed = t - start;
    if (elapsed > 50000){ //outputs cart position every 50ms
      start = micros();
      count += 5;
      t_count += 5;
      write_cart_position();
    }
}

