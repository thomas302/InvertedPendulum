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
  int m_ticks = m->get_motor_count();
  int p_ticks = m->get_pend_count();
  double cart_pos = static_cast<double>(m_ticks) * tick_to_cm;
  double pend_pos = static_cast<double>(p_ticks % 4096) * tick_to_deg;
  Serial.printf("time (s): %f ", (float)t_count);
  Serial.printf("cart_pos (cm): %f ", cart_pos);
  Serial.printf("pend_pos (deg): %f ", pend_pos);
  Serial.printf("cart_vel (cm/s): %f ", m->velocity * tick_to_cm);
  Serial.printf("pend_vel (deg/s): %f ", m->pend_vel * tick_to_deg);
  Serial.printf("ticks: %d \r\n", m_ticks);
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

  m = new Motor(33, 25, &motorEnc, &pendEnc);

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

#include <cmath>

// Function to calculate the underdamped response at a given time t
double underdamped_response(double t) {
    // Given parameters
    double settling_time = 10.0;   // Desired settling time (in seconds)
    double overshoot_percentage = 10.0;  // Desired overshoot (as a percentage)
    
    // Calculate damping ratio (zeta) using overshoot formula: Mp = exp(-pi * zeta / sqrt(1 - zeta^2))
    double Mp = overshoot_percentage / 100.0;  // Convert overshoot percentage to decimal
    double zeta = -log(Mp) / (M_PI * std::sqrt(1 + log(Mp) * log(Mp)));
    
    // Calculate natural frequency (omega_n) using settling time formula: ts = 4 / (zeta * omega_n)
    double omega_n = 4 / (settling_time * zeta);
    
    // Calculate damped natural frequency: omega_d = omega_n * sqrt(1 - zeta^2)
    double omega_d = omega_n * std::sqrt(1.0 - zeta * zeta);
    
    // Calculate the underdamped response using the standard formula
    double response = 1 - std::exp(-zeta * omega_n * t) *
                      (std::cos(omega_d * t) + (zeta / std::sqrt(1 - zeta * zeta)) * std::sin(omega_d * t));
    
    return response;
}

void loop() {
    m->write_output();
    //Run logging at a slower speed so it doesnt overwhelm the serial monitor
    u_long t = micros();
    u_long elapsed = t - start;

    if (elapsed > 1000000){ //outputs cart position every 50ms
      start = micros();
      count += 1;
      t_count += 50;
      write_cart_position();
      //Serial.printf(" cm: %f", underdamped_response((double) t_count/1000.0));
      //Serial.printf(" time: %f \r\n", (double) t_count/1000.0);
    }

    //if ((double) t_count/1000.0 > 30) t_count = 0;


    if (count == 300) {
      m->set_setpoint(25.0/tick_to_cm);
      //Serial.printf("setpoint: %f", 0.5);
    }else if (count == 600) {
      m->set_setpoint(0/tick_to_cm);
      //Serial.printf("setpoint: %f", 0.0);
    }else if (count == 900) {
      m->set_setpoint(-25/tick_to_cm);
      //Serial.printf("setpoint: %f", 1.0);
    } else if (count >= 1200){
      m->set_setpoint(10.0);
      //Serial.printf("setpoint: %f", .75);
      count = 0;
    }


    
}

