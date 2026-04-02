#include "Motor.h"
#include <Arduino.h>
#include <ESP32Encoder.h>

ESP32Encoder pendEnc, motorEnc;

Motor *m;

hw_timer_t *Timer0_Cfg = NULL;

u_long start;
u_long t_count = 0;
int offset_count = 0;

const double tick_to_cm = 2.0 * 60.0 / (10 * 4095); // belt_pitch (mm/th) * num_pulley_teeth (th/rev) / (10(mm/cm)*num_encoder_ticks(/rev) // (cm)
const double tick_to_deg = 360.0 / 8192.0;
const int ts_ms = 10;
double setpoint = 10;
double max_accel = 0;

bool swingup = true;
double appx_tanh(double x, double b){
  return (pow(b,x)-pow(b, -x))/(pow(b,x)+pow(b, -x));
}

double get_nrg_error() {
    double mp = 93.0 / 1000.0;
    double l  = 0.79 / 2.0;
    double g  = 9.81;

    double th  = m->pend_pos_rads;
    double dth = m->pend_vel_rads;

    double E = 0.5 * mp * l * l * dth * dth
             + mp * g * l * (1.0 - cos(th));

    double E_s = 0.0;

    return E - E_s;
}


double get_swingup_output(double k, double dist_threshold = 20) {
    double dist = (m->cart_pos)*tick_to_cm;

    double th  = m->pend_pos_rads;
    double dth = m->pend_vel_rads;

    double smoothing_factor = ((dth * cos(th) == 0) ? 1 : appx_tanh((dth * cos(th)), 1000));
    double output = k * get_nrg_error() * smoothing_factor;

    //Wait until output is moving towards the center when the cart is beyond ±dist_threshold
    //if ((dist > dist_threshold && output > 0) || (dist < -dist_threshold && output < 0)) output = 0;

    return output;
}

void write_positions()
{
  int m_ticks = m->get_motor_count();
  int p_ticks = m->get_pend_count();

  double cart_pos = static_cast<double>(m_ticks) * tick_to_cm; // paranoid conversion to double
  double pend_pos = static_cast<double>((p_ticks)) * tick_to_deg;

  Serial.printf("time (s): %f ", (float)t_count); // cast to float to ensure proper formatting
  Serial.printf("cart_pos (cm): %f ", cart_pos);
  Serial.printf("pend_pos (deg): %f ", pend_pos);
  Serial.printf("cart_vel (cm/s): %f ", m->cart_vel * tick_to_cm);
  Serial.printf("cart_accel (cm/s): %F", m->cart_accel * tick_to_cm);
  Serial.printf("pend_vel (deg/s): %f ", m->pend_vel * tick_to_deg);
  Serial.printf("setpoint (cm): %f ", setpoint);
  Serial.printf("output: %f ", m->output);
  Serial.printf("ticks: %d \r\n", m_ticks); // output ticks as signed number
}

double gain_k = 26;
void test_nrg(){
  Serial.printf("Nrg Error: %f \n", get_nrg_error());
  Serial.printf("Swingup Output: %f \n", get_swingup_output(gain_k));
}

double last_o = 0.0;
double max_increase = 5.0;
void IRAM_ATTR updatePID()
{
  if (abs(m->pend_pos)*tick_to_deg < 10 && !m->LQR_Enabled)
  {
    m->set_LQR_enable(true);
    swingup = false;
  }
  else if (abs(m->pend_pos)*tick_to_deg > 15  && m->LQR_Enabled)
  {
    swingup = true;
    m->set_PO_enable(true);
  }
  else if (!m->LQR_Enabled){
    swingup = true;
  }

  m->update_input(ts_ms);
  m->update_PID();
  m->update_LQR();
  
  if (swingup){
    double o = get_swingup_output(gain_k);
    
    m->set_percent_output(o);
  }

  m->write_output();
}

void setup()
{
  pinMode(16, INPUT);
  pinMode(17, INPUT);
  pendEnc.attachFullQuad(34, 35);
  pendEnc.clearCount();
  delay(100);

  pinMode(34, INPUT);
  pinMode(35, INPUT);
  motorEnc.attachFullQuad(16, 17);
  motorEnc.clearCount();
  delay(100);

  m = new Motor(33, &motorEnc, &pendEnc);
  

  Serial.begin(115200);

  m->set_setpoint(10.0 / tick_to_cm);
  m->config_PIDF(0.065, 0.0065, 0.004, 5);
  m->set_PID_enabled(false);

  m->set_LQR_enable(false);
  delay(1000);
  // ms
  // Sets timer to update pid on 10ms loop time
  Timer0_Cfg = timerBegin((uint32_t)1000000); // 1MHz frequency or 1 us per tick
  timerAttachInterrupt(Timer0_Cfg, &updatePID);
  timerAlarm(Timer0_Cfg, ts_ms * 1000, true, 0); // 20,000 ticks/us or 20ms
  timerStart(Timer0_Cfg);

  m->set_percent_output(0);
  start = micros();
}

double duty = 0;
void origin_offset()
{
  if (offset_count == 20)
  {
    m->origin_offset = 15;
    Serial.println("Offset: 15");
  }
  if (offset_count == 40)
  {
    m->origin_offset = -15;
    Serial.println("Offset: -15");
  }
  if (offset_count == 60)
  {
    m->origin_offset = 0;
    offset_count = 0;
    Serial.println("Offset: 0");
  }
}

void write_info()
{
  u_long t = micros();
  u_long elapsed = t - start;
  if (elapsed > 500000)
  { // outputs cart position every 500ms
    start = micros();
    if (m->LQR_Enabled) offset_count += 1; // Guarantees the pendulum will initially be balcnicng at the tracks centre point.
    else offset_count = 0;
    t_count += 5;
    write_positions();
    if(swingup) Serial.println("Swingin UP!");
    else if (m->LQR_Enabled) Serial.println("Runing LQR");
  }
}

int state = 0;

void loop()
{
  write_info();
  
  if (m->LQR_Enabled){
    origin_offset();
  }

  if (abs(m->pend_pos)*tick_to_deg < 10 && !m->LQR_Enabled)
  {
    Serial.println("LQR Enabled");
  }
  else if (abs(m->pend_pos)*tick_to_deg > 15  && m->LQR_Enabled)
  {
    Serial.println("LQR Disabled");
  }
    
}