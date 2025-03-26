#include <Arduino.h>
#include <ESP32Encoder.h>
#include <zPID.h>
#include <ESP32MotorControl.h>
#include "ESC.h"

int signum(double x) {
  return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
};


class Motor {
  public:
    double pend_pos = 0;
    double pend_pos_m1 = 0;
    double pend_vel = 0;

    double cart_pos = 0;
    double cart_pos_m1 = 0;
    double cart_vel = 0;

    double output = 0;
    

    Motor(int _forward, ESP32Encoder* drive, ESP32Encoder* pend) {
      /*
      Takes input control pins for h-bridge, enable pin for pwm control
      and 2 encoder pins, and an encoder mode (0 = quadrature, 1 = 2x mode, 2 = 1x mode)
      */
      signal_pin = _forward;
      
      pinMode(signal_pin, OUTPUT);

      cart_enc = drive;
      pend_enc = pend;

      motor = new ESC(33);

      cart_PID = new zPID(&cart_pos, &PID_out, &setpoint, 0, 0, 0, 0.01);
    } 

    void update_input() {
      cart_pos_m1 = cart_pos;
      cart_pos = static_cast<double>(get_motor_count());
      cart_vel = (cart_pos-cart_pos_m1) * 0.5/ (0.02);

      pend_pos_m1 = pend_pos;
      pend_pos = static_cast<double>((get_pend_count())) ;
      pend_vel = (pend_pos - pend_pos_m1) * 0.5 / (0.02);

      pend_pos_rads = pend_pos * 2 * PI / 8191;
      pend_vel_rads = pend_vel * 2 * PI / 8191;
    }

    void set_PID_enabled(bool enable) {
      PID_Enabled = enable;
      if (LQR_Enabled && PID_Enabled) LQR_Enabled = false;
    }

    void set_LQR_enable(bool enable) {
      LQR_Enabled = enable;
      if (LQR_Enabled && PID_Enabled) PID_Enabled = false;
    }

    void config_PIDF(double kP, double kI, double kD, double _kF) {
      cart_PID->set_tunings(kP, kI, kD);
      kF = _kF;
    }

    void set_setpoint(double _setpoint) {
      setpoint = _setpoint;
      cart_PID->reset();
    }

    void set_percent_output(double percent) {
      output = percent;
    }  

    void log_data() {
      cart_PID->log_data();
    }
    
    void update_PID() {
      cart_PID->update();
      if (PID_Enabled) output = PID_out + signum(cart_PID->get_error())*kF;
    }

    void update_LQR(){
      double out = k_gains[0]*cart_pos*tick_to_cm/100.0 + k_gains[1]*cart_vel*tick_to_cm/100.0 + k_gains[2] * pend_pos_rads  + k_gains[3]*pend_vel_rads;
      if (LQR_Enabled) output = out;
    }
    
    /**
     * @brief Appplies the current value of output in percent to the motor.
     */
    void write_output() {
      double o = output;

      //deadband, turn off is output is too small
      if ( abs(o) < 0.1){
        o = 0.0;
      }

      motor->setMotorSpeed(o);
    }

    int get_motor_count() {
      return -cart_enc->getCount();
    }

    int get_pend_count() {
      int norm_angle = ((-pend_enc->getCount() + 4096)%8192 +8192)%8192;
      return (norm_angle + 4096)%8192 - 4096;
    }
    
    void debugInfo() {
      Serial.println("********Motor Outputs*********");
      Serial.print("Encoder Position: ");
      Serial.println(cart_enc->getCount());
      Serial.print("PID Out: ");
      Serial.println(PID_out);
      Serial.print("Output: ");
      Serial.println(output);
    }

  private:
    const double tick_to_cm = 2.0*60.0/(10*4095);
    ESC* motor;
    zPID* cart_PID;
    double kF = 0;

    double setpoint = 0;
    double PID_out = 0; //Output from the PID controller withought Feedforward

    double pend_pos_rads = 0;
    double pend_vel_rads = 0;

    ESP32Encoder *cart_enc;
    ESP32Encoder  *pend_enc;

    double k_gains[4] = {-173.4245, -183.3347, -959.6590, -124.6091};
    
    int signal_pin; 

    bool PID_Enabled = false;
    bool LQR_Enabled = false;
};
