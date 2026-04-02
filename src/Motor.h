#include <Arduino.h>
#include <ESP32Encoder.h>
#include <zPID.h>
// #include <ESP32MotorControl.h>
#include "ESC.h"
// #include "MotorControl.h"

int signum(double x)
{
  return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
};

class Motor
{
public:
  double pend_pos = 0;
  double pend_pos_m1 = 0;
  double pend_vel = 0;
  double pend_vel_m1 = 0;
  double pend_accel = 0;
  double pend_pos_rads = 0;
  double pend_vel_rads = 0;
  double origin_offset = 0;

  double cart_pos = 0;
  double cart_pos_m1 = 0;
  double cart_vel = 0;
  double cart_vel_m1 = 0;
  double cart_accel = 0;
  double max_accel = 0;

  double output = 0;

  bool PID_Enabled = false;
  bool LQR_Enabled = false;
  bool PO_Enabled = false;

  Motor(int _signal_pin, ESP32Encoder *drive, ESP32Encoder *pend)
  {
    /*

    */
    signal_pin = _signal_pin;

    pinMode(signal_pin, OUTPUT);

    cart_enc = drive;
    pend_enc = pend;

    motor = new ESC(_signal_pin);

    cart_PID = new zPID(&cart_pos, &PID_out, &setpoint, 0, 0, 0, 0.01);
  }

  void update_input(int ts_ms)
  {
    /*Takes a time step in ms and updates all state variable*/
    double ts = ts_ms / 1000.0;
    cart_pos_m1 = cart_pos;                            // Set last cart position to current cart position in ticks
    cart_vel_m1 = cart_vel;                            // Set last cart velocity to current cart celocity in ticks
    cart_pos = static_cast<double>(get_motor_count()); // Update current cart position in ticks
    cart_vel = (cart_pos - cart_pos_m1) / (ts);        // calculate cart velocity in ticks
    cart_accel = (cart_vel - cart_vel_m1) / (ts);

    pend_pos_m1 = pend_pos; // Set last pendulum position to current pendulum position in ticks
    pend_vel_m1 = pend_vel;
    pend_pos = static_cast<double>((get_pend_count())); // Update current pendulum position in ticks
    pend_vel = (pend_pos - pend_pos_m1) / (ts);         // calculate pendulum velocity in ticks/s
    pend_accel = (pend_vel - pend_vel_m1) / (ts);
    pend_pos_rads = pend_pos * 2 * PI / 8191; // Convert pendulum position to radians
    pend_vel_rads = pend_vel * 2 * PI / 8191; // Convert pendulum velocity to radians/s

    if (abs(cart_accel) > max_accel) max_accel = abs(cart_accel);
  }

  void set_PID_enabled(bool enable)
  {
    PID_Enabled = enable;
    if (enable){
      LQR_Enabled = false;
      PO_Enabled = false;
    }

  }

  void set_LQR_enable(bool enable)
  {
    LQR_Enabled = enable;
    if (enable){
      PID_Enabled = false;
      PO_Enabled = false;
    }
  }

  void set_PO_enable(bool enable)
  {
    PO_Enabled = enable;
    if (enable){
      PID_Enabled = false;
      LQR_Enabled = false;
    }
  }

  void config_PIDF(double kP, double kI, double kD, double _kF)
  {
    cart_PID->set_tunings(kP, kI, kD);
    kF = _kF;
  }

  void set_setpoint(double _setpoint)
  {
    /*Set the target position for the PID Controller*/
    setpoint = _setpoint;
    cart_PID->reset();
  }

  void set_percent_output(double percent)
  {
    /*Sets the output for controlling the motor directly*/
    output = percent;
  }

  void log_data()
  {
    /*Write state data from the PID controller to the serial console*/
    cart_PID->log_data();
  }

  void update_PID()
  {
    // Set the output to the new PID output if PID is enabled
    cart_PID->update();
    if (PID_Enabled)
      output = -PID_out + signum(cart_PID->get_error()) * kF;
  }

  void update_LQR()
  {
    // Set the output to the value of k matrix times each state variable
    // if the LQR controller is Enabled
    double out = k_gains[0] * ((cart_pos)*tick_to_cm - origin_offset) / 100.0 + k_gains[1] * cart_vel * tick_to_cm / 100.0 + k_gains[2] * pend_pos_rads + k_gains[3] * pend_vel_rads;
    if (LQR_Enabled)
      output = out + signum(out) * 0;
  }

  /**
   * @brief Appplies the current value of output in percent to the motor.
   */
  void write_output()
  {
    double o = output;

    // deadband, turn off is output is too small
    if (abs(o) < 0.5 || cart_pos > 12000 || cart_pos < -12000)
    {
      o = 0.0;
      motor->setMotorSpeed(o);
    }
    else
    {
      motor->setMotorSpeed(o + signum(o) * 1.0); // output plus small static voltage to overcome static friction,
    };
  }

  // Gets the position of the cart in ticks
  int get_motor_count()
  {
    return cart_enc->getCount();
  }

  // Gets the pendulum angle normalized such that the angle is always in the [-180, 180] range
  int get_pend_count()
  {
    int norm_angle = ((-pend_enc->getCount() + 4096) % 8192 + 8192) % 8192;
    return (norm_angle + 4096) % 8192 - 4096;
  }

  void debugInfo()
  {
    Serial.println("********Motor Outputs*********");
    Serial.print("Encoder Position: ");
    Serial.println(cart_enc->getCount());
    Serial.print("PID Out: ");
    Serial.println(PID_out);
    Serial.print("Output: ");
    Serial.println(output);
  }

private:
  const double tick_to_cm = 2.0 * 60.0 / (10 * 4095);
  ESC *motor;
  zPID *cart_PID;
  double kF = 0;

  double setpoint = 0;
  double PID_out = 0; // Output from the PID controller withought Feedforward

  ESP32Encoder *cart_enc;
  ESP32Encoder *pend_enc;

  //HBridge_Control *mc;

  double k_gains[4] = {-171.0589, -145.1050, -613.1004, -98.3010}; //{-177.6944, -150.5793, -635.2362, -101.5714};//{-51.0502, -96.8535, -910.3226, -79.3228};//-51.3130, -97.2580, -913.2107, -79.5040

  int signal_pin;
};