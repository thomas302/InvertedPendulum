#include <Arduino.h>
#include <ESP32Encoder.h>
#include <zPID.h>
#include "ESP32MotorControl.h"

int signum(double x) {
  return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
};


class Motor {
  public:

    zPID* mPID;
    ESP32Encoder* driveEnc;

    ESP32MotorControl m = ESP32MotorControl();

    double kF = 0;
    double setpoint = 0;
    double output = 0;
    double input = 0;

    Motor(int _forward, int _reverse, int _enable, ESP32Encoder* drive){
      /*
      Takes input control pins for h-bridge, enable pin for pwm control
      and 2 encoder pins, and an encoder mode (0 = quadrature, 1 = 2x mode, 2 = 1x mode)
      */
      forward = _forward;
      reverse = _reverse;
      // enable tied high on h-bridge
      //enable = _enable;
      
      pinMode(forward, OUTPUT);
      pinMode(reverse, OUTPUT);
      //pinMode(enable, OUTPUT);

      driveEnc = drive;

      m.attachMotor(forward,reverse);
      //ledcAttach(enable, 5000, 10);
        
      mPID = new zPID(&input, &PID_out, &setpoint, 0,0,0,0);
    } 
    void update_input(){
      input = driveEnc->getCount();
    }

    void set_PID_enabled(bool enable){
      PID_Enabled = enable;
    }

    void config_PIDF(double kP, double kI, double kD, double _kF){
      mPID->set_tunings(kP, kI, kD);
      kF = _kF;
    }

    void set_setpoint(double _setpoint){
      setpoint = _setpoint;
    }

    void set_percent_output(double percent){
      output = percent;
    }  
    
    void update_PID(){
      mPID->update();
      output = PID_out + signum(mPID->get_error())*kF;
    }
    

    void write_output(){
      int state = 0;
      double o = output;
      if (output > 0){
        //Set Hbridge to forward
        state = 1;
      }
      else{
        //Set Hbridge to reverse
        state = 2;
        o*=-1;
      }

      //deadband, turn off is output is too small
      if ( o < 0.001){
        state = 0;
      }

      switch(state){
        case 1:
          m.motorForward(0, o);
        break;

        case 2:
          m.motorReverse(0, o);
        break;

        case 0:
        default:
          m.motorStop(0);
        break;
      }
    }

    int get_ticks(){
      return driveEnc->getCount();
    }
    
    void debugInfo(){
      Serial.println("********Motor Outputs*********");
      Serial.print("Encoder Position: ");
      Serial.println(driveEnc->getCount());
      Serial.print("PID Out: ");
      Serial.println(PID_out);
      Serial.print("Output: ");
      Serial.println(output);
    }

  private:
    double PID_out = 0;
    
    int forward; 
    int reverse; 
    //int enable;

    bool PID_Enabled = false;
};
