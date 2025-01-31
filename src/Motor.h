#include <Arduino.h>
#include <ESP32Encoder.h>
#include <zPID.h>
#include "ESP32MotorControl.h"

int signum(double x) {
  return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
};

void PWM_setup(const int pin, const int channel) {
    ledcSetup(channel,1200, 65536u);
    ledcAttachPin(pin, channel);
};

class Motor {
  public:

    zPID* mPID;
    ESP32Encoder* driveEnc;
    ESP32Encoder* pendEnc;

    Servo esc;

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
      
      PWM_setup(forward, 0);
      PWM_setup(reverse, 1);

      driveEnc = drive;
      //ledcAttach(enable, 5000, 10);
        
      mPID = new zPID(&input, &PID_out, &setpoint, 0,0,0,0);
    } 
    void updateInput(){
      input = driveEnc->getCount();
    }

    void setPID_Enabled(bool enable){
      PID_Enabled = enable;
    }

    void configPIDF(double kP, double kI, double kD, double _kF){
      mPID->set_tunings(kP, kI, kD);
      kF = _kF;
    }

    void setSetpoint(double _setpoint){
      setpoint = _setpoint;
    }

    void setPercentOutput(double percent){
      output = percent;
    }  
    
    void updatePID(){
      output = PID_out + signum(mPID->get_error())*kF;
    }
    

    void writeOutput(){
      int state = 0;
      double o = output;
      if (output > 0){
        //Set Hbridge to forward
        state = 1;
        //Serial.println("f");
      }
      else{
        //Set Hbridge to reverse
        state = 2;
        o*=-1;
        //Serial.println("r");
      }

      if ( o < 0.001){
        ledcWrite(
        if (brakeMode){
          state = -1;
        }
        else{
          state = 0;
        }
      }

      switch(state){
        case 1:
          digitalWrite(forward, HIGH);
          digitalWrite(reverse, LOW);
          analogWrite(enable, 1023*o);
        break;

        case 2:
          digitalWrite(forward, LOW);
          digitalWrite(reverse, HIGH);
          analogWrite(enable, 1023*o);
        break;

        case -1:
          digitalWrite(forward, HIGH);
          digitalWrite(reverse, HIGH);
          analogWrite(enable, 0);
        break;

        default:
          digitalWrite(forward, LOW);
          digitalWrite(reverse, LOW);
          analogWrite(enable, 0);
        break;
      }
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
