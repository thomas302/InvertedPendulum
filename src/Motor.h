#include <PID_v1.h>
#include <ESP32Encoder.h>
#include <Arduino.h>
#include <driver/ledc.h>

int signum(double x){
  return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
};

class Motor{
  public:
    PID *mPID;
    ESP32Encoder encoder;

    double kF = 0;
    double setpoint = 0;
    double output = 0;

    Motor(int _forward, int _reverse, int _enable, int _enc1, int _enc2, int encMode=0){
      /*
      Takes input control pins for h-bridge, enable pin for pwm control
      and 2 encoder pins, and an encoder mode (0 = quadrature, 1 = 2x mode, 2 = 1x mode)
      */
      forward = _forward;
      reverse = _reverse;
      enable = _enable;
      
      pinMode(forward, OUTPUT);
      pinMode(reverse, OUTPUT);
      pinMode(enable, OUTPUT);


      //ledcAttach(enable, 5000, 10);

      pinMode(_enc1, INPUT);
      pinMode(_enc2, INPUT);

      mPID = new PID(&input, &PID_out, &setpoint, 0,0,0,0);
      mPID->SetOutputLimits(-1.0, 1.0);
      mPID->SetSampleTime(25);
      mPID->SetTunings(1/15000, 0, 1, 1);
      mPID->SetMode(1);
      mPID->SetControllerDirection(DIRECT);

      ESP32Encoder::useInternalWeakPullResistors = puType::up;
      //encoder = ESP32Encoder::ESP32Encoder();
      switch(encMode){
        case 0:
          encoder.attachFullQuad(_enc1, _enc2);
        break;
        case 1:
          encoder.attachHalfQuad(_enc1, _enc2);
        break;
        case 2:
          encoder.attachSingleEdge(_enc1, _enc2);
        break;
        default:
          Serial.println("Encoder mode out of range, defaulted to full quad");
        break;
      }
    }
    void updateInput(){
      input = encoder.getCount();
    }

    void setPID_Enabled(bool enable){
      PID_Enabled = enable;
    }

    void configPIDF(double kP, double kI, double kD, double _kF){
      mPID->SetTunings(kP, kI, kD);
      kF = _kF;
    }

    void setSetpoint(double _setpoint){
      setpoint = _setpoint;
    }

    void setPercentOutput(double percent){
      output = percent;
    }

    void setBrake(){
      // Set h Bridge to brake
      output = 0;
    }

    void setBrakeMode(bool mode){
      // if true brake, else coast
      brakeMode = mode;
    }

    void updatePID(){
      input = encoder.getCount();
      bool outputUpdated = mPID->Compute();
      if(PID_Enabled){
        if (outputUpdated){
          setPercentOutput(PID_out);
          // + signum(PID_out) * kF;
        }
      }
    }

    void updatePIDNow(){
      mPID->calculateNow();
      output = PID_out;
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

      if ( o < 0.05){
        analogWrite(enable, 0);
        if (brakeMode){
          state = -1;
        }
        else{
          state = 0;
        }
        //Serial.println("brake");
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
      Serial.println(encoder.getCount());
      Serial.print("PID Out: ");
      Serial.println(PID_out);
      Serial.print("Output: ");
      Serial.println(output);
    }

  private:

    double input = 0;
    double PID_out = 0;
    
    int forward; 
    int reverse; 
    int enable;

    bool brakeMode = true;
    bool PID_Enabled = false;
};