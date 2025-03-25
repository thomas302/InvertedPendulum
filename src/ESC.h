#include <ESP32Servo.h>

class ESC {
    public:
        ESC(int pin){
            motor = new Servo();
            motor->attach(33,1000,2000);
        }
        void setMotorSpeed(double output){
            int state = 0;
            double o = output;
      
            if (output > 0.5 || output < -0.5){
              if (output > 0){
                //Set LEDs to forward
                state = 1;
              }
              else{
                //Set LEDs to reverse
                state = 2;
              }
              //adjusts output to esc range.
              //normalizes to output range which
              if (o > 100) o = 100.0;
              if (o < -100) o = -100.0;
              o = ((o/100.0)*90.0 + 90.0);
            }
      
            switch(state){
              case 1:
              case 2:
                motor->write(o);
              break;
              case 0:
              default:
                motor->write(90.0);
              break;
            }
            
          }
    private:
        Servo* motor;
};