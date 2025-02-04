#include <Arduino.h>
class zPID {
    public:
        double* input; 
        
        double* output;
        double last_output = 0;
 
        double* setpoint;
        
        double kP, kI, kD, T, iT;
        
        double error = 0;
        double error_m1 = 0;
        double error_m2 = 0;

        
        zPID(double* _input, double* _output, double* _setpoint, double _kP, double _kI, double _kD, double _T){
            input = _input;
            output = _output;
            setpoint = _setpoint;
            kP = _kP;
            kI = _kI;
            kD = _kD; 
            T = _T; 
            
            // inverse T
            iT = 1/T;
        }

        void log_data() {
            Serial.printf("Input: %f \n", *input);
            Serial.printf("Error: %f \n", error);
            Serial.printf("Output: %f \n", *output);
        }

        void update() {
            //input is the encoders current position
            double i = *input;
            double s = *setpoint;

            error = s-i;
            
            double o = last_output + kP * (error-error_m1) + kI * T * 0.5 * (error + error_m1) + kD * iT * (error - 2 * error_m1 + error_m2);
            
            *output = o;
            //update last values for next loop
            last_output = o;

            error_m2 = error_m1;

            error_m1 = error;
        }

        void set_tunings(double _kP, double _kI, double _kD) {
            kP = _kP;
            kI = _kI;
            kD = _kD;
        }

        double get_error() {
            return error;
        }
};