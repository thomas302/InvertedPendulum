#include <Arduino.h>
class zPID {
    public:
        zPID(double* _input, double* _output, double* _setpoint, double _kP, double _kI, double _kD, double _T){
            input = _input;
            output = _output;
            setpoint = _setpoint;

            kP = _kP;
            kI = _kI;
            kD = _kD;

            T = _T; 
            iT = 1/T;
        }

        void log_data() {
            Serial.printf("Input  : %f \n", *input);
            Serial.printf("Error  : %f \n", error);
            Serial.printf("Output : %f \n", *output);
        }

        void update() {
            //input is the encoders current position
            i = *input;
            s = *setpoint;

            error = s-i;

            *output = last_output + kP * (error-error_m1) + kI * T * 0.5 * (error + error_m1) + kD * iT * (error - 2 * error_m1 + error_m2);//o;

            // update last values for next loop
            last_output  =  *output;
            error_m2     =  error_m1;
            error_m1     =  error;
        }

        void set_tunings(double _kP, double _kI, double _kD) {
            kP = _kP;
            kI = _kI;
            kD = _kD;
        }

        void reset(){
            last_output = 0;
            error_m1 = 0;
            error_m2 = 0;
            error = 0;
        }

        double get_error() {
            return error;
        }

    private:
        double* input; 
        double* output;
        double* setpoint;

        double last_output = 0;
        
        double kP, kI, kD, T, iT; // iT = inverse T
        
        double error = 0;
        double error_m1 = 0;
        double error_m2 = 0;

        double i; //These really shouldnt be neccessary, but i had weird issues without them.
        double s;
};