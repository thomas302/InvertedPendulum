class zPID{
    double* input; 
    double* output;


    double last_output = 0;

    double setpoint;
    
    double kP, kI, kD, T;
    
    double error;
    double error_m1 = 0;
    double error_m2 = 0;

    
    zPID(double* _input, double* _output, double _setpoint,double _kP, double _kI, double _kD, double _T){
        input = _input;
        output = _output;
        setpoint = _setpoint;

        kP = _kP;
        kI = _kI;
        kD = _kD; 
        T = _T; 

    }

    double update(){
        //input is the encoders current position
        error = input - setpoint;

        output = last_output + (kP * (error-error_m1) + kI * (error + error_m1) + kD * (error - 2*error_m1 + error_m2));

        last_output = output

        error_m1 = error
        error_m2 = error_m1
    }
}
