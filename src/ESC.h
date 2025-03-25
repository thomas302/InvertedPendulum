#include <ESP32Servo.h>
#include <stdio.h>

#include "esp_system.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

int signum(double x) {
  return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
};

class ESC
{
public:
  ESC(int pin)
  {
    //motor = new Servo();
    //motor->attach(33, 1000, 2000);
    confMCPWM(33);
  }
  void setMotorSpeed(double output)
  {
    int state = 0;
    double o = output;
    int wo = 1500;

    if (output > 0.2 || output < -0.2)
    {
      state = 1;
      // normalizes output to -100% to +100% range
      o = (o > 100) ? 100.0: (o < -100) ? -100.0: o;
      //scales output to esc range.
      int wo = std::round((o / 100.0) * 460 + signum(o) * 40 + 1500);
    }

    switch (state)
    {
      case 1:
        writeMicros(wo);
      break;
      
      case 0:
      default:
        writeMicros(1500);
      break;
    }
  }

private:
  void confMCPWM(int pin)
  {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, pin);

    const int TIMER_FREQUENCY = 160000000;

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 180; // frequency (hz),
    pwm_config.cmpr_a = 0;      // duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;      // duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); // Configure PWM0A & PWM0B with above settings
    mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, TIMER_FREQUENCY);
  }

  void writeMicros(int time)
  {
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, time);
  }

  Servo *motor;
};