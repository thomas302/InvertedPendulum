#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

class HBridge_Control
{
  public:
    HBridge_Control(int pinF, int pinR)
    {
      conf_MCPWM(pinF, pinR);
    }

    void setMotorSpeed(double output) // output in percent, -100.0 to 100.0
    {
      double o = output;
      uint32_t wo = 0;

      if (o > 0.1 || o < -0.1) //smallest step size, appx .2% or (1/460)*100%
      {
        o = (o > 100) ? 100: (o < -100) ? -100: o;

        if (o > 0){
          wo = (uint32_t) (o/100.0*1000); 
          set_forward();
        }
        else if (o <= 0 ){
          wo = (uint32_t) (-o/100.0*1000);
          set_reverse();
        }
      }
      set_speed(wo);
    }

  private:
  mcpwm_cmpr_handle_t cmpA = NULL;
  mcpwm_cmpr_handle_t cmpB = NULL;
  mcpwm_gen_handle_t gena = NULL; 
  mcpwm_gen_handle_t genb = NULL;
  mcpwm_oper_handle_t oper;
  mcpwm_timer_handle_t timer = NULL;
  void conf_MCPWM(const int pinF, const int pinR)
  {

    // mcpwm timer
    mcpwm_timer_config_t timer_config = {
        .group_id = 1,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 6000000,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = 500,  
    };
    mcpwm_new_timer(&timer_config, &timer);

    mcpwm_operator_config_t operator_config = {
        .group_id = 1,
    };
    mcpwm_new_operator(&operator_config, &oper);

    mcpwm_operator_connect_timer(oper, timer);

    mcpwm_comparator_config_t comparator_config = {
        .flags= {.update_cmp_on_tez = true,}
    };
    mcpwm_new_comparator(oper, &comparator_config, &cmpB);
    mcpwm_new_comparator(oper, &comparator_config, &cmpA);

    // set the initial compare value for both comparators
    mcpwm_comparator_set_compare_value(cmpA, 0);
    mcpwm_comparator_set_compare_value(cmpB, 0);

    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = 33,
    };
    mcpwm_new_generator(oper, &generator_config, &gena);
    generator_config.gen_gpio_num = 25;
    mcpwm_new_generator(oper, &generator_config, &genb);

    mcpwm_generator_set_actions_on_timer_event(gena,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
            MCPWM_GEN_TIMER_EVENT_ACTION_END());
    mcpwm_generator_set_actions_on_compare_event(gena,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpA, MCPWM_GEN_ACTION_LOW),
            MCPWM_GEN_COMPARE_EVENT_ACTION_END());
    mcpwm_generator_set_actions_on_timer_event(genb,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
            MCPWM_GEN_TIMER_EVENT_ACTION_END());
    mcpwm_generator_set_actions_on_compare_event(genb,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpB, MCPWM_GEN_ACTION_LOW),
            MCPWM_GEN_COMPARE_EVENT_ACTION_END());

    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
    setMotorSpeed(-50);
    }

    void set_forward(){
      mcpwm_generator_set_force_level(gena, -1, true);
      mcpwm_generator_set_force_level(genb, 0, true);
    }

    void set_reverse(){
      mcpwm_generator_set_force_level(gena, 0, true);
      mcpwm_generator_set_force_level(genb, -1, true);
    }

    void set_speed(uint32_t speed){
      mcpwm_comparator_set_compare_value(cmpA, speed);
      mcpwm_comparator_set_compare_value(cmpB, speed);
    }

    void set_brake(){
      mcpwm_comparator_set_compare_value(cmpA, 0);
      mcpwm_comparator_set_compare_value(cmpB, 0);
    }

    static inline int signum(double x) {
      return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
    }
};