#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

class ESC
{
  public:
    ESC(int pin)
    {
      conf_MCPWM(33);
    }

    void setMotorSpeed(double output) // output in percent, -100.0 to 100.0
    {
      double o = output;
      uint32_t wo = 1500;

      if (output > 0.2 || output < -0.2) //smallest step size, appx .2% or (1/460)*100%
      {
        // normalizes output to -100% to +100% range
        o = (o > 100) ? 100.0: (o < -100) ? -100.0: o;

        //scales output to esc range. 1000-2000us, with an 80us deadband centered at 1500us
        wo = std::round((o / 100.0) * 460 + signum(o) * 40 + 1500);
        writeMicros(wo);
      }
      else
      {
        writeMicros(wo);
      }
    }

  private:
    mcpwm_cmpr_handle_t comparator;
    void conf_MCPWM(const int pin)
    {
      mcpwm_timer_handle_t timer = NULL;
      mcpwm_timer_config_t timer_config = {
          .group_id = 0,
          .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
          .resolution_hz = 1000000, // 1MHz, or 1us per tick
          .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
          .period_ticks = 5000 // 5000 ticks, 200hz, or 5ms
      };
      mcpwm_new_timer(&timer_config, &timer);

      mcpwm_oper_handle_t oper = NULL;
      mcpwm_operator_config_t operator_config = {
          .group_id = 0, // operator must be in the same group to the timer
      };
      mcpwm_new_operator(&operator_config, &oper);

      mcpwm_operator_connect_timer(oper, timer);

      comparator = NULL;
      mcpwm_comparator_config_t comparator_config = {
          .flags = {
            .update_cmp_on_tez = true
          }
      };
      mcpwm_new_comparator(oper, &comparator_config, &comparator);

      mcpwm_gen_handle_t generator = NULL;
      mcpwm_generator_config_t generator_config = {
          .gen_gpio_num = pin,
      };
      mcpwm_new_generator(oper, &generator_config, &generator);

      mcpwm_comparator_set_compare_value(comparator, 1500); // sets to neutral output for esc, 1500 ticks/1500 us
      
      mcpwm_generator_set_action_on_timer_event(generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)); // Sets Pwm signal high at 0/timer start
      mcpwm_generator_set_action_on_compare_event(generator,
          MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)); // Sets signal low when gretater than comparator value

      mcpwm_timer_enable(timer);
      mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
    }

    void writeMicros(uint32_t time)
    {
      mcpwm_comparator_set_compare_value(comparator, time);
    }

    static inline int signum(double x) {
      return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
    }
};