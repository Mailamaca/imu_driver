
#include "driver/pcnt.h"

class MailaEncoder {
  private:
    int pinPulse;
    int pinCtrl;
    pcnt_channel_t channel;
    pcnt_unit_t unit;
    int16_t upperLimit;
    int16_t lowerLimit;
    int16_t filter;
    
    int16_t value=0;    

  public:
    MailaEncoder(int pinPulse,
                 int pinCtrl,
                 pcnt_channel_t channel,
                 pcnt_unit_t unit,
                 int16_t upperLimit,
                 int16_t lowerLimit,
                 uint16_t filter) {
              
      this->pinPulse = pinPulse;
      this->pinCtrl = pinCtrl;
      this->channel = channel;
      this->unit = unit;
      this->upperLimit = upperLimit;
      this->lowerLimit = lowerLimit;
      this->filter = filter;

      pcnt_config_t pcnt_config;
      pcnt_config.pulse_gpio_num = this->pinPulse;
      pcnt_config.ctrl_gpio_num = this->pinCtrl;
      pcnt_config.channel = this->channel;
      pcnt_config.unit = this->unit;
      pcnt_config.pos_mode = PCNT_COUNT_INC;
      pcnt_config.neg_mode = PCNT_COUNT_DIS;
      pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
      pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
      pcnt_config.counter_h_lim = this->upperLimit;
      pcnt_config.counter_l_lim = this->lowerLimit;
      pcnt_unit_config(&pcnt_config);
      
      pcnt_set_filter_value(this->unit, this->filter);
      pcnt_filter_enable(this->unit);
      
      pcnt_counter_pause(this->unit);
      pcnt_counter_clear(this->unit);
      pcnt_counter_resume(this->unit);

    }

    int16_t getValue() {
      pcnt_get_counter_value(this->unit, &this->value);
      return this->value;
    }    

    int16_t getDelta() {
      int16_t prevValue = this->value;
      pcnt_get_counter_value(this->unit, &this->value);
      int16_t newValue = this->value - prevValue;
      if (newValue<0) {
        newValue = newValue + this->upperLimit;
      }
      return newValue;
    }

    void resetValue() {
      pcnt_counter_clear(this->unit);
    }
    
};
