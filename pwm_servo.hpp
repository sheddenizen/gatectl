#ifndef PWM_SERVO_H
#define PWM_SERVO_H

#include "Arduino.h"
#include "log.hpp"

class SrvPulseOut {
  static constexpr auto _pwm_bits = 14;
  static constexpr uint32_t us_to_pwm(uint32_t us, uint32_t f) { return (1 << (_pwm_bits-6)) * us * f / 15625; }
  public:
    SrvPulseOut(char const * name, char const * units, uint8_t pin, uint32_t cal1us, uint32_t cal1pos, uint32_t cal2us, uint32_t cal2pos, unsigned ledcidx, uint32_t freq = 200)
      : _name(name)
      , _units(units)
      , _ledcidx(ledcidx)
      , _num(us_to_pwm(cal2us - cal1us, freq))
      , _div(cal2pos - cal1pos)
      , _offs(us_to_pwm(cal1us, freq) - cal1pos * _num / _div)
    {
        ledcSetup(ledcidx, freq, _pwm_bits);
        ledcAttachPin(pin, ledcidx);
    }
    void go(int32_t pos) {
      int32_t pwm = pos * _num / _div + _offs;
      ledcWrite(_ledcidx, pwm);
      _last_pos = pos;
    }
    void go() { go(_last_pos); }
    void stop() {
      ledcWrite(_ledcidx, 0);
    }
    int32_t get_last_pos() const { return _last_pos; }
    uint32_t raw() const { return ledcRead(_ledcidx); }
    char const * name() const { return _name; }
    char const * units() const { return _units; }
  private:
    char const * const _name;
    char const * const _units;
    uint8_t const _ledcidx;
    int32_t const _num;
    int32_t const _div;
    int32_t const _offs;
    int32_t _last_pos = 0;
};

std::ostream & operator<<(std::ostream & os, SrvPulseOut const & spo) {
  return os << spo.name() << '=' << spo.get_last_pos() << spo.units() << '(' << spo.raw() << ')';
}


#endif // PWM_SERVO_H
