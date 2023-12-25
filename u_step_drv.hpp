#ifndef U_STEP_DRV_HPP
#define U_STEP_DRV_HPP

#include <stdint.h>
#include <iostream>
#include "Arduino.h"
#include "sin_lookup.hpp"

class UStepDrv {
  public:
    UStepDrv(std::array<uint8_t, 4> phase_pins, uint8_t inh_pin, unsigned ledc_offs = 0)
      : _ledc_offs(ledc_offs)
      , _phase_pins(phase_pins)
      , _inh_pin(inh_pin) {
      // Ensure drive is disabled before doing anything with PWMs
      pinMode(inh_pin, OUTPUT);
      digitalWrite(inh_pin,0);
      usleep(1000);
      for (int n = 0; n < 4; ++n) {
        ledcSetup(n + ledc_offs,_freq,_pwm_bits);
        ledcAttachPin(phase_pins[n], n + ledc_offs);
      }
      disable();
    }
    ~UStepDrv() {
      disable();
      for (int n = 0; n < 4; ++n) {
        ledcDetachPin(_phase_pins[n]);
      }
      // Let pin settle, then switch back to input
      usleep(1000);
      pinMode(_inh_pin, INPUT);
      _en = false;
    }
    void enable() {
      digitalWrite(_inh_pin, 1);
      _en = true;
      _mag = 0;
    }
    // Bottom 8 bits of a are phase angle, m is duty multiplier, 16 bit fractional
    void set(unsigned a, uint16_t m) {
      for (int n = 0; n < 2 ; ++n) {
        auto ph = sin_lookup(a + n * 64);
        uint32_t duty = dt_comp(_freq, _deadtime_ns[ (n << 1) + ph.second ? 1 : 0]) + ((m * ph.first) >> (32 - _pwm_bits));
        duty = duty > _pwm_max ? _pwm_max : duty;
        auto chan = _ledc_offs + n * 2; 
        ledcWrite(chan + (ph.second ? 1 : 0), _pwm_max - duty);
        ledcWrite(chan + (ph.second ? 0 : 1), _pwm_max);
      }
      _angle = a;
      _mag = m;
    }
    uint16_t mag() const { return _mag; }
    unsigned angle() const { return _angle; }
    bool enabled() const { return _en; }
    void disable() {
      digitalWrite(_inh_pin, 0);
      for (int n = _ledc_offs; n < _ledc_offs + 4; ++n) {
        ledcWrite(n, _pwm_max);
      }
      _en = false;
    }
    constexpr uint32_t freq() const { return _freq; }
  private:
    unsigned _ledc_offs;
    const std::array<uint8_t, 4> _phase_pins;
    const uint8_t _inh_pin;
    static constexpr auto _pwm_bits = 12;
    static const auto _pwm_max = (1 << _pwm_bits) -1;
    // ToDo: These should be configurable
    static constexpr std::array<uint32_t, 4> _deadtime_ns = { 4100, 5600, 5400, 8200 };
    static constexpr auto _freq = 15625;
    static constexpr uint32_t dt_comp(uint32_t freq, uint32_t deadtime_ns) { return ((deadtime_ns << _pwm_bits) / 1000) * _freq / 1000000; }
    unsigned _angle = 0;
    bool _en = false;
    uint16_t _mag = 0;
    friend std::ostream & operator << (std::ostream & os, UStepDrv const & drv);
};

inline std::ostream & operator << (std::ostream & os, UStepDrv const & drv) {
  os << "Drive: En=" << drv.enabled() 
     << " Mag=" << drv.mag() 
     << " Ang=" << drv.angle() << " (" << (drv.angle() % 256) << ")";
  return os;
}

#endif // U_STEP_DRV_HPP
