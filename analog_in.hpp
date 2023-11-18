#ifndef ANALOG_IN_HPP
#define ANALOG_IN_HPP

#include <stdint.h>
#include <ostream>
#include "Arduino.h"

class AnalogIn {
  public:
    AnalogIn(char const * name, char const * units, uint8_t pin, int16_t cal1raw, int cal1val, int16_t cal2raw, int cal2val, adc_attenuation_t atten = ADC_0db)
      : _name(name), _units(units), _pin(pin)
      ,  _num(cal2val - cal1val)
      ,  _div(cal2raw - cal1raw)
      ,  _offs(cal1val - cal1raw * _num / _div)
    {
      analogSetPinAttenuation(pin, atten);
    }

    int raw() const { return analogRead(_pin); }
    int scale(int raw) const { return raw * _num / _div + _offs; } 
    int operator() () const { return scale(raw()); }

    char const * name() const { return _name; };
    char const * units() const { return _units; };
    
  private:
    char const * _name;
    char const * _units;
    uint8_t _pin;
    int _num;
    int _div;
    int _offs;
    friend std::ostream & operator << (std::ostream & os, AnalogIn const & ai);
};

std::ostream & operator << (std::ostream & os, AnalogIn const & ai) {
  int raw=ai.raw();
  os << ai.name() << '=' << ai.scale(raw) << ai.units() << '(' << raw << ')';
  return os;
}

#endif // ANALOG_IN_HPP
