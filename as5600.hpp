#ifndef AS5600_HPP
#define AS5600_HPP

#include "Arduino.h"
#include "Wire.h"
#include <stdint.h>

class AS5600PosnSensor {
  public:
    static constexpr uint16_t resbits = 12;
    static constexpr uint16_t mask = (1 << resbits) - 1;
    explicit AS5600PosnSensor(char const * name, TwoWire & i2c, uint16_t offset) 
      : _name(name), _i2c(i2c), _offset(offset) {}
    uint16_t raw() const {
      _i2c.beginTransmission(0x36);
      _i2c.write(uint8_t(0xc));
      _i2c.endTransmission();
      _i2c.requestFrom(0x36, 2);
      uint8_t count = 0;

      while ((count = _i2c.available()) < 2) {
        Serial.printf("Wait %hhu\n", count);
        delay(100);
      }
      uint16_t val = _i2c.read();
      val = (val << 8) | _i2c.read();
      const_cast<uint16_t &>(_last_raw) = val;
      return val;      
    }
    uint16_t operator()() const { return (raw() + _offset) & mask; }
    uint16_t degr(uint16_t raw_val) const 
    { return ((raw_val + _offset) & mask) * 360 / (1 << resbits); }
    uint16_t degr() const { return degr(raw()); }
    uint16_t last_raw() const { return _last_raw; }
    char const * name() const { return _name; }

  private:
    char const * const _name;
    TwoWire & _i2c;
    uint16_t _offset;
    uint16_t _last_raw;
};

std::ostream & operator << (std::ostream & os, AS5600PosnSensor const & sens) {
  uint16_t raw=sens.raw();
  os << sens.name() << '=' << sens.degr(raw) << "deg(" << raw << ')';
  return os;
}

#endif // AS5600_HPP
