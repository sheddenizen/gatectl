#ifndef AS5600_HPP
#define AS5600_HPP

#include "log.hpp"

#include "Arduino.h"
#include "Wire.h"
#include <stdint.h>

class AS5600PosnSensor {
  public:
    static constexpr uint16_t resbits = 12;
    static constexpr uint16_t mask = (1 << resbits) - 1;
    explicit AS5600PosnSensor(char const * name, TwoWire & i2c, uint16_t offset) 
      : _name(name), _i2c(i2c), _offset(offset) {}
    uint16_t raw() {
      _i2c.beginTransmission(0x36);
      _i2c.write(uint8_t(0xc));
      _i2c.endTransmission();
      _i2c.requestFrom(0x36, 2);
      int retries = 1;
      int byte_count;

      while ((byte_count = _i2c.available()) < 2 && retries) {
        --retries;
        lg::W() << "Sensor " << _name << " got " << byte_count << " bytes with " << retries << " retries left";
        delay(1);
      }
      if (!retries && byte_count < 2) {
        _error = true;
        return _last_raw;
      }
      uint16_t val = _i2c.read();
      val = (val << 8) | _i2c.read();
      _last_raw = val;
      _error = false;
      return val;      
    }
    uint16_t operator()() { return (raw() + _offset) & mask; }
    uint16_t degr(uint16_t raw_val) const 
    { return ((raw_val + _offset) & mask) * 360 / (1 << resbits); }
    uint16_t degr() { return mrad(raw()); }
    uint16_t mrad(uint16_t raw_val) const 
    { return ((raw_val + _offset) & mask) * 100531 / (1 << (resbits+4)); }
    uint16_t mrad() { return mrad(raw()); }
    uint16_t last_raw() const { return _last_raw; }
    char const * name() const { return _name; }
    bool error() const { return _error; }

  private:
    char const * const _name;
    TwoWire & _i2c;
    uint16_t _offset;
    uint16_t _last_raw;
    bool _error = false;
};

std::ostream & operator << (std::ostream & os, AS5600PosnSensor const & sens) {
  uint16_t raw=sens.last_raw();
  os << sens.name() << '=' << sens.mrad(raw) << "mRadn(" << raw << ')';
  return os;
}

#endif // AS5600_HPP
