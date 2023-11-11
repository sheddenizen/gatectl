#ifndef PCF8574_HPP
#define PCF8574_HPP

#include "log.hpp"

#include "Arduino.h"
#include "Wire.h"
#include <stdint.h>

class PCF8574Gpio {
  public:
    explicit PCF8574Gpio(char const * name, TwoWire & i2c, uint8_t addroffs) 
      : _name(name), _i2c(i2c), _addr(0x20 + addroffs) {}
    uint8_t get() const {
      _i2c.beginTransmission(_addr);
      _i2c.endTransmission();
      _i2c.requestFrom(_addr, uint8_t(1));
      int retries = 1;
      int byte_count;

      while (!_i2c.available() && --retries) {
        lg::W() << "Sensor " << _name << " got " << byte_count << " bytes with " << retries << " retries left";
        delay(10);
        if (!retries)
          return _last_raw;
      }
      uint8_t val = _i2c.read();
      const_cast<uint8_t &>(_last_raw) = val;
      return val;      
    }
    bool get(uint8_t bit) const { return (get() >> bit) & 1; }
    void set(uint8_t val)
    {
      _i2c.beginTransmission(_addr);
      _i2c.write(val);
      _i2c.endTransmission();
      _last_set = val;
    }
    void set(uint8_t bit, bool val) {
      uint8_t mask = 1 << val;
      set(val ? _last_set | mask : _last_set & ~mask);
    }
    uint8_t last_set() const { return _last_set; }
    uint8_t last_get() const { return _last_raw; }
    char const * name() const { return _name; }
  private:
    char const * const _name;
    TwoWire & _i2c;
    uint8_t _addr;
    uint8_t _last_raw = 0xff;
    uint8_t _last_set = 0xff;
};

std::ostream & operator << (std::ostream & os, PCF8574Gpio const & gpio) {
  os << gpio.name() << "=0x" << std::hex << unsigned(gpio.last_get()) << "/0x" << std::hex << unsigned(gpio.last_set());
  return os;
}

#endif // PCF8574_HPP
