#include <array>
#include <Wire.h>
#include <sstream>
#include <utility>

class SerialStream {
  public:
    std::ostringstream _buf;
};

template<typename T>
SerialStream & operator << (SerialStream & ss, T const & x) {
  ss._buf << x;
  return ss;
}
SerialStream ss;

SerialStream & operator << (SerialStream & ss, std::ostream & (fn)( std::ostream & )){
  ss._buf << fn;
  Serial.print(ss._buf.str().c_str());
  ss._buf.str("");
  return ss;
}

class AnalogIn {
  public:
    AnalogIn(char const * name, char const * units, uint8_t pin, int16_t cal1raw, int cal1val, int16_t cal2raw, int cal2val)
      : _name(name), _units(units), _pin(pin),
        _num(cal2val - cal1val),
        _div(cal2raw - cal1raw),
        _offs(-cal1raw * _num / _div -cal1val)
    {
      analogSetPinAttenuation(pin, ADC_0db);
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
};

std::ostream & operator << (std::ostream & os, AnalogIn const & ai) {
  int raw=ai.raw();
  os << ai.name() << '=' << ai.scale(raw) << ai.units() << '(' << raw << ')';
}

class AS5600PosnSensor {
  public:
    static constexpr uint16_t resbits = 12;
    static constexpr uint16_t mask = (1 << resbits) - 1;
    explicit AS5600PosnSensor(char const * name, TwoWire & i2c, uint16_t offset) 
      : _name(name), _i2c(i2c), _offset(offset) {}
    uint16_t raw() const {
      _i2c.beginTransmission(0x36);
      _i2c.write(uint8_t(0xe));
      _i2c.endTransmission();
      _i2c.requestFrom(0x36, 2);
      uint8_t count = 0;

      while (count = _i2c.available() < 2) {
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
  uint16_t raw=sens.last_raw();
  os << sens.name() << '=' << sens.degr(raw) << "deg(" << raw << ')';
}

class UStepDrv {
  public:
    UStepDrv(std::array<uint8_t, 4> phase_pins,uint8_t inh_pin, unsigned ledc_offs = 0)
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
      ss << "dt_comp: " << _dt_comp << std::endl;
    }
    ~UStepDrv() {
      disable();
      for (int n = 0; n < 4; ++n) {
        ledcDetachPin(_phase_pins[n]);
      }
      // Let pin settle, then switch back to input
      usleep(1000);
      pinMode(_inh_pin, INPUT);
      _en = 0;
    }
    void enable() {
      digitalWrite(_inh_pin,1);
      _en = 1;
      _mag = 0;
    }
    // Bottom 8 bits of a are phase angle, m is duty multiplier, 16 bit fractional
    void set(unsigned a, uint16_t m) {
      for (int n = 0; n < 2 ; ++n) {
        auto ph = sin_lookup(a + n * 64);
        uint32_t duty = _dt_comp + ((m * ph.first) >> (32 - _pwm_bits));
        duty = duty > _pwm_max ? _pwm_max : duty;
        auto chan = _ledc_offs + n * 2; 
        ledcWrite(chan + (ph.second ? 1 : 0), _pwm_max - duty);
        ledcWrite(chan + (ph.second ? 0 : 1), _pwm_max);
      }
      _angle = uint8_t(a);
      _mag = m;
      ++_count;
    }
    uint16_t mag() const { return _mag; }
    unsigned angle() const { return _angle; }
    bool enabled() const { return bool(_en); }
    void disable() {
      digitalWrite(_inh_pin, 0);
      for (int n = _ledc_offs; n < _ledc_offs + 4; ++n) {
        ledcWrite(n, _pwm_max);
      }
    }
    unsigned _count = 0;
  private:
    unsigned _ledc_offs;
    const std::array<uint8_t, 4> _phase_pins;
    const uint8_t _inh_pin;
    static const auto _pwm_bits = 12;
    static const auto _pwm_max = (1 << _pwm_bits) -1;
    static const auto _deadtime_us = 6;
    static const auto _freq = 15625;
    static const auto _dt_comp = (1 << _pwm_bits) * _deadtime_us * _freq / 1000000;
    uint8_t _angle;
    uint8_t _en;
    uint16_t _mag;
    std::pair<uint16_t, bool> sin_lookup(unsigned x) {
      static const std::array<uint16_t, 65> sin_data = {
        0,      1608,   3216,   4821,   6424,   8022,   9616,   11204,
        12785,  14359,  15924,  17479,  19024,  20557,  22078,  23586,
        25079,  26557,  28020,  29465,  30893,  32302,  33692,  35061,
        36409,  37736,  39039,  40319,  41575,  42806,  44011,  45189,
        46340,  47464,  48558,  49624,  50659,  51664,  52638,  53580,
        54490,  55367,  56211,  57021,  57797,  58537,  59243,  59913,
        60546,  61144,  61704,  62227,  62713,  63161,  63571,  63943,
        64276,  64570,  64826,  65042,  65219,  65357,  65456,  65515,
        65535,
      };

      bool neg = x & 128;
      x = x & 127;
      if (x <= 64)
        return std::make_pair(sin_data[x], neg);
      else
        return std::make_pair(sin_data[128-x], neg);
    } 
};


UStepDrv * mot_drv;

class App {
  public:
    App()
      : _drive_angle("Drive", Wire, 0)
      , _mot_angle("Mot", Wire1, 0)
      , _mot_current_a("Mot Phase A", "mA", 34, 64, 0, 3000, 6000)
      , _mot_current_b("Mot Phase B", "mA", 35, 64, 0, 3000, 6000)
      , _torque("Torque", "mNm", 36, 1725, 0, 1560, 805)
      , _batt_mv("Battery", "mV", 39, 0, 0, 2675, 12140)
      , _mot_drive({25,26,27,13}, 23)
      {}
    void loop() {}
    static void motor_task_entry(void *me) {
      (*(App*)me).motor_task();
    }
    void test_task1() {
      unsigned pos = 0;
      const uint16_t m = 8000;
      int dir = 1;
      for (;;) {
        _mot_drive.set(pos,m);
        usleep(10000);
        unsigned a = _mot_angle.raw();
        unsigned ca = pos % 12800 * 4096 / 12800;
        ss << pos << ',' << a << ',' << (pos % 256) << ',' << (a * 50 % 4096 / 50) << ',' << ((4096 + a - ca) % 4096) << std::endl;
        pos += dir;
        if (pos == 25600)
          dir = -1;
        if (pos == 0)
          dir = 1;
      }
    }
    void motor_task() {
      unsigned pos = 0;
      const uint16_t m = 8000;
      int dir = 64;
      for (;;) {
        _mot_drive.set(pos,m);
        usleep(1000000);
        unsigned a = _mot_angle.raw();
        unsigned ca = pos % 12800 * 4096 / 12800;
        ss << pos << ',' << a << ',' << (pos % 256) << ',' << (a * 50 % 4096 / 50) << ',' << ((4096 + a - ca) % 4096) 
          << ',' << _mot_current_a.raw() << ',' << _mot_current_b.raw()
          << std::endl;
        pos += dir;
        if (pos == 25600)
          dir = -64;
        if (pos == 0)
          dir = 64;
      }
    }
    UStepDrv _mot_drive;
  private:
    AS5600PosnSensor _drive_angle;
    AS5600PosnSensor _mot_angle;
    AnalogIn _mot_current_a;
    AnalogIn _mot_current_b;
    AnalogIn _torque;
    AnalogIn _batt_mv;
    friend std::ostream & operator << (std::ostream & os, App const & app);
};

std::ostream & operator << (std::ostream & os, App const & app) {
  os << app._drive_angle << ' ' 
     << app._mot_angle << ' ' 
     << app._mot_current_a << ' ' 
     << app._mot_current_b << ' '
     << app._torque << ' '
     << app._batt_mv
     << " Mot e,m,a,c: " << app._mot_drive.enabled() << ',' 
     << app._mot_drive.mag() << ',' << app._mot_drive.angle() << ','
     << app._mot_drive._count;
}

static App * app;

void setup() {
  Serial.begin(115200);
  // Primary I2C uses default pins, but we'll be explicit 
  Wire.begin(21, 22);
  // Secondary, containing or 
  Wire1.begin(5,18);
  analogReadResolution(12);
  analogSetAttenuation(ADC_0db);
  app = new App;
  app->_mot_drive.enable();

  static TaskHandle_t mot_task;
  xTaskCreatePinnedToCore(App::motor_task_entry, "motor_task", 10000, app, 0, &mot_task, 1);
}

void loop() {
  // ss  << *app << std::endl;
  usleep(500000);
}
