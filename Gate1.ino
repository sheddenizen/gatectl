#include "sin_lookup.hpp"
#include "lp_filter.hpp"
#include "u_step_drv.hpp"
#include "as5600.hpp"
#include "cli.hpp"
#include "tunable.hpp"

#include <array>
#include <sstream>
#include <utility>

#include "cal_data.hpp"

class SerialStream {
  public:
    std::ostringstream _buf;
};
// Dodgy hackery!
template<typename T>
SerialStream & operator << (SerialStream & ss, T const & x) {
  using namespace cli;
  ss._buf << x;
  return ss;
}

SerialStream & operator << (SerialStream & ss, std::ostream & (fn)( std::ostream & )){
  ss._buf << fn;
  Serial.print(ss._buf.str().c_str());
  ss._buf.str("");
  return ss;
}

SerialStream ss;

class AnalogIn {
  public:
    AnalogIn(char const * name, char const * units, uint8_t pin, int16_t cal1raw, int cal1val, int16_t cal2raw, int cal2val)
      : _name(name), _units(units), _pin(pin)
      ,  _num(cal2val - cal1val)
      ,  _div(cal2raw - cal1raw)
      ,  _offs(-cal1raw * _num / _div -cal1val)
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
    friend std::ostream & operator << (std::ostream & os, AnalogIn const & ai);
};

std::ostream & operator << (std::ostream & os, AnalogIn const & ai) {
  int raw=ai.raw();
  os << ai.name() << '=' << ai.scale(raw) << ai.units() << '(' << raw << ')';
  return os;
}


/*
static unsigned borked_sens_to_phase(uint16_t sens) {
  auto sin16p = sin_lookup(sens/4);
  // Offset = 29.96 -> 49M / 2^19
  unsigned const stat_offs = 49095153;
//  unsigned const stat_offs = 60000000;
  // Magical fiddle factor = 19 sensor counts. 19 * 12800 /4096 = 475 / 8
  unsigned result = unsigned(sens) * 12800 / 4096 + 12800;
  if (sin16p.second)
    result -= ((stat_offs - 475 * unsigned(sin16p.first)) >> 19);
  else
    result -= ((stat_offs + 475 * unsigned(sin16p.first)) >> 19);
  return result % 12800;
}
*/
static unsigned sens_to_phase(uint16_t sens) {
  unsigned result = 12800 * sens / 4096;
  result = result + unsigned(cal_data[sens]);
  return result;
}

class CommutatorCtl {
  public:
    CommutatorCtl(UStepDrv & mot_drive, AS5600PosnSensor & rotor_pos, AnalogIn & supply_mv, unsigned supply_cutoff_mv)
      : _mot_drive(mot_drive), _rotor_pos(rotor_pos), _supply_mv(supply_mv), _supply_cutoff_mv(supply_cutoff_mv), _base_phase_advance("phaseadv", 64)
    {}
    void set_drive_mv(int mvolts) {
      _last_supply = _supply_mv();
/*      if (_last_supply < _supply_cutoff_mv) {
        if (_supply_filt > 100000) {
          _mot_drive.disable();
          _recover_count = 100;
          _supply_filt = 0;
          return;
        } else {
          _supply_filt += _supply_cutoff_mv - _last_supply;
        }
      }
      if (_recover_count > 0) {
        _recover_count--;
        return;
      }
*/
      _drive_phase = mvolts < 0 ? -_base_phase_advance : _base_phase_advance;
      unsigned abs_mvolts = mvolts < 0 ? -mvolts : mvolts;
      abs_mvolts = abs_mvolts < _last_supply ? abs_mvolts : _last_supply;
      _drive16 = _last_supply > abs_mvolts ? (abs_mvolts << 16) / _last_supply : (1 << 16) - 1;
      if (!_mot_drive.enabled()) {
        _mot_drive.enable();
      }
    }
    void disengage() {
      set_drive_mv(0);
      _mot_drive.disable();
    }
    void loop() {
      unsigned pos = _rotor_pos();
      unsigned phase_pos = sens_to_phase(pos);

      int delta = (25600 + phase_pos - _last_phase_pos) % 12800;
      delta -= delta > 6400 ? 12800 : 0;
      _last_phase_advance = _drive_phase + delta/2;
      if (_mot_drive.enabled()) {
        _mot_drive.set(phase_pos + _last_phase_advance, _drive16);
      }
      _last_pos = pos;
      _last_phase_pos = phase_pos;
    }
  private:
    UStepDrv & _mot_drive;
    AS5600PosnSensor & _rotor_pos;
    AnalogIn & _supply_mv;
    unsigned const _supply_cutoff_mv;
    int _last_supply = 0;
    unsigned _drive16 = 0;
    unsigned _last_pos = 0;
    unsigned _last_phase_pos = 0;
    int _last_phase_advance = 0;
    tunable::Item<int> _base_phase_advance;
    int _drive_phase;
    unsigned _recover_count = 0;
    unsigned _supply_filt = 0;
    friend std::ostream & operator << (std::ostream & os, CommutatorCtl const & comm);
};

std::ostream & operator << (std::ostream & os, CommutatorCtl const & comm) {
  os << "Commutator: " << "Rotor=" << comm._last_pos << " Duty=" << ((comm._drive16 * 100) >> 16) << "%(" << comm._drive16 << ") Phase=" << comm._last_phase_advance << " Supply=" << comm._last_supply << " Recovery: " << comm._recover_count;
  return os;
}


class Servo {
  public:
    Servo(CommutatorCtl & commutator, AnalogIn & actual, unsigned drive_limit, int gain_n, int gain_d)
      : _commutator(commutator)
      , _actual(actual)
      , _drive_limit(drive_limit)
      , _gain_n(gain_n)
      , _gain_d(gain_d)
      , _lpf(2, 60000)
    {}
    void set_target(int target) {
      _target = target;
    }
    void loop() {
      _last_actual = _actual();
      _filt_actual = _lpf(_last_actual);
      _drive = (_target - _filt_actual) * _gain_n / _gain_d;
      _drive = _drive > _drive_limit ? _drive_limit : _drive;
      _drive = _drive < -_drive_limit ? -_drive_limit : _drive;
      _commutator.set_drive_mv(_drive);
    }
  private:
    CommutatorCtl & _commutator;
    AnalogIn & _actual;
    int _target = 0;
    int _last_actual = 0;
    int _filt_actual = 0;
    int const _drive_limit;
    int const _gain_n;
    int const _gain_d;
    LpFilter _lpf;
    int _drive = 0;
    friend std::ostream & operator << (std::ostream & os, Servo const & srv);
};

std::ostream & operator << (std::ostream & os, Servo const & srv) {
  os << "Servo: Target: " << srv._target << " Actual " << srv._actual.name() << ": " << srv._last_actual << srv._actual.units() << " Filt:" << srv._filt_actual << " Drive: " << srv._drive;
  return os;
}


class App {
  public:
    App()
      : _drive_angle("Drive", Wire, 0)
      , _mot_angle("Mot", Wire1, 2048)
      , _mot_current_a("Mot Phase A", "mA", 34, 64, 0, 1305, 3320)
      , _mot_current_b("Mot Phase B", "mA", 35, 62, 0, 1200, 3001)
      , _torque("Torque", "mNm", 36, 1725, 0, 1560, -805)
      , _batt_mv("Battery", "mV", 39, 0, 0, 2675, 12140)
      , _mot_drive({25,26,27,13}, 23)
      , _commutator(_mot_drive, _mot_angle, _batt_mv, 10000)
      , _torque_servo(_commutator, _torque, 2000, 2000, 1000)
      {}
    void loop() {}
    static void motor_task_entry(void *me) {
      (*(App*)me).motor_task();
    }
    void test_motor_task() {
      const uint16_t m = 8000;
      unsigned pos = 0;
      unsigned sha = _mot_angle();
      unsigned sha_p = sha;
      _mot_drive.enable();
      _mot_drive.set(pos,m);
      usleep(100000);
      while (sha < 4000 || sha_p > 100) {
        sha_p = sha;
        sha = _mot_angle();
        _mot_drive.set(pos--,m);
        usleep(1000);
      }
      _mot_drive.set(--pos,m);
      usleep(10000);
      sha = _mot_angle();
      sha_p = sha;
      pos = pos & 255;
      unsigned pos_p = pos;
      ss << std::endl;
      for (;;) {
        for (; pos < 12800 * 5 ; ++pos) {
          sha_p = sha;
          sha = _mot_angle();
          if (sha != sha_p && sha > sha_p) {
            ss << pos << ',' << sha << ',' << pos_p << ',' << sha_p << ',' << ((pos+pos_p)/2) << std::endl;
            pos_p = pos;
          }
          _mot_drive.set(pos,m);
          usleep(10000);
        }
        ss << std::endl;
        for (; pos <= 12800 * 5 ; --pos) {
          sha_p = sha;
          sha = _mot_angle();
          if (sha != sha_p && sha < sha_p) {
            ss << pos << ',' << sha << ',' << pos_p << ',' << sha_p << ',' << ((pos+pos_p)/2) << std::endl;
            pos_p = pos;
          }
          _mot_drive.set(pos,m);
          usleep(10000);
        }
        ss << std::endl;
      }
    }
    void motor_task() {
      for (;;) {
        if (_torque_mode)
          _torque_servo.loop();
        _commutator.loop();
      }
    }
    void set_torque(int mnm) {
      _torque_mode = true;
      _torque_servo.set_target(mnm);
    }
    void set_drive(int mv) {
      _torque_mode = false;
      _commutator.set_drive_mv(mv);
    }
    void disengage() {
      _torque_mode = false;
      _commutator.disengage();
    }
  private:
    AS5600PosnSensor _drive_angle;
    AS5600PosnSensor _mot_angle;
    AnalogIn _mot_current_a;
    AnalogIn _mot_current_b;
    AnalogIn _torque;
    AnalogIn _batt_mv;
    UStepDrv _mot_drive;
    CommutatorCtl _commutator;
    Servo _torque_servo;
    bool _torque_mode = false;
    friend std::ostream & operator << (std::ostream & os, App const & app);
};

std::ostream & operator << (std::ostream & os, App const & app) {
  os << app._drive_angle << ' ' 
     << app._mot_current_a << ' ' 
     << app._mot_current_b << ' '
     << app._mot_drive << ' '
     << app._commutator;
  if (app._torque_mode)
     os << ' ' << app._torque_servo;
  return os;
}

static App * app;

static void print_state() {
    ss << (*(App*)app) << std::endl;
}

std::string testcmd(int i, std::string s, float f)
{
  std::ostringstream os;
  os << "This is a test: " << i << ' ' << s << ' ' << f;
  return os.str();
}

cli::Executor cli_exec;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2000);
  // Primary I2C uses default pins, but we'll be explicit 
  Wire.begin(21, 22);
  // Secondary, containing or 
  Wire1.begin(5,18);
  analogReadResolution(12);
  analogSetAttenuation(ADC_0db);
  tunable::set_cli_executor(cli_exec);
  app = new App;

  cli_exec.add_command("test", testcmd, "test command <int> <string> <float>");
  cli_exec.add_command("torque", [](int mnm){app->set_torque(mnm); return true;}, "Set torque target, mNm");
  cli_exec.add_command("drive", [](int mV){app->set_drive(mV); return true;}, "Set drive voltage, mV");
  cli_exec.add_command("stop", [](){app->disengage(); return true;}, "Disengage drive");

  ss << "Start Control Task" << std::endl;
  static TaskHandle_t mot_task;
  xTaskCreatePinnedToCore(App::motor_task_entry, "motor_task", 10000, app, 0, &mot_task, 1);
}


void loop() {
  //static unsigned n = 0;
  //auto s = sin_lookup(n);
  //int sp = int(s.first) / 64 * (s.second ? -1 : 1);
  //int sp = 1300;
  //app->set_drive(sp);
  //app->set_torque(0);
  //usleep(100000);

  std::string line;
  char esc = 0;
  for (;;) {
    char buf[2] = {};
    int n = Serial.readBytes(buf, 1);
    if (n == 1) {
      if (esc > 0) {
        if ((esc == '[' || (esc & 0x2f) == esc) && (buf[0] & 0x40) == 0x40 ) {
          esc = 0;
        } else {
          esc = buf[0];
        }
        continue;
      }
      if (buf[0] == '\n' || buf[0] == '\r') {
        Serial.println("");
        if (!line.empty())
          break;
        continue;
      }
      if (buf[0] >= ' ' && buf[0] <= '~') {
        line+=buf[0];
        Serial.write(buf[0]);
      } else if (buf[0] == 127 && line.size()) {
        Serial.print('\r');
        for ([[maybe_unused]]auto x : line)
          Serial.print(' ');
        line.erase(line.end()-1);
        Serial.print('\r');
        Serial.print(line.c_str());
      } else if (buf[0] == 27) {
        esc = buf[0];
      } else {
        Serial.print('#');
        Serial.println(int(buf[0]));
        Serial.print(line.c_str());
      }
    } else {
      esc = 0;
      if (line.empty()) {
        print_state();
      }
    }
  }
  Serial.println(cli_exec(line).c_str());
}
