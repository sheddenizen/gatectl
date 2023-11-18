#ifndef MOTOR_CTL_HPP
#define MOTOR_CTL_HPP

#include "log.hpp"

// Servo
#include "tunable.hpp"
#include "lp_filter.hpp"

// MotorControl
#include "as5600.hpp"
#include "commutator_ctl.hpp"
#include "analog_in.hpp"

#include "Arduino.h"
#include <string>
#include <ostream>
#include <iomanip>

class Servo {
  public:
    Servo(std::string name, unsigned drive_limit, int gain_n, int gain_d, int lp_ff_percent)
      : _name(name)
      , _drive_limit(name+ "lim", drive_limit)
      , _gain_n(name + "pgain", gain_n)
      , _gain_d(gain_d)
      , _lpf(lp_ff_percent, 60000)
    {}
    void set_target(int target) {
      _target = target;
    }
    int loop(int actual) {
      _last_actual = actual;
      _filt_actual = _lpf(_last_actual);
      _drive = (_target - _filt_actual) * _gain_n / _gain_d;
      _drive = _drive > _drive_limit ? _drive_limit : _drive;
      _drive = _drive < -_drive_limit ? -_drive_limit : _drive;
      return _drive;
    }
  private:
    std::string _name;
    int _target = 0;
    int _last_actual = 0;
    int _filt_actual = 0;
    tunable::Item<int> _drive_limit;
    tunable::Item<int> _gain_n;
    int const _gain_d;
    LpFilter _lpf;
    int _drive = 0;
    friend std::ostream & operator << (std::ostream & os, Servo const & srv);
};

std::ostream & operator << (std::ostream & os, Servo const & srv) {
  os << srv._name << " servo: Target: " << srv._target << " Actual" << ": " << srv._last_actual << " Filt:" << srv._filt_actual << " Drive: " << srv._drive;
  return os;
}

class MotorControl {
  public:
    struct stats {
      unsigned start_pos = 0;
      unsigned last_pos = 0;
      int mot_i_a = 0;
      int mot_i_b = 0;
      int torque = 0;
      int count = 0;
      int drive = 0;
      uint32_t tstart = 0;
      uint32_t tlast = 0;
      int speed = 0;
      int duration_us = 0;
      unsigned rate = 0;
    };
    MotorControl(AnalogIn & batt_mv)
      : _mot_angle("Mot", Wire1, 0)
      , _mot_current_a("Mot Phase A", "mA", 34, 64, 0, 1305, 3320)
      , _mot_current_b("Mot Phase B", "mA", 35, 62, 0, 1200, 3001)
      , _torque("Torque", "mNm", 36, 1725, 0, 1560, -7897)
      , _batt_mv(batt_mv)
      , _mot_drive({25,26,27,13}, 23)
      , _commutator(_mot_drive, _mot_angle, _batt_mv, 10000)
      , _torque_servo("torq", 3000, 500, 1000, 2)
    {
      start_motor_task();
    }

    bool run()
    {
      if (_run)
        vTaskDelay(1);
      vTaskResume(_mot_task);
      vTaskDelay(1);
      return _run;
    }
    bool stop()
    {
      _run = false;
      vTaskDelay(1);
      return !_run;
    }
    bool run_test() { 
      if (!_run) {
        test_motor_task();
        return true;
      }
      return false;
    }
    bool running() const { return _run; }

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
    int mot_direct_step(unsigned angle, unsigned magmv) {
      if (_run)
        return -1;
      unsigned magabs = (magmv << 16) / _batt_mv();
      _mot_drive.enable();
      _mot_drive.set(angle, magabs);
      // Result: pulse width in us, trying not to overflow - 15625/1024 = 10^6 / 2^16
      return magabs * 15625 / _mot_drive.freq() / 1024;
    }
    bool error() {
      //todo...
      return _mot_angle.error();
    }
    std::tuple<int, int> get_phase_currents() {
      int aav = 0;
      int bav = 0;
      constexpr int count = 1000;
      for (int n = 0 ; n < count; ++n) {
        aav += _mot_current_a();
        bav += _mot_current_b();
        usleep(1000);
      }
      return std::make_tuple(aav/count,bav/count);
    }
    void reset_stats()
    {
      _last_stats = calc_stats();
      _stats = {};
      _stats.tstart = esp_timer_get_time();
      _stats.tlast = _stats.tstart; 
      _stats.start_pos = _commutator.last_pos();
    }
    stats const & get_last_stats() const { return _last_stats; }

  private:
    void motor_task() {
      for (;;) {
        while (_run) {
          auto torque = _torque();
          _stats.torque += torque;
          if (_torque_mode) {
            auto drive = _torque_servo.loop(torque);
            _commutator.set_drive_mv(drive);
          }
          _stats.drive += _commutator.get_drive_mv();
          _commutator.loop();
          _count++;
          _stats.count++;
          _stats.last_pos = _commutator.last_pos();
          _stats.mot_i_a += _mot_current_a();
          _stats.mot_i_b += _mot_current_b();
          _stats.tlast = esp_timer_get_time();
        }
        disengage();
        vTaskSuspend(_mot_task);
        _run = true;
      }
    }
    void start_motor_task()
    {
      lg::I() << "Start Motor Task";
      xTaskCreatePinnedToCore([](void *motor_ctl){ (*(MotorControl*)motor_ctl).motor_task(); }, "motor_task", 15000, this, 0, &_mot_task, 1);
    }
    stats calc_stats() {
      stats s = _stats;
      s.duration_us = s.tlast - s.tstart;
      if (s.count == 0 || s.duration_us == 0)
        return s;
      s.mot_i_a /= s.count;
      s.mot_i_b /= s.count;
      s.drive /= s.count;
      s.torque /= s.count;
      s.speed = (s.last_pos - s.start_pos) & 4095;
      s.speed = s.speed >= 2048 ? -(4096 - s.speed) : s.speed;
      s.speed = s.speed * 1000000 / s.duration_us;
      s.rate = s.count * 1000000 / s.duration_us;
      return s;
    }
    void test_motor_task();

    AS5600PosnSensor _mot_angle;
    AnalogIn _mot_current_a;
    AnalogIn _mot_current_b;
    AnalogIn _torque;
    AnalogIn & _batt_mv;
    UStepDrv _mot_drive;
    CommutatorCtl _commutator;
    Servo _torque_servo;
    bool _torque_mode = false;
    TaskHandle_t _mot_task = 0;
    bool _run = false;
    unsigned _count = 0;
    stats _stats;
    stats _last_stats;
    friend std::ostream & operator << (std::ostream & os, MotorControl const & mctl);
};

std::ostream & operator << (std::ostream & os, MotorControl::stats const & s) {
  using std::setw;
  os << "f=" << setw(4) << s.rate << "Hz "
     << "sp=" << setw(4) << s.speed << "Hz "
     << "Ia=" << setw(4) << s.mot_i_a << "mA "
     << "Ib=" << setw(4) << s.mot_i_b << "mA "
     << "Vm=" << setw(4) << s.drive << "mV "
     << "T=" << setw(4) << s.torque << "mNm";
     return os;
}

std::ostream & operator << (std::ostream & os, MotorControl const & mctl) {
  os << (mctl._run ? "Run " : "Stop ")
     << mctl.get_last_stats() << ' '
     << mctl._commutator
     << ' ' << mctl._mot_current_a << ' ' << mctl._mot_current_b;
  return os;
}

void MotorControl::test_motor_task() {
  const uint16_t m = 8000;
  const auto delay = 10;
  unsigned pos = 0;
  unsigned sha = _mot_angle();
  unsigned sha_p = sha;
  usleep(100000);
  lg::Raw() << "Preparing initial test conditions" << std::endl;
  _mot_drive.enable();
  _mot_drive.set(pos,m);
  while (sha < 4000 || sha_p > 100) {
    sha_p = sha;
    sha = _mot_angle();
    _mot_drive.set(pos--,m);
    usleep(1000);
  }
  lg::Raw() << "------------------------- Test Start -------------------------" << std::endl;
  _mot_drive.set(--pos,m);
  usleep(delay * 1000);
  sha = _mot_angle();
  sha_p = sha;
  pos = pos & 255;
  unsigned pos_p = pos;
  lg::Raw() << std::endl;
  for (; pos < 12800 * 5; ++pos) {
    sha_p = sha;
    sha = _mot_angle();
    if (sha != sha_p && sha > sha_p) {
      lg::Raw() << pos << ',' << sha << ',' << pos_p << ',' << sha_p << ',' << ((pos+pos_p)/2) << std::endl;
      pos_p = pos;
    }
    _mot_drive.set(pos,m);
    usleep(delay * 1000);
  }
  lg::Raw() << std::endl;
  for (; pos <= 12800 * 5; --pos) {
    sha_p = sha;
    sha = _mot_angle();
    if (sha != sha_p && sha < sha_p) {
      lg::Raw() << pos << ',' << sha << ',' << pos_p << ',' << sha_p << ',' << ((pos+pos_p)/2) << std::endl;
      pos_p = pos;
    }
    _mot_drive.set(pos,m);
    usleep(delay * 1000);
  }
  lg::Raw() << std::endl;
  lg::Raw() << "------------------------- Test End -------------------------" << std::endl;
}


#endif // MOTOR_CTL_HPP
