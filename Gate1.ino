#include "log.hpp"
#include "motor_ctl.hpp"
#include "as5600.hpp"
#include "pcf8574.hpp"
#include "analog_in.hpp"
#include "cli.hpp"
#include "tunable.hpp"
#include "netw.hpp"
#include "mqtt.hpp"
#include "Update.h"

#include <array>
#include <sstream>
#include <iomanip>
#include <utility>
#include <functional>
#include <unordered_map>

#include "cal_data.hpp"

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

template<typename A>
struct StateMachine {
    struct Event { char const * name; };
    struct State { char const * name; };
    using STKey = std::pair<Event, State>;
    using ActionFn = void (A::*)();
    using STVal = std::pair<ActionFn, State>;
    struct STKeyHash {
      // Hash derived from the address of name strings, not the state/event objects
      std::size_t operator()(STKey const &k) const { return std::hash<char const *>()(k.first.name) + (std::hash<char const *>()(k.second.name) << 1); }
    };
    struct STKeyEq {
      bool operator()(STKey const & a, STKey const & b) const { return a.first.name == b.first.name && a.second.name == b.second.name; }
    };
    using StateTable = std::unordered_map<STKey, STVal, STKeyHash, STKeyEq>;
    A & _app;
    StateTable _state_table;
    State _current_state;
    Event _last_event = {0};
    void handle_event(Event e) {
      auto it = _state_table.find({ e, _current_state });
      if (it == _state_table.end()) {
        return;
      }
      if (_current_state.name != it->second.second.name)
        lg::I() << "Got " << e.name << " event in state " << _current_state.name << ", moving to " << it->second.second.name;
      else
        lg::D() << "Got " << e.name << " event in state " << _current_state.name << ", no change";
      _current_state = it->second.second;
      _last_event = e;
      // Calls to action function may recurse
      (_app.*it->second.first)();
    }
    void handle_event_by_name(std::string const & name) {
      for (auto & ref : _state_table) {
        if (ref.first.first.name == name) {
          handle_event(ref.first.first);
          break;
        }
      }
    }
    char const * current_state_name() const { return _current_state.name; }
    char const * get_and_clear_last_event_name() {
      char const * result = _last_event.name ? _last_event.name : "";
      _last_event.name = 0;
      return result;
    }
};

class App {
    using sm = StateMachine<App>;
    struct {
      sm::Event const gobuttonpush = { "gobuttonpush" };
      sm::Event const stopbuttonpush = { "stopbuttonpush" };
      sm::Event const hardwareerror = { "hardwareerror" };
      sm::Event const unlatched = { "unlatched" };
      sm::Event const inmainswing = { "inmainswing" };
      sm::Event const indockingrange = { "indockingrange" };
      sm::Event const inlatchingrange = { "inlatchingrange" };
      sm::Event const manuallatch = { "manuallatch" };
      sm::Event const latched = { "latched" };
      sm::Event const timeout = { "timeout" };
      sm::Event const noretries = { "noretries" };
    } const ev;
    struct {
      sm::State const free = { "free" };
      sm::State const unlatching = { "unlatching" };
      sm::State const unlatched = { "unlatched" };
      sm::State const moving = { "moving" };
      sm::State const docking = { "docking" };
      sm::State const latching = { "latching" };
      sm::State const docked = { "docked" };
      sm::State const goback = { "goback" };
      sm::State const hwerror = { "hwerror" };
    } const st;
    sm _appsm = {*this, {
      { {ev.gobuttonpush, st.free},         { &App::goliftlatch, st.unlatching} },
      { {ev.gobuttonpush, st.docked},       { &App::goliftlatch, st.unlatching} },
      { {ev.timeout, st.unlatching},        { &App::befree, st.free} },
      { {ev.unlatched, st.unlatching},      { &App::preparemove, st.unlatched} },
      { {ev.noretries, st.unlatching},      { &App::befree, st.free} },

      { {ev.inmainswing, st.unlatched},     { &App::startmainswing, st.moving} },
      { {ev.timeout, st.unlatched},         { &App::startmainswing, st.moving} },
      { {ev.indockingrange, st.unlatched},  { &App::goback, st.goback} },
      { {ev.inlatchingrange, st.unlatched}, { &App::goback, st.goback} },

      { {ev.indockingrange, st.moving},     { &App::gotodockspeed, st.docking} },
      { {ev.inlatchingrange, st.docking},   { &App::droplatch, st.latching} },

      { {ev.latched, st.latching},          { &App::finish, st.docked} },
      { {ev.timeout, st.latching},          { &App::retryliftlatch, st.unlatching} },


      { {ev.timeout, st.moving},            { &App::befree, st.free} },
      { {ev.timeout, st.docking},           { &App::retryliftlatch, st.unlatching} },

      { {ev.timeout, st.goback},            { &App::befree, st.free} },
      { {ev.inmainswing, st.goback},        { &App::gotodockspeed, st.docking} },

      { {ev.inmainswing, st.docked},        { &App::befree, st.free} },
      { {ev.manuallatch, st.free},          { &App::finish, st.docked} },

      { {ev.stopbuttonpush, st.unlatching}, { &App::befree, st.free} },
      { {ev.stopbuttonpush, st.moving},     { &App::befree, st.free} },
      { {ev.stopbuttonpush, st.docking},    { &App::befree, st.free} },
      { {ev.stopbuttonpush, st.goback},     { &App::befree, st.free} },
      { {ev.stopbuttonpush, st.latching},   { &App::befree, st.free} },

      { {ev.hardwareerror, st.unlatching},  { &App::errorfree, st.hwerror} },
      { {ev.hardwareerror, st.moving},      { &App::errorfree, st.hwerror} },
      { {ev.hardwareerror, st.docking},     { &App::errorfree, st.hwerror} },
      { {ev.hardwareerror, st.goback},      { &App::errorfree, st.hwerror} },
      { {ev.hardwareerror, st.latching},    { &App::errorfree, st.hwerror} },
      { {ev.hardwareerror, st.docked},      { &App::errorfree, st.hwerror} },
      { {ev.hardwareerror, st.hwerror},     { &App::errorpersist, st.hwerror} },
      { {ev.hardwareerror, st.free},        { &App::errorfree, st.hwerror} },
      { {ev.timeout, st.hwerror},           { &App::errorrecover, st.free} },

    }, st.free };

    struct Target {
      Target const * const opposite;
      // Point at which we drop the latch and beyond which we check for successful docking
      tunable::Item<int16_t> latch_threshold_mrad;
      // Linear velocity, mm/s, that we approach the dock at
      tunable::Item<int16_t> docking_velocity;
      // Linear velocity, mm/s, that we limit ourselves to for the main swing 
      tunable::Item<int16_t> move_velocity;
      // Name of the target
      char const * name;
      uint8_t const btn_go_msk;
      uint8_t const btn_stop_msk;
      bool const posdir;
    };
    static constexpr int intervalms = 10;
    static constexpr int recoverytimems = 5000;
    static constexpr uint16_t min_telem_count = 10;
  public:
    struct Btn {
      enum mask {
        open = 1 << 6,
        close = 1 << 5,
        stop = 1 << 4
      };
    };
    struct Telltale {
      enum mode {
        telem_short,
        telem_long,
        error_blink,
        slow_wink,
      };
    };
    using TelemCollectFn = std::function<void(std::ostream &)>;
    App()
      : _i2cgpio1("RemoteGpio", Wire, 0)
      , _drive_angle("DrvAng", Wire, 800)
      , _batt_mv("Battery", "mV", 39, 0, 0, 2675, 12140)
      , _latch_pos("Latch", "mm", 32, 2580, 13, 1584, 44, ADC_6db)
      , _latch_lift("LatchLift", "mm", 19, 1250, 13, 1750, 44, 8)
      , _mctl(_batt_mv)
      , _vel_filter(2,8000)
      , _vel_servo("vel", 25000, 500, 2000, 100)
      {
        start_control_task();
      }
    void set_telem_req_fn(std::function<void(TelemCollectFn const &)> fn) { _telem_req_fn = fn; }
    PCF8574Gpio const get_i2c_gpio1() { return _i2cgpio1; }
    MotorControl & get_motor_control() { return _mctl; }
    MotorControl const & get_motor_control() const { return _mctl; }
    void latch_go(int32_t pos) { _latch_target = pos; }
    void latch_lock() { _latch_target = _latch_secure_pos - _latch_drive_margin; }
    void set_target_vel(int val) {
      // Enable velocity control servo, but don't start motor control task
      _vel_servo.set_target(val);
      _mot_run = true;
    }
    void clear_target_vel() {
      // Stop velocity control and make safe disengage, but don't stop motor control task if started
      _vel_servo.reset();
      _mot_run = false;
      _mctl.disengage();
    }
    void sim_btn_push(uint8_t btn_bits) { _btn_sim = btn_bits;}
    void sim_btn_push(Btn::mask btn_mask) { _btn_sim = btn_mask;}
    void set_telem_rate(uint16_t rate) {
      _telem_rate = rate;
      if (_telem_countdown > min_telem_count)
        _telem_countdown = min_telem_count;
    };
    void set_telltale(Telltale::mode mode) { _telltale_mode = mode; }
    void collect_and_serialize_stats(std::ostream & os) {
          _mctl.reset_stats();
          auto s = _mctl.get_last_stats();
          os << "\"run\":" << _mctl.running()
             << ",\"loop_count\":" << _count
             << ",\"drv_a_mrad\":" << _drive_angle.mrad(_drive_angle.last_raw())
             << ",\"drv_vact_mmps\":" << _velocity_actual
             << ",\"batt_V_mV\":" << s.batt_mv
             << ",\"batt_Vmin_mV\":" << s.batt_min_mv
             << ",\"gate_state\":\"" << _appsm.current_state_name()
             << "\",\"ev_last\":\"" << _appsm.get_and_clear_last_event_name()
             << "\",\"latch_pos_mm\":" << _latch_actual
             << ",\"mot_Ia_mA\":" << s.mot_i_a
             << ",\"mot_Ib_mA\":" << s.mot_i_b
             << ",\"drv_Tqact_nNm\":" << s.torque
             << ",\"err_flags\":" << _error.val
            ;
          if (_error)
            os << ",\"error_desc\":\"" << _error << "\"";
          else
            os << ",\"error_desc\":\"None\"";

          if (_mctl.running()) {
            os << ",\"mot_f_Hz\":" << s.rate
              << ",\"mot_sp_Hz\":" << s.speed
              << ",\"mot_V_mV\":" << s.drive
              << ",\"drv_Tqtgt_nNm\":" << _mctl.get_torque_tgt()
              << ",\"mot_srv_intg\":" << _mctl.get_integral()
              << ",\"drv_vtgt_mmps\":" << _vel_servo.get_target()
              << ",\"drv_srv_intg\":" << _vel_servo.get_integral()
              << ",\"mot_Iapk_mA\":" << s.mot_ipk_a
              << ",\"mot_Ibpk_mA\":" << s.mot_ipk_b
              ;
          }
    }
  private:
    void set_timeout_ms(int t) { _timer_count = (t + intervalms - 1) / intervalms; }
    void set_timeout_s(int t) { _timer_count = 1000 * t / intervalms; }
    void stop_timeout() { _timer_count = 0; }

    // Action functions
    void liftlatch() {
      _latch_target = _latch_lifted_pos + _latch_drive_margin;
      _docked_target = 0;
      set_timeout_ms(_unlatch_timeout_ms);
      set_telem_rate(_telem_high_rate);
    }
    void goliftlatch() {
      _retry_count = 0;
      liftlatch();
      set_telltale(Telltale::telem_short);
    }
    void preparemove() {
      _vel_servo.reset();
      _mctl.run();
      _mot_run = true;
      // Token timout - kick into next state if not in docking range
      set_timeout_ms(intervalms*5);
    }
    void startmainswing() {
      _vel_servo.set_target(_target->move_velocity * (_target->posdir ? 1 : -1));
      set_timeout_s(_move_timeout);
      _telem_countdown = 0;
    }
    void gotodockspeed() {
      int vtarg = _target->docking_velocity * (_target->posdir ? 1 : -1);
      _vel_servo.set_target(vtarg * int(100 + _retry_vinc_percent * _retry_count) / 100);
      set_timeout_s(_dock_timeout);
      _telem_countdown = 0;
    }
    void goback() {
      _vel_servo.set_target(_target->move_velocity * (_target->posdir ? -1 : 1));
      set_timeout_s(_goback_timeout);
      _telem_countdown = 0;
    }
    void droplatch() {
      _latch_target = _latch_secure_pos - _latch_drive_margin;
      _vel_servo.set_target(0);
      set_timeout_ms(_latch_timeout_ms);
      _telem_countdown = 0;
    }
    void retryliftlatch() {
      _retry_count++;
      if (_retry_count > _dock_retries) {
        _appsm.handle_event(ev.noretries);
      } else {
        liftlatch();
        set_telltale(Telltale::telem_long);
      }
    }
    void finish() {
      _mctl.stop();
      _mot_run = false;
      _vel_servo.reset();
      _docked_target = _target;
      if (_target)
        _target = _target->opposite;
      else
        lg::E() << "NULL Target at finish!";
      stop_timeout();
      set_telem_rate(_telem_low_rate);
      set_telltale(Telltale::slow_wink);
    }
    void befree() {
      _mctl.stop();
      _mot_run = false;
      _vel_servo.reset();

      lg::D() << "befree: Clearing target";
      _target = 0;
      _docked_target = 0;
      _latch_target = _latch_secure_pos - _latch_drive_margin;
      _vel_servo.reset();
      stop_timeout();
      set_telem_rate(_telem_low_rate);
      set_telltale(Telltale::telem_short);
    }
    void errorfree() {
      befree();
      lg::E() << "Transition into error state: " << _error;
      set_timeout_ms(recoverytimems);
      set_telltale(Telltale::error_blink);
    }
    void errorpersist() {
      set_timeout_ms(recoverytimems);
    }
    void errorrecover() {
      stop_timeout();
      _target = 0;
      _telem_countdown = 1;
      set_telltale(Telltale::telem_short);
    }

    void control_task()
    {
      uint8_t last_ctl_in = 0x0f;
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      lg::I() << "Start control loop";

      unsigned long tms = millis();
      unsigned long ttarg = tms - tms % intervalms;
      int16_t last_drive_angle = _drive_angle.mrad();

      // Start latch in the latch positon
      _latch_target = _latch_secure_pos - _latch_drive_margin;
      TelemCollectFn const telem_collect_fn([this](std::ostream &os){ this->collect_and_serialize_stats(os); });
      for (;;) {
        ttarg += intervalms;
        tms = millis();
        int sleept = int(ttarg - tms);
        if (sleept < -intervalms || sleept > intervalms) {
          lg::E() << "Control loop time correction, target: " << ttarg << ", now: " << tms;
          ttarg = tms - tms % intervalms + intervalms;
          sleept = int(ttarg - tms);
          unsigned errints = (tms - ttarg) / intervalms;
          _telem_countdown = (_telem_countdown < errints) ? 0 : _telem_countdown - errints;
        }
        if (sleept < 0)
          sleept = 0;
        vTaskDelay(sleept / portTICK_PERIOD_MS);
        ++_count;
        if (_timer_count > 0) {
          --_timer_count;
          if (_timer_count == 0)
            _appsm.handle_event(ev.timeout);
        }

        int16_t drive_angle = _drive_angle.mrad();
        _latch_actual = _latch_pos();
        uint8_t ctl_in = _i2cgpio1.get();

        errorcode error;
        error.raiseifset(_drive_angle.error(), errorcode::driveSensor);
        error.raiseifset(_latch_actual < 0 || _latch_actual > 90, errorcode::latchSensor);
        error |= _mctl.error();
        if (! (_error == error))
          lg::W() << "Error status change, now: " << error;
        _error = error;

        if (error)
          _appsm.handle_event(ev.hardwareerror);

        if (_btn_sim) {
          lg::I() << "Simulated Button Push: 0x" << std::hex << unsigned(_btn_sim);
          ctl_in |= _btn_sim;
          _btn_sim = 0;
        }
        uint8_t ctl_new = (~last_ctl_in & ctl_in);
        last_ctl_in = ctl_in;

        if (_target && (ctl_new & _target->btn_stop_msk)) {
          _appsm.handle_event(ev.stopbuttonpush);
        } else if (!_target) {
          if (ctl_new & _open_target.btn_go_msk)
              _target = &_open_target;
          if (ctl_new & _close_target.btn_go_msk)
              _target = &_close_target;
          if (_target) {
              lg::I() << "GO Button push: Setting target to " << _target->name;
              _appsm.handle_event(ev.gobuttonpush);
          }
        } else if (ctl_new & _target->btn_go_msk) {
          _appsm.handle_event(ev.gobuttonpush);
        }

        if (!_target && (_latch_actual < _latch_secure_pos)) {
          Target * docktarg = 0;
          if ((drive_angle > _open_target.latch_threshold_mrad) == _open_target.posdir)
            docktarg = &_open_target;
          if ((drive_angle > _close_target.latch_threshold_mrad) == _close_target.posdir)
            docktarg = &_close_target;
          if (docktarg) {
            lg::I() << "Manual latch: Setting target to " << docktarg->name;
            _target = docktarg;
            _appsm.handle_event(ev.manuallatch);
          }
        }

        // Do we know where we're going?
        if (_target) {
          // Work out where in the swing we are and send event (repeatedly)
          if ((drive_angle > _target->latch_threshold_mrad) == _target->posdir) {
            _appsm.handle_event(ev.inlatchingrange);
          } else {
            const int docking_mrad = _target->latch_threshold_mrad + int32_t(_docking_range) * (_target->posdir ? -1000 : 1000) / int32_t(_gate_radius);
            if ((drive_angle > docking_mrad) == _target->posdir)
              _appsm.handle_event(ev.indockingrange);
            else if ((drive_angle > _target->opposite->latch_threshold_mrad) != _target->opposite->posdir || _latch_actual > _latch_secure_pos)
              // Only if not in latching range of the other end
              _appsm.handle_event(ev.inmainswing);
          }
        }
        // delta in mRad/1000 * radius in mm * 1000mms/interval in ms / 2 (linkage) -> velocity mm/s
        int da_delta = int(drive_angle - last_drive_angle) * int(_gate_radius) / intervalms / 2;
        last_drive_angle = drive_angle;
        _velocity_actual = _vel_filter(da_delta);

        // Motor control is negative wrt drive shaft because gears. Don't set unless we are actually running - this makes manual override/testing easier
        if (_mot_run && _mctl.running()) {
          _torq_target  = _vel_servo.loop(_velocity_actual);
          _mctl.set_torque(-_torq_target);
        }

        if (_latch_actual < _latch_secure_pos)
          _appsm.handle_event(ev.latched);
        if (_latch_actual > _latch_lifted_pos)
          _appsm.handle_event(ev.unlatched);

        // Slow lift, fast drop
        int lp = _latch_lift.get_last_pos();
        if (_latch_target > lp && _count % 4 == 0)
          _latch_lift.go(lp + 1);
        if (_latch_target < lp)
          _latch_lift.go(lp - 2 < _latch_target ? _latch_target : lp - 2);

        teltale_update();
        if (_telem_countdown == 0) {
          _telem_countdown = _telem_rate;
          _telem_req_fn(telem_collect_fn);
        }
        _telem_countdown--;

      }
    }
    void set_telltale_led(bool on) {
      _i2cgpio1.set(0, !on, false);
    }
    void teltale_update() {
      switch (_telltale_mode) {
        case Telltale::telem_short:
          set_telltale_led(_telem_countdown == 2);
          break;
        case Telltale::telem_long:
          set_telltale_led(_telem_countdown > 1 && _telem_countdown < min_telem_count);
          break;
        case Telltale::error_blink:
          {
            unsigned tms = millis();
            set_telltale_led((tms % 3000) < 1000);
            break;
          }
        case Telltale::slow_wink:
          set_telltale_led((_count & 511) == 0);
          break;        
      }
    }
    void start_control_task()
    {
      lg::I() << "Start Control Task";
      xTaskCreatePinnedToCore([](void *app){ (*(App*)app).control_task(); }, "control_task", 15000, this, 0, &_ctrl_task, 0);
    }
    PCF8574Gpio _i2cgpio1;    
    AS5600PosnSensor _drive_angle;
    AnalogIn _batt_mv;
    AnalogIn _latch_pos;
    SrvPulseOut _latch_lift;
    TaskHandle_t _ctrl_task =0;
    MotorControl _mctl;
    LpFilter _vel_filter;
    std::function<void(TelemCollectFn const &)> _telem_req_fn;
    friend std::ostream & operator << (std::ostream & os, App const & app);
    tunable::Item<uint16_t> _latch_lifted_pos = {"latch-lift-thres-mm", 33};
    tunable::Item<uint16_t> _latch_secure_pos = {"latch-secure-thres-mm", 18};
    tunable::Item<uint16_t> _latch_drive_margin = {"latch-overdrive-mm", 5};
    tunable::Item<uint16_t> _latch_timeout_ms = {"latch-timeout-ms", 3000};
    tunable::Item<uint16_t> _unlatch_timeout_ms = {"unlatch-timeout-ms", 5000};
    tunable::Item<uint16_t> _dock_timeout = {"dock-timeout-s", 10};
    tunable::Item<uint16_t> _move_timeout = {"move-timeout-s", 120};
    tunable::Item<uint16_t> _dock_retries = {"dock-retries", 5};
    tunable::Item<uint16_t> _retry_vinc_percent = {"retry-vinc-pc", 20};
    tunable::Item<uint16_t> _goback_timeout = {"goback-timeout-s", 30};
    int16_t _latch_target = 0;
    int16_t _latch_actual = 0;
    tunable::Item<uint16_t> _docking_range = {"dock-range-mm", 500};
    tunable::Item<uint16_t> _gate_radius = {"gate-radius-mm", 2850};
    tunable::Item<uint16_t> _telem_high_rate = {"telem-hi-rate", 25};
    tunable::Item<uint16_t> _telem_low_rate = {"telem-lo-rate", 200};
    Target const * _target = 0;
    Target const * _docked_target = 0;
    Target _open_target = {&_close_target, {"op-thres-mrad", 4935}, {"op-vdock-mmps", 500}, {"op-v-mmps", 400}, "open", Btn::open, Btn::close | Btn::stop, true };
    Target _close_target = {&_open_target, {"cl-thres-mrad", 1760}, {"cl-vdock-mmps", 400}, {"cl-v-mmps", 400}, "close", Btn::close, Btn::open | Btn::stop, false };
    Servo _vel_servo;
    int16_t _velocity_actual = 0;
    int16_t _torq_target = 0;
    int _timer_count = 0;
    int _retry_count = 0;
    bool _mot_run = false;
    uint16_t _telem_rate = _telem_low_rate;
    unsigned _telem_countdown = 0;
    uint8_t _btn_sim = 0;
    errorcode _error;
    uint32_t _count = 0;
    Telltale::mode _telltale_mode = Telltale::telem_short;

};

std::ostream & operator << (std::ostream & os, App const & app) {
  os << "State=" << app._appsm.current_state_name() << " ";
  
  if (app._target) {
    os << " Tgt="
      << app._target->name 
      << "(Dt=" << app._target->latch_threshold_mrad << ' '
      << app._vel_servo << ") ";
  }

  os << app._drive_angle << ' '
     << "vdrive=" << app._velocity_actual << "mm/s "
     << app._i2cgpio1 << ' '
     << app._latch_pos << ' '
     << app._latch_lift << ' '
     << app.get_motor_control();
  return os;
}


// Wait indefinitely for user to enter a line of text on the serial interface, allowing for backspace, but ignoring ANSI
// terminal sequences for now. If no text is in the line buffer, idlefn() will be called on each serial timeout
std::string serial_get_line(std::function<void()> idlefn)
{
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
      if (line.empty() && idlefn) {
        idlefn();
      }
    }
  }
  return line;
}

static std::string taskinfo(std::string name) {
  using std::setw;
  std::ostringstream out;
  auto handle = xTaskGetHandle(name.c_str());

  std::array<char const *, 6> statenames = {{
    "eRunning",
    "eReady",
    "eBlocked",
    "eSuspended",
    "eDeleted",
    "eInvalid"
  }};

  if (handle) {
    eTaskState state = eTaskGetState(handle);
    out << name << " handle=" << handle << " state=" << statenames[state] << " (" << state << ")" << " stack high water=" << uxTaskGetStackHighWaterMark(handle);
  } else {
    out << "Task, " << name << " not found";
  }
  return out.str();
}


static App * app;
static Netw * netw;
static Mqtt * mqtt;

static void print_state() {
  netw->poll();
  lg::I() << (*app) << " Wifi: " << *netw;
  lg::LogStream::Instance().print();
}

cli::Executor cli_exec;

void add_motor_ctl_cli_cmds(MotorControl & mctl)
{
  cli_exec.add_command("torque", [&mctl](int mnm){mctl.set_torque(mnm); return "Ok";}, "Set torque target, mNm");
  cli_exec.add_command("drive", [&mctl](int mV){mctl.set_drive(mV); return "Ok";}, "Set drive voltage, mV");
  cli_exec.add_command("disengage", [&mctl](){mctl.disengage(); return "Ok"; }, "Disengage drive");
  cli_exec.add_command("stop", [&mctl](){return mctl.stop(); }, "Disengage drive, stop control task");
  cli_exec.add_command("run", [&mctl](){return mctl.run(); }, "Start motor control task");
  cli_exec.add_command("running", [&mctl](){return mctl.running(); }, "Is motor control task running?");
  cli_exec.add_command("run-test", [&mctl](){return mctl.run_test(); }, "Run position calibration (if stopped)");
  cli_exec.add_command("abs-test", [&mctl](unsigned angle, unsigned mv){return mctl.mot_direct_step(angle, mv); }, "Set absolute step angle and voltage (if stopped), return pwm on time, us");
  cli_exec.add_command("mot-current", [&mctl](){return mctl.get_phase_currents(); }, "Read motor phase currents");
}

void add_control_app_cmds(App & app)
{
  cli_exec.add_command("get-i2c-gpio", [&app](){return app.get_i2c_gpio1(); }, "Get i2c gpio 1 state");
  cli_exec.add_command("get-i2c-gpio1", [&app](){return unsigned(app.get_i2c_gpio1().get()); }, "Get i2c gpio 1 inputs");
  cli_exec.add_command("latch-go", [&app](int pos){app.latch_go(pos); return "Ok";}, "Go to latch position mm");
  cli_exec.add_command("latch-lock", [&app](){app.latch_lock(); return "Ok"; }, "Drop latch to secure position");
  cli_exec.add_command("vel", [&app](int vel){app.set_target_vel(vel); return "Ok";}, "Set gate target velocity and enable vel servo");
  cli_exec.add_command("veloff", [&app](){app.clear_target_vel(); return "Ok";}, "Reset target velocity and disable vel servo");
  cli_exec.add_command("telemrate", [&app](uint16_t rate){app.set_telem_rate(rate); return "Ok";}, "Set telemetry rate in control intervals");
  cli_exec.add_command("telltale", [&app](unsigned mode){app.set_telltale(App::Telltale::mode(mode)); return "Ok";}, "Set set telltale mode");
  cli_exec.add_command("btn-open", [&app](){app.sim_btn_push(App::Btn::open); return "Ok";}, "Simulate open button push");
  cli_exec.add_command("btn-close", [&app](){app.sim_btn_push(App::Btn::close); return "Ok";}, "Simulate close button push");
  cli_exec.add_command("btn-stop", [&app](){app.sim_btn_push(App::Btn::stop); return "Ok";}, "Simulate stop button push");
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1000);
  // Primary I2C uses default pins, but we'll be explicit 
  Wire.begin(21, 22);
  // Secondary, containing our motor positon sensor
  Wire1.begin(5,18, 400000);
  analogReadResolution(12);
  analogSetAttenuation(ADC_0db);

  //begin(unsigned long baud, uint32_t Serial1config=SERIAL_8N1, int8_t rxPin=-1, int8_t txPin=-1, bool invert=false, unsigned long timeout_ms = 20000UL, uint8_t rxfifo_full_thrhd = 112);
  //  void end(bool fullyTerminate = true);
  tunable::set_cli_executor(cli_exec);
  app = new App;

  add_motor_ctl_cli_cmds(app->get_motor_control());
  add_control_app_cmds(*app);
  cli_exec.add_command("period", [](int ms){Serial.setTimeout(ms); return "Ok"; }, "Set status print interval, ms");
  cli_exec.add_command("taskinfo", taskinfo, "Dump status of specified task");
  cli_exec.add_command("mainstack", [](){ return uxTaskGetStackHighWaterMark(xTaskGetCurrentTaskHandle()); }, "High water mark of main stack");
  cli_exec.add_command("heap", [](){ return ESP.getFreeHeap(); }, "Free heap");
  cli_exec.add_command("reboot", [](){ ESP.restart(); return "OK"; }, "Reboot");
  cli_exec.add_command("log", [](unsigned level){lg::LogStream::Instance().set_log_level(level); return level; }, "Set log level, 0-5");
  cli_exec.add_command("scan", [](){ netw->scan_now(); return "Requesting Scan"; }, "Force SSID scan");


  netw = new Netw("gatectl", [](bool connected) {
    lg::D() << "Notify network " << (connected ? "connected" : "disconnected");
    if (connected)
      mqtt->start();
  });
  mqtt = new Mqtt(netw->hostname(), std::string("gatectl/v1/") + netw->hostname() + "/");
  mqtt->subscribe("cmd", [](std::string topic, std::string payload) {
    // Mind the stack!
    mqtt->send("response", cli_exec(payload));
  });
  mqtt->subscribe("prog/begin", [](std::string topic, std::string payload) {
    if (Update.begin()) {
      mqtt->send("prog/begin/ack", "");
    } else {
      mqtt->send("prog/begin/nack", "");
    }
  });
  mqtt->subscribe("prog/write", [](std::string topic, std::string payload) {
    if (Update.write(reinterpret_cast<uint8_t *>(const_cast<char *>(payload.c_str())), payload.size() )) {
      mqtt->send("prog/write/ack", std::to_string(payload.size()));
    } else {
      mqtt->send("prog/write/nack", "");
    }
  });

  auto telem_req_fn = [](std::function<void(std::ostream &)> const & telem_collect_fn) {
    std::ostringstream os;
    os << '{';
    telem_collect_fn(os);
    netw->collect_telem(',', os);
    os << '}';
    mqtt->send_deferred("status", os.str());
  };

  app->set_telem_req_fn(telem_req_fn);

  //  netw->start();
  netw->setup_net();
}

void loop()
{
  std::string line = serial_get_line(print_state);
  lg::Raw() << cli_exec(line) << std::endl;
  lg::LogStream::Instance().print();
}
