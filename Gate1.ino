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
    void handle_event(Event e) {
      auto it = _state_table.find({ e, _current_state });
      if (it == _state_table.end()) {
        return;
      }
      lg::I() << "Got " << e.name << " event in state " << _current_state.name << ", moving to " << it->second.second.name;
      _current_state = it->second.second;
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
};

class App {
    struct Btn {
      enum {
        open = 1 << 7,
        close = 1 << 6,
        stop = 1 << 4
      };
    };

    using sm = StateMachine<App>;
    struct {
      sm::Event const gobuttonpush = { "gobuttonpush" };
      sm::Event const stopbuttonpush = { "stopbuttonpush" };
      sm::Event const hardwareerror = { "hardwareerror" };
      sm::Event const unlatched = { "unlatched" };
      sm::Event const atdecelpoint = { "atdecelpoint" };
      sm::Event const atlatchpoint = { "atdecelpoint" };
      sm::Event const latched = { "latched" };
      sm::Event const timeout = { "timeout" };
      sm::Event const noretries = { "noretries" };
    } const ev;
    struct {
      sm::State const free = { "free" };
      sm::State const unlatching = { "unlatching" };
      sm::State const moving = { "moving" };
      sm::State const decelerating = { "decelerating" };
      sm::State const latching = { "latching" };
      sm::State const docked = { "docked" };
    } const st;
    sm _appsm = {*this, {
      { {ev.gobuttonpush, st.free},         { &App::liftlatch, st.unlatching} },
      { {ev.gobuttonpush, st.docked},       { &App::liftlatch, st.unlatching} },
      { {ev.unlatched, st.unlatching},      { &App::startmoving, st.moving} },
      { {ev.atdecelpoint, st.moving},       { &App::decelerate, st.decelerating} },
      { {ev.atlatchpoint, st.decelerating}, { &App::droplatch, st.latching} },
      { {ev.latched, st.decelerating},      { &App::finish, st.free} },
    }, st.free };

    struct Target {
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
  public:
    App()
      : _i2cgpio1("RemoteGpio", Wire, 0)
      , _drive_angle("Drive", Wire, -1260)
      , _batt_mv("Battery", "mV", 39, 0, 0, 2675, 12140)
      , _latch_pos("Latch", "mm", 32, 2580, 13, 1584, 44, ADC_6db)
      , _latch_lift("LatchLift", "mm", 19, 1250, 13, 1750, 44, 8)
      , _mctl(_batt_mv)
      {
        start_control_task();
      }
    void set_telem_sender(std::function<void(std::string const &)> fn) { _telem_send_fn = fn; }
    PCF8574Gpio const get_i2c_gpio1() { return _i2cgpio1; }
    MotorControl & get_motor_control() { return _mctl; }
    MotorControl const & get_motor_control() const { return _mctl; }
    void latch_go(int32_t pos) { _latch_lift.go(pos); }
    void latch_stop() { _latch_lift.stop(); }

    // Action functions
    void liftlatch() {}
    void startmoving() {}
    void decelerate() {}
    void droplatch() {}
    void finish() {}


  private:
    void control_task()
    {
      constexpr int intervalms = 10;
      unsigned count = 0;
      uint8_t last_ctl_in = 0x0f;
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      lg::I() << "Start control loop";

      unsigned long tms = millis();
      unsigned long ttarg = tms - tms % intervalms;
 
      for (;;) {
        ttarg += intervalms;
        tms = millis();
        int sleept = int(ttarg - tms);
        if (sleept < -intervalms || sleept > intervalms) {
          lg::E() << "Control loop time correction, target: " << ttarg << ", now: " << tms;
          ttarg = tms - tms % intervalms + intervalms;
          sleept = int(ttarg - tms);
        }
        if (sleept < 0)
          sleept = 0;
        vTaskDelay(sleept / portTICK_PERIOD_MS);
        tms = millis();
        ++count;
        uint8_t ctl_in = _i2cgpio1.get();
        uint8_t ctl_new = (~last_ctl_in & ctl_in);
        last_ctl_in = ctl_in;

        if (_target && (ctl_new & _target->btn_stop_msk))
          _appsm.handle_event(ev.stopbuttonpush);
        if (!_target && (ctl_new & (Btn::close | Btn::close)))
            _target = (ctl_new & Btn::close) ? &_close_target : &_open_target;
        if (ctl_new & _target->btn_go_msk)
          _appsm.handle_event(ev.gobuttonpush);

        int16_t drive_angle = _drive_angle.mrad();
        if (_target) {
          if ((drive_angle > _target->latch_threshold_mrad) == _target->posdir)
            _appsm.handle_event(ev.atlatchpoint);
          if ((drive_angle > _target->latch_threshold_mrad) == _target->posdir)
            _appsm.handle_event(ev.atlatchpoint);
        }

        if (count % (_mctl.running() ? 50 : 500) == 0) {
          collect_and_send_stats();
        }

      }
    }

    static constexpr bool open_btn(uint8_t x) { return (x & 1<<7) != 0; }
    static constexpr bool close_btn(uint8_t x) { return (x & 1<<6) != 0; }
    static constexpr bool stop_btn(uint8_t x) { return (x & 1<<4) != 0; }
    void old_control_task()
    {
      constexpr int intervalms = 10;
      int torq_target = 0;
      unsigned count = 0;
      uint8_t last_ctl_in = 0x0f;
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      lg::I() << "Start control loop";
      unsigned long tms = millis();
      unsigned long ttarg = tms - tms % intervalms;
      unsigned error_count = 0;
 
      for (;;) {
        ttarg += intervalms;
        tms = millis();
        int sleept = int(ttarg - tms);
        if (sleept < -intervalms || sleept > intervalms) {
          lg::E() << "Control loop time correction, target: " << ttarg << ", now: " << tms;
          ttarg = tms - tms % intervalms + intervalms;
          sleept = int(ttarg - tms);
        }
        if (sleept < 0)
          sleept = 0;
        vTaskDelay(sleept / portTICK_PERIOD_MS);
        tms = millis();
        ++count;
        uint8_t ctl_in = _i2cgpio1.get();
        uint8_t ctl_new = (~last_ctl_in & ctl_in);
        last_ctl_in = ctl_in;
        if (open_btn(ctl_new)) {
          lg::I() << "Open Btn " << _i2cgpio1;
          torq_target -= 5000;
          _mctl.run();
        }
        if (close_btn(ctl_new)) {
          torq_target += 5000;
          lg::I() << "Close Btn " << _i2cgpio1;
          _mctl.run();
        }
        if (stop_btn(ctl_new)) {
          torq_target = 0;
          lg::I() << "STOP Btn " << _i2cgpio1;
          _mctl.stop();
        }
        if (ctl_new)
          _mctl.set_torque(torq_target);

        /* int16_t drive_angle = */ _drive_angle();

        if (_drive_angle.error() || _mctl.error()) {
          if (error_count == 0)
            lg::E() << "Sensor Error, Drive: " << _drive_angle.error() << " Motor Ctl: " << _mctl.error();
          error_count++;
        } else if (error_count) {
          lg::W() << "Sensor Error cleared at error count: " << error_count;
          error_count = 0;
        }

        if (count % (_mctl.running() ? 50 : 500) == 0) {
          collect_and_send_stats();
        }
      }
    }
    void start_control_task()
    {
      lg::I() << "Start Control Task";
      xTaskCreatePinnedToCore([](void *app){ (*(App*)app).control_task(); }, "control_task", 15000, this, 0, &_ctrl_task, 0);
    }
    void collect_and_send_stats() {
          _mctl.reset_stats();
          auto s = _mctl.get_last_stats();
          std::ostringstream os;
          os << "{\"run\":" << _mctl.running()
             << ",\"dr_a\":" << _drive_angle.mrad(_drive_angle.last_raw())
             << ",\"Vb_mV\":" << _batt_mv()
             /* << ",\"Tup_ms\":" << tms
             << ",\"N\":" << count */;
          if (_mctl.running()) {
            os << ",\"f_Hz\":" << s.rate
              << ",\"sp_Hz\":" << s.speed
              << ",\"Ia_mA\":" << s.mot_i_a
              << ",\"Ib_mA\":" << s.mot_i_b
              << ",\"Vd_mV\":" << s.drive
              << ",\"Ta_Nm\":" << s.torque
              /* << ",\"Tt_Nm\":" << torq_target */;
          }
          os << "}";
          _telem_send_fn(os.str());
    }
    PCF8574Gpio _i2cgpio1;    
    AS5600PosnSensor _drive_angle;
    AnalogIn _batt_mv;
    AnalogIn _latch_pos;
    SrvPulseOut _latch_lift;
    TaskHandle_t _ctrl_task =0;
    MotorControl _mctl;
    std::function<void(std::string const &)> _telem_send_fn;
    friend std::ostream & operator << (std::ostream & os, App const & app);
    tunable::Item<uint16_t> _latch_lifted_pos = {"Latch lift threshold mm", 40};
    tunable::Item<uint16_t> _latch_secure_pos = {"Latch secure threshold, mm", 10};
    tunable::Item<uint16_t> _latch_drive_margin = {"Overdrive latch margin, mm", 5};
    Target const * _target = 0;
    Target _open_target = {{"Open threshold, mrad", 1570}, {"Open docking velocity, mm/s", 700}, {"Opening velocity, mm/s", 1500}, "open", Btn::close, Btn::open | Btn::stop, false };
    Target _close_target = {{"Close threshold, mrad", 4712}, {"Close docking velocity, mm/s", 700}, {"Closing velocity, mm/s", 1500}, "close", Btn::open, Btn::close | Btn::stop, true };
};

std::ostream & operator << (std::ostream & os, App const & app) {
  os << "Drive=" << app._drive_angle << ' ' 
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

/*
static std::string taskdump() {
  using std::setw;
  std::ostringstream out;
  int ntask = uxTaskGetNumberOfTasks();
  std::vector<TaskStatus_t> taskinfo(ntask);
  uint32_t totalRunTime = 0;
  uxTaskGetSystemState( &taskinfo[0], ntask, &totalRunTime );
  for (auto task : taskinfo) {
    out << setw(10) << task.pcTaskName << setw(3) << task.eCurrentState << setw(9) << task.ulRunTimeCounter << setw(4) << (task.ulRunTimeCounter * 100 / totalRunTime) << '%' << std::endl;
  }
  out << "Total Run time: " << totalRunTime;
  return out.str();
}
*/

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
  lg::I() << (*app) << " Wifi: " << netw;
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
  cli_exec.add_command("latch-stop", [&app](){app.latch_stop(); return "Ok"; }, "Disengage latch servo");
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
    void end(bool fullyTerminate = true);
  tunable::set_cli_executor(cli_exec);
  app = new App;

  add_motor_ctl_cli_cmds(app->get_motor_control());
  add_control_app_cmds(*app);
  cli_exec.add_command("period", [](int ms){Serial.setTimeout(ms); return "Ok"; }, "Set status print interval, ms");
  cli_exec.add_command("taskinfo", taskinfo, "Dump status of specified task");
  cli_exec.add_command("mainstack", [](){ return uxTaskGetStackHighWaterMark(xTaskGetCurrentTaskHandle()); }, "High water mark of main stack");
  cli_exec.add_command("heap", [](){ return ESP.getFreeHeap(); }, "Free heap");
  cli_exec.add_command("log", [](unsigned level){lg::LogStream::Instance().set_log_level(level); return level; }, "Set log level, 0-5");

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

  app->set_telem_sender([](std::string const & s){ mqtt->send("status", s); });
//  netw->start();
  netw->setup_net();
}

void loop()
{
  std::string line = serial_get_line(print_state);
  lg::Raw() << cli_exec(line) << std::endl;
  lg::LogStream::Instance().print();
}
