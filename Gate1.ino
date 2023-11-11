#include "log.hpp"
#include "sin_lookup.hpp"
#include "lp_filter.hpp"
#include "commutator_ctl.hpp"
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

#include "cal_data.hpp"


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


class App {
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
    App()
      : _i2cgpio1("RemoteGpio", Wire, 0)
      , _drive_angle("Drive", Wire, -1260)
      , _mot_angle("Mot", Wire1, 0)
      , _mot_current_a("Mot Phase A", "mA", 34, 64, 0, 1305, 3320)
      , _mot_current_b("Mot Phase B", "mA", 35, 62, 0, 1200, 3001)
      , _torque("Torque", "mNm", 36, 1725, 0, 1560, -7897)
      , _batt_mv("Battery", "mV", 39, 0, 0, 2675, 12140)
      , _mot_drive({25,26,27,13}, 23)
      , _commutator(_mot_drive, _mot_angle, _batt_mv, 10000)
      , _torque_servo("torq", 3000, 500, 1000, 2)
      {
        start_motor_task();
        start_control_task();
      }
    void set_telem_sender(std::function<void(std::string const &)> fn) { _telem_send_fn = fn; }
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
    PCF8574Gpio const get_i2c_gpio1() { return _i2cgpio1; }
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
      xTaskCreatePinnedToCore([](void *app){ (*(App*)app).motor_task(); }, "motor_task", 15000, this, 0, &_mot_task, 1);
    }
    static constexpr bool open_btn(uint8_t x) { return (x & 1<<7) != 0; }
    static constexpr bool close_btn(uint8_t x) { return (x & 1<<6) != 0; }
    static constexpr bool stop_btn(uint8_t x) { return (x & 1<<4) != 0; }
    void control_task()
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
          run();
        }
        if (close_btn(ctl_new)) {
          torq_target += 5000;
          lg::I() << "Close Btn " << _i2cgpio1;
          run();
        }
        if (stop_btn(ctl_new)) {
          torq_target = 0;
          lg::I() << "STOP Btn " << _i2cgpio1;
          stop();
        }
        if (ctl_new)
          set_torque(torq_target);

        int16_t drive_angle = _drive_angle();

        if (_drive_angle.error() || _mot_angle.error()) {
          if (error_count == 0)
            lg::E() << "Sensor Error, Drive: " << _drive_angle.error() << " Motor: " << _mot_angle.error();
          error_count++;
        } else if (error_count) {
          lg::W() << "Sensor Error cleared at error count: " << error_count;
          error_count = 0;
        }

        if (count % (running() ? 50 : 500) == 0) {
          reset_stats();
          auto s = get_last_stats();
          std::ostringstream os;
          os << "{\"run\":" << running()
             << ",\"dr_a\":" << drive_angle
             << ",\"Vb_mV\":" << _batt_mv()
             << ",\"Tup_ms\":" << tms
             << ",\"N\":" << count;
          if (running()) {
            os << ",\"f_Hz\":" << s.rate
              << ",\"sp_Hz\":" << s.speed
              << ",\"Ia_mA\":" << s.mot_i_a
              << ",\"Ib_mA\":" << s.mot_i_b
              << ",\"Vd_mV\":" << s.drive
              << ",\"Ta_Nm\":" << s.torque 
              << ",\"Tt_Nm\":" << torq_target;
          }
          os << "}";
          _telem_send_fn(os.str());
        }
      }
    }
    void start_control_task()
    {
      lg::I() << "Start Control Task";
      xTaskCreatePinnedToCore([](void *app){ (*(App*)app).control_task(); }, "control_task", 15000, this, 0, &_ctrl_task, 0);
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

    PCF8574Gpio _i2cgpio1;
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
    TaskHandle_t _mot_task = 0;
    TaskHandle_t _ctrl_task =0;
    bool _run = false;
    unsigned _count = 0;
    stats _stats;
    stats _last_stats;
    std::function<void(std::string const &)> _telem_send_fn;
    friend std::ostream & operator << (std::ostream & os, App const & app);
};

std::ostream & operator << (std::ostream & os, App::stats const & s) {
  using std::setw;
  os << "f=" << setw(4) << s.rate << "Hz "
     << "sp=" << setw(4) << s.speed << "Hz "
     << "Ia=" << setw(4) << s.mot_i_a << "mA "
     << "Ib=" << setw(4) << s.mot_i_b << "mA "
     << "Vm=" << setw(4) << s.drive << "mV "
     << "T=" << setw(4) << s.torque << "mNm";
     return os;
}
std::ostream & operator << (std::ostream & os, App const & app) {
  os << (app._run ? "Run " : "Stop ")
     << app.get_last_stats() << ' '
     << "Drive=" << app._drive_angle << ' ' 
     << app._commutator
     << ' ' << app._mot_current_a << ' ' << app._mot_current_b
     << app._i2cgpio1;
  return os;
}

void App::test_motor_task() {
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

  cli_exec.add_command("torque", [](int mnm){app->set_torque(mnm); return "Ok";}, "Set torque target, mNm");
  cli_exec.add_command("drive", [](int mV){app->set_drive(mV); return "Ok";}, "Set drive voltage, mV");
  cli_exec.add_command("disengage", [](){app->disengage(); return "Ok"; }, "Disengage drive");
  cli_exec.add_command("stop", [](){return app->stop(); }, "Disengage drive, stop control task");
  cli_exec.add_command("run", [](){return app->run(); }, "Start motor control task");
  cli_exec.add_command("running", [](){return app->running(); }, "Is motor control task running?");
  cli_exec.add_command("run-test", [](){return app->run_test(); }, "Run position calibration (if stopped)");
  cli_exec.add_command("abs-test", [](unsigned angle, unsigned mv){return app->mot_direct_step(angle, mv); }, "Set absolute step angle and voltage (if stopped), return pwm on time, us");
  cli_exec.add_command("mot-current", [](){return app->get_phase_currents(); }, "Read motor phase currents");
  cli_exec.add_command("period", [](int ms){Serial.setTimeout(ms); return "Ok"; }, "Set status print interval, ms");
  cli_exec.add_command("get-i2c-gpio", [](){return app->get_i2c_gpio1(); }, "Get i2c gpio 1 state");
  cli_exec.add_command("get-i2c-gpio1", [](){return unsigned(app->get_i2c_gpio1().get()); }, "Get i2c gpio 1 inputs");
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
