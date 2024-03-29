#ifndef LOG_HPP
#define LOG_HPP

#include "Arduino.h"

#include <sstream>
#include <vector>

namespace lg {


class LogStream {
    static constexpr auto sizeThres = 4096;
  public:
    static LogStream & Instance()
    {
      static LogStream ls;
      return ls;
    }
    void add(unsigned level, std::ostringstream & ss) {
      if (level > _log_level)
        return;
      if (xSemaphoreTake(_mutex, 1000 / portTICK_PERIOD_MS)) {
        if (_log.size() < sizeThres) {
          _log += ss.str();
        } else {
          ++_overflow_count;
        }
        xSemaphoreGive(_mutex);
      }
    }
    void print() {
      if (xSemaphoreTake(_mutex, 1000 / portTICK_PERIOD_MS)) {
        Serial.print(_log.c_str());
        if (_overflow_count) {
          Serial.print("Log overflow, ");
          Serial.print(_overflow_count);
          Serial.println(" entries lost");
          _overflow_count = 0;
        }
        _log.clear();
        xSemaphoreGive(_mutex);
      }
    }
    void set_log_level(unsigned level) { _log_level = level; }
  private:
    LogStream()
      : _mutex(xSemaphoreCreateMutex())
    {}
    std::string _log;
    SemaphoreHandle_t _mutex;
    unsigned _overflow_count = 0;
    unsigned _log_level = 4;
};

class Raw {
  public:
    Raw() {}
    ~Raw() { LogStream::Instance().add(0, _ss); }
    std::ostream & operator()() { return _ss; }
    template <typename T>
    std::ostream & operator << (T const & v) { return _ss << v; }
    std::ostream & operator << (std::ostream & (fn)( std::ostream & )){ return _ss << fn; }
    template<typename... Args>
    void printf(char const * fmt, Args const &...args) {
        std::vector<char> buf(1024, 0);
        snprintf(&buf[0], buf.size()-1, fmt, args...);
        _ss << &buf[0];
    }
  private:
    std::ostringstream _ss;
};

template <int level, char... prefx>
class Entry {
  public:
    Entry() {
      char p[] = { prefx... , ':', ' ', 0 };
      if (! _ss.bad())
        _ss << p;
    }
    ~Entry()
    {
      _ss << std::endl;
      LogStream::Instance().add(level, _ss);
    }
    std::ostream & operator()() { return _ss; }
    std::ostream & if_true(bool cond) {
      if (cond)
        _ss.setstate(std::ios_base::badbit);
      else
        _ss.clear(std::ios_base::badbit);
      return _ss;
    }
      
    template <typename T>
    std::ostream & operator << (T const & v) { return _ss << v; }

    std::ostream & operator << (std::ostream & (fn)( std::ostream & )){ return _ss << fn; }

    template<typename... Args>
    void printf(char const * fmt, Args const &...args) {
        std::vector<char> buf(1024, 0);
        snprintf(&buf[0], buf.size()-1, fmt, args...);
        _ss << &buf[0];
    }

  private:
    std::ostringstream _ss;
};

using E = Entry<1, 'E', 'r', 'r'>;
using W = Entry<2, 'W', 'r', 'n'>;
using N = Entry<3, 'N', 'o', 't'>;
using I = Entry<4, 'I', 'n', 'f'>;
using D = Entry<5, 'D', 'b', 'g'>;

}


#endif // LOG_HPP