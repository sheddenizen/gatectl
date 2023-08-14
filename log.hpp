#ifndef LOG_HPP
#define LOG_HPP

#include "Arduino.h"

#include <sstream>

namespace lg {

class Raw {
  public:
    Raw() {}
    ~Raw() { Serial.print(_ss.str().c_str()); }
    std::ostream & operator()() { return _ss; }
    template <typename T>
    std::ostream & operator << (T const & v) { return _ss << v; }
    std::ostream & operator << (std::ostream & (fn)( std::ostream & )){ return _ss << fn; }
    template<typename... Args>
    void printf(char const * fmt, Args const &...args) {
      Serial.printf(fmt, args...);
    }
  private:
    std::ostringstream _ss;
};

template <int level, char... prefx>
class Entry {
  public:
    Entry() {
      char p[] = { prefx... , ':', ' ', 0 };
      _ss << p;
    }
    ~Entry()
    {
      _ss << std::endl;
      Serial.print(_ss.str().c_str());
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
        constexpr auto smax  = 1024;
        char *buf = new char[smax];
        snprintf(buf, smax-1, fmt, args...);
        buf[smax-1] = 0;
        _ss << buf;
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