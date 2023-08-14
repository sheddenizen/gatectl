#ifndef NETW_HPP
#define NETW_HPP

#include <stdint.h>
#include <string>
#include <functional>

class Netw {
  public:
    Netw() {}
    void start();
  private:
    void task();
    std:: string _hostname;
};


class Mqtt {
  public:
    Mqtt() {}
  private:

};

#endif // NETW_HPP

