#ifndef NETW_HPP
#define NETW_HPP

#include "Arduino.h"
#include "WiFi.h"

#include <stdint.h>
#include <string>
#include <ostream>
#include <functional>

class Netw {
  public:
    explicit Netw(std::string hostnamebase, std::function<void(bool)> notifyfn = std::function<void(bool)>(), unsigned delayms = 400);
    void start();
    void set_notify(std::function<void(bool)> notifyfn) { _notifyfn = notifyfn; }
    void set_delay(unsigned delayms) { _delayms = delayms; }
    bool connected() const { return _connected; }
    wl_status_t wifi_state() const { return WiFi.status(); }
    void poll();
    void setup_net();
    void collect_telem(std::ostream & os);
    std::string const & hostname() const { return _hostname; }
  private:
    void task();
    std::string _hostname;
    bool _connected = false;
    TaskHandle_t _net_task = 0;
    std::function<void(bool)> _notifyfn;
    unsigned _delayms;
    wl_status_t _wifi_last = WL_NO_SHIELD;
    unsigned _scan_countdown = 1;
};

std::ostream & operator << (std::ostream & os,  wl_status_t state);
std::ostream & operator << (std::ostream & os,  Netw const & nw);


#endif // NETW_HPP

