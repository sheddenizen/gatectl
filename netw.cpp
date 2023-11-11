#include "netw.hpp"
#include "log.hpp"

#include "Arduino.h"
#include "WiFi.h"
#include "WiFiMulti.h"
#include "wpa_secrets.hpp"

#include <string>
#include <iomanip>



Netw::Netw(std::string hostnamebase, std::function<void(bool)> notifyfn, unsigned delayms)
  : _notifyfn(notifyfn)
  , _delayms(delayms)
{
  std::ostringstream h;

  uint8_t mac[6];

  WiFi.macAddress(mac);
  // So much more horrible than printf
  h << hostnamebase << '-' << std::hex << std::setfill('0') << std::setw(2) << (int)mac[4] << std::setw(2) << (int)mac[5];

  _hostname = h.str();
}


static void find_ap(){

  static WiFiMulti wifiMulti;
  static bool first = true;

  if (first) {
    for (auto & ssp : wpa_ssids) {
      lg::D() << "Adding SSID, %s" << ssp.first;
      wifiMulti.addAP(ssp.first, ssp.second);
    }
    first = false;
  }
  lg::D() << "wifi multi run";
  uint8_t wifi_init = wifiMulti.run();
  lg::D() << "wifi multi result %d " <<  wifi_init;
}


void Netw::setup_net()
{
  lg::I() << "Starting Wi-fi";
  WiFi.setHostname(_hostname.c_str());

  // WiFi.begin(ssid, password);
  wl_status_t wifi_res = WiFi.begin();
  lg::I() <<"Enabled wifi, initial result: " << wifi_res;
  if (wifi_res == WL_CONNECT_FAILED) {
    lg::I() <<"Initial connect failed, performing scan";
    find_ap();
  }
}

void Netw::start()
{
  vTaskDelay(_delayms / portTICK_PERIOD_MS);
  lg::I() << "Start Net Task";
  xTaskCreatePinnedToCore([](void *obj){ (*(Netw*)obj).task(); }, "net_task", 15000, this, 0, &_net_task, 0);
}

void Netw::poll()
{
    // 100ms makes it bootloop a few times, 200ms is ok... wtf
    wl_status_t _wifi_state = WiFi.status();

    if (_wifi_state != _wifi_last) {
      lg::I() << "Wifi state change: " << _wifi_state << " (was "  << _wifi_last << ")" << " ssid: " << WiFi.SSID();
      _wifi_last = _wifi_state;
    } else {
          vTaskDelay(_delayms / portTICK_PERIOD_MS);
    }
    if ((_wifi_state == WL_CONNECTED) != _connected) {
      _connected = _wifi_state == WL_CONNECTED;
      if (_notifyfn)
        _notifyfn(_connected);
    }
    if (_wifi_state ==  WL_NO_SSID_AVAIL && --_scan_countdown == 0) {
      _scan_countdown = 5;
      lg::I() <<"No SSID, scanning";
      find_ap();
      lg::I() <<"Status now " << WiFi.status();
    }  
}

void Netw::task()
{
  //uint32_t last_send = 0;
  lg::D() << "Comms Task" ;
  setup_net();
  for (;;) {
  }
}

std::ostream & operator << (std::ostream & os,  wl_status_t state) {
  static const std::array<char const *, 7> statenames = {{
    "WL_IDLE_STATUS",
    "WL_NO_SSID_AVAIL",
    "WL_SCAN_COMPLETED",
    "WL_CONNECTED",
    "WL_CONNECT_FAILED",
    "WL_CONNECTION_LOST",
    "WL_DISCONNECTED"
  }};
  if (state < statenames.size())
    os << statenames[state];
  else if (state == 255)
    os << "WL_NO_SHIELD";
  else
    os << "Unknown";
  return os;
}

std::ostream & operator << (std::ostream & os,  Netw const & nw) {
  return os << WiFi.status() << "(" << WiFi.SSID().c_str() << ")";
}
