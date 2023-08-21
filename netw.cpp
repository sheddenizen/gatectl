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
    for (auto ssp = &wpa_ssids[0]; ssp->first[0] != 0; ++ssp) {
      lg::D() << "Adding SSID, %s" << ssp->first;
      wifiMulti.addAP(ssp->first, ssp->second);
    }
    first = false;
  }
  lg::D() << "wifi multi run";
  uint8_t wifi_init = wifiMulti.run();
  lg::D() << "wifi multi result %d " <<  wifi_init << " on ssid " << WiFi.SSID();
}


void Netw::setup_net() {

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
  lg::I() << "Start Net Task";
  xTaskCreatePinnedToCore([](void *obj){ (*(Netw*)obj).task(); }, "net_task", 10000, this, 0, &_net_task, 0);
}

void Netw::task()
{
  //uint32_t last_send = 0;
  wl_status_t wifi_last = WL_NO_SHIELD;
  unsigned scan_countdown = 1;
  lg::D() << "Comms Task" ;

  // 100ms makes it bootloop a few times, 200ms is ok... wtf
  vTaskDelay(_delayms / portTICK_PERIOD_MS);
  lg::I() << "Starting Wi-fi";

  setup_net();
  for (;;) {

    wl_status_t wifi_state = WiFi.status();

    if (wifi_state != wifi_last) {
      lg::I() << "Wifi state change: " << wifi_state << " (was "  << wifi_last << ")";
      lg::I() << "Wifi ssid: " << WiFi.SSID();
      wifi_last = wifi_state; 
    } else {
          vTaskDelay(_delayms / portTICK_PERIOD_MS);
    }
    if ((wifi_state == WL_CONNECTED) != _connected) {
      _connected = wifi_state == WL_CONNECTED;
      if (_notifyfn)
        _notifyfn(_connected);
    }
    if (wifi_state ==  WL_NO_SSID_AVAIL && --scan_countdown == 0) {
      scan_countdown = 5;
      lg::I() <<"No SSID, scanning";
      find_ap();
      lg::I() <<"Status now " << WiFi.status();
    }
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
