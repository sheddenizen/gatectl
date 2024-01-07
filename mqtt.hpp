#ifndef MQTT_HPP
#define MQTT_HPP

#include "mqtt_client.h"

#include <string>
#include <map>
#include <functional>

class Mqtt {
    using rx_callback_fn = std::function<void(std::string const & topic, std::string const & payload)>;
  public:
    explicit Mqtt(std::string clientname, std::string basetopic = "", std::string uri = "mqtt://iothub");
    void subscribe(std::string const & topic, rx_callback_fn callback);
    bool send(std::string const & topic, std::string const & payload);
    // Fire and forget, overwrites last message if not sent
    void send_deferred(std::string && topic, std::string && payload);
    void start();
  private:
    void send_task();
    esp_err_t mqtt_event_handler_cb(int32_t event_id, esp_mqtt_event_handle_t event);
    std::string _clientname;
    std::string _basetopic;
    std::string _uri;
    std::map<std::string, rx_callback_fn> _subscriptions;
    bool _connected = false;
    esp_mqtt_client_handle_t _client = 0;
    TaskHandle_t _send_task = 0;
    QueueHandle_t _sendq;
};

#endif // MQTT_HPP
