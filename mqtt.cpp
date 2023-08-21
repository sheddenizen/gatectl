#include "mqtt.hpp"

#include "log.hpp"
#include "Arduino.h"
#include "mqtt_client.h"


typedef unsigned short mqtt_size_t;

// Callback function for subscribed messages
typedef void (*mqtt_msg_rx_fn)(char const * topic, mqtt_size_t topic_len,
                               char const * msg, mqtt_size_t msg_len);


typedef struct mqtt_subscription
{
  char const * topic;
  mqtt_msg_rx_fn handler;

} mqtt_subscription_t;


esp_err_t Mqtt::mqtt_event_handler_cb(int32_t event_id, esp_mqtt_event_handle_t event)
{
  switch (event_id) {
    case MQTT_EVENT_CONNECTED:
      lg::I() <<"MQTT_EVENT_CONNECTED, subscribing";
      for (auto sub : _subscriptions) {
        lg::D() << "Subscribing to topic " << sub.first;
        auto res = esp_mqtt_client_subscribe(_client, sub.first.c_str(), 1);
        lg::E().if_true(res) << "Error " << res << " subscribing to topic " << sub.first;
      }
      _connected = true;
      break;
    case MQTT_EVENT_DISCONNECTED:
      lg::I() <<"MQTT_EVENT_DISCONNECTED";
      _connected = false;
      break;
    case MQTT_EVENT_SUBSCRIBED: {
      lg::I() <<"MQTT_EVENT_SUBSCRIBED, msg_id=" << event->msg_id;
      int msg_id = esp_mqtt_client_publish(_client, "/topic/qos0", "data", 0, 0, 0);
      lg::I() <<"sent publish successful, msg_id=" << msg_id;
      break;
    }
    case MQTT_EVENT_UNSUBSCRIBED:
      lg::I() <<"MQTT_EVENT_UNSUBSCRIBED, msg_id=" << event->msg_id;
      break;
    case MQTT_EVENT_PUBLISHED:
      lg::D() << "MQTT_EVENT_PUBLISHED, msg_id=" << event->msg_id << ", outbox=" << esp_mqtt_client_get_outbox_size(_client);
      break;
    case MQTT_EVENT_DATA: {
      std::string topic(event->topic, event->topic_len);
      auto res = _subscriptions.find(topic);
      if (res != _subscriptions.end()) {
        std::string payload(event->data, event->data_len);
        lg::D() << "MQTT_EVENT_DATA Topic=" << topic << " payload=" << payload;
        res->second(topic, payload);
      } else {
        lg::W() << "Unable to find subscription for topic " << topic;          
      }
      break;
    }
    case MQTT_EVENT_ERROR:
      lg::I() << "MQTT_EVENT_ERROR";
      if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
        lg::E().if_true(event->error_handle->esp_tls_last_esp_err) << "reported from esp-tls";
        lg::E().if_true(event->error_handle->esp_tls_stack_err) << "reported from tls stack";
        lg::E().if_true(event->error_handle->esp_transport_sock_errno) << "captured as transport's socket errno: " << event->error_handle->esp_transport_sock_errno;
        lg::I() << "Last errno string (" << strerror(event->error_handle->esp_transport_sock_errno) << ")";
      }
      break;
    default:
      lg::I() << "Other event id: " << event_id;
      break;
  }
  return ESP_OK;
}

void Mqtt::start()
{
  if (_client) {
    lg::W() << "MQTT Client already started";
    return;
  }

  esp_mqtt_client_config_t mqtt_cfg = {};
  mqtt_cfg.uri = _uri.c_str();

  _client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_register_event(_client, esp_mqtt_event_id_t(ESP_EVENT_ANY_ID), 
        [](void* event_handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
            reinterpret_cast<Mqtt *>(event_handler_arg)->mqtt_event_handler_cb(event_id, (esp_mqtt_event_handle_t)event_data);
        }, 
        this);
  esp_mqtt_client_start(_client);
}

Mqtt::Mqtt(std::string clientname, std::string basetopic, std::string uri)
  : _clientname(clientname)
  , _basetopic(basetopic)
  , _uri(uri)
{}

void Mqtt::subscribe(std::string const & topic, rx_callback_fn callback)
{
  _subscriptions[_basetopic + topic] = callback;
  if (_connected) {
    lg::W() << "Subscription after connect of topic " << topic;
    auto res = esp_mqtt_client_subscribe(_client, topic.c_str(), 1);
    lg::E().if_true(res) << "Error " << res << " subscribing to topic " << topic;
  }
}

bool Mqtt::send(std::string const & topic, std::string const & payload) {
  if (!_client) {
    lg::E() << "Unable to send message " << topic << "to topic, client not started";
    return false;
  }
  int msg_id = esp_mqtt_client_publish(_client, (_basetopic + topic).c_str(), payload.c_str(), payload.size(), 1, false);
  if (msg_id) {
    lg::D() << "Enqueued message " << msg_id << " topic " << topic << " " << payload.size() << " bytes while " << (_connected ? "connected" : "disconnected");
    return true;
  } else {
    lg::W() << "Failed to enqueue message, topic " << topic;
    return false;
  }
}

