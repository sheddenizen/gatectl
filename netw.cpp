#include "netw.hpp"
#include "log.hpp"

#include "Arduino.h"
#include "WiFi.h"
#include "WiFiMulti.h"
#include "wpa_secrets.hpp"
#include "mqtt_client.h"

#include <string>

#define debug Serial.printf
#define info Serial.printf
#define warn Serial.printf
#define error Serial.printf

#define EOL "\r\n"

#define log_error_if_nonzero(msg,x) do { if (0==(x)) Serial.printf(msg); } while(0)


static char hostname[30];


typedef unsigned short mqtt_size_t;

// Callback function for subscribed messages
typedef void (*mqtt_msg_rx_fn)(char const * topic, mqtt_size_t topic_len,
                               char const * msg, mqtt_size_t msg_len);


typedef struct mqtt_subscription
{
  char const * topic;
  mqtt_msg_rx_fn handler;

} mqtt_subscription_t;

static bool mqtt_connected = true;
static esp_mqtt_client_handle_t client = 0;


static void set_position_handler(char const * topic, mqtt_size_t topic_len,
              char const * msg, mqtt_size_t msg_len){}

mqtt_subscription_t mqtt_subscriptions[] = {
  { "set", set_position_handler },
};
size_t const mqtt_num_subscriptions = sizeof(mqtt_subscriptions) / sizeof(mqtt_subscriptions[0]);


static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
  esp_mqtt_client_handle_t client = event->client;
  int msg_id;
  size_t n;
  // your_context_t *context = event->context;
  switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
      lg::I() <<"MQTT_EVENT_CONNECTED";
      for (n = 0 ; n < mqtt_num_subscriptions ; ++n ) {
        esp_mqtt_client_subscribe(client, mqtt_subscriptions[n].topic, 1);
      }
      mqtt_connected = true;
      break;
    case MQTT_EVENT_DISCONNECTED:
      lg::I() <<"MQTT_EVENT_DISCONNECTED";
      mqtt_connected = false;
      break;
    case MQTT_EVENT_SUBSCRIBED:
      lg::I() <<"MQTT_EVENT_SUBSCRIBED, msg_id=" << event->msg_id;
      msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
      lg::I() <<"sent publish successful, msg_id=" << msg_id;
      break;
    case MQTT_EVENT_UNSUBSCRIBED:
      lg::I() <<"MQTT_EVENT_UNSUBSCRIBED, msg_id=" << event->msg_id;
      break;
    case MQTT_EVENT_PUBLISHED:
      lg::D() << "MQTT_EVENT_PUBLISHED, msg_id=" << event->msg_id << ", outbox=" << esp_mqtt_client_get_outbox_size(client);
      break;
    case MQTT_EVENT_DATA:
      lg::I().printf("MQTT_EVENT_DATA, MSG Rx Topic=%.*s Data=%.*s", event->topic_len, event->topic, event->data_len, event->data);
      for (n = 0 ; n < mqtt_num_subscriptions ; ++n ) {
        if (strncmp(mqtt_subscriptions[n].topic, event->topic, event->topic_len) == 0) {
          // Copy and null-terminate the topic name and payload for the sake of convenience
          char *topic_buf = (char *)malloc(event->topic_len + 1);
          char *payload_buf = (char *)malloc(event->data_len + 1);

          if (!topic_buf || !payload_buf)
          {
            lg::E() << "MQTT Unable to allocate memory for message!";
          } else {
            memcpy(topic_buf, event->topic, event->topic_len);
            topic_buf[event->topic_len] = 0;
            memcpy(payload_buf, event->data, event->data_len);
            payload_buf[event->data_len] = 0;

            mqtt_subscriptions[n].handler(topic_buf, event->topic_len, payload_buf, event->data_len);
          }
          free(payload_buf);
          free(topic_buf);

          break;
        }
      }
      if (n == mqtt_num_subscriptions) {
        lg::I().printf("MSG Rx Topic=%.*s (%u) Unable to find handler!\r\n", event->topic_len, event->topic, event->topic_len);
      }
      break;
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
      lg::I() << "Other event id: " << event->event_id;
      break;
  }
  return ESP_OK;
}


static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
  ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
  mqtt_event_handler_cb((esp_mqtt_event_handle_t)event_data);
}

static void mqtt_app_start()
{
  esp_mqtt_client_config_t mqtt_cfg = {};
  mqtt_cfg.uri = "mqtt://iothub";
/*
  // Prepend prefix and hostname to 
  size_t ex_len = strlen(mqtt_topic_pre) + strlen(hostname) + 2;
  for (int n=0; n < mqtt_num_subscriptions; n++) {
    char * topic = (char *)malloc(strlen(mqtt_subscriptions[n].topic) + ex_len); 
    sprintf(topic, "%s%s/%s", mqtt_topic_pre, hostname, mqtt_subscriptions[n].topic);
    mqtt_subscriptions[n].topic = topic;
  }
  mqtt_status_topic = (char *)malloc(strlen(mqtt_status_topic_sfx) + ex_len);
  sprintf(mqtt_status_topic, "%s%s/%s", mqtt_topic_pre, hostname, mqtt_status_topic_sfx);
*/

  client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_register_event(client, esp_mqtt_event_id_t(ESP_EVENT_ANY_ID), mqtt_event_handler, client);
  esp_mqtt_client_start(client);
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


static void setup_net() {
  uint8_t mac[6];

  WiFi.macAddress(mac);
  sprintf(hostname, "gatectl-%02x%02x", mac[4], mac[5]);

  WiFi.setHostname(hostname);

  // WiFi.begin(ssid, password);
  wl_status_t wifi_res = WiFi.begin();
  lg::I() <<"Enabled wifi, initial result: " << wifi_res;
  if (wifi_res == WL_CONNECT_FAILED) {
    lg::I() <<"Initial connect failed, performing scan";
    find_ap();
  }
}


void Netw::task()
{
  //uint32_t last_send = 0;
  wl_status_t wifi_last = WL_NO_SHIELD;
  bool first_connect = true;
  lg::D() << "Comms Task" ;

  // 100ms makes it bootloop a few times, 200ms is ok... wtf
  delay(200);
  lg::I() <<"Starting comms";

  setup_net();

  for (;;) {
    delay(200);
    uint32_t now_ms = uint32_t(esp_timer_get_time()/1000ll);

    wl_status_t wifi_state = WiFi.status();

    while (wifi_state != wifi_last) {
      lg::I() << "Wifi state change: " << wifi_state << " (was "  << wifi_last << ")";
      lg::I() << "Wifi ssid: " << WiFi.SSID();
      if (wifi_state == WL_CONNECTED && first_connect) {
        lg::D() << "Wifi good, starting mqtt";
        mqtt_app_start();
        lg::D() << "Started mqtt";
        first_connect = false;
      }
      wifi_last = wifi_state; 
    }
    if (wifi_state ==  WL_NO_SSID_AVAIL) {
      lg::I() <<"No SSID, scanning";
      find_ap();
      lg::I() <<"Status now " << WiFi.status();
    }

    if (wifi_state != WL_CONNECTED) {
      return;
    }

    if (!mqtt_connected) {
      lg::W() << "MQTT not connected after " << now_ms << "ms";
      return;
    }

    static bool mqtt_pending = false;
    if (esp_mqtt_client_get_outbox_size(client)) {
      if (!mqtt_pending)
        lg::W() << "MQTT message pending " << mqtt_pending;
      mqtt_pending = true;
    } else {
      if (mqtt_pending)
        lg::W() << "MQTT message cleared " << mqtt_pending;
      mqtt_pending = false;
    }
/*
    uint32_t send_interval_ms = state == state_idle ? send_interval_idle_ms : send_interval_active_ms;
    if ((last_send == 0  || now_ms - last_send > send_interval_ms) && !mqtt_pending) {
      // mqtt_send_status(client);
      last_send = now_ms;
    }
*/
  }
}
