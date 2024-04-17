#ifndef BEACON_HPP
#define BEACON_HPP


#include "driver/rmt.h"
#include "log.hpp"

#include <array>
#include <stdint.h>
#include <functional>

template <uint8_t LED_RMT_TX_GPIO, unsigned NUM_LEDS, unsigned BITS_PER_LED_CMD = 24, rmt_channel_t LED_RMT_TX_CHANNEL = RMT_CHANNEL_0>
class Ws2812 {
  public:
    typedef uint32_t value_type;
    typedef std::array<value_type, NUM_LEDS> array_type;

    Ws2812()
    {
          rmt_config_t config = {};
          config.rmt_mode = RMT_MODE_TX,
          config.channel = LED_RMT_TX_CHANNEL,
          config.gpio_num = gpio_num_t(LED_RMT_TX_GPIO),
          config.clk_div = 2,
          config.mem_block_num = 3,
          config.tx_config.idle_output_en = true;

          lg::I() << "Setting up " << NUM_LEDS << " WS2812s, RMT " << LED_RMT_TX_CHANNEL << " on GPIO " << int(LED_RMT_TX_GPIO);
          lg::E().if_non_zero(rmt_config(&config)) << "Configuring RMT channel " << LED_RMT_TX_CHANNEL << " GPIO " << int(LED_RMT_TX_GPIO);
          lg::E().if_non_zero(rmt_driver_install(config.channel, 0, 0)) << "Installing RMT channel " << LED_RMT_TX_CHANNEL << " GPIO " << int(LED_RMT_TX_GPIO);
      }
    array_type const & data() const { return _input_data; }
    array_type & data() { return _input_data; }
    void write_leds()
    {
      // Wait if it hasn't finished the last one
      lg::E().if_non_zero(rmt_wait_tx_done(LED_RMT_TX_CHANNEL, portMAX_DELAY)) << "Failed to wait for RMT transmission to finish";
      setup_rmt_data_buffer();
      lg::E().if_non_zero(rmt_write_items(LED_RMT_TX_CHANNEL, &_led_data_buffer[0], _led_data_buffer.size(), false)) << "Failed to write items to RMT";
    }

  private:
    static constexpr auto _T0H = 14;
    static constexpr auto _T1H = 52;
    static constexpr auto _T0L = 52;
    static constexpr auto _T1L = 52;

    void setup_rmt_data_buffer()
    {
      for (uint32_t led = 0; led < NUM_LEDS; led++) {
        uint32_t bits_to_send = _input_data[led];
        uint32_t mask = 1 << (BITS_PER_LED_CMD - 1);

        for (uint32_t bit = 0; bit < BITS_PER_LED_CMD; bit++) {
          uint32_t bit_is_set = bits_to_send & mask;
          _led_data_buffer[(led * BITS_PER_LED_CMD) + bit] = bit_is_set ?
                                                          (rmt_item32_t){{{_T1H, 1, _T1L, 0}}} :
                                                          (rmt_item32_t){{{_T0H, 1, _T0L, 0}}};
          mask >>= 1;
        }
      }
    }
    array_type _input_data;
    std::array<rmt_item32_t, NUM_LEDS * BITS_PER_LED_CMD> _led_data_buffer;
};

class Beacon {
    static constexpr auto ledcount = 35;
    static constexpr auto rotdiv1 = 32;
    static constexpr auto hold = 6000;
    static constexpr auto faderate = 5000;
    static constexpr auto strobeint = 2000;
    static constexpr uint32_t colscale(uint32_t col, uint32_t scale256) {
      uint32_t out = 0;
      for (int i = 0; i < 3; i++) {
        out = (out << 8) | ((col & 0xff0000) * scale256) >> 24;
        col <<= 8;
      }
      return out;
    }
  public:
    enum pattern {
      off,
      solid,
      rotatingcw,
      rotatingccw,
      fadeoff,
      strobe,
    };
    enum colour {
      black,
      red,
      green,
      blue,
      amber,
      yellow,
      magenta,
      turquoise,
    };
    static uint32_t colours(colour c) {
      static const uint32_t clrs[] = {
        0,
        0x00ff00,
        0xff0000,
        0x0000ff,
        0x30ff00,
        0x50ff00,
        0x00ff7f,
        0x7f00ff,
      };
      return clrs[c];
    }

    Beacon(std::function<void(bool)> pwr_ctl_fn)
      : _pwr_ctl_fn(pwr_ctl_fn)
    {}
    void set_mode(pattern p, colour c) {
      if (p != _p)
        _ts = millis();
      _p = p;
      _c = colours(c);
    }
    void loop() {
      // If we need to switch on/off, do it, then wait until next cycle to start
      if ((_p != off) != _pwr_state) {
        _pwr_state = _p != off;
        _pwr_ctl_fn(_pwr_state);
        return;
      }
      // Light off nothing to do
      if (!_pwr_state)
        return;

      switch (_p) {
        case solid:
          std::fill(_ledring.data().begin(), _ledring.data().end(), _c);
          break;
        case rotatingcw:
        case rotatingccw: {
          uint32_t td = millis() - _ts;
          uint32_t s = 256 * (td % rotdiv1) / rotdiv1;
          int ln = td / rotdiv1;
          for(int i = 1; i < ledcount; ++i) {
            unsigned n = unsigned(i + ((_p == rotatingcw) ? ln : - ln)) % unsigned(ledcount);
            if (i == 0)
              _ledring.data()[n] = colscale(_c, 256 - s);
            else if (i < 4)
              _ledring.data()[n] = _c;
            else if (i == 4)
              _ledring.data()[n] = colscale(_c, s);
            else
              _ledring.data()[n] = 0;            
          }
          break;
        }
        case fadeoff: {
          uint32_t td = millis() - _ts;
          if (td < hold) {
            std::fill(_ledring.data().begin(), _ledring.data().end(), _c);
          } else {
            int s = 256 - 256 * (td - hold) / faderate;
            if (s < 0) {
              s = 0;
              _p = off;
            }
            std::fill(_ledring.data().begin(), _ledring.data().end(), colscale(_c,s));
          }
          break;
        }
        case strobe: {
          uint32_t td = millis() - _ts;
          if (td > strobeint) {
            std::fill(_ledring.data().begin(), _ledring.data().end(), _c);
            _ts = millis();
          }
          else {
            std::fill(_ledring.data().begin(), _ledring.data().end(), 0);
          }
          break;
        }
      }
      _ledring.write_leds();
    }
  private:
    Ws2812<4, 35> _ledring;
    std::function<void(bool)> _pwr_ctl_fn;
    pattern _p = off;
    uint32_t _c = black;
    uint32_t _ts;
    bool _pwr_state = true;
};


#endif // BEACON_HPP
