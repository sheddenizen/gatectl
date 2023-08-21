#ifndef UTIL_HPP
#define UTIL_HPP

#include "Arduino.h"

constexpr void task_sleep(unsigned ms)
{
  vTaskDelay(ms / portTICK_PERIOD_MS);
}

template<typename T>
class Q {
  public:
    Q(size_t capacity)
      : _capacity(capacity)
      , qhandle(xQueueCreate( capacity, sizeof(T*))
    {}
    enq(T const & v) {
      copy = new T(v);

    }
  private:
    size_t _capacity;
    QueueHandle_t _qhandle;
}





#endif // UTIL_HPP
