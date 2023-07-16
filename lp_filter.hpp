#ifndef LP_FILTER_HPP
#define LP_FILTER_HPP

#include "stdint.h"
#include <array>


class LpFilter {
    struct lpf_params_t {
        int32_t gain;
        int32_t coeff1;
        int32_t coeff2;
    };
    static std::array<LpFilter::lpf_params_t, 11> const cooked_params;
    static auto constexpr order = 2;
  public:
    LpFilter(int32_t ff_percent, int32_t input_max_val);
    int32_t operator() (int32_t sample);
  private:
    lpf_params_t _params;
    int32_t _shift;
    std::array<int32_t, order+1> _in = {};
    std::array<int32_t, order+1> _out = {};
};


#endif // LP_FILTER_HPP
