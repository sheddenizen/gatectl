#include "lp_filter.hpp"

constexpr auto base_shift = 29;
constexpr auto radix_bits = 3;

std::array<LpFilter::lpf_params_t, 11> const LpFilter::cooked_params {{
    {   507178  ,   1026066113  ,   -491223911  },
    {   1944375 ,   978551887   ,   -449458477  },
    {   4198442 ,   931323806   ,   -411246662  },
    {   7172166 ,   884473343   ,   -376291094  },
    {   10782175    ,   838065198   ,   -344322985  },
    {   14957098    ,   792142531   ,   -315100012  },
    {   14957098    ,   792142531   ,   -315100012  },
    {   19635965    ,   746731218   ,   -288404168  },
    {   24766823    ,   701843306   ,   -264039684  },
    {   30305537    ,   657479811   ,   -241831049  },
    {   36214774    ,   613632985   ,   -221621170  },
}};

LpFilter::LpFilter(int32_t ff_percent, int32_t input_max_val) {
  /*if (ff_percent <= 0 || ff_percent > MAX_FF)
  {
      ESP_LOGE(__func__, "Unable to create filter: state=%p, ff_percent=%d, input_max_val=%d", state, ff_percent, input_max_val);
      return;
  }
  */
  lpf_params_t params = cooked_params[ff_percent - 1];

  int32_t req_bits;
  for (req_bits = 1; input_max_val >> req_bits; ++req_bits);

  _params.gain = (params.gain + (1 << (req_bits - 1)) ) >> req_bits;

  req_bits += radix_bits;

  _params.coeff1 = (params.coeff1 + (1 << (req_bits - 1)) ) >> req_bits;
  _params.coeff2 = (params.coeff2 + (1 << (req_bits - 1)) ) >> req_bits;
  _shift = base_shift - req_bits;
/*
  ESP_LOGI(__func__, "Created LP filter: ff_percent=%d, input_max_val=%d, required bits %d, scaling_fact=%d, gain=%d, coeff1=%d, coeff2=%d",
                          ff_percent,
                          input_max_val,
                          req_bits,
                          1 << _shift,
                          _params.gain,
                          _params.coeff1,
                          _params.coeff2);
*/
}


int32_t LpFilter::operator() (int32_t sample) {
    _in[0] = _in[1];
    _in[1] = _in[2];
    _in[2] = sample;
    _out[0] = _out[1];
    _out[1] = _out[2];
    _out[2] =  ( (_in[0] + 2 * _in[1] + _in[2]) * _params.gain
                     + ( _params.coeff2 * _out[0])
                     + ( _params.coeff1 * _out[1]) )  >> _shift;

    return _out[2] >> radix_bits;
}
