#ifndef COMMUTATOR_CTL_HPP
#define COMMUTATOR_CTL_HPP

#include "cal_data.hpp"
#include "u_step_drv.hpp"
#include "as5600.hpp"
#include "analog_in.hpp"
#include "tunable.hpp"
#include <stdint.h>
#include <ostream>

class CommutatorCtl {
  public:
    CommutatorCtl(UStepDrv & mot_drive, AS5600PosnSensor & rotor_pos, AnalogIn & supply_mv, unsigned supply_cutoff_mv)
      : _mot_drive(mot_drive), _rotor_pos(rotor_pos), _supply_mv(supply_mv), _supply_cutoff_mv(supply_cutoff_mv), _base_phase_advance("phaseadv", 64)
    {}
    void set_drive_mv(int mvolts) {
      _last_supply = _supply_mv();
/*      if (_last_supply < _supply_cutoff_mv) {
        if (_supply_filt > 100000) {
          _mot_drive.disable();
          _recover_count = 100;
          _supply_filt = 0;
          return;
        } else {
          _supply_filt += _supply_cutoff_mv - _last_supply;
        }
      }
      if (_recover_count > 0) {
        _recover_count--;
        return;
      }
*/
      _drive_phase = mvolts < 0 ? -_base_phase_advance : _base_phase_advance;
      unsigned abs_mvolts = mvolts < 0 ? -mvolts : mvolts;
      abs_mvolts = abs_mvolts < _last_supply ? abs_mvolts : _last_supply;
      _drive16 = _last_supply > abs_mvolts ? (abs_mvolts << 16) / _last_supply : (1 << 16) - 1;
      if (!_mot_drive.enabled()) {
        _mot_drive.enable();
      }
      _drive_mv = mvolts;
    }
    void disengage() {
      set_drive_mv(0);
      _mot_drive.disable();
    }
    void loop() {
      unsigned pos = _rotor_pos();
      unsigned phase_pos = sens_to_phase(pos);

      int delta = (25600 + phase_pos - _last_phase_pos) % 12800;
      delta -= delta > 6400 ? 12800 : 0;
      _last_phase_advance = _drive_phase + delta/2;
      if (_mot_drive.enabled()) {
        _mot_drive.set(phase_pos + _last_phase_advance, _drive16);
      }
      _last_pos = pos;
      _last_phase_pos = phase_pos;
    }
    unsigned last_pos() const { return _last_pos; }
    int get_drive_mv() const { return _drive_mv; }
  private:
    unsigned sens_to_phase(uint16_t sens) {
      unsigned result = 12800 * sens / 4096;
      result = result + unsigned(cal_data[sens]);
      return result;
    }
    UStepDrv & _mot_drive;
    AS5600PosnSensor & _rotor_pos;
    AnalogIn & _supply_mv;
    unsigned const _supply_cutoff_mv;
    int _last_supply = 0;
    int _drive_mv = 0;
    unsigned _drive16 = 0;
    unsigned _last_pos = 0;
    unsigned _last_phase_pos = 0;
    int _last_phase_advance = 0;
    tunable::Item<int16_t> _base_phase_advance;
    int _drive_phase;
    unsigned _recover_count = 0;
    unsigned _supply_filt = 0;
    friend std::ostream & operator << (std::ostream & os, CommutatorCtl const & comm);
};

std::ostream & operator << (std::ostream & os, CommutatorCtl const & comm) {
  os << "Comtr: " << "Rot=" << comm._last_pos << " Duty=" << ((comm._drive16 * 100) >> 16)
     << "%(" << comm._drive16 << ") Ph=" << comm._last_phase_advance 
     << " Vs=" << comm._last_supply << " Recovery: " << comm._recover_count;
  return os;
}


#endif // COMMUTATOR_CTL_HPP
