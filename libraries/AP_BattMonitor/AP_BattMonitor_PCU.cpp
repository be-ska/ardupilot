#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor_PCU.h"

#if HAL_PCU_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#define PCU_NOT_HEALTY_MICROS 5000000
extern const AP_HAL::HAL& hal;


/// Constructor
AP_BattMonitor_PCU::AP_BattMonitor_PCU(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    // do nothing
}

// read - read the voltage and current
void
AP_BattMonitor_PCU::read()
{
    const uint32_t tnow = AP_HAL::micros();

    if (!_pcu1.is_initialized())
    {
        _pcu1.init_serial(0);
        return;
    }

    // PCU integration
    _pcu1.send_message();
    if (_pcu1.parse_message()){
        
        // Healthy if messages are received
        _state.healthy = true;

        // get voltage
        _state.voltage = _pcu1.get_volt();

        // calculate time since last current read
        const uint32_t dt_us = tnow - _state.last_time_micros;
        // read current
        _state.current_amps = _pcu1.get_amps();
        update_consumed(_state, dt_us);
        // record time
        _state.last_time_micros = tnow;
    }
    // Not healthy if no messages for 5 seconds
    else if (((tnow - _state.last_time_micros) > PCU_NOT_HEALTY_MICROS)){
        _state.healthy = false;
    }
}

#endif
