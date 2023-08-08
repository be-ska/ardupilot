#pragma once
#include "AP_ProtocolPCU/AP_ProtocolPCU.h"

#if HAL_PCU_ENABLED
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"


class AP_BattMonitor_PCU : public AP_BattMonitor_Backend
{
public:

    /// Constructor
    AP_BattMonitor_PCU(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /// Read the battery voltage and current.  Should be called at 10hz
    virtual void read() override;

    /// returns true if battery monitor instance provides current info
    bool has_current() const override { return true; };

    /// returns true if battery monitor provides consumed energy info
    virtual bool has_consumed_energy() const override { return true; }

    virtual void init(void) override {}

protected:

    // Parameters
    AP_ProtocolPCU _pcu1;                /// feed voltage and current from the PCU hardware connected to serial2

};

#endif
