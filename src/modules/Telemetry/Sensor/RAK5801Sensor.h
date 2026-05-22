#pragma once

#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && defined(RAK_4631) && RAK_4631 == 1

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "TelemetrySensor.h"

#ifndef RAK5801_PWR_EN_PIN
#define RAK5801_PWR_EN_PIN WB_IO1
#endif
#ifndef RAK5801_A0_PIN
#define RAK5801_A0_PIN WB_IO4
#endif
#ifndef RAK5801_A1_PIN
#define RAK5801_A1_PIN WB_A1
#endif
#ifndef RAK5801_MV_PER_MA
#define RAK5801_MV_PER_MA 149.9f
#endif
#ifndef RAK5801_SAMPLE_COUNT
#define RAK5801_SAMPLE_COUNT 15
#endif

extern bool rak5801Present;

bool detectRAK5801();

class RAK5801Sensor : public TelemetrySensor
{
  public:
    RAK5801Sensor();
    virtual bool getMetrics(meshtastic_Telemetry *measurement) override;
    virtual bool initDevice(TwoWire *bus, ScanI2C::FoundDevice *dev) override;

};

#endif
