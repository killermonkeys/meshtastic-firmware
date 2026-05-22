#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && defined(RAK_4631) && RAK_4631 == 1

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "RAK5801Sensor.h"
#include <Arduino.h>

#ifdef ARCH_NRF52
#include "Nrf52SaadcLock.h"
#endif

bool rak5801Present = false;

#ifndef RAK5801_MIN_VALID_MA
#define RAK5801_MIN_VALID_MA 3.0f
#endif
#ifndef RAK5801_MAX_VALID_MA
#define RAK5801_MAX_VALID_MA 22.0f
#endif
#ifndef RAK5801_PWR_SETTLE_MS
#define RAK5801_PWR_SETTLE_MS 500
#endif

static void rak5801EnableSensorPower()
{
    pinMode(RAK5801_PWR_EN_PIN, OUTPUT);
    digitalWrite(RAK5801_PWR_EN_PIN, HIGH);
}

static float rak5801ReadChannelCurrentMa(uint8_t pin)
{
    pinMode(pin, INPUT);

    uint32_t raw = 0;
#ifdef ARCH_NRF52
    concurrency::LockGuard saadcGuard(concurrency::nrf52SaadcLock);
#endif
    analogReadResolution(BATTERY_SENSE_RESOLUTION_BITS);
    for (uint32_t i = 0; i < RAK5801_SAMPLE_COUNT; i++) {
        raw += analogRead(pin);
    }
    raw = raw / RAK5801_SAMPLE_COUNT;
    const float voltageMv =
        ((1000.0f * AREF_VOLTAGE) / (float)(1 << BATTERY_SENSE_RESOLUTION_BITS)) * (float)raw;
    return voltageMv / RAK5801_MV_PER_MA;
}

static bool rak5801CurrentLooksValid(float currentMa)
{
    return currentMa >= RAK5801_MIN_VALID_MA && currentMa <= RAK5801_MAX_VALID_MA;
}

static void rak5801LogProbe(const char *phase, float a0, float a1)
{
    LOG_INFO("RAK5801 %s: A0=%.2f mA, A1=%.2f mA (valid range %.1f-%.1f mA)", phase, a0, a1, (float)RAK5801_MIN_VALID_MA,
             (float)RAK5801_MAX_VALID_MA);
}

bool detectRAK5801()
{
    rak5801EnableSensorPower();
    delay(RAK5801_PWR_SETTLE_MS);

    const float a0 = rak5801ReadChannelCurrentMa(RAK5801_A0_PIN);
    const float a1 = rak5801ReadChannelCurrentMa(RAK5801_A1_PIN);
    rak5801LogProbe("probe", a0, a1);

    rak5801Present = rak5801CurrentLooksValid(a0) || rak5801CurrentLooksValid(a1);
    if (rak5801Present) {
        LOG_INFO("RAK5801 detected. With GNSS installed, use Sensor Slot D (PPS on IO5), not Slot A.");
    } else {
        LOG_WARN("RAK5801 not detected (check IO-slot module, 12V sensor power, and A0/A1 wiring)");
    }
    return rak5801Present;
}

RAK5801Sensor::RAK5801Sensor() : TelemetrySensor(meshtastic_TelemetrySensorType_RAK5801, "RAK5801") {}

bool RAK5801Sensor::initDevice(TwoWire *bus, ScanI2C::FoundDevice *dev)
{
    if (!rak5801Present) {
        detectRAK5801();
    }
    if (!rak5801Present) {
        delay(RAK5801_PWR_SETTLE_MS);
        detectRAK5801();
    }

    rak5801EnableSensorPower();
    status = true;
    initialized = true;
    LOG_INFO("Init sensor: %s (present=%d)", sensorName, rak5801Present);
    return true;
}

bool RAK5801Sensor::getMetrics(meshtastic_Telemetry *measurement)
{
    if (!rak5801Present) {
        detectRAK5801();
    }

    rak5801EnableSensorPower();
    delay(50);

    bool valid = false;
    const float a0 = rak5801ReadChannelCurrentMa(RAK5801_A0_PIN);
    if (rak5801CurrentLooksValid(a0)) {
        measurement->variant.environment_metrics.has_current_a0 = true;
        measurement->variant.environment_metrics.current_a0 = a0;
        valid = true;
    }
    const float a1 = rak5801ReadChannelCurrentMa(RAK5801_A1_PIN);
    if (rak5801CurrentLooksValid(a1)) {
        measurement->variant.environment_metrics.has_current_a1 = true;
        measurement->variant.environment_metrics.current_a1 = a1;
        valid = true;
    }

    if (!valid) {
        rak5801LogProbe("read", a0, a1);
    }
    return valid;
}

#endif
