#include "PumpOutput.h"

#include "../config/sdk_config.h"
#include "driver/gpio.h"
#include "esp_timer.h"

PumpOutput::PumpOutput() : enabled_(false), startedMs_(0) {}

bool PumpOutput::begin() {
  if (!mutex_.begin()) return false;
  const gpio_config_t configuration = {
      .pin_bit_mask = 1ULL << RF_PUMP_PIN,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  return gpio_config(&configuration) == ESP_OK;
}

void PumpOutput::setEnabled(bool enabled, TickType_t timeout) {
  ScopedDeviceLock lock(mutex_, timeout);
  if (!lock.acquired()) return;
  writeUnlocked(enabled);
}

void PumpOutput::stop(TickType_t timeout) {
  setEnabled(false, timeout);
}

bool PumpOutput::update(uint32_t nowMs, TickType_t timeout) {
  ScopedDeviceLock lock(mutex_, timeout);
  if (!lock.acquired()) return false;

  if (enabled_ && startedMs_ != 0 &&
      nowMs - startedMs_ >= RF_PUMP_MAX_ON_MS) {
    writeUnlocked(false);
    return true;
  }
  return false;
}

void PumpOutput::writeUnlocked(bool enabled) {
  gpio_set_level(static_cast<gpio_num_t>(RF_PUMP_PIN),
                 enabled ? (RF_PUMP_ACTIVE_HIGH ? 1 : 0)
                         : (RF_PUMP_ACTIVE_HIGH ? 0 : 1));
  enabled_ = enabled;
  startedMs_ = enabled
                   ? static_cast<uint32_t>(esp_timer_get_time() / 1000ULL)
                   : 0;
}
