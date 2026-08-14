#include "app/RoboFishRuntime.h"
#include "config/sdk_config.h"

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr char kTag[] = "main";

void confirmRunningOtaImage(bool coreReady) {
  const esp_partition_t *running = esp_ota_get_running_partition();
  if (!running) {
    ESP_LOGE(kTag, "Cannot resolve the running OTA partition");
    return;
  }

  esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
  esp_err_t error = esp_ota_get_state_partition(running, &state);
  if (error == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY) {
    if (!coreReady) {
      ESP_LOGE(kTag,
               "Core runtime unavailable; leaving OTA image pending so the "
               "bootloader can roll it back");
      return;
    }
    error = esp_ota_mark_app_valid_cancel_rollback();
    if (error == ESP_OK) {
      ESP_LOGI(kTag, "Running OTA image marked valid");
    } else {
      ESP_LOGE(kTag, "Failed to mark OTA image valid: %s",
               esp_err_to_name(error));
    }
  }
}

}  // namespace

extern "C" void app_main(void) {
  static RoboFishRuntime runtime;

  const bool coreReady = runtime.begin();
  confirmRunningOtaImage(coreReady);

  for (;;) {
    runtime.update();
    vTaskDelay(pdMS_TO_TICKS(RF_MAIN_LOOP_DELAY_MS));
  }
}
