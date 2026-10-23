#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
bool usbTelemetryBegin(QueueHandle_t snapshots);
unsigned usbTelemetryTestBaud();
