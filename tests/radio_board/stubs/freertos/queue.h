#pragma once
struct HostQueue;
using QueueHandle_t = HostQueue *;
int xQueueReceive(QueueHandle_t, void *, unsigned);
int xQueueReset(QueueHandle_t);
