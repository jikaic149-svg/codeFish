#pragma once
#include <stddef.h>
#include <stdint.h>
#include <deque>
#include <vector>
struct HostQueue {
  size_t capacity = 0, itemBytes = 0;
  std::deque<std::vector<uint8_t>> entries;
};
using StaticQueue_t = HostQueue;
using QueueHandle_t = HostQueue *;
QueueHandle_t xQueueCreateStatic(unsigned, unsigned, uint8_t *, StaticQueue_t *);
int xQueueSend(QueueHandle_t, const void *, unsigned);
int xQueueReceive(QueueHandle_t, void *, unsigned);
int xQueueOverwrite(QueueHandle_t, const void *);
int xQueuePeek(QueueHandle_t, void *, unsigned);
