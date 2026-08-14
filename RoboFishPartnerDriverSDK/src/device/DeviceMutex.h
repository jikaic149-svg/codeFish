#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class DeviceMutex {
 public:
  DeviceMutex() : handle_(nullptr), storage_() {}

  bool begin() {
    if (!handle_) handle_ = xSemaphoreCreateMutexStatic(&storage_);
    return handle_ != nullptr;
  }

  bool take(TickType_t timeout) {
    return handle_ && xSemaphoreTake(handle_, timeout) == pdTRUE;
  }

  void give() {
    if (handle_) xSemaphoreGive(handle_);
  }

 private:
  SemaphoreHandle_t handle_;
  StaticSemaphore_t storage_;
};

class ScopedDeviceLock {
 public:
  ScopedDeviceLock(DeviceMutex &mutex, TickType_t timeout)
      : mutex_(mutex), acquired_(mutex_.take(timeout)) {}

  ~ScopedDeviceLock() {
    if (acquired_) mutex_.give();
  }

  bool acquired() const {
    return acquired_;
  }

 private:
  DeviceMutex &mutex_;
  bool acquired_;
};
