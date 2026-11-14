#ifndef SHARED_DATA_LOCK_H_
#define SHARED_DATA_LOCK_H_

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Guards the data shared between the calculation tasks and the inverter tasks:
//  - the global "rules" object (published snapshot from rules_task)
//  - the global "currentMonitor" struct (published by the current monitor tasks)
//
// Writers build new values in a private copy, then copy them in whilst holding
// this lock. Readers that use several related values together (e.g. building an
// inverter CAN/RS485 message) hold a SharedDataLock so they never see a half
// updated object.
//
// Keep the lock held only briefly. Never wait on another mutex while holding it,
// except for the existing order: RS485 or VSPI mutex first, then this lock.
extern SemaphoreHandle_t shared_data_mutex;

class SharedDataLock
{
public:
    SharedDataLock()
    {
        if (shared_data_mutex != nullptr)
        {
            xSemaphoreTake(shared_data_mutex, portMAX_DELAY);
        }
    }
    ~SharedDataLock()
    {
        if (shared_data_mutex != nullptr)
        {
            xSemaphoreGive(shared_data_mutex);
        }
    }
    SharedDataLock(const SharedDataLock &) = delete;
    SharedDataLock &operator=(const SharedDataLock &) = delete;
};

#endif
