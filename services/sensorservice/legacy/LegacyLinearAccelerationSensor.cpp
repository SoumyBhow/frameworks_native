/*
 * Copyright (C) 2010 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdint.h>
#include <math.h>
#include <sys/types.h>

#include <utils/Errors.h>

#include <hardware/sensors.h>

#include "LegacyLinearAccelerationSensor.h"
#include "../SensorDevice.h"
#include "../SensorFusion.h"

namespace android {
// ---------------------------------------------------------------------------

LegacyLinearAccelerationSensor::LegacyLinearAccelerationSensor(sensor_t const* list, size_t count)
    : mGravitySensor(list, count)
{
    mData[0] = mData[1] = mData[2] = 0;

    const Sensor &gsensor = mGravitySensor.getSensor();
    const sensor_t sensor = {
        .name       = "Linear Acceleration Sensor",
        .vendor     = "AOSP",
        .version    = gsensor.getVersion(),
        .handle     = '_lin',
        .type       = SENSOR_TYPE_LINEAR_ACCELERATION,
        .maxRange   = gsensor.getMaxValue(),
        .resolution = gsensor.getResolution(),
        .power      = gsensor.getPowerUsage(),
        .minDelay   = gsensor.getMinDelay(),
    };
    mSensor = Sensor(&sensor);
}

bool LegacyLinearAccelerationSensor::process(sensors_event_t* outEvent,
        const sensors_event_t& event)
{
    bool result = mGravitySensor.process(outEvent, event);
    if (result) {
        if (event.type == SENSOR_TYPE_ACCELEROMETER) {
            mData[0] = event.acceleration.x;
            mData[1] = event.acceleration.y;
            mData[2] = event.acceleration.z;
        }
        outEvent->data[0] = mData[0] - outEvent->data[0];
        outEvent->data[1] = mData[1] - outEvent->data[1];
        outEvent->data[2] = mData[2] - outEvent->data[2];
        outEvent->sensor = '_lin';
        outEvent->type = SENSOR_TYPE_LINEAR_ACCELERATION;
        return true;
    }
    return false;
}

status_t LegacyLinearAccelerationSensor::activate(void* ident, bool enabled) {
    return mGravitySensor.activate(ident, enabled);
}

status_t LegacyLinearAccelerationSensor::setDelay(void* ident, int handle, int64_t ns) {
    return mGravitySensor.setDelay(ident, handle, ns);
}

// ---------------------------------------------------------------------------
}; // namespace android

