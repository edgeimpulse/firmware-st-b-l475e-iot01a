/* Edge Impulse ingestion SDK
 * Copyright (c) 2020 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _EI_ACCELEROMETER_H_
#define _EI_ACCELEROMETER_H_

#include "stm32l475e_iot01_accelero.h"
#include "ei_sampler.h"

extern EdgeSampler *sampler;
extern DigitalOut led;

bool ei_accelerometer_init() {
    return BSP_ACCELERO_Init() == ACCELERO_OK;
}

void ei_accelerometer_sample_read_data(float *values, size_t value_size) {
    int16_t accel_data[3] = { 0 };
    BSP_ACCELERO_AccGetXYZ(accel_data);

    values[0] = static_cast<float>(accel_data[0]) / 100.0f;
    values[1] = static_cast<float>(accel_data[1]) / 100.0f;
    values[2] = static_cast<float>(accel_data[2]) / 100.0f;
}

/**
 * Sample raw data
 */
bool ei_accelerometer_sample_start() {
    if (ei_config_get_config()->sample_interval_ms < 10.0f) {
        ei_config_set_sample_interval(10.0f);
    }

    sensor_aq_payload_info payload = {
        // Unique device ID (optional), set this to e.g. MAC address or device EUI **if** your device has one
        WiFiInterface::get_default_instance()->get_mac_address(),
        // Device type (required), use the same device type for similar devices
        EDGE_STRINGIZE(TARGET_NAME),
        // How often new data is sampled in ms. (100Hz = every 10 ms.)
        ei_config_get_config()->sample_interval_ms,
        // The axes which you'll use. The units field needs to comply to SenML units (see https://www.iana.org/assignments/senml/senml.xhtml)
        { { "accX", "m/s2" }, { "accY", "m/s2" }, { "accZ", "m/s2" } }
    };

    sampler->start_sampling(&payload, 2000, &ei_accelerometer_sample_read_data, &led);

    return true;
}

#endif // _EI_ACCELEROMETER_H_
