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

#ifndef _EI_MAX30102_H_
#define _EI_MAX30102_H_

#include "MAX30102.h"
#include "ei_sampler.h"

extern EdgeSampler *sampler;

/**
 * Initialize the sensor
 * @returns boolean whether the sensor was present
 */
bool ei_max30102_init() {
    maxim_max30102_reset();
    uint8_t uch_dummy;
    bool max30102_present = maxim_max30102_read_reg(0, &uch_dummy);
    if (max30102_present) {
        maxim_max30102_init();
    }
    return max30102_present;
}

void ei_max30102_sample_read_data(float *values, size_t value_size) {
    static DigitalIn INT(D2);
    while (INT.read() == 1);

    uint32_t red;
    uint32_t ir;

    maxim_max30102_read_fifo(&red, &ir);

    values[0] = static_cast<float>(red);
    values[1] = static_cast<float>(ir);
}

bool ei_max30102_sample_start() {
    ei_config_set_sample_interval(10); // this sensor does not have settable interval...

    sensor_aq_payload_info payload = {
        // Unique device ID (optional), set this to e.g. MAC address or device EUI **if** your device has one
        WiFiInterface::get_default_instance()->get_mac_address(),
        // Device type (required), use the same device type for similar devices
        EDGE_STRINGIZE(TARGET_NAME),
        // How often new data is sampled in ms. (100Hz = every 10 ms.)
        10.0f,
        // The axes which you'll use. The units field needs to comply to SenML units (see https://www.iana.org/assignments/senml/senml.xhtml)
        { { "red", "lx" }, { "ir", "lx" } }
    };

    // warm up the sensor
    Timer t;
    t.start();
    while (t.read_ms() < 2000) {
        float values[2];
        ei_max30102_sample_read_data(values, 2);
    }

    sampler->start_sampling(&payload, 0, &ei_max30102_sample_read_data);

    return true;
}

#endif // _EI_MAX30102_H_
