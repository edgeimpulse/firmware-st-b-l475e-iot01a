/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
