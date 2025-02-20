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

#include "mbed.h"
#include "ei_classifier_porting.h"

/**
 * This file overrides the __weak functions in the ei_classifier_porting.cpp
 * file for Mbed OS
 * This adds cancelable events to the inference loop
 */

#define EI_SIGNAL_TERMINATE_THREAD_REQ      0x1
#define EI_SIGNAL_TERMINATE_THREAD_RES      0x4

/**
 * Verify if our thread was canceled
 */
EI_IMPULSE_ERROR ei_run_impulse_check_canceled() {
    uint32_t signal = osThreadFlagsGet();
    osThreadFlagsClear(signal);
    if (signal == EI_SIGNAL_TERMINATE_THREAD_REQ) {
        return EI_IMPULSE_CANCELED;
    }
    return EI_IMPULSE_OK;
}

/**
 * Cancelable sleep, can be triggered with signal from other thread
 */
EI_IMPULSE_ERROR ei_sleep(int32_t time_ms) {
#if DEVICE_LPTICKER
    LowPowerTimer timer;
#else
    Timer timer;
#endif
    timer.start();

    while (1) {
        int32_t sleep_time = 100;
        if (time_ms - timer.read_ms() < 100) {
            sleep_time = time_ms - timer.read_ms();
        }
        ThisThread::sleep_for(sleep_time);

        uint32_t signal = osThreadFlagsGet();
        osThreadFlagsClear(signal);
        if (signal == EI_SIGNAL_TERMINATE_THREAD_REQ) {
            return EI_IMPULSE_CANCELED;
        }

        if (timer.read_ms() >= time_ms) {
            return EI_IMPULSE_OK;
        }
    }
}
