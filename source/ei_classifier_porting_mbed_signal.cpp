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
