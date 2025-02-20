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

/* Include ----------------------------------------------------------------- */
#include "edge-impulse-sdk/CMSIS/DSP/Include/arm_math.h"
#include "ei_classifier_porting.h"
#include "mbed.h"
#include "stdlib.h"
#include "stm32l475e_iot01_audio.h"
#include "ei_microphone_inference.h"
#include "numpy.hpp"

/**
 * Double buffered struct for collecting and handling sampling data
 */
typedef struct {
    int16_t *buffers[2];
    uint8_t buf_select;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

using namespace ei;

/**
 * Used sample buffer size in samples
 */
#define SAMPLE_BUFFER_SIZE 1600

/* Private variables ------------------------------------------------------- */
static inference_t inference;
static signed short sampleBuffer[SAMPLE_BUFFER_SIZE];
static bool record_ready = false;

void (*audio_handle_callback)(bool halfOrCompleteTransfer) = 0;

/* Private functions ------------------------------------------------------- */

/**
 * @brief      Copy sample data in selected buf and signal ready when buffer is full
 *
 * @param[in]  n_bytes  Number of bytes to copy
 * @param[in]  offset   offset in sampleBuffer
 */
static void audio_buffer_inference_callback(uint32_t n_bytes, uint32_t offset)
{
    for (uint32_t i = 0; i<n_bytes>> 1; i++) {
        inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[offset + i];

        if (inference.buf_count >= inference.n_samples) {
            inference.buf_select ^= 1;
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }
}

/**
 * @brief      Callback from BSP audio function
 *
 * @param[in]  halfOrCompleteTransfer  Signal half or complete to indicate offset in sampleBuffer
 */
static void pdm_data_ready_inference_callback(bool halfOrCompleteTransfer)
{
    if (record_ready == true) {
        audio_buffer_inference_callback(SAMPLE_BUFFER_SIZE,
                                        halfOrCompleteTransfer ? SAMPLE_BUFFER_SIZE >> 1 : 0);
    }
}

/* Public functions -------------------------------------------------------- */

/**
 * @brief      Setup inference struct and and start audio sampling
 *
 * @param[in]  n_samples  Number of samples needed per inference
 *
 * @return     false on error
 */
bool ei_microphone_inference_start(uint32_t n_samples)
{
    inference.buffers[0] = (int16_t *)malloc(n_samples * sizeof(int16_t));

    if (inference.buffers[0] == NULL) {
        return false;
    }

    inference.buffers[1] = (int16_t *)malloc(n_samples * sizeof(int16_t));

    if (inference.buffers[1] == NULL) {
        free(inference.buffers[0]);
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count = 0;
    inference.n_samples = n_samples;
    inference.buf_ready = 0;

    ei_microphone_inference_set_bsp_callback(&pdm_data_ready_inference_callback);

    int32_t ret;
    uint32_t state;
    ret = BSP_AUDIO_IN_GetState(AUDIO_INSTANCE, &state);
    if (ret != BSP_ERROR_NONE) {
        printf("Cannot start recording: Error getting audio state (%ld)\n", ret);
        return false;
    }
    if (state == AUDIO_IN_STATE_RECORDING) {
        printf("Cannot start recording: Already recording\n");
        return false;
    }

    ret = BSP_AUDIO_IN_Record(AUDIO_INSTANCE, (uint8_t *)sampleBuffer, SAMPLE_BUFFER_SIZE * 2);

    if (ret != BSP_ERROR_NONE) {
        printf("Error failed to start recording (%ld)\n", ret);
        return false;
    }

    record_ready = true;

    return true;
}

/**
 * @brief      Wait for a full buffer
 *
 * @return     In case of an buffer overrun return false
 */
bool ei_microphone_inference_record(void)
{
    bool ret = true;

    if (inference.buf_ready == 1) {
        printf(
            "Error sample buffer overrun. Decrease the number of slices per model window "
            "(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)\n");
        ret = false;
    }

    while (inference.buf_ready == 0) {
        ThisThread::sleep_for(1);
    }

    inference.buf_ready = 0;

    return ret;
}

/**
 * Get raw audio signal data
 */
int ei_microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);

    return 0;
}

/**
 * @brief      Stop audio sampling, release sampling buffers
 *
 * @return     false on error
 */
bool ei_microphone_inference_end(void)
{
    int32_t ret = BSP_AUDIO_IN_Stop(AUDIO_INSTANCE);
    if (ret != BSP_ERROR_NONE) {
        printf("Error failed to stop recording (%ld)\n", ret);
        return false;
    }

    record_ready = false;
    free(inference.buffers[0]);
    free(inference.buffers[1]);

    return true;
}

/**
 * @brief      Set audio handle callback
 * @details    Creates a seperation between sample collection for ingestion and inferencing
 *
 * @param[in]  sample_handle  Callback function
 */
void ei_microphone_inference_set_bsp_callback(void (*sample_handle)(bool halfOrCompleteTransfer))
{
    audio_handle_callback = sample_handle;
}

/**
 * @brief      Called by stm audio driver after completion of half the buffer DMA transfer
 *
 * @param[in]  Instance  Audio instance number
 */
void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)
{
    if (audio_handle_callback) {
        audio_handle_callback(0);
    }
}

/**
 * @brief      Called by stm audio driver after completion of a full buffer DMA transfer
 *
 * @param[in]  Instance  Audio instance number
 */
void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
    if (audio_handle_callback) {
        audio_handle_callback(1);
    }
}