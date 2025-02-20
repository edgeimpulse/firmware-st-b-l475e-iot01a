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

#ifndef _EI_MICROPHONE_H_
#define _EI_MICROPHONE_H_

#include "stm32l475e_iot01_audio.h"
#include "ei_sampler.h"
#include "ei_microphone_inference.h"

/**
 * todo: we're using raw block device here because it's much faster
 *       however, we don't do wear-leveling at this point because of this
 *       we need to implement this.
 */

#define BD_ERASE_SIZE                   4096

static uint16_t PCM_Buffer[PCM_BUFFER_LEN / 2];
static BSP_AUDIO_Init_t mic_params;

// semaphore to fire when one of the buffers is full
static Semaphore buffer_full_semaphore(0);

// buffer, needs to be aligned to temp flash erase size
#define AUDIO_BUFFER_NB_SAMPLES         2048

// we'll use only half the buffer at the same time, because we have flash latency
// of up to 70 ms.
static int16_t AUDIO_BUFFER[AUDIO_BUFFER_NB_SAMPLES];
static size_t AUDIO_BUFFER_IX = 0;

#define AUDIO_THREAD_STACK_SIZE             4096
static uint8_t AUDIO_THREAD_STACK[AUDIO_THREAD_STACK_SIZE];

// skip the first 2000 ms.
static bool is_recording = false;
static uint32_t audio_event_count = 0;
static uint32_t max_audio_event_count = 0;
static size_t mic_header_length = 0;

extern EdgeSampler *sampler;
extern SlicingBlockDevice temp_bd;
extern EdgeWsClient *ws_client;
static EventQueue mic_queue;
static bool is_uploaded = false;
static size_t TEMP_FILE_IX = 0;

extern void print_memory_info();
extern EventQueue main_application_queue;
extern DigitalOut led;

static unsigned char ei_mic_ctx_buffer[1024];
static sensor_aq_signing_ctx_t ei_mic_signing_ctx;
static sensor_aq_mbedtls_hs256_ctx_t ei_mic_hs_ctx;
static sensor_aq_ctx ei_mic_ctx = {
    { ei_mic_ctx_buffer, 1024 },
    &ei_mic_signing_ctx,
    &fwrite,
    &fseek,
    &time
};

static void ei_microphone_blink() {
    led = !led;
}

enum AUDIO_BUFFER_EVENT {
    HALF = 0,
    FULL = 1,
    FINALIZE = 2,
    NONE = 3
};

static AUDIO_BUFFER_EVENT last_audio_buffer_event = NONE;

static void audio_buffer_callback(AUDIO_BUFFER_EVENT event) {
    int16_t *start = AUDIO_BUFFER + ((AUDIO_BUFFER_NB_SAMPLES / 2) * event);
    size_t buffer_size = (AUDIO_BUFFER_NB_SAMPLES / 2) * sizeof(int16_t);

    if (event == FINALIZE) {
        if (last_audio_buffer_event == HALF) {
            start = AUDIO_BUFFER + ((AUDIO_BUFFER_NB_SAMPLES / 2) * FULL);
            buffer_size = (AUDIO_BUFFER_IX - (AUDIO_BUFFER_NB_SAMPLES / 2)) * sizeof(int16_t);
        }
        else {
            start = AUDIO_BUFFER + ((AUDIO_BUFFER_NB_SAMPLES / 2) * HALF);
            buffer_size = AUDIO_BUFFER_IX * sizeof(int16_t);
        }
    }

    int ret = temp_bd.program(start, TEMP_FILE_IX, buffer_size);
    TEMP_FILE_IX += buffer_size;

    ei_mic_ctx.signature_ctx->update(ei_mic_ctx.signature_ctx, (uint8_t*)start, buffer_size);

    last_audio_buffer_event = event;

    if (ret != 0) {
        printf("ERR: audio_buffer_callback %d write failed (%d)\n", event, ret);
    }
}

static void finish_and_upload(char *filename, uint32_t sample_length_ms) {
    printf("Done sampling, total bytes collected: %u\n", TEMP_FILE_IX);

    {
        Ticker t;
        t.attach(&ei_microphone_blink, 1.0f);

        if (ws_client->is_connected()) {
            ws_client->send_sample_uploading();
        }

        printf("[1/1] Uploading file to Edge Impulse...\n");

        Timer upload_timer;
        upload_timer.start();

        sampler->upload_buffer(&temp_bd, 0, TEMP_FILE_IX, filename);

        upload_timer.stop();
        printf("[1/1] Uploading file to Edge Impulse OK (took %d ms.)\n", upload_timer.read_ms());

        is_uploaded = true;
    }

    led = 0;

    printf("OK\n");
}

/**
 * @brief      Called from the BSP audio callback function
 *
 * @param[in]  halfOrCompleteTransfer  Signal half or complete to indicate offset in sampleBuffer
 */
static void get_audio_from_dma_buffer(bool halfOrCompleteTransfer)
{
    if (!is_recording) return;
    if (audio_event_count >= max_audio_event_count) return;
    audio_event_count++;

    uint32_t buffer_size = PCM_BUFFER_LEN / 2;           /* Half Transfer */
    uint32_t nb_samples = buffer_size / sizeof(int16_t); /* Bytes to Length */

    if ((AUDIO_BUFFER_IX + nb_samples) > AUDIO_BUFFER_NB_SAMPLES) {
        return;
    }

    /* Copy second half of PCM_Buffer from Microphones onto Fill_Buffer */
    if (halfOrCompleteTransfer) {
        memcpy(AUDIO_BUFFER + AUDIO_BUFFER_IX, PCM_Buffer + nb_samples, buffer_size);
    } else {
        memcpy(AUDIO_BUFFER + AUDIO_BUFFER_IX, PCM_Buffer, buffer_size);
    }
    AUDIO_BUFFER_IX += nb_samples;

    if (AUDIO_BUFFER_IX == (AUDIO_BUFFER_NB_SAMPLES / 2)) {
        // half full
        mic_queue.call(&audio_buffer_callback, HALF);
    } else if (AUDIO_BUFFER_IX == AUDIO_BUFFER_NB_SAMPLES) {
        // completely full
        AUDIO_BUFFER_IX = 0;
        mic_queue.call(&audio_buffer_callback, FULL);
    }
}

/**
  * @brief  Manages the BSP audio in error event.
  * @param  Instance Audio in instance.
  * @retval None.
  */
void BSP_AUDIO_IN_Error_CallBack(uint32_t Instance) {
    printf("BSP_AUDIO_IN_Error_CallBack\n");
}

bool ei_microphone_init() {
    mic_params.BitsPerSample = 16;
    mic_params.ChannelsNbr = AUDIO_CHANNELS;
    mic_params.Device = AUDIO_IN_DIGITAL_MIC1;
    mic_params.SampleRate = AUDIO_SAMPLING_FREQUENCY;
    mic_params.Volume = 32;

    int32_t ret = BSP_AUDIO_IN_Init(AUDIO_INSTANCE, &mic_params);
    if (ret != BSP_ERROR_NONE) {
        printf("Error Audio Init (%ld)\r\n", ret);
        return false;
    }
    return true;
}

bool ei_microphone_record(uint32_t sample_length_ms, uint32_t start_delay_ms, bool print_start_messages) {
    if (is_recording) return false;

    Thread queue_thread(osPriorityHigh, AUDIO_THREAD_STACK_SIZE, AUDIO_THREAD_STACK, "audio-thread-stack");
    queue_thread.start(callback(&mic_queue, &EventQueue::dispatch_forever));

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

    AUDIO_BUFFER_IX = 0;
    is_recording = false;
    is_uploaded = false;
    audio_event_count = 0;
    max_audio_event_count = sample_length_ms; // each event is 1ms., very convenient
    TEMP_FILE_IX = 0;

    ei_microphone_inference_set_bsp_callback(&get_audio_from_dma_buffer);

    ret = BSP_AUDIO_IN_Record(AUDIO_INSTANCE, (uint8_t *) PCM_Buffer, PCM_BUFFER_LEN);
    if (ret != BSP_ERROR_NONE) {
        printf("Error failed to start recording (%ld)\n", ret);
        return false;
    }

    if (print_start_messages) {
        printf("Starting in %lu ms... (or until all flash was erased)\n", start_delay_ms);
    }

    // clear out flash memory
    size_t blocks = static_cast<size_t>(ceil(
        (static_cast<float>(sample_length_ms) * static_cast<float>(AUDIO_SAMPLING_FREQUENCY / 1000) * 2.0f) /
        BD_ERASE_SIZE)) + 1;
    // printf("Erasing %lu blocks\n", blocks);

    Timer erase_timer;
    erase_timer.start();

    for (size_t ix = 0; ix < blocks; ix++) {
        int r = temp_bd.erase(ix * BD_ERASE_SIZE, BD_ERASE_SIZE);
        if (r != 0) {
            printf("Failed to erase block device #2 (%d)\n", r);
            return false;
        }
    }

    // Gonna write the header to the header block
    // the issue here is that we bound the context already to FILE
    // which we don't have here... hmm
    {
        mic_header_length = 0;

        sensor_aq_init_mbedtls_hs256_context(&ei_mic_signing_ctx, &ei_mic_hs_ctx, ei_config_get_config()->sample_hmac_key);

        sensor_aq_payload_info payload = {
            WiFiInterface::get_default_instance()->get_mac_address(),
            EDGE_STRINGIZE(TARGET_NAME),
            1000.0f / static_cast<float>(AUDIO_SAMPLING_FREQUENCY),
            { { "audio", "wav" } }
        };

        int tr = sensor_aq_init(&ei_mic_ctx, &payload, NULL, true);
        if (tr != AQ_OK) {
            printf("sensor_aq_init failed (%d)\n", tr);
            return 1;
        }

        // then we're gonna find the last byte that is not 0x00 in the CBOR buffer.
        // That should give us the whole header
        size_t end_of_header_ix = 0;
        for (int ix = static_cast<int>(ei_mic_ctx.cbor_buffer.len) - 1; ix >= 0; ix--) {
            if (((uint8_t*)ei_mic_ctx.cbor_buffer.ptr)[ix] != 0x0) {
                end_of_header_ix = ix;
                break;
            }
        }

        if (end_of_header_ix == 0) {
            printf("Failed to find end of header\n");
            return false;
        }

        // Write to blockdevice
        tr = temp_bd.program(ei_mic_ctx.cbor_buffer.ptr, 0, end_of_header_ix);
        if (tr != 0) {
            printf("Failed to write to header blockdevice (%d)\n", tr);
            return false;
        }

        mic_header_length += end_of_header_ix;

        // Then we're gonna write our reference ("Ref-BINARY-i16" as CBOR string and then 0xFF to finish up the infinite length array)
        uint8_t ref[] = {
            0x6E, 0x52, 0x65, 0x66, 0x2D, 0x42, 0x49, 0x4E, 0x41, 0x52, 0x59, 0x2D, 0x69, 0x31, 0x36, 0xFF
        };
        tr = temp_bd.program(ref, end_of_header_ix, sizeof(ref));
        if (tr != 0) {
            printf("Failed to write to header blockdevice (%d)\n", tr);
            return false;
        }

        mic_header_length += sizeof(ref);

        // and update the signature
        tr = ei_mic_ctx.signature_ctx->update(ei_mic_ctx.signature_ctx, ref, sizeof(ref));
        if (tr != 0) {
            printf("Failed to update signature from header (%d)\n", tr);
            return false;
        }

        TEMP_FILE_IX = mic_header_length;
    }

    erase_timer.stop();
    // printf("Erase timer took %d ms.\n", erase_timer.read_ms());

    if (erase_timer.read_ms() > static_cast<int>(start_delay_ms)) {
        start_delay_ms = 0;
    }
    else {
        start_delay_ms = start_delay_ms - erase_timer.read_ms();
    }

    ThisThread::sleep_for(start_delay_ms);

    if (ws_client->is_connected()) {
        ws_client->send_sample_started();
    }

    if (print_start_messages) {
        printf("Sampling...\n");
    }

    led = 1;

    is_recording = true;

    ThisThread::sleep_for(sample_length_ms);

    // we're not perfectly aligned  with the audio interrupts, so give some extra time to ensure
    // we always have at least the sample_length_ms number of frames
    int32_t extra_time_left = 1000;
    while (audio_event_count < max_audio_event_count) {
        ThisThread::sleep_for(10);
        extra_time_left -= 10;
        if (extra_time_left <= 0) {
            break;
        }
    }

    led = 0;

    is_recording = false;

    ret = BSP_AUDIO_IN_Stop(AUDIO_INSTANCE);
    if (ret != BSP_ERROR_NONE) {
        printf("Error failed to stop recording (%ld)\n", ret);
        return false;
    }

    // give the mic queue some time to process last events
    // mbed eventqueue should really have a way of knowing when the queue is empty
    ThisThread::sleep_for(100);

    audio_buffer_callback(FINALIZE);

    // too much data? cut it
    if (TEMP_FILE_IX > mic_header_length + (sample_length_ms * (AUDIO_SAMPLING_FREQUENCY / 1000) * 2)) {
        TEMP_FILE_IX = mic_header_length + (sample_length_ms * (AUDIO_SAMPLING_FREQUENCY / 1000) * 2);
    }

    int ctx_err = ei_mic_ctx.signature_ctx->finish(ei_mic_ctx.signature_ctx, ei_mic_ctx.hash_buffer.buffer);
    if (ctx_err != 0) {
        printf("Failed to finish signature (%d)\n", ctx_err);
        return false;
    }

    // load the first page in flash...
    uint8_t *page_buffer = (uint8_t*)malloc(BD_ERASE_SIZE);
    if (!page_buffer) {
        printf("Failed to allocate a page buffer to write the hash\n");
        return false;
    }

    int j = temp_bd.read(page_buffer, 0, BD_ERASE_SIZE);
    if (j != 0) {
        printf("Failed to read first page (%d)\n", j);
        free(page_buffer);
        return false;
    }

    // update the hash
    uint8_t *hash = ei_mic_ctx.hash_buffer.buffer;
    // we have allocated twice as much for this data (because we also want to be able to represent in hex)
    // thus only loop over the first half of the bytes as the signature_ctx has written to those
    for (size_t hash_ix = 0; hash_ix < ei_mic_ctx.hash_buffer.size / 2; hash_ix++) {
        // this might seem convoluted, but snprintf() with %02x is not always supported e.g. by newlib-nano
        // we encode as hex... first ASCII char encodes top 4 bytes
        uint8_t first = (hash[hash_ix] >> 4) & 0xf;
        // second encodes lower 4 bytes
        uint8_t second = hash[hash_ix] & 0xf;

        // if 0..9 -> '0' (48) + value, if >10, then use 'a' (97) - 10 + value
        char first_c = first >= 10 ? 87 + first : 48 + first;
        char second_c = second >= 10 ? 87 + second : 48 + second;

        page_buffer[ei_mic_ctx.signature_index + (hash_ix * 2) + 0] = first_c;
        page_buffer[ei_mic_ctx.signature_index + (hash_ix * 2) + 1] = second_c;
    }

    j = temp_bd.erase(0, BD_ERASE_SIZE);
    if (j != 0) {
        printf("Failed to erase first page (%d)\n", j);
        free(page_buffer);
        return false;
    }

    j = temp_bd.program(page_buffer, 0, BD_ERASE_SIZE);

    free(page_buffer);

    if (j != 0) {
        printf("Failed to write first page with updated hash (%d)\n", j);
        return false;
    }

    return true;
}

/**
 * Sample raw data
 */
bool ei_microphone_sample_start() {
    // this sensor does not have settable interval...
    ei_config_set_sample_interval(static_cast<float>(1000) / static_cast<float>(AUDIO_SAMPLING_FREQUENCY));

    printf("Sampling settings:\n");
    printf("\tInterval: %.5f ms.\n", (float)ei_config_get_config()->sample_interval_ms);
    printf("\tLength: %lu ms.\n", ei_config_get_config()->sample_length_ms);
    printf("\tName: %s\n", ei_config_get_config()->sample_label);
    printf("\tHMAC Key: %s\n", ei_config_get_config()->sample_hmac_key);

    char filename[256];
    int fn_r = snprintf(filename, 256, "/fs/%s", ei_config_get_config()->sample_label);
    if (fn_r <= 0) {
        printf("ERR: Failed to allocate file name\n");
        return false;
    }

    printf("\tFile name: %s\n", filename);

    bool r = ei_microphone_record(ei_config_get_config()->sample_length_ms, 2000, true);
    if (!r) {
        return r;
    }

    // print_memory_info();

    // finalize and upload the file
    finish_and_upload(filename, ei_config_get_config()->sample_length_ms);

    while (1) {
        if (is_uploaded) break;
        ThisThread::sleep_for(10);
    }

    return true;
}

/**
 * Get raw audio signal data
 */
static int raw_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
    offset += mic_header_length;

    EIDSP_i16 *temp = (EIDSP_i16*)calloc(length, sizeof(EIDSP_i16));
    if (!temp) {
        printf("raw_audio_signal_get_data - malloc failed\n");
        return EIDSP_OUT_OF_MEM;
    }

    int r = temp_bd.read(temp, offset * sizeof(EIDSP_i16), length * sizeof(EIDSP_i16));
    if (r != 0) {
        printf("raw_audio_signal_get_data - BD read failed (%d)\n", r);
        free(temp);
        return r;
    }

    r = numpy::int16_to_float(temp, out_ptr, length);
    free(temp);

    return r;
}

/**
 * Get signal from the just recorded data (not yet processed)
 */
signal_t ei_microphone_get_signal() {
    signal_t signal;
    signal.total_length = (TEMP_FILE_IX - mic_header_length) / sizeof(int16_t);
    signal.get_data = &raw_audio_signal_get_data;
    return signal;
}

#endif // _EI_MICROPHONE_H_
