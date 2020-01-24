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

#ifndef _EI_MICROPHONE_H_
#define _EI_MICROPHONE_H_

#include "stm32l475e_iot01_audio.h"
#include "ei_sampler.h"

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

extern EdgeSampler *sampler;
extern SlicingBlockDevice temp_bd;
extern EdgeWsClient *ws_client;
static EventQueue mic_queue;
static bool is_uploaded = false;
static size_t TEMP_FILE_IX = 0;

extern void print_memory_info();
extern EventQueue main_application_queue;
extern DigitalOut led;

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

    last_audio_buffer_event = event;

    if (ret != 0) {
        printf("ERR: audio_buffer_callback %d write failed (%d)\n", event, ret);
    }
}

static void finish_and_upload(FILE *file, char *filename, uint32_t sample_length_ms) {
    printf("Done sampling, total bytes collected: %u\n", TEMP_FILE_IX);
    {
        Ticker t;
        t.attach(&ei_microphone_blink, 2.0f);

        printf("Processing...\n");

        if (ws_client->is_connected()) {
            ws_client->send_sample_processing();
        }

        printf("[1/3] Copying from tempfile to %s (this might take a while)...\n", filename);

        Timer copy_timer;
        copy_timer.start();

        int16_t read_buffer[1024];
        size_t bytes_left = TEMP_FILE_IX;
        size_t curr_temp_ix = 0;
        while (1) {
            size_t bytes_to_read = sizeof(read_buffer);
            if (bytes_to_read > bytes_left) {
                bytes_to_read = bytes_left;
            }

            int r = temp_bd.read(read_buffer, curr_temp_ix, bytes_to_read);
            if (r != 0) {
                printf("Failed to read from block device (ix=%u, bytes=%u) (%d)\n",
                    curr_temp_ix, bytes_to_read, r);
                return;
            }

            int ret = sampler->write_sensor_data_batch(read_buffer, bytes_to_read / 2);
            if (ret != AQ_OK) {
                printf("sampler->write_sensor_data_batch failed (%d)\n", ret);
                return;
            }

            curr_temp_ix += bytes_to_read;
            bytes_left -= bytes_to_read;

            if (bytes_left == 0) {
                break;
            }
        }

        copy_timer.stop();

        printf("[1/3] Copying from tempfile to %s OK (took %d ms.)\n", filename, copy_timer.read_ms());
        printf("[2/3] Calculating hash...\n");

        sampler->finish();

        printf("[2/3] Calculating hash OK\n");

        printf("Done processing\n");
    }

    {
        Ticker t;
        t.attach(&ei_microphone_blink, 1.0f);

        if (ws_client->is_connected()) {
            ws_client->send_sample_uploading();
        }

        printf("[3/3] Uploading file to Edge Impulse...\n");

        Timer upload_timer;
        upload_timer.start();

        sampler->upload_file(file, filename);

        upload_timer.stop();
        printf("[3/3] Uploading file to Edge Impulse OK (took %d ms.)\n", upload_timer.read_ms());

        is_uploaded = true;

        fclose(file);
    }

    printf("OK\n");
}

/**
* @brief  Half Transfer user callback, called by BSP functions.
* @param  None
* @retval None
*/
void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance) {
    if (!is_recording) return;
    if (audio_event_count >= max_audio_event_count) return;
    audio_event_count++;

    uint32_t buffer_size = PCM_BUFFER_LEN / 2; /* Half Transfer */
    uint32_t nb_samples = buffer_size / sizeof(int16_t); /* Bytes to Length */

    if ((AUDIO_BUFFER_IX + nb_samples) > AUDIO_BUFFER_NB_SAMPLES) {
        return;
    }

    /* Copy first half of PCM_Buffer from Microphones onto Fill_Buffer */
    memcpy(AUDIO_BUFFER + AUDIO_BUFFER_IX, PCM_Buffer, buffer_size);
    AUDIO_BUFFER_IX += nb_samples;

    if (AUDIO_BUFFER_IX == (AUDIO_BUFFER_NB_SAMPLES / 2)) {
        // half full
        mic_queue.call(&audio_buffer_callback, HALF);
    }
    else if (AUDIO_BUFFER_IX == AUDIO_BUFFER_NB_SAMPLES) {
        // completely full
        AUDIO_BUFFER_IX = 0;
        mic_queue.call(&audio_buffer_callback, FULL);
    }
}

/**
* @brief  Transfer Complete user callback, called by BSP functions.
* @param  None
* @retval None
*/
void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance) {
    if (!is_recording) return;
    if (audio_event_count >= max_audio_event_count) return;
    audio_event_count++;

    uint32_t buffer_size = PCM_BUFFER_LEN / 2; /* Half Transfer */
    uint32_t nb_samples = buffer_size / sizeof(int16_t); /* Bytes to Length */

    if ((AUDIO_BUFFER_IX + nb_samples) > AUDIO_BUFFER_NB_SAMPLES) {
        return;
    }

    /* Copy second half of PCM_Buffer from Microphones onto Fill_Buffer */
    memcpy(AUDIO_BUFFER + AUDIO_BUFFER_IX, PCM_Buffer + nb_samples, buffer_size);
    AUDIO_BUFFER_IX += nb_samples;

    if (AUDIO_BUFFER_IX == (AUDIO_BUFFER_NB_SAMPLES / 2)) {
        // half full
        mic_queue.call(&audio_buffer_callback, HALF);
    }
    else if (AUDIO_BUFFER_IX == AUDIO_BUFFER_NB_SAMPLES) {
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
        BD_ERASE_SIZE));
    // printf("Erasing %lu blocks\n", blocks);

    Timer erase_timer;
    erase_timer.start();

    for (size_t ix = 0; ix < blocks; ix++) {
        int r = temp_bd.erase(ix * BD_ERASE_SIZE, BD_ERASE_SIZE);
        if (r != 0) {
            printf("Failed to erase block device (%d)\n", r);
            return false;
        }
    }

    erase_timer.stop();
    // printf("Erase timer took %d ms.\n", erase_timer.read_ms());

    if (erase_timer.read_ms() > start_delay_ms) {
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
    if (TEMP_FILE_IX > sample_length_ms * (AUDIO_SAMPLING_FREQUENCY / 1000) * 2) {
        TEMP_FILE_IX = sample_length_ms * (AUDIO_SAMPLING_FREQUENCY / 1000) * 2;
    }

    return true;
}

/**
 * Sample raw data
 */
bool ei_microphone_sample_start() {
    // this sensor does not have settable interval...
    ei_config_set_sample_interval(static_cast<float>(1000) / static_cast<float>(AUDIO_SAMPLING_FREQUENCY));

    sensor_aq_payload_info payload = {
        // Unique device ID (optional), set this to e.g. MAC address or device EUI **if** your device has one
        WiFiInterface::get_default_instance()->get_mac_address(),
        // Device type (required), use the same device type for similar devices
        EDGE_STRINGIZE(TARGET_NAME),
        // How often new data is sampled in ms.
        ei_config_get_config()->sample_interval_ms,
        // The axes which you'll use. The units field are recommended to comply to SenML units (see https://www.iana.org/assignments/senml/senml.xhtml)
        { { "audio", "wav" } }
    };

    // print_memory_info();

    printf("Sampling settings:\n");
    printf("\tInterval: %.5f ms.\n", (float)ei_config_get_config()->sample_interval_ms);
    printf("\tLength: %lu ms.\n", ei_config_get_config()->sample_length_ms);
    printf("\tName: %s\n", ei_config_get_config()->sample_label);
    printf("\tHMAC Key: %s\n", ei_config_get_config()->sample_hmac_key);
    char filename[128];
    FILE *file = sampler->start(&payload, (const char*)ei_config_get_config()->sample_label, filename);
    if (!file) {
        return false;
    }
    printf("\tFile name: %s\n", filename);

    bool r = ei_microphone_record(ei_config_get_config()->sample_length_ms, 2000, true);
    if (!r) {
        return r;
    }

    // print_memory_info();

    // finalize and upload the file
    finish_and_upload(file, filename, ei_config_get_config()->sample_length_ms);

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
    signal.total_length = TEMP_FILE_IX / sizeof(int16_t);
    signal.get_data = &raw_audio_signal_get_data;
    return signal;
}

#endif // _EI_MICROPHONE_H_
