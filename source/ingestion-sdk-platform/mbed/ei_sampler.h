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

#ifndef _EDGE_IMPULSE_AT_COMMANDS_SAMPLER_H_
#define _EDGE_IMPULSE_AT_COMMANDS_SAMPLER_H_

#define EDGE_STRINGIZE_(x) #x
#define EDGE_STRINGIZE(x) EDGE_STRINGIZE_(x)

#include "sensor_aq.h"
#include "sensor_aq_mbedtls_hs256.h"
#include "ei_config.h"
#include "mbed_fs_commands.h"
#include "ei_ws_client.h"
#include <map>
#include <string>
#include <string.h>

#define EDGE_SAMPLER_BUFFER_SIZE            512

/**
 * Sample data at a high frequency and then write them to disk
 */
class EdgeSampler {
public:
    EdgeSampler(ei_config_ctx_t *context, ei_config_t *config, EdgeWsClient *ws_client)
        : _context(context),
          _config(config),
          _ws_client(ws_client),
          _has_time(false),
          _sample_thread(NULL)
    {
        sensor_aq_init_mbedtls_hs256_context(&_signing_ctx, &_hs_ctx, _config->sample_hmac_key);

        _aq_ctx.buffer = { _sample_buffer, EDGE_SAMPLER_BUFFER_SIZE };
        _aq_ctx.signature_ctx = &_signing_ctx;
        _aq_ctx.fwrite = &fwrite;
        _aq_ctx.fseek = &fseek;
        _aq_ctx.time = NULL;

        // read all current files to determine what the next file should be...
        _context->list_files(callback(this, &EdgeSampler::list_files_cb));
    }

    /**
     * Call this after NTP sync is completed
     */
    void set_has_time(bool value) {
        _has_time = value;
        if (value) {
            _aq_ctx.time = &time;
        }
        else {
            _aq_ctx.time = NULL;
        }
    }

    /**
     * Start full sampling cycle
     * @param payload Header information
     * @param start_delay How long we need to wait before sampling
     * @param data_fn Function to retrieve data from sensors
     */
    void start_sampling(
            sensor_aq_payload_info *payload,
            int start_delay,
            Callback<void(float*, size_t)> data_fn,
            DigitalOut *statusLed = NULL)
    {
        // it's possible that we get terminated by the REPL
        // so clean up after ourselves
        if (_sample_thread != NULL) {
            _sample_thread->terminate();
            delete _sample_thread;
        }

        printf("Sampling settings:\n");
        printf("\tInterval: %.2f ms.\n", (float)_config->sample_interval_ms);
        printf("\tLength: %lu ms.\n", _config->sample_length_ms);
        printf("\tName: %s\n", _config->sample_label);
        printf("\tHMAC Key: %s\n", _config->sample_hmac_key);

        char filename[128];
        FILE *file = start(payload, (const char*)_config->sample_label, filename);
        if (!file) {
            return;
        }

        printf("\tFile name: %s\n", filename);

        _sample_thread = new Thread(osPriorityLow, OS_STACK_SIZE, nullptr, "sample-thread");
        _sample_thread->start(callback(&_sample_queue, &EventQueue::dispatch_forever));

        size_t sensors_count = 0;

        for (int s = 0; s < EI_MAX_SENSOR_AXES; s++) {
            if (payload->sensors[s].name != 0x0) {
                sensors_count++;
            }
            else {
                break;
            }
        }

        printf("Starting in %d seconds...\n", start_delay);

        ThisThread::sleep_for(start_delay);

        if (_ws_client->is_connected()) {
            _ws_client->send_sample_started();
        }

        printf("Sampling...\n");

        if (statusLed) {
            statusLed->write(1);
        }

        Timer timer;

        uint64_t total_sample_count = 0;
        uint64_t samples_timing_missed = 0;

        int64_t break_at = timer.read_us() + (_config->sample_length_ms * 1000);

        timer.start();

        while (1) {
            int64_t next_tick = timer.read_us() + static_cast<int64_t>(_config->sample_interval_ms * 1000);

            // so... we allocate a new buffer here... and it will be cleared after we write to
            // sensor_aq_add_data
            // this is done because file system write is not guaranteed to be less than sampling freq.
            float *values = (float*)malloc(sizeof(float) * _aq_ctx.axis_count);
            data_fn(values, _aq_ctx.axis_count);

            _sample_queue.call(callback(this, &EdgeSampler::write_sensor_data_internal), &_aq_ctx, values, sensors_count);

            total_sample_count++;

            if (timer.read_us() > next_tick) {
                samples_timing_missed++;
            }

            // so... now we have this issue that we need to be able to yield back to the sample thread
            // so we cannot busy-loop here, but I still want accurate timing

            // let's sleep for (wait_time / 1000) - 1 ms. then busy loop from there
            uint64_t wait_time = next_tick - timer.read_us();

            // sleep OK (/1000 already floors it)
            ThisThread::sleep_for((wait_time / 1000) - 1);

            // busy loop til next tick
            while (next_tick > timer.read_us());

            if (timer.read_us() > break_at) break;
        }

        if (statusLed) {
            statusLed->write(0);
        }

        // give the queue some time to empty just in case (better yet: is there something on EventQueue to check this?)
        ThisThread::sleep_for(100);

        // todo: should check if queue is empty!!!
        _sample_thread->terminate();
        delete _sample_thread;
        _sample_thread = NULL;

        // end it
        printf("Done sampling\n");
        printf("Total samples collected: %llu\n", total_sample_count);

        if (_ws_client->is_connected()) {
            _ws_client->send_sample_uploading();
        }

        finish();
        upload_file(file, filename);
        fclose(file);

        printf("OK\n");
    }

    /**
     * Initializes context and writes the header
     */
    FILE *start(sensor_aq_payload_info *payload, const char *label, char filename[128]) {
        if (!get_unused_filename(label, filename)) {
            printf("Failed to get an unused filename\n");
            return NULL;
        }

        FILE *file = fopen(filename, "w+");

        // also set the HMAC key on the signing context
        memcpy(_hs_ctx.hmac_key, _config->sample_hmac_key, 33);

        // Initialize the context, this verifies that all requirements are present
        // it also writes the initial CBOR structure
        int res;
        res = sensor_aq_init(&_aq_ctx, payload, file);
        if (res != AQ_OK) {
            printf("sensor_aq_init failed (%d)\n", res);
            return NULL;
        }

        return file;
    }

    /**
     * Writes sensor data to the context, does not clear the buffer passed in!
     */
    int write_sensor_data(float *values, size_t values_size) {
        return sensor_aq_add_data(&_aq_ctx, values, values_size);
    }

    /**
     * Writes sensor data to the context, does not clear the buffer passed in!
     */
    int write_sensor_data(int16_t *values, size_t values_size) {
        return sensor_aq_add_data_i16(&_aq_ctx, values, values_size);
    }

    /**
     * Writes batch sensor data to the context, does not clear the buffer passed in!
     */
    // int write_sensor_data_batch(float *values, size_t values_size) {
    //     return sensor_aq_add_data_batch(&_aq_ctx, values, values_size);
    // }

    /**
     * Writes batch sensor data to the context, does not clear the buffer passed in!
     */
    int write_sensor_data_batch(int16_t *values, size_t values_size) {
        return sensor_aq_add_data_batch(&_aq_ctx, values, values_size);
    }

    /**
     * Closes file
     */
    void finish() {
        // When you're done call sensor_aq_finish - this will calculate the finalized signature and close the CBOR file
        int res = sensor_aq_finish(&_aq_ctx);
        if (res != AQ_OK) {
            printf("sensor_aq_finish failed (%d)\n", res);
            return;
        }
    }

    /**
     * Upload a file (if there is a connection)
     */
    void upload_file(FILE *file, char filename[128]) {
        if (_context->upload_file) {
            if (_context->wifi_connection_status()) {
                printf("Uploading... '%s' to %s%s...\n", filename,
                    ei_config_get_config()->upload_host,
                    ei_config_get_config()->upload_path);
                _context->upload_file(file, (const char*)filename);
            }
            else {
                printf("Not uploading file, not connected to WiFi\n");
            }
        }
    }

    /**
     * Upload buffer from blockdevice
     */
    void upload_buffer(BlockDevice *bd, size_t bd_start, size_t bd_end, const char *filename) {
        if (_context->upload_file) {
            if (_context->wifi_connection_status()) {
                printf("Uploading... '%s' to %s%s...\n", filename,
                    ei_config_get_config()->upload_host,
                    ei_config_get_config()->upload_path);
                _context->upload_from_blockdevice(bd, bd_start, bd_end, (const char*)filename);
            }
            else {
                printf("Not uploading file, not connected to WiFi. Used buffer, from=%u, to=%u.\n",
                    bd_start, bd_end);
            }
        }
    }

private:

    void write_sensor_data_internal(sensor_aq_ctx *ctx, float *values, size_t values_size) {
        int r;
        if ((r = sensor_aq_add_data(ctx, values, values_size)) != AQ_OK) {
            printf("Writing sensor data failed (%d)\n", r);
        }

        free(values);
    }

    /**
     * Get an unused filename for the current label
     * @returns false if failed to get a filename
     */
    bool get_unused_filename(const char *label, char filename_buffer[128]) {
        std::map<std::string, int>::iterator it;

        std::string category(label);
        it = _classes.find(category);

        // new category?
        if (it == _classes.end()) {
            // @todo check boundaries
            // add category0 to the map
            _classes.insert(std::pair<std::string, int>(category, 0));

            if (snprintf(filename_buffer, 128, "%s%s.0", EI_FS_PREFIX, label) < 0) {
                return false;
            }

            return true;
        }

        it->second++;
        if (snprintf(filename_buffer, 128, "%s%s.%d", EI_FS_PREFIX, label, it->second) < 0) {
            return false;
        }
        return true;
    }

    /**
     * List files callback to determine what the next empty file will be...
     */
    void list_files_cb(char *curr) {
        curr = curr + strlen(EI_FS_PREFIX);

        int cx;
        for (cx = strlen(curr) - 1; cx >= 0; cx--) {
            if (curr[cx] < '0' || curr[cx] > '9') {
                break;
            }
        }
        cx += 1;

        if (static_cast<size_t>(cx) == strlen(curr)) return;

        std::string category(curr, cx);
        std::string number(curr + cx, strlen(curr) - cx);
        int n = atoi(number.c_str());

        std::map<std::string, int>::iterator it = _classes.find(category);

        if (it == _classes.end()) {
            _classes.insert(std::pair<std::string, int>(category, n));
        }
        else if (n > it->second) {
            it->second = n;
        }
    }

    ei_config_ctx_t *_context;
    ei_config_t *_config;
    EdgeWsClient *_ws_client;
    bool _has_time;

    uint8_t _sample_buffer[EDGE_SAMPLER_BUFFER_SIZE];
    Thread *_sample_thread;
    EventQueue _sample_queue;

    // The sensor format supports signing the data, set up a signing context
    sensor_aq_signing_ctx_t _signing_ctx;

    // We'll use HMAC SHA256 signatures, which can be created through Mbed TLS
    // If you use a different crypto library you can implement your own context
    sensor_aq_mbedtls_hs256_ctx_t _hs_ctx;

    sensor_aq_ctx _aq_ctx;

    // list of classes and their file names
    std::map<std::string, int> _classes;
};

#endif // _EDGE_IMPULSE_AT_COMMANDS_SAMPLER_H_
