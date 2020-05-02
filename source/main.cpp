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
#include "mbed_mem_trace.h"
#include "LittleFileSystem.h"
#include "model-parameters/model_metadata.h"
#include "ei_run_classifier.h"
#include "NTPClient.h"
#include "at_cmd_repl_mbed.h"
#include "mbed_fs_commands.h"
#include "ei_sampler.h"
#include "sensor_aq.h"
#include "ei_ws_client.h"
#include "model_metadata.h"
#include "ei_max30102.h"
#include "ei_accelerometer.h"
#include "ei_microphone.h"

// set up two slices on block device, one for the file system, and one for temporary files
static BlockDevice *full_bd = BlockDevice::get_default_instance();
static SlicingBlockDevice fs_bd(full_bd, 0x0, 6 * 1024 * 1024);
SlicingBlockDevice temp_bd(full_bd, 6 * 1024 * 1024, 8 * 1024 * 1024);

static LittleFileSystem fs("fs", &fs_bd, 256, 512, 512);
EdgeWsClient *ws_client;
EventQueue main_application_queue;
EdgeSampler *sampler;
DigitalOut led(LED1);

static unsigned char repl_stack[8 * 1024];
static AtCmdRepl repl(&main_application_queue, sizeof(repl_stack), repl_stack);

static WiFiInterface *network = NULL;
static bool network_connected = false;

// this lists all the sensors that this device has (initialized later on, when we know whether
// sensors are present on the board)
static ei_sensor_t sensor_list[3] = { 0 };

static bool max30102_present;
static bool microphone_present;

void print_memory_info() {
    // Grab the heap statistics
    mbed_stats_heap_t heap_stats;
    mbed_stats_heap_get(&heap_stats);
    printf("Heap size: %lu / %lu bytes (max: %lu)\r\n", heap_stats.current_size, heap_stats.reserved_size, heap_stats.max_size);
}

bool connect_to_wifi() {
    network_connected = false;

    nsapi_error_t err;
    if (network != NULL) {
        err = network->disconnect();
        if (err != NSAPI_ERROR_OK) {
            printf("Disconnecting from existing network failed (%d) - continuing\n", err);
        }
        network = NULL;
    }

    network = WiFiInterface::get_default_instance();
    if (!network) {
        printf("Could not get network handle\n");
        return false;
    }

    char *ssid;
    char *password;
    ei_config_security_t security;
    bool connected;

    if (ei_config_get_wifi(&ssid, &password, &security, &connected) != EI_CONFIG_OK) {
        printf("Failed to get WiFi credentials from config\n");
        return false;
    }

    printf("Connecting to '%s'\n", ssid);
    err = network->connect(ssid, password, (nsapi_security)security);
    if (err != NSAPI_ERROR_OK) {
        printf("Connecting to WiFi network failed (%d)\n", err);
        return false;
    }

    printf("Connected to WiFi, will now automatically upload samples\n");

    printf("Getting NTP info...\n");
    NTPClient ntp(network);
    time_t curr_time = ntp.get_timestamp();
    printf("Current time is %s", ctime(&curr_time));

    if (curr_time > 100) {
        set_time(static_cast<int64_t>(curr_time));
        sampler->set_has_time(true);
    }
    else {
        sampler->set_has_time(false);
    }

    printf("Connecting to WS Client returned %d\n", ws_client->connect());

    network_connected = true;

    return true;
}

#if defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_ACCELEROMETER

void run_nn(bool debug) {
    // summary of inferencing settings (from model_metadata.h)
    printf("Inferencing settings:\n");
    printf("\tInterval: %.2f ms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
    printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    printf("\tSample length: %.0f ms.\n",
        1000.0f * static_cast<float>(EI_CLASSIFIER_RAW_SAMPLE_COUNT) / (1000.0f / static_cast<float>(EI_CLASSIFIER_INTERVAL_MS)));
    printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

    printf("Starting inferencing, press 'b' to break\n");

    while (1) {
        // also store the data, for this we need the payload too, if you don't want to persist data, you can get rid of this
        // and pass NULL instead of sampler to `run_impulse`
        sensor_aq_payload_info payload = {
            WiFiInterface::get_default_instance()->get_mac_address(),
            EDGE_STRINGIZE(TARGET_NAME),
            EI_CLASSIFIER_INTERVAL_MS,    // <-- don't use the config here, use the value we get from model_metadata !
            { { "accX", "m/s2" }, { "accY", "m/s2" }, { "accZ", "m/s2" } }
        };
        char filename[128] = { 0 }; // out filename
        FILE *file = sampler->start(&payload, "device-classification", filename);
        printf("Sampling... Storing in file name: %s\n", filename);

        // run the impulse: DSP, neural network and the Anomaly algorithm
        ei_impulse_result_t result = { 0 };
        EI_IMPULSE_ERROR err = run_impulse(sampler, &result, &ei_accelerometer_sample_read_data, debug);
        if (err != EI_IMPULSE_OK) {
            printf("Failed to run impulse (%d)\n", err);
            break;
        }

        // print the predictions
        printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
        }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
        printf("    anomaly score: %.3f\n", result.anomaly);
#endif

        sampler->finish();
        fclose(file);
        printf("Finished inferencing, raw data is stored in '%s'. Use AT+UPLOADFILE to send back to Edge Impulse.\n",
            filename);

        printf("Starting inferencing in 2 seconds...\n");

        // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
        if (ei_sleep(2000) != EI_IMPULSE_OK) {
            break;
        }
    }
}

#elif defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_MICROPHONE

void run_nn(bool debug) {
    // summary of inferencing settings (from model_metadata.h)
    printf("Inferencing settings:\n");
    printf("\tInterval: %.2f ms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
    printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

    printf("Starting inferencing, press 'b' to break\n");

    while (1) {
        printf("Starting inferencing in 2 seconds...\n");

        // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
        if (ei_sleep(2000) != EI_IMPULSE_OK) {
            break;
        }

        printf("Recording\n");

        bool m = ei_microphone_record(EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16, 20, false);
        if (!m) {
            printf("ERR: Failed to record audio...\n");
            break;
        }

        printf("Recording OK\n");

        signal_t signal = ei_microphone_get_signal();
        ei_impulse_result_t result = { 0 };

        EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug);
        if (r != EI_IMPULSE_OK) {
            printf("ERR: Failed to run classifier (%d)\n", r);
            break;
        }

        // print the predictions
        printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
        }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
        printf("    anomaly score: %.3f\n", result.anomaly);
#endif
    }
}
#else

#error "EI_CLASSIFIER_SENSOR not configured, cannot configure `run_nn`"

#endif // EI_CLASSIFIER_SENSOR

void run_nn_normal() {
    run_nn(false);
}

void run_nn_debug() {
    run_nn(true);
}

int get_device_id(uint8_t out_buffer[32], size_t *out_size) {
    if (!WiFiInterface::get_default_instance()) {
        *out_size = 0;
        return -1;
    }

    const char *mac = WiFiInterface::get_default_instance()->get_mac_address();
    memcpy(out_buffer, mac, strlen(mac));
    *out_size = strlen(mac);
    return 0;
}

bool wifi_connection_status() {
    return network_connected;
}

bool wifi_scan(Callback<void(const char*, ei_config_security_t, int8_t)> data_fn) {
    if (!WiFiInterface::get_default_instance()) {
        return false;
    }

    WiFiAccessPoint res[20];
    nsapi_size_or_error_t sz = WiFiInterface::get_default_instance()->scan(res, 20);
    if (sz < 0) {
        printf("scan failed (%d)\n", sz);
    }
    else {
        for (int ix = 0; ix < sz; ix++) {
            data_fn(res[ix].get_ssid(), (ei_config_security_t)res[ix].get_security() /*compatible struct as Mbed */, res[ix].get_rssi());
        }
    }
    return true;
}

bool ws_client_is_connected() {
    if (!ws_client) return false;
    return ws_client->is_connected();
}

bool ws_client_last_error(char *buff, size_t buff_size) {
    if (!ws_client) return false;

    char *err = ws_client->get_last_error();
    if (strlen(err) > buff_size - 1) {
        memcpy(buff, err, buff_size - 1);
        buff[buff_size] = 0;
    }
    else {
        memcpy(buff, err, strlen(err));
    }
    return strcmp(err, "") != 0;
}

int get_sensor_list(const ei_sensor_t **list, size_t *list_size) {
    size_t sensor_count = 1;

    int ix = 0;

    sensor_list[ix].name = "Built-in accelerometer";
    sensor_list[ix].start_sampling_cb = &ei_accelerometer_sample_start;
    sensor_list[ix].max_sample_length_s = 5 * 60;
    sensor_list[ix].frequencies[0] = 62.5f;
    sensor_list[ix].frequencies[1] = 100.0f;
    ix++;
    if (microphone_present) {
        sensor_list[ix].name = "Built-in microphone";
        sensor_list[ix].start_sampling_cb = &ei_microphone_sample_start;
        sensor_list[ix].max_sample_length_s = 1 * 60;
        sensor_list[ix].frequencies[0] = 16000.0f;
        ix++;
        sensor_count++;
    }
    if (max30102_present) {
        sensor_list[ix].name = "MAX30102 HR / oximeter";
        sensor_list[ix].start_sampling_cb = &ei_max30102_sample_start;
        sensor_list[ix].max_sample_length_s = 5 * 60;
        sensor_list[ix].frequencies[0] = 100.0f;
        ix++;
        sensor_count++;
    }

    *list = sensor_list;
    *list_size = sensor_count;
    return 0;
}

void fill_memory() {
    size_t size = 8 * 1024;
    size_t allocated = 0;
    while (1) {
        void *ptr = malloc(size);
        if (!ptr) {
            if (size == 1) break;
            size /= 2;
        }
        else {
            allocated += size;
        }
    }
    printf("Allocated: %lu bytes\n", allocated);
}
int main() {
    // mbed_mem_trace_set_callback(mbed_mem_trace_default_callback);

    printf("\n\nHello from the Edge Impulse Device SDK.\n");
    printf("Compiled on %s at %s\n",__DATE__,__TIME__);
    int err = fs.mount(&fs_bd);
    printf("Mounting file system: %s\n", (err ? "Fail :(" : "OK"));
    if (err) {
        // Reformat if we can't mount the filesystem
        // this should only happen on the first boot
        printf("No filesystem found, formatting... ");
        fflush(stdout);
        err = fs.reformat(&fs_bd);
        printf("%s\n", (err ? "Fail :(" : "OK"));
        if (err) {
            error("error: %s (%d)\n", strerror(-err), err);
        }
        err = fs.mount(&fs_bd);
        if (err) {
            printf("Failed to mount file system (%d)\n", err);
            return 1;
        }
    }

    err = temp_bd.init();
    if (err != 0) {
        printf("Failed to initialize temp blockdevice\n");
        return 1;
    }

    // Sensor init
    ei_accelerometer_init();
    max30102_present = ei_max30102_init();
    microphone_present = ei_microphone_init();

    // intialize configuration
    ei_config_ctx_t config_ctx = { 0 };
    ei_mbed_fs_init(&config_ctx, WiFiInterface::get_default_instance());

    config_ctx.get_device_id = get_device_id;
    config_ctx.wifi_connection_status = wifi_connection_status;
    config_ctx.wifi_settings_changed = connect_to_wifi;
    config_ctx.scan_wifi = wifi_scan;
    config_ctx.mgmt_is_connected = ws_client_is_connected;
    config_ctx.mgmt_get_last_error = ws_client_last_error;
    config_ctx.get_sensor_list = get_sensor_list;

    EI_CONFIG_ERROR cr = ei_config_init(&config_ctx);

    if (cr != EI_CONFIG_OK) {
        printf("Failed to initialize configuration (%d)\n", cr);
    }
    else {
        printf("Loaded configuration\n");
    }

    ws_client = new EdgeWsClient(
        &main_application_queue,
        WiFiInterface::get_default_instance(),
        ei_config_get_context(),
        ei_config_get_config(),
        &repl);
    sampler = new EdgeSampler(ei_config_get_context(), ei_config_get_config(), ws_client);

    ei_at_register_generic_cmds();
    ei_at_cmd_register("FILLMEMORY", "Try and fill the full RAM, to report free heap stats", fill_memory);
    ei_at_cmd_register("RUNIMPULSE", "Run the impulse", run_nn_normal);
    ei_at_cmd_register("RUNIMPULSEDEBUG", "Run the impulse with debug messages", run_nn_debug);

    if (ei_config_has_wifi()) {
        connect_to_wifi();
    }

    printf("Type AT+HELP to see a list of commands.\n");

    // print_memory_info();

    repl.start_repl();

    main_application_queue.dispatch_forever();
}
