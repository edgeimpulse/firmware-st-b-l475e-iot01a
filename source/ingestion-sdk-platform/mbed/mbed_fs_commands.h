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

#ifndef _EDGE_IMPULSE_AT_COMMANDS_FS_H_
#define _EDGE_IMPULSE_AT_COMMANDS_FS_H_

#define MBED_FS_EDGE_STRINGIZE_(x) #x
#define MBED_FS_EDGE_STRINGIZE(x) MBED_FS_EDGE_STRINGIZE_(x)

#include "mbed.h"
#include "ei_config.h"
#include "at_cmd_interface.h"
#include "http_request.h"

#ifndef EI_FS_PREFIX
#define EI_FS_PREFIX "/fs/"
#endif

#ifdef EI_SENSOR_AQ_BLOCKDEVICE
#include "SlicingBlockDevice.h"
extern SlicingBlockDevice temp_bd;
#endif

/**
 * Command context bindings with Mbed OS file system
 */
static int ei_mbed_fs_load_config(ei_config_t *config) {
    FILE *f = fopen(EI_FS_PREFIX "config", "r+");
    if (!f) {
        return 0;
    }

    fread(config, sizeof(ei_config_t), 1, f);
    fclose(f);
    return 0;
}

static int ei_mbed_fs_save_config(const ei_config_t *config) {
    FILE *f = fopen(EI_FS_PREFIX "config", "w+");
    if (!f) {
        return -1;
    }

    int write_result = fwrite(config, sizeof(ei_config_t), 1, f);
    fclose(f);
    if (write_result != 1) {
        return -2;
    }
    else {
        return 0;
    }
}

/**
 * List the files in file system (full path)
 */
#ifdef __MBED__
static void ei_mbed_fs_list_files(Callback<void(char*)> data_fn) {
#else
static void ei_mbed_fs_list_files(void(*data_fn)(char*)) {
#endif
    DIR *d;
    struct dirent *dir;
    d = opendir("/fs");
    if (d) {
        while ((dir = readdir(d)) != NULL) {
            if (strcmp(dir->d_name, ".") == 0) continue;
            if (strcmp(dir->d_name, "..") == 0) continue;
            if (strcmp(dir->d_name, "config") == 0) continue;

            char n[256] = { 0 };
            if (snprintf(n, 256, EI_FS_PREFIX "%s", dir->d_name) < 0) {
                data_fn(n);
            }
            data_fn(n);
        }
        closedir(d);
    }
}

/**
 * Read a single file from file system
 */
#ifdef __MBED__
static bool ei_mbed_fs_read_file(const char *path, Callback<void(uint8_t*, size_t)> data_fn) {
#else
static bool ei_mbed_fs_read_file(const char *path, void(*data_fn)(uint8_t*, size_t)) {
#endif
    FILE *file = fopen(path, "r+");
    if (!file) {
        return false;
    }

    // we're encoding as base64 in AT+READFILE, so this needs to be divisable by 3
    uint8_t buffer[513];
    size_t read;
    while ((read = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        data_fn(buffer, read);
    }
    return true;
}

/**
 * Read from the temp buffer
 */
#ifdef __MBED__
static bool ei_mbed_fs_read_buffer(size_t begin, size_t length, Callback<void(uint8_t*, size_t)> data_fn) {
#else
static bool ei_mbed_fs_read_buffer(size_t begin, size_t length, void(*data_fn)(uint8_t*, size_t)) {
#endif

    size_t pos = begin;
    size_t bytes_left = length;

    // we're encoding as base64 in AT+READFILE, so this needs to be divisable by 3
    uint8_t buffer[513];
    while (1) {
        size_t bytes_to_read = sizeof(buffer);
        if (bytes_to_read > bytes_left) {
            bytes_to_read = bytes_left;
        }
        if (bytes_to_read == 0) {
            return true;
        }

        int r = temp_bd.read(buffer, pos, bytes_to_read);
        if (r != 0) {
            return false;
        }
        data_fn(buffer, bytes_to_read);

        pos += bytes_to_read;
        bytes_left -= bytes_to_read;
    }

    return true;
}

/**
 * Unlink a single file from file system
 */
static bool ei_mbed_fs_unlink_file(const char *path) {
    return remove(path) == 0;
}

static NetworkInterface *ei_mbed_fs_network = NULL;
static FILE *ei_mbed_fs_upload_file_handler = NULL;
static const size_t mbed_fs_upload_file_buffer_size = 8 * 1024;
static uint8_t *ei_mbed_fs_upload_file_buffer;

const void * ei_mbed_fs_upload_file_get_chunk(uint32_t* out_size) {
    size_t size = fread(ei_mbed_fs_upload_file_buffer, 1, mbed_fs_upload_file_buffer_size, ei_mbed_fs_upload_file_handler);

    *out_size = size;

    return (const void*)ei_mbed_fs_upload_file_buffer;
}

bool ei_mbed_fs_upload_file(FILE *file, const char *filename) {
    if (strcmp(ei_config_get_config()->upload_host, "") == 0) {
        printf("ERR: Not uploading file, upload_host not set\n");
        return false;
    }
    if (strcmp(ei_config_get_config()->upload_path, "") == 0) {
        printf("ERR: Not uploading file, upload_path not set\n");
        return false;
    }

    if (!ei_mbed_fs_network) {
        printf("ERR: Not uploading file, no network interface\n");
        return false;
    }

    fseek(file, 0, SEEK_SET);
    ei_mbed_fs_upload_file_handler = file;

    char url[257] = { 0 }; // max length of each of these fields is 128...
    const char *host = ei_config_get_config()->upload_host;
    const char *path = ei_config_get_config()->upload_path;
    if (strlen(host) + strlen(path) > 256) {
        printf("ERR: Host and path are longer than 256 characters, potential overflow!\n");
        return false;
    }
    memcpy(url, host, strlen(host));
    memcpy(url + strlen(host), path, strlen(path));

    ei_mbed_fs_upload_file_buffer = (uint8_t*)malloc(mbed_fs_upload_file_buffer_size);
    if (!ei_mbed_fs_upload_file_buffer) {
        printf("ERR: Could not allocate upload file buffer\n");
        return false;
    }

    HttpRequest* req = new HttpRequest(ei_mbed_fs_network, HTTP_POST, (const char*)url);
    // all our files are mounted under /fs/, so skip the first 4 bytes
    req->set_header("Content-Type", "application/cbor");
    req->set_header("X-File-Name", filename + 4);
    req->set_header("X-API-Key", ei_config_get_config()->upload_api_key);

    HttpResponse* res = req->send(&ei_mbed_fs_upload_file_get_chunk);

    free(ei_mbed_fs_upload_file_buffer);

    bool err = false;

    if (res == NULL) {
        printf("ERR: Failed to make request, error %d\n", req->get_error());
        err = true;
    }
    else if (res->get_status_code() != 200) {
        printf("ERR: Failed to upload, response code was not 200, but %d\n", res->get_status_code());
        printf("%s\n", res->get_body_as_string().c_str());
        err = true;
    }
    else {
        if (remove(filename) != 0) {
            printf("ERR: Failed to unlink '%s'\n", filename);
            err = true;
        }
    }

    // printf("Response: %d - %s\n", res->get_status_code(), res->get_status_message().c_str());

    // printf("Headers:\n");
    // for (size_t ix = 0; ix < res->get_headers_length(); ix++) {
    //     printf("\t%s: %s\n", res->get_headers_fields()[ix]->c_str(), res->get_headers_values()[ix]->c_str());
    // }
    // printf("\nBody (%lu bytes):\n\n%s\n", res->get_body_length(), res->get_body_as_string().c_str());

    delete req;

    return !err;
}

static BlockDevice *ei_mbed_fs_upload_bd_handler = NULL;
static const size_t mbed_fs_upload_bd_buffer_size = 8 * 1024;
static size_t mbed_fs_upload_bd_current_pos = 0;
static size_t mbed_fs_upload_bd_total_size = 0;
static uint8_t *ei_mbed_fs_upload_bd_buffer;

const void * ei_mbed_fs_upload_bd_get_chunk(uint32_t* out_size) {
    size_t bytes_to_read = mbed_fs_upload_bd_buffer_size;
    if (mbed_fs_upload_bd_total_size - mbed_fs_upload_bd_current_pos < bytes_to_read) {
        bytes_to_read = mbed_fs_upload_bd_total_size - mbed_fs_upload_bd_current_pos;
    }

    if (bytes_to_read == 0) {
        *out_size = 0;
        return (const void*)ei_mbed_fs_upload_bd_buffer;
    }

    int r = ei_mbed_fs_upload_bd_handler->read(ei_mbed_fs_upload_bd_buffer, mbed_fs_upload_bd_current_pos, bytes_to_read);
    if (r != 0) {
        printf("ERR: Failed to read %lu bytes from block device (%d)\n", bytes_to_read, r);
        *out_size = 0;
    }
    else {
        mbed_fs_upload_bd_current_pos += bytes_to_read;
        *out_size = bytes_to_read;
    }

    return (const void*)ei_mbed_fs_upload_bd_buffer;
}

bool ei_mbed_fs_upload_from_blockdevice(BlockDevice *bd, size_t bd_start, size_t bd_end, const char *filename) {
    if (strcmp(ei_config_get_config()->upload_host, "") == 0) {
        printf("ERR: Not uploading file, upload_host not set\n");
        return false;
    }
    if (strcmp(ei_config_get_config()->upload_path, "") == 0) {
        printf("ERR: Not uploading file, upload_path not set\n");
        return false;
    }

    if (!ei_mbed_fs_network) {
        printf("ERR: Not uploading file, no network interface\n");
        return false;
    }

    mbed_fs_upload_bd_current_pos = bd_start;
    mbed_fs_upload_bd_total_size = bd_end;
    ei_mbed_fs_upload_bd_handler = bd;

    char url[257] = { 0 }; // max length of each of these fields is 128...
    const char *host = ei_config_get_config()->upload_host;
    const char *path = ei_config_get_config()->upload_path;
    if (strlen(host) + strlen(path) > 256) {
        printf("ERR: Host and path are longer than 256 characters, potential overflow!\n");
        return false;
    }
    memcpy(url, host, strlen(host));
    memcpy(url + strlen(host), path, strlen(path));

    ei_mbed_fs_upload_bd_buffer = (uint8_t*)malloc(mbed_fs_upload_bd_buffer_size);
    if (!ei_mbed_fs_upload_bd_buffer) {
        printf("ERR: Could not allocate upload file buffer\n");
        return false;
    }

    char *label;
    float interval;
    uint32_t length;
    char *hmac_key;

    EI_CONFIG_ERROR r = ei_config_get_sample_settings(&label, &interval, &length, &hmac_key);
    if (r != EI_CONFIG_OK) {
        printf("ERR: Failed to retrieve sample settings (%d)\n", r);
        return false;
    }

    HttpRequest* req = new HttpRequest(ei_mbed_fs_network, HTTP_POST, (const char*)url);
    req->set_header("Content-Type", "application/octet-stream");
    // all our files are mounted under /fs/, so skip the first 4 bytes
    req->set_header("X-File-Name", filename + 4);
    req->set_header("X-API-Key", ei_config_get_config()->upload_api_key);
    req->set_header("X-Label", label);

    char content_len_buf[10];
    int cont_r = snprintf(content_len_buf, 10, "%lu", bd_end - bd_start);
    if (cont_r <= 0) {
        printf("ERR: Failed to print content length (%d)\n", cont_r);
        return false;
    }

    HttpResponse* res = req->send(&ei_mbed_fs_upload_bd_get_chunk);

    free(ei_mbed_fs_upload_bd_buffer);

    bool err = false;

    if (res == NULL) {
        printf("ERR: Failed to make request, error %d\n", req->get_error());
        err = true;
    }
    else if (res->get_status_code() != 200) {
        printf("ERR: Failed to upload, response code was not 200, but %d\n", res->get_status_code());
        printf("%s\n", res->get_body_as_string().c_str());
        err = true;
    }

    // printf("Response: %d - %s\n", res->get_status_code(), res->get_status_message().c_str());

    // printf("Headers:\n");
    // for (size_t ix = 0; ix < res->get_headers_length(); ix++) {
    //     printf("\t%s: %s\n", res->get_headers_fields()[ix]->c_str(), res->get_headers_values()[ix]->c_str());
    // }
    // printf("\nBody (%lu bytes):\n\n%s\n", res->get_body_length(), res->get_body_as_string().c_str());

    delete req;

    return !err;
}

int ei_mbed_fs_get_device_type(uint8_t out_buffer[32], size_t *out_size) {
    const char *type = MBED_FS_EDGE_STRINGIZE(TARGET_NAME);
    memcpy(out_buffer, type, strlen(type));
    *out_size = strlen(type);
    return 0;
}

/**
 * Initialize the file system commands
 * @param cfg, pass a config context in, will set:
 *       load_config, save_config, list_files and read_file functions to the right values...
 */
int ei_mbed_fs_init(ei_config_ctx_t *cfg, NetworkInterface *network) {
    ei_mbed_fs_network = network;

    cfg->load_config = &ei_mbed_fs_load_config;
    cfg->save_config = &ei_mbed_fs_save_config;
    cfg->list_files = &ei_mbed_fs_list_files;
    cfg->read_file = &ei_mbed_fs_read_file;
    cfg->read_buffer = &ei_mbed_fs_read_buffer;
    cfg->unlink_file = &ei_mbed_fs_unlink_file;
    cfg->upload_file = &ei_mbed_fs_upload_file;
    cfg->upload_from_blockdevice = &ei_mbed_fs_upload_from_blockdevice;
    cfg->get_device_type = &ei_mbed_fs_get_device_type;

    return 0;
}

#endif // _EDGE_IMPULSE_AT_COMMANDS_FS_H_
