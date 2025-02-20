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

#ifndef _EDGE_IMPULSE_WS_CLIENT_H_
#define _EDGE_IMPULSE_WS_CLIENT_H_

#include "ei_config.h"
#include "ws_client.h"
#include "qcbor.h"
#include "at_cmd_repl_mbed.h"

class EdgeWsClient {
public:
    EdgeWsClient(EventQueue *queue, NetworkInterface *network, ei_config_ctx_t *context, ei_config_t *config, AtCmdRepl *repl)
        : _queue(queue),
          _network(network),
          _context(context),
          _config(config),
          _client(_queue, _network, _config->mgmt_url),
          _repl(repl)
    {
        _ws_callbacks.rx_callback = callback(this, &EdgeWsClient::rx_callback);
        _ws_callbacks.disconnect_callback = callback(this, &EdgeWsClient::disconnect_callback);
        _is_connected = false;
        _cbor_buf.len = 512;
        _cbor_buf.ptr = _cbor_buf_raw;
        memset(_last_ws_error, 0, 128);
    }

    ~EdgeWsClient() {

    }

    char *get_last_error() {
        return _last_ws_error;
    }

    int connect() {
        printf("ws connect\n");
        _is_connected = false;

        if (strcmp(_config->mgmt_url, "") == 0) {
            return -3301;
        }

        if (strcmp(_config->upload_host, "") == 0) {
            return -3301;
        }

        // Cache the mgmt URL and remote mgmt URL in DNS Cache before connecting
        // see https://github.com/edgeimpulse/edgeimpulse/issues/224
        // printf("Caching DNS entries...\n");

        SocketAddress address;
        nsapi_error_t dns_res;

        ParsedUrl parsed_mgmt_url(_config->mgmt_url);
        dns_res = _network->gethostbyname(parsed_mgmt_url.host(), &address);
        if (dns_res != NSAPI_ERROR_OK) {
            return dns_res;
        }

        ParsedUrl parsed_ingestion_url(_config->upload_host);
        dns_res = _network->gethostbyname(parsed_ingestion_url.host(), &address);
        if (dns_res != NSAPI_ERROR_OK) {
            return dns_res;
        }

        // printf("Caching DNS entries OK\n");

        _client.set_url(_config->mgmt_url);

        int rv = _client.connect(&_ws_callbacks);
        if (rv != MBED_WS_ERROR_OK) {
            disconnect_callback();
            return rv;
        }

        // authenticate...
        rv = send_hello();
        if (rv != MBED_WS_ERROR_OK) {
            printf("client.send returned %d\n", rv);
            disconnect_callback();
            return rv;
        }

        printf("ws connect ok\n");

        return MBED_WS_ERROR_OK;
    }

    bool is_connected() {
        return _is_connected;
    }

    int send_sample_started() {
        // authenticate...
        QCBOREncodeContext ec;

        QCBOREncode_Init(&ec, _cbor_buf);

        QCBOREncode_OpenMap(&ec);
        QCBOREncode_AddBoolToMap(&ec, "sampleStarted", true);
        QCBOREncode_CloseMap(&ec);

        UsefulBufC encoded;
        if(QCBOREncode_Finish(&ec, &encoded)) {
            return -3302;
        }

        return _client.send(WS_BINARY_FRAME, _cbor_buf_raw, encoded.len);
    }

    int send_sample_processing() {
        // authenticate...
        QCBOREncodeContext ec;

        QCBOREncode_Init(&ec, _cbor_buf);

        QCBOREncode_OpenMap(&ec);
        QCBOREncode_AddBoolToMap(&ec, "sampleProcessing", true);
        QCBOREncode_CloseMap(&ec);

        UsefulBufC encoded;
        if(QCBOREncode_Finish(&ec, &encoded)) {
            return -3302;
        }

        return _client.send(WS_BINARY_FRAME, _cbor_buf_raw, encoded.len);
    }

    int send_sample_uploading() {
        // authenticate...
        QCBOREncodeContext ec;

        QCBOREncode_Init(&ec, _cbor_buf);

        QCBOREncode_OpenMap(&ec);
        QCBOREncode_AddBoolToMap(&ec, "sampleUploading", true);
        QCBOREncode_CloseMap(&ec);

        UsefulBufC encoded;
        if(QCBOREncode_Finish(&ec, &encoded)) {
            return -3302;
        }

        return _client.send(WS_BINARY_FRAME, _cbor_buf_raw, encoded.len);
    }

private:
    int send_hello() {
        // authenticate...
        QCBOREncodeContext ec;

        QCBOREncode_Init(&ec, _cbor_buf);

        QCBOREncode_OpenMap(&ec);
        QCBOREncode_OpenMapInMap(&ec, "hello");

        uint8_t id_out_buffer[32] = { 0 };
        size_t id_out_size;
        _context->get_device_id(id_out_buffer, &id_out_size);

        uint8_t type_out_buffer[32] = { 0 };
        size_t type_out_size;
        _context->get_device_type(type_out_buffer, &type_out_size);

        QCBOREncode_AddInt64ToMap(&ec, "version", 2);
        UsefulBufC api_key_b = { _config->upload_api_key, strlen(_config->upload_api_key) };
        QCBOREncode_AddTextToMap(&ec, "apiKey", api_key_b);
        UsefulBufC id_b = { id_out_buffer, id_out_size };
        QCBOREncode_AddTextToMap(&ec, "deviceId", id_b);
        UsefulBufC type_b = { type_out_buffer, type_out_size };
        QCBOREncode_AddTextToMap(&ec, "deviceType", type_b);

        if (_context->get_sensor_list != NULL) {
            const ei_sensor_t *list;
            size_t list_size;

            int r = _context->get_sensor_list(&list, &list_size);
            if (r != 0) {
                printf("ERR: Failed to get sensor list from context (%d)\n", r);
            }
            else {
                QCBOREncode_OpenArrayInMap(&ec, "sensors");
                for (size_t ix = 0; ix < list_size; ix++) {
                    QCBOREncode_OpenMap(&ec); // map for this sensor
                    UsefulBufC name_b = { list[ix].name, strlen(list[ix].name) };
                    QCBOREncode_AddTextToMap(&ec, "name", name_b);
                    QCBOREncode_AddInt64ToMap(&ec, "maxSampleLengthS", list[ix].max_sample_length_s);
                    QCBOREncode_OpenArrayInMap(&ec, "frequencies");
                    for (size_t fx = 0; fx < EDGE_IMPULSE_MAX_FREQUENCIES; fx++) {
                        if (list[ix].frequencies[fx] == 0.0f) continue;
                        QCBOREncode_AddDouble(&ec, list[ix].frequencies[fx]);
                    }
                    QCBOREncode_CloseArray(&ec); // frequencies
                    QCBOREncode_CloseMap(&ec); // map for this sensor
                }
                QCBOREncode_CloseArray(&ec); // sensor array
            }
        }

        QCBOREncode_CloseMap(&ec); // hello map
        QCBOREncode_CloseMap(&ec); // main object map

        UsefulBufC encoded;
        if(QCBOREncode_Finish(&ec, &encoded)) {
            return -3302;
        }

        return _client.send(WS_BINARY_FRAME, _cbor_buf_raw, encoded.len);
    }

    int send_sample_start_received() {
        // authenticate...
        QCBOREncodeContext ec;

        QCBOREncode_Init(&ec, _cbor_buf);

        QCBOREncode_OpenMap(&ec);
        QCBOREncode_AddBoolToMap(&ec, "sample", true);
        QCBOREncode_CloseMap(&ec);

        UsefulBufC encoded;
        if(QCBOREncode_Finish(&ec, &encoded)) {
            return -3302;
        }

        return _client.send(WS_BINARY_FRAME, _cbor_buf_raw, encoded.len);
    }

    int send_sample_start_error(char *label) {
        // authenticate...
        QCBOREncodeContext ec;

        QCBOREncode_Init(&ec, _cbor_buf);

        QCBOREncode_OpenMap(&ec);
        QCBOREncode_AddBoolToMap(&ec, "sample", false);
        UsefulBufC err_key = { label, strlen(label) };
        QCBOREncode_AddTextToMap(&ec, "error", err_key);
        QCBOREncode_CloseMap(&ec);

        memcpy(_last_ws_error, label, strlen(label) + 1);

        UsefulBufC encoded;
        if(QCBOREncode_Finish(&ec, &encoded)) {
            return -3302;
        }

        return _client.send(WS_BINARY_FRAME, _cbor_buf_raw, encoded.len);
    }

    int send_sample_finished() {
        // authenticate...
        QCBOREncodeContext ec;

        QCBOREncode_Init(&ec, _cbor_buf);

        QCBOREncode_OpenMap(&ec);
        QCBOREncode_AddBoolToMap(&ec, "sampleFinished", true);
        QCBOREncode_CloseMap(&ec);

        UsefulBufC encoded;
        if(QCBOREncode_Finish(&ec, &encoded)) {
            return -3302;
        }

        return _client.send(WS_BINARY_FRAME, _cbor_buf_raw, encoded.len);
    }

    void rx_callback(WS_OPCODE opcode, uint8_t* buffer, size_t buffer_size) {
        if (opcode != WS_BINARY_FRAME) return;

        QCBORDecodeContext ctx;
        QCBORItem item;

        QCBORDecode_Init(&ctx, (UsefulBufC){ buffer, buffer_size }, QCBOR_DECODE_MODE_NORMAL);

        // first one needs to be a map...
        if (QCBORDecode_GetNext(&ctx, &item) != QCBOR_SUCCESS || item.uDataType != QCBOR_TYPE_MAP) {
            printf("[MGMT] err expected map\n");
            for (size_t ix = 0; ix < buffer_size; ix++) {
                printf("%02x ", buffer[ix]);
            }
            printf("\n");
            return;
        }

        // then we expect labels and handle them
        while (QCBORDecode_GetNext(&ctx, &item) == QCBOR_SUCCESS && item.uLabelType == QCBOR_TYPE_TEXT_STRING) {
            char label_buf[128] = { 0 };
            int rv;

            memcpy(label_buf, item.label.string.ptr, item.label.string.len > 127 ? 127 : item.label.string.len);

            if (strcmp(label_buf, "hello") == 0) {
                if (item.uDataType == QCBOR_TYPE_TRUE) {
                    _is_connected = true;
                }
                else {
                }
            }
            else if (strcmp(label_buf, "err") == 0) {
                if (item.uDataType != QCBOR_TYPE_TEXT_STRING) {
                    printf("[MGMT] Unexpected type for 'err'\n");
                    continue;
                }
                memset(_last_ws_error, 0, 128);
                memcpy(_last_ws_error, item.val.string.ptr, item.val.string.len > 127 ? 127 : item.val.string.len);
            }
            else if (strcmp(label_buf, "sample") == 0) {
                if (item.uDataType != QCBOR_TYPE_MAP) {
                    printf("[MGMT] Unexpected type for 'sample'\n");
                    continue;
                }

                char path_buffer[128] = { 0 };
                char label_buffer[64] = { 0 };
                uint32_t length = 0;
                float interval = 0;
                char hmac_key_buffer[64] = { 0 };
                char sensor_buffer[64] = { 0 };
                uint8_t fields_found = 0;

                while (QCBORDecode_GetNext(&ctx, &item) == QCBOR_SUCCESS && item.uLabelType == QCBOR_TYPE_TEXT_STRING) {
                    memset(label_buf, 0, sizeof(label_buf));
                    memcpy(label_buf, item.label.string.ptr, item.label.string.len > 127 ? 127 : item.label.string.len);

                    if (strcmp(label_buf, "path") == 0 && item.uDataType == QCBOR_TYPE_TEXT_STRING) {
                        fields_found++;
                        memcpy(path_buffer, item.val.string.ptr, item.val.string.len >= sizeof(path_buffer) ? sizeof(path_buffer) : item.val.string.len);
                    }
                    else if (strcmp(label_buf, "label") == 0 && item.uDataType == QCBOR_TYPE_TEXT_STRING) {
                        fields_found++;
                        memcpy(label_buffer, item.val.string.ptr, item.val.string.len >= sizeof(label_buffer) ? sizeof(label_buffer) : item.val.string.len);
                    }
                    else if (strcmp(label_buf, "hmacKey") == 0 && item.uDataType == QCBOR_TYPE_TEXT_STRING) {
                        fields_found++;
                        memcpy(hmac_key_buffer, item.val.string.ptr, item.val.string.len >= sizeof(hmac_key_buffer) ? sizeof(hmac_key_buffer) : item.val.string.len);
                    }
                    else if (strcmp(label_buf, "interval") == 0 && item.uDataType == QCBOR_TYPE_INT64) {
                        fields_found++;
                        interval = (float)item.val.int64;
                    }
                    else if (strcmp(label_buf, "interval") == 0 && item.uDataType == QCBOR_TYPE_DOUBLE) {
                        fields_found++;
                        interval = item.val.dfnum;
                    }
                    else if (strcmp(label_buf, "length") == 0 && item.uDataType == QCBOR_TYPE_INT64) {
                        fields_found++;
                        length = (uint32_t)item.val.int64;
                    }
                    else if (strcmp(label_buf, "sensor") == 0 && item.uDataType == QCBOR_TYPE_TEXT_STRING) {
                        fields_found++;
                        memcpy(sensor_buffer, item.val.string.ptr, item.val.string.len >= sizeof(sensor_buffer) ? sizeof(sensor_buffer) : item.val.string.len);
                    }
                    else {
                        printf("[MGMT] Unknown field for sample... %s (%d)\n", label_buf, item.uDataType);
                        rv = send_sample_start_error(label_buf);
                        if (rv != MBED_WS_ERROR_OK) {
                            printf("Failed to send sample error message (%d)\n", rv);
                        }
                        return;
                    }
                }

                EI_CONFIG_ERROR r;

                r = ei_config_set_sample_settings(label_buffer, interval, length, hmac_key_buffer);
                if (r != EI_CONFIG_OK) {
                    printf("Failed to persist sample settings (%d)\n", r);
                }

                r = ei_config_set_upload_path_settings(path_buffer);
                if (r != EI_CONFIG_OK) {
                    printf("Failed to persist upload settings (%d)\n", r);
                }

                rv = send_sample_start_received();
                if (rv != MBED_WS_ERROR_OK) {
                    printf("Failed to send sample start received message (%d)\n", r);
                }

                _client.pause_disconnect_checker();

                printf("[MGMT] start sampling request, fields_found=%u, path_buffer=%s, label=%s, hmac=%s, iv=%.2f, len=%lu, sensor=%s\n",
                    fields_found, path_buffer, label_buffer, hmac_key_buffer, interval, length, sensor_buffer);

                char repl_command[128] = { 0 };
                snprintf(repl_command, 128, "AT+SAMPLESTART=%s", sensor_buffer);
                _repl->exec_command(repl_command);

                rv = send_sample_finished();
                if (rv != MBED_WS_ERROR_OK) {
                    printf("Failed to send sample finished message (%d)\n", rv);
                }

                _client.resume_disconnect_checker();
            }
            else {
                printf("[MGMT] Unknown field... %s (%d)\n", label_buf, item.uDataType);
            }
        }

        QCBORDecode_Finish(&ctx);

        _repl->reprintReplState();

        if (!_is_connected) {
            _client.disconnect();

            disconnect_callback();
        }
    }

    void disconnect_callback() {
        printf("ws disconnect_callback\n");
        _repl->reprintReplState();
        _is_connected = false;

        _queue->call_in(5000, callback(this, &EdgeWsClient::connect));
    }

    EventQueue *_queue;
    NetworkInterface *_network;
    ei_config_ctx_t *_context;
    ei_config_t *_config;
    WsClient _client;
    AtCmdRepl *_repl;
    // AtCmdRepl *_repl;
    ws_callbacks_t _ws_callbacks;
    bool _is_connected;
    unsigned char _cbor_buf_raw[512];
    UsefulBuf _cbor_buf;
    char _last_ws_error[128];
};

#endif // _EDGE_IMPULSE_WS_CLIENT_H_
