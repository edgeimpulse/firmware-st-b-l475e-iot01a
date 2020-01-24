#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "qcbor.h"
#include "sensor_aq.h"
#include "sensor_aq_mbedtls_hs256.h"

int main() {
    // The sensor format supports signing the data, set up a signing context
    sensor_aq_signing_ctx_t signing_ctx;

    // We'll use HMAC SHA256 signatures, which can be created through Mbed TLS
    // If you use a different crypto library you can implement your own context
    sensor_aq_mbedtls_hs256_ctx_t hs_ctx;
    // Set up the context, the last argument is the HMAC key
    sensor_aq_init_mbedtls_hs256_context(&signing_ctx, &hs_ctx, "fed53116f20684c067774ebf9e7bcbdc");

    // Set up the sensor acquisition basic context
    sensor_aq_ctx ctx = {
        // We need a single buffer. The library does not require any dynamic allocation (but your TLS library might)
        { (unsigned char*)malloc(1024), 1024 },

        // Pass in the signing context
        &signing_ctx,

        // And pointers to fwrite and fseek - note that these are pluggable so you can work with them on
        // non-POSIX systems too. Just override the EI_SENSOR_AQ_STREAM macro to your custom file type.
        &fwrite,
        &fseek,
        // if you set the time function this will add 'iat' (issued at) field to the header with the current time
        // if you don't include it, this will be omitted
        &time
    };

    // Payload header
    sensor_aq_payload_info payload = {
        // Unique device ID (optional), set this to e.g. MAC address or device EUI **if** your device has one
        "ac:87:a3:0a:2d:1b",
        // Device type (required), use the same device type for similar devices
        "DISCO-L475VG-IOT01A",
        // How often new data is sampled in ms. (100Hz = every 10 ms.)
        10,
        // The axes which you'll use. The units field needs to comply to SenML units (see https://www.iana.org/assignments/senml/senml.xhtml)
        { { "accX", "m/s2" }, { "accY", "m/s2" }, { "accZ", "m/s2" } }
    };

    // Place to write our data.
    // The library streams data, and does not cache everything in buffers
    FILE *file = fopen("test/encoded.cbor", "w+");

    // Initialize the context, this verifies that all requirements are present
    // it also writes the initial CBOR structure
    int res;
    res = sensor_aq_init(&ctx, &payload, file);
    if (res != AQ_OK) {
        printf("sensor_aq_init failed (%d)\n", res);
        return 1;
    }

    // Periodically call `sensor_aq_add_data` (every 10 ms. in this example) to append data
    float values[][3] = {
        { -9.81, 0.03, 1.21 },
        { -9.83, 0.04, 1.28 },
        { -9.12, 0.03, 1.23 },
        { -9.14, 0.01, 1.25 }
    };
    for (size_t ix = 0; ix < sizeof(values) / sizeof(values[0]); ix++) {
        res = sensor_aq_add_data(&ctx, values[ix], 3);
        if (res != AQ_OK) {
            printf("sensor_aq_add_data failed (%d)\n", res);
            return 1;
        }
    }

    // When you're done call sensor_aq_finish - this will calculate the finalized signature and close the CBOR file
    res = sensor_aq_finish(&ctx);
    if (res != AQ_OK) {
        printf("sensor_aq_finish failed (%d)\n", res);
        return 1;
    }

    // For convenience we'll write the encoded file. You can throw this directly in http://cbor.me to decode
    printf("Encoded file:\n");

    // Print the content of the file here:
    fseek(file, 0, SEEK_END);
    size_t len = ftell(file);
    uint8_t *buffer = (uint8_t*)malloc(len);

    fseek(file, 0, SEEK_SET);
    fread(buffer, len, 1, file);

    for (size_t ix = 0; ix < len; ix++) {
        printf("%02x ", buffer[ix]);
    }
    printf("\n");

    // Verify if the file can be decoded:
    //
    // $ npm install cbor
    // $ node cbor.js
    //
    // If you changed the HMAC key make sure to update it in cbor.js too
}
