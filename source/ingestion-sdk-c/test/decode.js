const cbor = require('cbor');
const fs = require('fs');
const crypto = require('crypto');
const Path = require('path');

// same as in main.cpp
const hmac_key = "fed53116f20684c067774ebf9e7bcbdc";

// read the file we just generated
let buffer = fs.readFileSync(Path.join(__dirname, 'encoded.cbor'));

// decode it
let decoded = cbor.decode(buffer);
console.log('Full object', decoded);

// verify the protected header. The C library can also output 'none' signatures but this is not recommended.
if (!decoded.protected) {
    throw 'Missing protected';
}
if (decoded.protected.alg !== 'HS256') {
    throw 'Only "HS256" supported at the moment';
}
if (decoded.protected.ver !== 'v1') {
    throw 'Invalid version, expecting "v1"';
}

// so to verify that this is correctly signed, we need to find where the signature is in the original stream...
let sig_ix = buffer.lastIndexOf(decoded.signature);
let sig_length = buffer[sig_ix - 1];

if (sig_length !== 64) { // hex encoded 32 bytes
    throw 'Signature length is not 64, but ' + sig_length + ' (note: indefinite length strings are not supported)';
}

// so we set the signature to "000000" (etc.) in ascii
for (let ix = sig_ix; ix < sig_ix + sig_length; ix++) {
    buffer[ix] = '0'.charCodeAt(0); // set to ascii 0
}

// then calculate the HMAC...
let hmac = crypto.createHmac('sha256', hmac_key);
hmac.update(buffer);
let signature = hmac.digest();

if (signature.toString('hex') !== decoded.signature) {
    throw 'Signature verification failed (calculated: ' + signature.toString('hex') + ', but received: ' + decoded.signature + ')';
}

console.log('Signature verification OK');

console.log('Payload', decoded.payload);
