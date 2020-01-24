const cbor = require('cbor');
const fs = require('fs');
const crypto = require('crypto');
const Path = require('path');

// same as in main.cpp
const hmac_key = "fed53116f20684c067774ebf9e7bcbdc";

// empty signature (all zeros). HS256 gives 32 byte signature, and we encode in hex, so we need 64 characters here
let emptySignature = Array(64).fill('0').join('');

let data = {
    protected: {
        ver: "v1",
        alg: "HS256",
        iat: Math.floor(Date.now() / 1000) // epoch time, seconds since 1970
    },
    signature: emptySignature,
    payload: {
        device_name: "ac:87:a3:0a:2d:1b",
        device_type: "DISCO-L475VG-IOT01A",
        interval_ms: 10,
        sensors: [
            { name: "accX", units: "m/s2" },
            { name: "accY", units: "m/s2" },
            { name: "accZ", units: "m/s2" }
        ],
        values: [
            [ -9.81, 0.03, 1.21 ],
            [ -9.83, 0.04, 1.27 ],
            [ -9.12, 0.03, 1.23 ],
            [ -9.14, 0.01, 1.25 ]
        ]
    }
};

let encoded = cbor.encode(data);

// now calculate the HMAC and fill in the signature
let hmac = crypto.createHmac('sha256', hmac_key);
hmac.update(encoded);
let signature = hmac.digest().toString('hex');

// now find the empty signature in our encoded buffer
let sigIx = encoded.indexOf(emptySignature);
if (sigIx === -1) {
    throw 'Could not find empty signature in encoded CBOR object';
}

// replace the empty signature in the buffer, we don't re-encode the object
// as this might lead to canonicalization errors (although the chance here is small)
encoded = Buffer.concat([
    encoded.slice(0, sigIx),
    Buffer.from(signature, 'ascii'),
    encoded.slice(sigIx + signature.length)
]);

// representation of the buffer with spaces between bytes
console.log([...encoded].map(b => {
    let v = b.toString(16);
    return v.length === 1 ? '0' + v : v;
}).join(' '));

fs.writeFileSync(Path.join(__dirname, 'encoded.cbor'), encoded);
