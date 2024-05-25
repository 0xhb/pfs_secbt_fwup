#!/bin/bash

# Generate a 256-bit ECDSA private key using the NIST P-256 curve
echo "Generating private key..."
openssl ecparam -genkey -name prime256v1 -noout -out private_key.pem
echo " "

# Extract the public key from the private key
echo "Generating public key..."
openssl ec -in private_key.pem -pubout -out public_key.pem

echo " "
echo "Converting the public key to a C header file"
# Convert the public key to DER format
openssl ec -in private_key.pem -pubout -outform DER -out public_key.der

#create header file
xxd -i public_key.der public_key.h

echo " "
# Convert the DER format to a C array
xxd -i public_key.der public_key.c
