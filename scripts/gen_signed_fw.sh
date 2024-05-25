#!/bin/bash
#generate the  fw signature
openssl dgst -sha256 -sign private_key.pem -out firmware.sig ./Application/Debug/Application.bin

#concatenate signature with firmware.
cat ./Application/Debug/Application.bin firmware.sig > signed_firmware.bin
ech "Done."
