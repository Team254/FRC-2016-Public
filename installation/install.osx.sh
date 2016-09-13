#!/bin/bash

# Run this file from your mac to install the nessisary software

if [ -z $1 ]
then
    echo "Usage: $0 <roborio-hostname>"
    exit -1;
fi;

ssh admin@$1 "rm -r ~/*"

FILES_TO_COPY="\
./install_mosquitto.roborio.sh \
./libwebsockets.so \
./libzlib.so \
./mosquitto \
./mosquitto.init.sh \
./mosquitto.roborio.conf \
./RIOdroid.roborio.sh \
./RIOdroid.tar.gz"

scp $FILES_TO_COPY admin@$1:~/
scp -r logger admin@$1:~/logger

ssh admin@$1 "./install_mosquitto.roborio.sh && ./RIOdroid.roborio.sh"
