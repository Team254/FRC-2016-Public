#!/bin/sh

/usr/local/opt/mosquitto/sbin/mosquitto -c ${BASH_SOURCE%/*}/mosquitto.conf
