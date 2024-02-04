#!/bin/bash

mosquitto_sub -W1 -h iothub -t 'gatectl/v1/gatectl-5200/response' &

mosquitto_pub -h iothub -t 'gatectl/v1/gatectl-5200/cmd' -m "$*"

wait

