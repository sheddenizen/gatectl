#!/bin/bash

mosquitto_sub -h iothub -t 'gatectl/v1/gatectl-5200/response' &

