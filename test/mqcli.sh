#!/bin/bash

mosquitto_sub -h iothub -t 'gatectl/v1/gatectl-5200/response' &
subpid=$!

while read -e line
do
  [ "$line" == "exit" ] && break
  mosquitto_pub -h iothub -t 'gatectl/v1/gatectl-5200/cmd' -m "$line"
done

echo done, killing subscriber, $subpid
kill $subpid
wait

