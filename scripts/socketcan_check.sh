#!/bin/bash
#
if [ -z $1 ]; then
  echo "Failed: invalid arguments."
  exit 1
fi

if ip addr ls dev $1 | grep "state UP" > /dev/null 2>&1; then
  bitrate=$(ip -det link show can0 | awk '/bitrate/ {print $2}')
  echo "Interface $1 is UP (bitrate = "$bitrate")."
  exit 0
else
  echo "Interface $1 is DOWN."
  exit 2
fi
