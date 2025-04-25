#!/bin/bash
#
if [ -z $1 ] || [ -z $2 ]; then
  echo "Failed: invalid arguments."
  exit 1
fi

if ip addr ls dev $1 > /dev/null 2>&1; then
  echo "Found device $1."
  if ip link set $1 down; then
    ip link set $1 type can bitrate $2 restart-ms 100
    ip link set $1 up
    echo "Enabled SocketCAN interface $1 (bitrate = $2)."
    exit 0
  else
    echo "Failed to configure and enable SocketCAN interface $1."
    exit 3
  fi
else
  echo "Failed to found device $1."
  exit 2
fi
