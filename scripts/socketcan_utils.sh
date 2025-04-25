#!/bin/bash
#
choose_action() {
  local action=$(whiptail --title "SocketCAN" --menu "Choose action" 25 78 16 \
  "Enable" "" \
  "Check" "" 3>&1 1>&2 2>&3)
  echo "$action"
}


choose_interface() {
  local interface=$(whiptail --separate-output --title "Interface" --radiolist "Choose interface" 20 78 2 \
  "can0" "" ON \
  "can1" "" OFF 3>&1 1>&2 2>&3)
  echo "$interface"
}


choose_bitrate() {
  local -A bitrate
  bitrate["125 kbit/s"]="125000"
  bitrate["250 kbit/s"]="250000"
  bitrate["500 kbit/s"]="500000"
  bitrate["1 Mbit/s"]="1000000"

  local bitrate_key=$(whiptail --separate-output --title "Bitrate" --radiolist "Choose bitrate" 20 78 4 \
  "125 kbit/s" "" ON \
  "250 kbit/s" "" OFF \
  "500 kbit/s" "" OFF \
  "1 Mbit/s" "" OFF 3>&1 1>&2 2>&3)

  if [ -z "$bitrate_key" ]; then
    echo
  else
    echo "${bitrate["$bitrate_key"]}"
  fi
}


socketcan_enable() {
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
}


socketcan_check() {
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
}


action="$(choose_action)"


if [ -z "$action" ]; then
  echo "No action was chosen."
  exit 1
elif test "$action" = "Enable"; then
  interface="$(choose_interface)"
  if [ -z "$interface" ] ; then
    exit 1
  fi
  bitrate="$(choose_bitrate)"
  echo "$bitrate"
  if [ -z "$bitrate" ]; then
    exit 1
  fi
  socketcan_enable "$interface" "$bitrate"
elif test "$action" = "Check"; then
  interface="$(choose_interface)"
  socketcan_check "$interface"
fi
