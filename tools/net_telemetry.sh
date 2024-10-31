#!/bin/bash

# specify network interface
INTERFACE="enp2s0"  # connection to switch

# log file

LOG_FILE="network-telemetry.log"
echo "Timestamp, Outgoing Rate (Mbps), TX Packet Loss" > "$LOG_FILE"

prev_tx_bytes=$(cat /sys/class/net/$INTERFACE/statistics/tx_bytes)
prev_tx_packets=$(cat /sys/class/net/$INTERFACE/statistics/tx_packets)
prev_tx_errors=$(cat /sys/class/net/$INTERFACE/statistics/tx_errors)

while true; do
    # polling rate
    sleep 1

    curr_tx_bytes=$(cat /sys/class/net/$INTERFACE/statistics/tx_bytes)
    curr_tx_packets=$(cat /sys/class/net/$INTERFACE/statistics/tx_packets)
    curr_tx_errors=$(cat /sys/class/net/$INTERFACE/statistics/tx_errors)
    tx_rate_bps=$(( (curr_tx_bytes - prev_tx_bytes) * 8 ))  # Convert bytes to bits
    tx_rate_mbps=$(echo "scale=2; $tx_rate_bps / 1000000" | bc)  # Convert to Mbps
    packet_loss=$((curr_tx_errors - prev_tx_errors))
    prev_tx_bytes=$curr_tx_bytes
    prev_tx_packets=$curr_tx_packets
    prev_tx_errors=$curr_tx_errors
    TIMESTAMP=$(date +"%Y-%m-%d %H:%M:%S")
    echo "$TIMESTAMP, $tx_rate_mbps, $packet_loss" >> "$LOG_FILE"
done