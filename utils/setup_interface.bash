#!/bin/bash

IFACE="$1"
IP="$2"

if [ -z "$IFACE" ] || [ -z "$IP" ]; then
    echo "Usage: $0 <interface> <static-ip>"
    echo "  Example: $0 wlP1p1s0 192.168.10.200"
    exit 1
fi

# Validate IP format (accepts x.x.x.x or x.x.x.x/prefix)
IP_ADDR="${IP%%/*}"
PREFIX="${IP##*/}"
[ "$PREFIX" = "$IP" ] && PREFIX="24"

if ! echo "$IP_ADDR" | grep -qE '^([0-9]{1,3}\.){3}[0-9]{1,3}$'; then
    echo "Error: '$IP_ADDR' is not a valid IP address."
    exit 1
fi

# Check interface exists
if ! ip link show "$IFACE" &>/dev/null; then
    echo "Error: interface '$IFACE' not found."
    ip link show | grep -E '^[0-9]+:' | awk -F': ' '{print "  " $2}'
    exit 1
fi

GATEWAY="${IP_ADDR%.*}.1"

# Find the active nmcli connection on this interface
CONN=$(nmcli -t -f NAME,DEVICE connection show --active | grep ":${IFACE}$" | cut -d':' -f1)

if [ -z "$CONN" ]; then
    # No active connection — create a new one
    CONN="static-${IFACE}"
    echo "No active connection on $IFACE. Creating '$CONN'..."
    sudo nmcli connection add \
        type ethernet \
        ifname "$IFACE" \
        con-name "$CONN" \
        ipv4.method manual \
        ipv4.addresses "${IP_ADDR}/${PREFIX}" \
        ipv4.gateway "$GATEWAY" \
        ipv6.method ignore
else
    echo "Configuring existing connection '$CONN' on $IFACE..."
    sudo nmcli connection modify "$CONN" \
        ipv4.method manual \
        ipv4.addresses "${IP_ADDR}/${PREFIX}" \
        ipv4.gateway "$GATEWAY" \
        ipv6.method ignore
fi

sudo nmcli connection up "$CONN"

echo ""
echo "Interface : $IFACE"
echo "IP        : ${IP_ADDR}/${PREFIX}"
echo "Gateway   : $GATEWAY"
echo "Connection: $CONN"
echo ""
ip addr show "$IFACE" | grep 'inet '
