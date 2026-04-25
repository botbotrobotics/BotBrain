#!/bin/bash

IP="$1"

PROFILE="/home/booster/Desktop/BotBrain/botbrain_ws/fastdds_profile.xml"

if [ -z "$IP" ]; then
    echo "Usage: $0 <ip-address>"
    echo "  Example: $0 192.168.10.200"
    exit 1
fi

if ! echo "$IP" | grep -qE '^([0-9]{1,3}\.){3}[0-9]{1,3}$'; then
    echo "Error: '$IP' is not a valid IP address."
    exit 1
fi

# --- whitelist ---

if grep -q "<address>${IP}</address>" "$PROFILE"; then
    echo "  [SKIP] $IP already in whitelist"
else
    TAB=$'\t'
    sed -i "/<\/interfaceWhiteList>/i\\    ${TAB}${TAB}${TAB}<address>${IP}<\/address>" "$PROFILE"
    echo "  [OK]   Added $IP to whitelist"
fi

# --- .bashrc ---

add_export() {
    local line="$1"
    if grep -qF "$line" ~/.bashrc; then
        echo "  [SKIP] Already in ~/.bashrc: $line"
    else
        echo "$line" >> ~/.bashrc
        echo "  [OK]   Added to ~/.bashrc: $line"
    fi
}

add_export "export FASTRTPS_DEFAULT_PROFILES_FILE=${PROFILE}"
add_export "export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS"

echo ""
echo "Current whitelist ($PROFILE):"
grep '<address>' "$PROFILE" | sed 's/^/    /'
