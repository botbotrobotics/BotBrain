#!/bin/bash

IP="$1"

PROFILE_SHM="/opt/booster/BoosterRos2/fastdds_profile.xml"
PROFILE_UDP="/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml"

if [ -z "$IP" ]; then
    echo "Usage: $0 <ip-address>"
    echo "  Example: $0 192.168.10.200"
    exit 1
fi

# Validate IP format
if ! echo "$IP" | grep -qE '^([0-9]{1,3}\.){3}[0-9]{1,3}$'; then
    echo "Error: '$IP' is not a valid IP address."
    exit 1
fi

add_to_whitelist() {
    local file="$1"

    if [ ! -f "$file" ]; then
        echo "  [SKIP] File not found: $file"
        return 1
    fi

    if grep -q "<address>${IP}</address>" "$file"; then
        echo "  [SKIP] $IP already in whitelist: $file"
        return 0
    fi

    # Insert new <address> line before </interfaceWhiteList>
    TAB=$'\t'
    sudo sed -i "/<\/interfaceWhiteList>/i\\${TAB}${TAB}<address>${IP}<\/address>" "$file"
    echo "  [OK]   Added $IP to: $file"
}

echo "Adding $IP to FastDDS whitelists..."
add_to_whitelist "$PROFILE_SHM"
add_to_whitelist "$PROFILE_UDP"
echo ""
echo "Current whitelists:"
for f in "$PROFILE_SHM" "$PROFILE_UDP"; do
    echo ""
    echo "  $f"
    grep '<address>' "$f" | sed 's/^/    /'
done
