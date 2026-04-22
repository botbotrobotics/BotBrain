#!/bin/bash
# If the K1 need any wifi setup, you can uses this .bash

# === Read network interface from robot_config.yaml ===
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_CONFIG_PATH="$SCRIPT_DIR/../../robot_config.yaml"

if [ ! -f "$ROBOT_CONFIG_PATH" ]; then
    echo "Error: robot_config.yaml not found at $ROBOT_CONFIG_PATH"
    return 1 2>/dev/null || exit 1
fi

NETWORK_IFACE=$(python3 -c "import yaml; config = yaml.safe_load(open('$ROBOT_CONFIG_PATH')); print(config['robot_configuration']['network_interface'])" 2>/dev/null)
if [ -z "$NETWORK_IFACE" ]; then
    echo "Error: Could not read network_interface from robot_config.yaml"
    return 1 2>/dev/null || exit 1
fi

echo "Network interface from config: $NETWORK_IFACE"

# === Network Configuration Check ===
if ! command -v nmcli &> /dev/null; then
    echo "Info: nmcli command not found. Skipping network configuration."
else
    IFACE="$NETWORK_IFACE"
    echo "Ethernet interface: $IFACE"
    IP="192.168.127.101"
    NETMASK="255.255.0.0"
    GATEWAY="192.168.127.1"

    echo "Searching for the active connection on interface $IFACE..."
    CONNECTION_NAME=$(nmcli -t -f DEVICE,NAME connection show --active | grep "^$IFACE" | cut -d':' -f2)

    if [ -z "$CONNECTION_NAME" ]; then
        echo "Error: No active connection found on interface $IFACE."
    else
        echo "Found active connection: '$CONNECTION_NAME'"
        echo "Modifying connection '$CONNECTION_NAME' with new IP: $IP..."

        sudo nmcli connection modify "$CONNECTION_NAME" ipv4.method manual
        sudo nmcli connection modify "$CONNECTION_NAME" ipv4.addresses "$IP/16"
        sudo nmcli connection modify "$CONNECTION_NAME" ipv4.gateway "$GATEWAY"

        echo "Reapplying connection '$CONNECTION_NAME' to activate the new settings."
        sudo nmcli connection up "$CONNECTION_NAME"

        echo "Interface $IFACE has been reconfigured successfully."
    fi
fi
