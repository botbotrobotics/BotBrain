#!/bin/bash

# === ROS Environment Variables ===
unset ROS_DOMAIN_ID
unset RMW_IMPLEMENTATION
unset CYCLONEDDS_URI

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#export ROS_DOMAIN_ID=42
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Export CYCLONEDDS_URI pointing to the XML file in the same directory
export CYCLONEDDS_URI="file://${SCRIPT_DIR}/../../cyclonedds_config.xml"
# === Network Configuration Check ===
if ! command -v nmcli &> /dev/null
then
    echo "Info: nmcli command not found. Skipping network configuration."
    # The script continues to run without exiting
else
    # === Network Configuration Variables ===
    # IFACE=$(nmcli -t -f DEVICE,TYPE device status | awk -F: '$2=="ethernet" {print $1; exit}')
    IFACE="eno1"
    echo "Ethernet interface: $IFACE"
    IP="192.168.123.174"
    NETMASK="255.255.255.0"
    GATEWAY="192.168.123.1"

    # === Step 1a: Find the active connection name for the specified interface ===
    echo "Searching for the active connection on interface $IFACE..."
    CONNECTION_NAME=$(nmcli -t -f DEVICE,NAME connection show --active | grep "^$IFACE" | cut -d':' -f2)

    if [ -z "$CONNECTION_NAME" ]; then
        echo "Error: No active connection found on interface $IFACE. Please ensure the interface is connected and has a profile."
        # The script will just finish here, without an error code
    else
        echo "Found active connection: '$CONNECTION_NAME'"

        # === Step 2a: Modify the connection's IPv4 settings ===
        echo "Modifying connection '$CONNECTION_NAME' with new IP: $IP..."

        # Set the IPv4 method to manual (static IP)
        sudo nmcli connection modify "$CONNECTION_NAME" ipv4.method manual

        # Set the new IP address and netmask
        sudo nmcli connection modify "$CONNECTION_NAME" ipv4.addresses "$IP/16"

        # Set the new gateway
        sudo nmcli connection modify "$CONNECTION_NAME" ipv4.gateway "$GATEWAY"

        # === Step 3a: Reapply the connection to make changes take effect ===
        echo "Reapplying connection '$CONNECTION_NAME' to activate the new settings."
        sudo nmcli connection up "$CONNECTION_NAME"

        echo "Interface $IFACE has been reconfigured successfully."
    fi
fi