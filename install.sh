#!/bin/bash
USER_HOME="$(getent passwd "${SUDO_USER:-$USER}" | cut -d: -f6)"
export PATH="$USER_HOME/.local/bin:$PATH"

# Install pip3 if not already installed
echo "=========================================="
echo "Checking for pip3..."
echo "=========================================="

if ! command -v pip3 &> /dev/null; then
    echo "pip3 is not installed. Installing python3-pip..."
    apt update
    apt install -y python3-pip

    if [ $? -eq 0 ]; then
        echo "✓ pip3 installed successfully!"
    else
        echo "✗ Warning: Failed to install pip3"
        echo "Please install it manually with: sudo apt install python3-pip"
    fi
else
    echo "✓ pip3 is already installed."
fi

echo ""

# Install jtop for Jetson Nano monitoring
echo "=========================================="
echo "Installing jtop (Jetson stats tool)..."
echo "=========================================="

if ! command -v jtop &> /dev/null && ! sudo -u "$SUDO_USER" command -v jtop &> /dev/null; then
    echo "jtop is not installed. Installing jetson-stats..."
    echo "Running: sudo -H pip3 install -U jetson-stats"

    sudo -H pip3 install -U jetson-stats
    INSTALL_RESULT=$?

    if [ $INSTALL_RESULT -eq 0 ]; then
        echo "✓ jetson-stats installed successfully!"
        echo "Note: You may need to reboot for jtop to work properly."
    else
        echo "✗ Warning: Failed to install jetson-stats (exit code: $INSTALL_RESULT)"
        echo "You can manually install it later with: sudo -H pip3 install -U jetson-stats"
    fi
else
    echo "✓ jtop is already installed."
fi

echo ""

# Check if dialog is installed, if not install it
if ! command -v dialog &> /dev/null; then
    echo "Dialog is not installed. Installing dialog..."

    # Detect the operating system and install dialog
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        OS=$ID
    elif [ "$(uname)" = "Darwin" ]; then
        OS="macos"
    else
        echo "Unable to detect operating system."
        exit 1
    fi

    case $OS in
        ubuntu|debian)
            sudo apt-get update
            sudo apt-get install -y dialog
            ;;
        fedora|rhel|centos)
            sudo dnf install -y dialog || sudo yum install -y dialog
            ;;
        arch|manjaro)
            sudo pacman -S --noconfirm dialog
            ;;
        macos)
            if command -v brew &> /dev/null; then
                brew install dialog
            else
                echo "Homebrew is not installed. Please install Homebrew first: https://brew.sh"
                exit 1
            fi
            ;;
        *)
            echo "Unsupported operating system: $OS"
            echo "Please install 'dialog' manually and run this script again."
            exit 1
            ;;
    esac

    # Verify installation
    if ! command -v dialog &> /dev/null; then
        echo "Failed to install dialog. Please install it manually."
        exit 1
    fi

    echo "Dialog installed successfully!"
fi

# Set purple color scheme for dialog with different tones
export DIALOGRC="/tmp/dialogrc_custom"
cat > "$DIALOGRC" << 'EOF'
# Dialog color configuration
use_colors = ON
screen_color = (WHITE,BLACK,OFF)
shadow_color = (BLACK,BLACK,ON)
dialog_color = (WHITE,MAGENTA,OFF)
title_color = (WHITE,MAGENTA,ON)
border_color = (WHITE,MAGENTA,OFF)
border2_color = (WHITE,MAGENTA,OFF)
button_active_color = (BLACK,WHITE,ON)
button_inactive_color = (WHITE,MAGENTA,OFF)
button_key_active_color = (BLACK,WHITE,ON)
button_key_inactive_color = (WHITE,MAGENTA,OFF)
button_label_active_color = (BLACK,WHITE,ON)
button_label_inactive_color = (WHITE,MAGENTA,OFF)
inputbox_color = (WHITE,MAGENTA,OFF)
inputbox_border_color = (WHITE,MAGENTA,OFF)
inputbox_border2_color = (WHITE,MAGENTA,OFF)
searchbox_color = (WHITE,MAGENTA,OFF)
searchbox_title_color = (WHITE,MAGENTA,ON)
searchbox_border_color = (WHITE,MAGENTA,OFF)
searchbox_border2_color = (WHITE,MAGENTA,OFF)
position_indicator_color = (WHITE,MAGENTA,ON)
menubox_color = (WHITE,MAGENTA,OFF)
menubox_border_color = (WHITE,MAGENTA,OFF)
menubox_border2_color = (WHITE,MAGENTA,OFF)
item_color = (WHITE,MAGENTA,OFF)
item_selected_color = (BLACK,WHITE,ON)
tag_color = (WHITE,MAGENTA,OFF)
tag_selected_color = (BLACK,WHITE,ON)
tag_key_color = (WHITE,MAGENTA,OFF)
tag_key_selected_color = (BLACK,WHITE,ON)
check_color = (WHITE,MAGENTA,OFF)
check_selected_color = (BLACK,WHITE,ON)
uarrow_color = (WHITE,MAGENTA,ON)
darrow_color = (WHITE,MAGENTA,ON)
EOF

# Initial welcome page
dialog --title "BotBrain Workspace Installer" \
       --msgbox "\nWelcome to the BotBrain Workspace Installation Wizard\n\nThis installer will guide you through the setup process for your BotBrain workspace environment.\n\nPress OK to continue or ESC to exit." 12 70

# Check if user cancelled
if [ $? -ne 0 ]; then
    clear
    echo "Installation cancelled by user."
    exit 1
fi

# Robot model selection page
ROBOT_MODEL=$(dialog --title "Robot Model Selection" \
                     --menu "\nSelect your robot model:" 15 70 4 \
                     "go2" "Unitree Go2" \
                     "g1" "Unitree G1" \
                     "tita" "Tita" \
                     "other" "Custom Robot" \
                     3>&1 1>&2 2>&3)

# Check if user cancelled
if [ $? -ne 0 ]; then
    clear
    echo "Installation cancelled by user."
    exit 1
fi

echo "Selected robot model: $ROBOT_MODEL"

# If user selected "other", prompt for custom model name
if [ "$ROBOT_MODEL" = "other" ]; then
    CUSTOM_MODEL=$(dialog --title "Custom Robot Model" \
                          --inputbox "\nEnter your robot model name:" 10 70 \
                          3>&1 1>&2 2>&3)

    if [ $? -ne 0 ]; then
        clear
        echo "Installation cancelled by user."
        exit 1
    fi

    ROBOT_MODEL=$CUSTOM_MODEL
    echo "Custom robot model: $ROBOT_MODEL"
fi

# If Tita robot is selected, prompt for namespace
if [ "$ROBOT_MODEL" = "tita" ]; then
    TITA_NAMESPACE=$(dialog --title "Tita Namespace Configuration" \
                            --inputbox "\nEnter the Tita namespace for your robot:\n\n(This is used for robot-specific communication)" 12 70 \
                            3>&1 1>&2 2>&3)

    if [ $? -ne 0 ]; then
        clear
        echo "Installation cancelled by user."
        exit 1
    fi

    echo "Tita namespace: $TITA_NAMESPACE"
fi

# Description file type selection page
DESCRIPTION_TYPE=$(dialog --title "Description File Type" \
                          --menu "\nSelect the robot description file format:" 15 70 2 \
                          "xacro" "Xacro (XML Macros for URDF)" \
                          "urdf" "URDF (Unified Robot Description Format)" \
                          3>&1 1>&2 2>&3)

# Check if user cancelled
if [ $? -ne 0 ]; then
    clear
    echo "Installation cancelled by user."
    exit 1
fi

echo "Selected description file type: $DESCRIPTION_TYPE"

# Robot name configuration page
dialog --title "Robot Name Configuration" \
       --yesno "\nWould you like to set a custom name for your robot?\n\nThis name will be used as the namespace for:\n- ROS2 Topics\n- Services\n- Actions\n- Nodes\n- And other robot-specific identifiers\n\nIf you skip this step, no custom namespace will be set." 15 70

if [ $? -eq 0 ]; then
    # User selected Yes
    ROBOT_NAME=$(dialog --title "Robot Name" \
                        --inputbox "\nEnter a name for your robot:\n\n(Use lowercase letters, numbers, and underscores only)" 12 70 \
                        3>&1 1>&2 2>&3)

    if [ $? -ne 0 ]; then
        clear
        echo "Installation cancelled by user."
        exit 1
    fi

    echo "Robot name: $ROBOT_NAME"
else
    # User selected No
    ROBOT_NAME=""
    echo "No robot name set."
fi

# Network interface selection page
# Get list of available network interfaces (works on both Linux and macOS)
if [ "$(uname)" = "Darwin" ]; then
    # macOS: ifconfig -l lists all interfaces in one line
    INTERFACES=$(ifconfig -l | tr ' ' '\n' | grep -v "lo0" | awk '{printf "%s \"%s\" ", $1, $1}')
else
    # Linux: parse ifconfig output
    INTERFACES=$(ifconfig -a | grep -E "^[a-z]" | awk '{print $1}' | sed 's/://g' | grep -v "lo" | awk '{printf "%s \"%s\" ", $1, $1}')
fi

if [ -z "$INTERFACES" ]; then
    # If no interfaces found, allow manual entry
    NETWORK_INTERFACE=$(dialog --title "Network Interface Configuration" \
                               --inputbox "\nNo network interfaces detected.\n\nPlease enter the network interface name manually:" 12 70 \
                               3>&1 1>&2 2>&3)

    if [ $? -ne 0 ]; then
        clear
        echo "Installation cancelled by user."
        exit 1
    fi
else
    # Add "custom" option to the menu
    INTERFACES="$INTERFACES custom \"Enter custom interface name\""

    # Create the menu dynamically with available interfaces
    eval "NETWORK_INTERFACE=\$(dialog --title \"Network Interface Selection\" \
                                      --menu \"\nSelect the network interface for ROS2 communication:\n\nThis interface will be used for DDS communication.\" 18 70 8 \
                                      $INTERFACES \
                                      3>&1 1>&2 2>&3)"

    # Check if user cancelled
    if [ $? -ne 0 ]; then
        clear
        echo "Installation cancelled by user."
        exit 1
    fi

    # If user selected custom, prompt for manual entry
    if [ "$NETWORK_INTERFACE" = "custom" ]; then
        NETWORK_INTERFACE=$(dialog --title "Custom Network Interface" \
                                   --inputbox "\nEnter the network interface name:" 10 70 \
                                   3>&1 1>&2 2>&3)

        if [ $? -ne 0 ]; then
            clear
            echo "Installation cancelled by user."
            exit 1
        fi
    fi
fi

echo "Selected network interface: $NETWORK_INTERFACE"

# Wi-Fi interface selection page
# Get list of available network interfaces (works on both Linux and macOS)
if [ "$(uname)" = "Darwin" ]; then
    # macOS: ifconfig -l lists all interfaces in one line
    WIFI_INTERFACES=$(ifconfig -l | tr ' ' '\n' | grep -v "lo0" | awk '{printf "%s \"%s\" ", $1, $1}')
else
    # Linux: parse ifconfig output
    WIFI_INTERFACES=$(ifconfig -a | grep -E "^[a-z]" | awk '{print $1}' | sed 's/://g' | grep -v "lo" | awk '{printf "%s \"%s\" ", $1, $1}')
fi

if [ -z "$WIFI_INTERFACES" ]; then
    # If no interfaces found, allow manual entry
    WIFI_INTERFACE=$(dialog --title "Wi-Fi Interface Configuration" \
                            --inputbox "\nNo network interfaces detected.\n\nPlease enter the Wi-Fi interface name manually:" 12 70 \
                            3>&1 1>&2 2>&3)

    if [ $? -ne 0 ]; then
        clear
        echo "Installation cancelled by user."
        exit 1
    fi
else
    # Add "custom" option to the menu
    WIFI_INTERFACES="$WIFI_INTERFACES custom \"Enter custom interface name\""

    # Create the menu dynamically with available interfaces
    eval "WIFI_INTERFACE=\$(dialog --title \"Wi-Fi Interface Selection\" \
                                   --menu \"\nSelect the Wi-Fi interface:\n\nThis interface will be used for wireless connectivity.\" 18 70 8 \
                                   $WIFI_INTERFACES \
                                   3>&1 1>&2 2>&3)"

    # Check if user cancelled
    if [ $? -ne 0 ]; then
        clear
        echo "Installation cancelled by user."
        exit 1
    fi

    # If user selected custom, prompt for manual entry
    if [ "$WIFI_INTERFACE" = "custom" ]; then
        WIFI_INTERFACE=$(dialog --title "Custom Wi-Fi Interface" \
                                --inputbox "\nEnter the Wi-Fi interface name:" 10 70 \
                                3>&1 1>&2 2>&3)

        if [ $? -ne 0 ]; then
            clear
            echo "Installation cancelled by user."
            exit 1
        fi
    fi
fi

echo "Selected Wi-Fi interface: $WIFI_INTERFACE"

# Wi-Fi credentials configuration page
# Create a temporary file for the form
TEMP_FORM=$(mktemp)
dialog --title "Wi-Fi Credentials Configuration" \
       --form "\nEnter the default Wi-Fi credentials:\n\nThese will be used for automatic Wi-Fi connection." 14 70 2 \
       "SSID:" 1 1 "" 1 20 40 0 \
       "Password:" 2 1 "" 2 20 40 0 \
       2> "$TEMP_FORM"

# Check if user cancelled
if [ $? -ne 0 ]; then
    rm -f "$TEMP_FORM"
    clear
    echo "Installation cancelled by user."
    exit 1
fi

# Read the form values
WIFI_SSID=$(sed -n '1p' "$TEMP_FORM")
WIFI_PASSWORD=$(sed -n '2p' "$TEMP_FORM")
rm -f "$TEMP_FORM"

echo "Wi-Fi SSID: $WIFI_SSID"
echo "Wi-Fi Password: [hidden]"

# OpenAI API Key configuration page
dialog --title "OpenAI API Key Configuration" \
       --yesno "\nWould you like to configure an OpenAI API key?\n\nThis is optional and can be used for GPT-powered features in your robot.\n\nYou can skip this step and configure it later in robot_config.yaml if needed." 12 70

if [ $? -eq 0 ]; then
    # User selected Yes
    OPENAI_API_KEY=$(dialog --title "OpenAI API Key" \
                            --inputbox "\nEnter your OpenAI API key:\n\n(You can find this at https://platform.openai.com/api-keys)" 12 70 \
                            3>&1 1>&2 2>&3)

    if [ $? -ne 0 ]; then
        clear
        echo "Installation cancelled by user."
        exit 1
    fi

    echo "OpenAI API key configured"
else
    # User selected No
    OPENAI_API_KEY=""
    echo "OpenAI API key skipped"
fi

# Supabase configuration page
dialog --title "Supabase Configuration" \
       --yesno "\nWould you like to configure Supabase credentials for the web server?\n\nThis is required for the web interface to work properly.\n\nYou can skip this step and configure it later in the .env file if needed." 12 70

if [ $? -eq 0 ]; then
    # Create temporary file for the form
    TEMP_FORM=$(mktemp)
    dialog --title "Supabase Credentials" \
           --form "\nEnter your Supabase configuration:\n" 14 100 2 \
           "Supabase URL:" 1 1 "" 1 20 70 0 \
           "Anon Key:" 2 1 "" 2 20 70 300 \
           2> "$TEMP_FORM"

    if [ $? -ne 0 ]; then
        rm -f "$TEMP_FORM"
        clear
        echo "Installation cancelled by user."
        exit 1
    fi

    # Read the form values
    SUPABASE_URL=$(sed -n '1p' "$TEMP_FORM")
    SUPABASE_ANON_KEY=$(sed -n '2p' "$TEMP_FORM")
    rm -f "$TEMP_FORM"

    echo "Supabase URL: $SUPABASE_URL"
    echo "Supabase Anon Key: [hidden]"

    # Create .env file in root directory (for docker-compose.yaml)
    ENV_FILE=".env"
    cat > "$ENV_FILE" << EOF
NEXT_PUBLIC_SUPABASE_URL=$SUPABASE_URL
NEXT_PUBLIC_SUPABASE_ANON_KEY=$SUPABASE_ANON_KEY
EOF

    echo "Created Supabase .env file at $ENV_FILE"
else
    SUPABASE_URL=""
    SUPABASE_ANON_KEY=""
    echo "Supabase configuration skipped (you can configure it later in the .env file)"
fi

# Front camera selection page
FRONT_CAMERA=$(dialog --title "Front Camera Selection" \
                      --menu "\nSelect the front camera model:" 16 70 4 \
                      "d435i" "Intel RealSense D435i" \
                      "d455" "Intel RealSense D455" \
                      "d555" "Intel RealSense D555" \
                      "none" "No front camera" \
                      3>&1 1>&2 2>&3)

# Check if user cancelled
if [ $? -ne 0 ]; then
    clear
    echo "Installation cancelled by user."
    exit 1
fi

echo "Selected front camera: $FRONT_CAMERA"

# If front camera is not "none", ask for serial number
FRONT_SERIAL=""
if [ "$FRONT_CAMERA" != "none" ]; then
    FRONT_SERIAL=$(dialog --title "Front Camera Serial Number" \
                          --inputbox "\nEnter the serial number for the front camera:\n\n(You can find this on the camera label or using 'rs-enumerate-devices')" 12 70 \
                          3>&1 1>&2 2>&3)

    if [ $? -ne 0 ]; then
        clear
        echo "Installation cancelled by user."
        exit 1
    fi

    echo "Front camera serial number: $FRONT_SERIAL"
fi

# Rear camera selection page
REAR_CAMERA=$(dialog --title "Rear Camera Selection" \
                     --menu "\nSelect the rear camera model:" 16 70 4 \
                     "d435i" "Intel RealSense D435i" \
                     "d455" "Intel RealSense D455" \
                     "d555" "Intel RealSense D555" \
                     "none" "No rear camera" \
                     3>&1 1>&2 2>&3)

# Check if user cancelled
if [ $? -ne 0 ]; then
    clear
    echo "Installation cancelled by user."
    exit 1
fi

echo "Selected rear camera: $REAR_CAMERA"

# If rear camera is not "none", ask for serial number
REAR_SERIAL=""
if [ "$REAR_CAMERA" != "none" ]; then
    REAR_SERIAL=$(dialog --title "Rear Camera Serial Number" \
                         --inputbox "\Enter the serial number for the rear camera:\n\n(You can find this on the camera label or using 'rs-enumerate-devices')" 12 70 \
                         3>&1 1>&2 2>&3)

    if [ $? -ne 0 ]; then
        clear
        echo "Installation cancelled by user."
        exit 1
    fi

    echo "Rear camera serial number: $REAR_SERIAL"
fi

# Update camera_config.yaml in the selected robot package
CAMERA_CONFIG_FILE="botbrain_ws/src/${ROBOT_MODEL}_pkg/config/camera_config.yaml"

if [ -f "$CAMERA_CONFIG_FILE" ]; then
    # Update front camera type if not "none"
    if [ "$FRONT_CAMERA" != "none" ]; then
        sed -i.bak "s/\(front:.*\)/\1/" "$CAMERA_CONFIG_FILE"
        sed -i.bak "/front:/,/type:/ s/type: \".*\"/type: \"$FRONT_CAMERA\"/" "$CAMERA_CONFIG_FILE"
        sed -i.bak "/front:/,/serial_number:/ s/serial_number: \".*\"/serial_number: \"$FRONT_SERIAL\"/" "$CAMERA_CONFIG_FILE"
        echo "Updated front camera configuration in $CAMERA_CONFIG_FILE"
    fi

    # Update rear camera type if not "none"
    if [ "$REAR_CAMERA" != "none" ]; then
        sed -i.bak "/back:/,/type:/ s/type: \".*\"/type: \"$REAR_CAMERA\"/" "$CAMERA_CONFIG_FILE"
        sed -i.bak "/back:/,/serial_number:/ s/serial_number: \".*\"/serial_number: \"$REAR_SERIAL\"/" "$CAMERA_CONFIG_FILE"
        echo "Updated rear camera configuration in $CAMERA_CONFIG_FILE"
    fi

    # Deletes backup file
    rm -f "${CAMERA_CONFIG_FILE}.bak"
else
    echo "Warning: $CAMERA_CONFIG_FILE not found. Skipping camera configuration."
fi

# Update cyclonedds_config.xml with selected network interface
CYCLONEDDS_CONFIG="botbrain_ws/cyclonedds_config.xml"

if [ -f "$CYCLONEDDS_CONFIG" ]; then
    # Update the ethernet interface in cyclonedds_config.xml (keeping loopback interface unchanged)
    sed -i.bak "/<NetworkInterface name=\"lo\"/! s/<NetworkInterface name=\"[^\"]*\" priority=\"10\"/<NetworkInterface name=\"$NETWORK_INTERFACE\" priority=\"10\"/" "$CYCLONEDDS_CONFIG"

    # Delete backup file
    rm -f "${CYCLONEDDS_CONFIG}.bak"

    echo "Updated cyclonedds_config.xml with network interface: $NETWORK_INTERFACE"
else
    echo "Warning: $CYCLONEDDS_CONFIG not found. Skipping CycloneDDS configuration."
fi

# Update robot_config.yaml with all selections at the end
CONFIG_FILE="botbrain_ws/robot_config.yaml"

if [ -f "$CONFIG_FILE" ]; then
    # Create temporary file with all updates
    cp "$CONFIG_FILE" "${CONFIG_FILE}.tmp"

    # Apply all sed replacements to the temporary file
    sed -i "s/robot_model: \".*\"/robot_model: \"$ROBOT_MODEL\"/" "${CONFIG_FILE}.tmp"

    # If Tita was selected, also update the namespace
    if [ "$ROBOT_MODEL" = "tita" ] && [ -n "$TITA_NAMESPACE" ]; then
        sed -i "s/tita_namespace: \".*\"/tita_namespace: \"$TITA_NAMESPACE\"/" "${CONFIG_FILE}.tmp"
    fi

    sed -i "s/description_file_type: \".*\"/description_file_type: \"$DESCRIPTION_TYPE\"/" "${CONFIG_FILE}.tmp"
    sed -i "s/robot_name: \".*\"/robot_name: \"$ROBOT_NAME\"/" "${CONFIG_FILE}.tmp"
    sed -i "s/network_interface: \".*\"/network_interface: \"$NETWORK_INTERFACE\"/" "${CONFIG_FILE}.tmp"
    sed -i "s/wifi_interface: \".*\"/wifi_interface: \"$WIFI_INTERFACE\"/" "${CONFIG_FILE}.tmp"
    sed -i "s/wifi_ssid: \".*\"/wifi_ssid: \"$WIFI_SSID\"/" "${CONFIG_FILE}.tmp"
    sed -i "s/wifi_password: \".*\"/wifi_password: \"$WIFI_PASSWORD\"/" "${CONFIG_FILE}.tmp"
    sed -i "s|openai_api_key: \".*\"|openai_api_key: \"$OPENAI_API_KEY\"|" "${CONFIG_FILE}.tmp"
    sed -i "s/front_camera: \".*\"/front_camera: \"$FRONT_CAMERA\"/" "${CONFIG_FILE}.tmp"
    sed -i "s/rear_camera: \".*\"/rear_camera: \"$REAR_CAMERA\"/" "${CONFIG_FILE}.tmp"
    sed -i "s/front_serial_number: \".*\"/front_serial_number: \"$FRONT_SERIAL\"/" "${CONFIG_FILE}.tmp"
    sed -i "s/rear_serial_number: \".*\"/rear_serial_number: \"$REAR_SERIAL\"/" "${CONFIG_FILE}.tmp"

    # Replace the original file with the temporary file
    mv "${CONFIG_FILE}.tmp" "$CONFIG_FILE"

    echo "Updated robot_config.yaml with all configuration settings"
else
    clear
    echo "Error: $CONFIG_FILE not found!"
    exit 1
fi

# Pre-installation confirmation
dialog --title "Installation Confirmation" \
       --yesno "\nConfiguration complete!\n\nYour BotBrain Workspace will be configured with:\n\n• Robot Model: $ROBOT_MODEL\n• Description Type: $DESCRIPTION_TYPE\n• Robot Name: ${ROBOT_NAME:-Not set}\n• Network Interface: $NETWORK_INTERFACE\n• Wi-Fi Interface: $WIFI_INTERFACE\n• Wi-Fi SSID: $WIFI_SSID\n• Front Camera: $FRONT_CAMERA\n• Rear Camera: $REAR_CAMERA\n\nThe installation will:\n\n1. Pull Docker images from docker-compose\n2. Set up and enable botbrain.service\n3. Set up and enable web_server.service\n4. Start builder services (builder_base, builder_yolo, web_server_builder)\n\nThis process may take several minutes.\n\nDo you want to continue with the installation?" 25 70

if [ $? -ne 0 ]; then
    clear
    echo "Installation cancelled by user."
    exit 1
fi

# Installation with progress tracking using mixedgauge
clear

# Clear/create log file at the start
> /tmp/install_log.txt

# Function to show progress (static version for completion)
show_progress() {
    local task1=$1
    local task2=$2
    local task3=$3
    local task4=$4

    # Format status indicators
    local status1="   "
    local status2="   "
    local status3="   "
    local status4="   "

    [[ "$task1" == "0" ]] && status1=" ✓ "
    [[ "$task1" == "7" ]] && status1=" ● "
    [[ "$task2" == "0" ]] && status2=" ✓ "
    [[ "$task2" == "7" ]] && status2=" ● "
    [[ "$task3" == "0" ]] && status3=" ✓ "
    [[ "$task3" == "7" ]] && status3=" ● "
    [[ "$task4" == "0" ]] && status4=" ✓ "
    [[ "$task4" == "7" ]] && status4=" ● "

    dialog --title "Installing BotBrain Workspace" --infobox "\n[$status1] Pull Docker images\n\n[$status2] Setup botbrain.service\n\n[$status3] Setup web_server.service\n\n[$status4] Start builder services" 13 70
}

# Animated progress display for running tasks
show_progress_animated() {
    local task1=$1
    local task2=$2
    local task3=$3
    local task4=$4

    local dots=0
    while true; do
        local animation=""
        case $((dots % 4)) in
            0) animation="   " ;;
            1) animation=".  " ;;
            2) animation=".. " ;;
            3) animation="..." ;;
        esac

        # Format status indicators
        local status1="   "
        local status2="   "
        local status3="   "
        local status4="   "

        [[ "$task1" == "0" ]] && status1=" ✓ "
        [[ "$task1" == "7" ]] && status1="$animation"
        [[ "$task2" == "0" ]] && status2=" ✓ "
        [[ "$task2" == "7" ]] && status2="$animation"
        [[ "$task3" == "0" ]] && status3=" ✓ "
        [[ "$task3" == "7" ]] && status3="$animation"
        [[ "$task4" == "0" ]] && status4=" ✓ "
        [[ "$task4" == "7" ]] && status4="$animation"

        dialog --title "Installing BotBrain Workspace" --infobox "\n[$status1] Pull Docker images\n\n[$status2] Setup botbrain.service\n\n[$status3] Setup web_server.service\n\n[$status4] Start builder services" 13 70

        dots=$((dots + 1))
        sleep 0.5
    done
}

# Step 1: Pull Docker images
if [ -f "docker-compose.yml" ] || [ -f "docker-compose.yaml" ]; then
    show_progress_animated "7" "-" "-" "-" &
    DIALOG_PID=$!

    docker compose pull --quiet > /dev/null 2>&1
    PULL_RESULT=$?

    kill $DIALOG_PID 2>/dev/null
    wait $DIALOG_PID 2>/dev/null

    echo "Docker pull exit code: $PULL_RESULT" >> /tmp/install_log.txt

    if [ $PULL_RESULT -ne 0 ]; then
        dialog --title "Installation Error" --msgbox "Failed to pull Docker images.\n\nCheck /tmp/install_log.txt for details." 10 70
        clear
        exit 1
    fi
else
    dialog --title "Installation Error" --msgbox "docker-compose.yaml file not found in the current directory." 10 70
    clear
    exit 1
fi

# Step 2: Set up botbrain.service
show_progress_animated "0" "7" "-" "-" &
DIALOG_PID=$!

if [ -f "botbrain.service" ]; then
    # Get the absolute path of the current directory
    WORKSPACE_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

    # Create a temporary service file with the correct working directory
    sed "s|BOTBRAIN_WORKSPACE_PATH|$WORKSPACE_PATH|g" botbrain.service > /tmp/botbrain.service.tmp

    # Copy the modified service file to systemd
    cp /tmp/botbrain.service.tmp /etc/systemd/system/botbrain.service >> /tmp/install_log.txt 2>&1
    systemctl daemon-reload >> /tmp/install_log.txt 2>&1
    systemctl enable botbrain.service >> /tmp/install_log.txt 2>&1

    # Clean up temporary file
    rm -f /tmp/botbrain.service.tmp

    if [ $? -ne 0 ]; then
        kill $DIALOG_PID 2>/dev/null
        wait $DIALOG_PID 2>/dev/null
        dialog --title "Installation Error" --msgbox "Failed to set up botbrain.service.\n\nCheck /tmp/install_log.txt for details." 10 70
        clear
        exit 1
    fi
else
    kill $DIALOG_PID 2>/dev/null
    wait $DIALOG_PID 2>/dev/null
    dialog --title "Installation Error" --msgbox "botbrain.service file not found in the current directory." 10 70
    clear
    exit 1
fi

kill $DIALOG_PID 2>/dev/null
wait $DIALOG_PID 2>/dev/null

# Step 3: Set up web_server.service
show_progress_animated "0" "0" "7" "-" &
DIALOG_PID=$!

if [ -f "web_server.service" ]; then
    # Get the absolute path of the current directory
    WORKSPACE_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

    # Create a temporary service file with the correct working directory
    sed "s|BOTBRAIN_WORKSPACE_PATH|$WORKSPACE_PATH|g" web_server.service > /tmp/web_server.service.tmp

    # Copy the modified service file to systemd
    cp /tmp/web_server.service.tmp /etc/systemd/system/web_server.service >> /tmp/install_log.txt 2>&1
    systemctl daemon-reload >> /tmp/install_log.txt 2>&1
    systemctl enable web_server.service >> /tmp/install_log.txt 2>&1

    # Clean up temporary file
    rm -f /tmp/web_server.service.tmp

    if [ $? -ne 0 ]; then
        kill $DIALOG_PID 2>/dev/null
        wait $DIALOG_PID 2>/dev/null
        dialog --title "Installation Error" --msgbox "Failed to set up web_server.service.\n\nCheck /tmp/install_log.txt for details." 10 70
        clear
        exit 1
    fi
else
    kill $DIALOG_PID 2>/dev/null
    wait $DIALOG_PID 2>/dev/null
    dialog --title "Installation Error" --msgbox "web_server.service file not found in the current directory." 10 70
    clear
    exit 1
fi

kill $DIALOG_PID 2>/dev/null
wait $DIALOG_PID 2>/dev/null

# Verify .env file exists before building
if [ -f ".env" ]; then
    echo "Found .env file, contents:" >> /tmp/install_log.txt
    cat .env >> /tmp/install_log.txt
else
    echo "WARNING: .env file not found before build!" >> /tmp/install_log.txt
fi

# Step 4: Run builder services
show_progress_animated "0" "0" "0" "7" &
DIALOG_PID=$!

docker compose up -d builder_base builder_yolo web_server_builder > /dev/null 2>&1
BUILD_RESULT=$?

echo "Docker compose up exit code: $BUILD_RESULT" >> /tmp/install_log.txt

if [ $BUILD_RESULT -ne 0 ]; then
    kill $DIALOG_PID 2>/dev/null
    wait $DIALOG_PID 2>/dev/null
    dialog --title "Installation Error" --msgbox "Failed to start builder services.\n\nCheck /tmp/install_log.txt for details." 10 70
    clear
    exit 1
fi

# Wait for all builder services to complete
# These containers are designed to build and then exit, so we wait for them to finish
echo "Waiting for builder services to complete..." >> /tmp/install_log.txt
BUILDER_BASE_ID=$(docker compose ps -q builder_base)
BUILDER_YOLO_ID=$(docker compose ps -q builder_yolo)
WEB_SERVER_BUILDER_ID=$(docker compose ps -q web_server_builder)

# Wait for each container to finish
if [ -n "$BUILDER_BASE_ID" ]; then
    echo "Waiting for builder_base to complete..." >> /tmp/install_log.txt
    docker wait "$BUILDER_BASE_ID" >> /tmp/install_log.txt 2>&1
fi

if [ -n "$BUILDER_YOLO_ID" ]; then
    echo "Waiting for builder_yolo to complete..." >> /tmp/install_log.txt
    docker wait "$BUILDER_YOLO_ID" >> /tmp/install_log.txt 2>&1
fi

if [ -n "$WEB_SERVER_BUILDER_ID" ]; then
    echo "Waiting for web_server_builder to complete..." >> /tmp/install_log.txt
    docker wait "$WEB_SERVER_BUILDER_ID" >> /tmp/install_log.txt 2>&1
fi

kill $DIALOG_PID 2>/dev/null
wait $DIALOG_PID 2>/dev/null

# Check if any of the builder services failed (non-zero exit code)
BUILDER_BASE_STATUS=$(docker inspect -f '{{.State.ExitCode}}' "$BUILDER_BASE_ID" 2>/dev/null || echo "1")
BUILDER_YOLO_STATUS=$(docker inspect -f '{{.State.ExitCode}}' "$BUILDER_YOLO_ID" 2>/dev/null || echo "1")
WEB_SERVER_BUILDER_STATUS=$(docker inspect -f '{{.State.ExitCode}}' "$WEB_SERVER_BUILDER_ID" 2>/dev/null || echo "1")

echo "builder_base exit code: $BUILDER_BASE_STATUS" >> /tmp/install_log.txt
echo "builder_yolo exit code: $BUILDER_YOLO_STATUS" >> /tmp/install_log.txt
echo "web_server_builder exit code: $WEB_SERVER_BUILDER_STATUS" >> /tmp/install_log.txt

if [ "$BUILDER_BASE_STATUS" -ne 0 ] || [ "$BUILDER_YOLO_STATUS" -ne 0 ] || [ "$WEB_SERVER_BUILDER_STATUS" -ne 0 ]; then
    dialog --title "Installation Error" --msgbox "One or more builder services failed during build.\n\nCheck /tmp/install_log.txt and docker logs for details." 10 70
    clear
    exit 1
fi

# Show completion
show_progress "0" "0" "0" "0"
sleep 2

# Installation complete
clear
dialog --title "Installation Complete" \
       --msgbox "\nBotBrain Workspace installation completed successfully!\n\nAll components have been configured and installed:\n\n✓ Robot configuration saved\n✓ Docker images pulled\n✓ botbrain.service enabled\n✓ web_server.service enabled\n✓ builder_base service running\n✓ builder_yolo service running\n✓ web_server_builder service running\n\nYou can now start using your BotBrain workspace." 19 70

clear
echo "Installation completed successfully!"
echo ""
echo "Configuration summary:"
echo "  - Robot model: $ROBOT_MODEL"
echo "  - Description type: $DESCRIPTION_TYPE"
echo "  - Robot name: $ROBOT_NAME"
echo "  - Network interface: $NETWORK_INTERFACE"
echo "  - Wi-Fi interface: $WIFI_INTERFACE"
echo "  - Wi-Fi SSID: $WIFI_SSID"
echo "  - Front camera: $FRONT_CAMERA"
echo "  - Rear camera: $REAR_CAMERA"
echo ""
echo "Services status:"
echo "  - botbrain.service: enabled"
echo "  - web_server.service: enabled"
echo ""
echo "Installation complete!"