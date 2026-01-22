SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="$SCRIPT_DIR/robot_config.yaml"

if [ ! -f "$CONFIG_FILE" ]; then
    echo "⚠️  Robot Setup Warning: Configuration file not found at $CONFIG_FILE"
    return 1
fi

ROBOT_MODEL=$(grep "^\s*robot_model:" "$CONFIG_FILE" | sed -e 's/#.*//' -e 's/.*://' -e 's/"//g' -e "s/'//g" -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//')

if [ -z "$ROBOT_MODEL" ]; then
    echo "⚠️  Robot Setup Warning: 'robot_model' key not found or is empty in $CONFIG_FILE"
    return 1
fi

SETUP_SCRIPT_PATH="$SCRIPT_DIR/src/${ROBOT_MODEL}_pkg/${ROBOT_MODEL}_setup.bash"

if [ -f "$SETUP_SCRIPT_PATH" ]; then
    source "$SETUP_SCRIPT_PATH"
else
    echo "⚠️  Robot Setup Warning: Setup script for model '$ROBOT_MODEL' not found at: $SETUP_SCRIPT_PATH"
    return 1
fi