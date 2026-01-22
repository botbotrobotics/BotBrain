import subprocess
import json
import base64
import tempfile
import os
from typing import Tuple, Optional
from langchain.agents import tool
import openai
import time


def execute_ros_command(command: str) -> Tuple[bool, str]:
    """
    Execute a ROS2 command.

    :param command: The ROS2 command to execute.
    :return: A tuple containing a boolean indicating success and the output of the command.
    """
    # Validate the command is a proper ROS2 command
    cmd = command.split(" ")
    valid_ros2_commands = ["service", "topic"]

    if len(cmd) < 2:
        raise ValueError(f"'{command}' is not a valid ROS2 command.")
    if cmd[0] != "ros2":
        raise ValueError(f"'{command}' is not a valid ROS2 command.")
    if cmd[1] not in valid_ros2_commands:
        raise ValueError(f"'ros2 {cmd[1]}' is not a valid ros2 subcommand.")

    try:
        output = subprocess.check_output(command, shell=True, stderr=subprocess.STDOUT, timeout=30).decode()
        return True, output
    except subprocess.CalledProcessError as e:
        return False, e.output.decode() if e.output else str(e)
    except subprocess.TimeoutExpired:
        return False, "Command timed out after 30 seconds"
    except Exception as e:
        return False, str(e)


@tool
def select_mode(mode: str) -> str:
    """
    Changes robot mode. If the response returns failed try again one more time.

    :param mode: mode of the robot between the following:

        "stand_down": Makes the robot stand down on the ground. Execute this function in any situation related to lie down, like sleep or rest. If in "balance_stand" mode, the robot should go first to "stop_move".
        "stand_up": Makes the robot stand up on the ground.
    """
    # Call the mode service using ros2 service call CLI
    cmd = f'ros2 service call /mode bot_custom_interfaces/srv/Mode "{{mode: \'{mode}\'}}"'
    success, output = execute_ros_command(cmd)

    if not success:
        return f"Failed to call mode service: {output}"

    # Parse the response to check if it was successful
    if "success: true" in output.lower() or "success=true" in output.lower():
        return "Successfully changed robot mode"
    else:
        return f"Failed to change robot mode: {output}"

