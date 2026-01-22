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
        "stand_up": Makes the robot stand up on the ground. Execute this function in any situation related to stand up, like waking. For this to work, you must first go to stand_down mode. In this mode the robot joints are locked and it can not move.
        "damp": Makes the robot enter damp mode. This is like an emergency break, but controlled.
        "hello": Makes the robot say hello. Use this every time someone say hi, hello, greet you or anything related.
        "balance_stand": Makes the robot stand up balancing. It allow for the robot to move. The robot must be standing up for this mode to be called.
        "stop_move": Makes the robot lock its motors and stop movements when in "balance_stand" mode.
        "sit": Makes the robot sit down.
        "rise_sit": Makes the robot stand up from sitting down.
        "stretch": Makes the robot stretch itself.
        "dance1": Makes the robot dance.
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

@tool
def light_control(control: bool, brightness: int = 5) -> str:
    """
    Control the light of the robot on and off with it's intensity.

    :param control: "true" to turn on the lights and "false" to turn off.
    :param brightness: Brightness of the light, it can be a value from 1 to 10. Default value is 5.
    """
    # Call the light_control service using ros2 service call CLI
    control_str = "true" if control else "false"
    cmd = f'ros2 service call /light_control bot_custom_interfaces/srv/LightControl "{{control: {control_str}, brightness: {brightness}}}"'
    success, output = execute_ros_command(cmd)

    if not success:
        return f"Failed to call light_control service: {output}"

    return "Successfully controlled lights"



