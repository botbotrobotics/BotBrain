#!/usr/bin/env python3
import rclpy
import importlib
from rosa import ROSA
from bot_rosa.llm import get_llm
from bot_rosa.prompts import get_prompts

class Agent(ROSA):

    def __init__(self, robot_model: str = "go2", streaming: bool = False, verbose: bool = True):
        self.__blacklist = ["master", "docker"]
        self.__prompts = get_prompts()
        self.__llm = get_llm()

        # Dynamically import the tools module based on robot_model
        # e.g., "go2" -> import go2_pkg.tools.go2
        module_path = f"{robot_model}_pkg.tools.{robot_model}"
        try:
            robot_tools = importlib.import_module(module_path)
            if verbose:
                print(f"Successfully loaded tools for robot: {robot_model} (from {module_path})")
        except ModuleNotFoundError as e:
            raise ImportError(
                f"Failed to import tools for robot '{robot_model}'. "
                f"Expected module path: {module_path}. "
                f"Make sure the package '{robot_model}_pkg' exists and contains 'tools/{robot_model}.py'. "
                f"Error: {e}"
            )

        if not rclpy.ok():
            rclpy.init(args=None)

        super().__init__(
            ros_version=2,
            llm=self.__llm,
            tools=[],
            tool_packages=[robot_tools],
            blacklist=self.__blacklist,
            prompts=self.__prompts,
            verbose=verbose,
            accumulate_chat_history=True,
            streaming=streaming,
        )

    def get_response(self, query: str):
        """
        Process a query and return the agent's response.

        Args:
            query (str): The input query to process.

        Returns:
            str: The agent's response.
        """
        response = self.invoke(query)
        return response