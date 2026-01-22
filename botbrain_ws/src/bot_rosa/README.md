# bot_rosa

The AI assistant package for the BotBrain robot platform. This package provides an intelligent conversational interface using ROSA (Robot Operating System Assistant) framework with LLM integration for natural language interaction and robot control.

## Description

The `bot_rosa` package serves as the AI assistant system for the R1 robot platform, providing natural language processing capabilities and intelligent robot control through conversational interfaces. It integrates with the ROSA framework to enable voice commands, question answering, and autonomous robot operations using Large Language Models (LLMs).

## Directory Structure

```
bot_rosa/
├── bot_rosa/                    # Python package
│   ├── __init__.py             # Package initialization
│   ├── agent.py                # Main ROSA agent implementation
│   ├── llm.py                  # LLM configuration and setup
│   └── prompts.py              # System prompts and personality configuration
├── launch/                     # Launch files
│   └── rosa.launch.py          # Main ROSA service launcher
├── scripts/                    # Executable scripts
│   └── rosa_service.py         # ROSA service server
├── CMakeLists.txt              # CMake build configuration
└── package.xml                 # Package manifest
```

## Files Explanation

### Core Components

#### `bot_rosa/agent.py`
The main ROSA agent implementation that provides:

- **ROSA Integration**: Extends the ROSA framework for robot-specific functionality
- **LLM Integration**: Connects to language models for natural language processing
- **ROS2 Integration**: Manages ROS2 services and topics for robot control
- **Interactive Interface**: Provides conversational interaction capabilities
- **Dynamic Tool Loading**: Loads robot-specific tools from the corresponding robot package (e.g., tools for a Go2 robot are loaded from the go2_pkg package)

#### `bot_rosa/llm.py`
LLM configuration and setup:

- **Model Configuration**: Centralized configuration for language model selection
- **LLM Provider Setup**: Configures the LLM backend (OpenAI, Ollama, etc.)
- **Temperature Settings**: Controls response randomness and creativity

#### `bot_rosa/prompts.py`
System prompts and personality configuration:

- **System Prompts**: Defines the AI assistant's behavior and capabilities
- **Robot Context**: Provides context about the robot's environment and constraints
- **Response Guidelines**: Sets rules for how the agent should respond to users

### Launch Files

#### `launch/rosa.launch.py`
Main ROSA service launcher:

- **Service Server**: Launches the ROSA service for external interaction
- **Robot Configuration**: Reads robot configuration from robot_config.yaml
- **Namespace Support**: Uses robot name for multi-robot deployments

### Scripts

#### `scripts/rosa_service.py`
ROSA service server implementation:

- **Service Interface**: Provides LLMPrompt service for external queries
- **Agent Integration**: Connects to ROSA agent for processing requests
- **Response Handling**: Returns AI-generated responses to service clients

---

**Note**: This package requires proper LLM API access and robot hardware. Ensure OpenAI API credentials are configured and the robot is properly connected before use.
