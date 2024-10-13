https://github.com/user-attachments/assets/6def582f-1843-479f-badb-4657942ef397

## TurtleSim A* Path Planning with Meta LLaMA-3.1-405B-Instruct model powered by NVIDIA / LlamaIndex Agent
Integrate the <b>ROS (Robot Operating System)</b>, <b>A* path planning algorithm</b>, <b>LlamaIndex</b>, and <b>Meta's LLaMA-3.1-405B-Instruct model</b> to control a turtle in a simulated environment. The turtle navigates from a starting position to a goal while avoiding obstacles. Users can interact with the agent via terminal commands to create obstacles and direct the turtle to specific locations.

# Table of Contents

- [Table of Contents](#table-of-contents)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Running the Program](#running-the-program)
- [Usage](#usage)
  - [Example Interaction](#example-interaction)
- [Code Overview](#code-overview)
  - [Key Methods](#key-methods)
- [License](#license)

# Features
- NVIDIA Language Model: Employs <b>Meta LLaMA-3.1-405B-Instruct</b> NVIDIA's LLM for natural language processing of commands.
- LlamaIndex Integration: Utilizes LlamaIndex for building tools for the NVIDIA LLM agent.
- A* Path Planning: Calculates the shortest path while avoiding obstacles.
- Dynamic Command Processing: Send commands to the agent in real-time via the terminal.
- Obstacle Management: Create and clear obstacles in the simulation.
- Visual Grid Display: Shows the grid, obstacles, and the path in the terminal.
- ROS Integration: Utilizes ROS for simulation and turtle control.

# Prerequisites
- Python 3.6+
- NVIDIA LLM SDK: Access to NVIDIA's language model API.
- ROS (Robot Operating System) – Melodic or Noetic recommended
- Turtlesim Package: Comes pre-installed with ROS.

# Installation
If you don't have a ROS workspace set up, create one:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/taherfattahi/turtlesim-astar-nvidia-llm.git
cd ..
catkin_make
source devel/setup.bash
```
1. Install Required Python Packages

```bash
pip install --upgrade --quiet llama-index-llms-nvidia
```
Note: Ensure you have pip for Python 3. You might need to use pip3 instead.

2. NVIDIA LLM SDK Setup:
- Sign up for access to NVIDIA's language model API <a href="https://build.nvidia.com/">Link</a>.
- Obtain your ```NVIDIA_API_KEY```.

3. Configure Environment Variables
Export your NVIDIA API key as an environment variable:

```bash
export NVIDIA_API_KEY="your_nvapi_key_here"
```

# Running the Program
1. Start ROS Core
In a new terminal window:

```bash
roscore
```

2. Launch Turtlesim Node
In another terminal window:
```bash
rosrun turtlesim turtlesim_node
```

3. Run the Turtle Controller
In a new terminal window, navigate to the project directory and run:
```bash
python3 turtlesim_nvidia_llm.py
```

# Usage
Once the program is running, you can interact with the agent via the terminal:
- Create Obstacles

```bash
Enter command for agent (or 'exit' to quit): Create 8 obstacles in simulation
```

- Move the Turtle to a Goal Position
```bash
Enter command for agent (or 'exit' to quit): Move to this position 8 8
```

- Exit the Program
```bash
Enter command for agent (or 'exit' to quit): exit
```

## Example Interaction
```bash
Enter command for agent (or 'exit' to quit): Create 5 obstacles in simulation
Agent response: Obstacles created successfully.

Enter command for agent (or 'exit' to quit): Move to this position 5 5
Agent response: Moving to position (5, 5).

Grid (11x11):
. . . . . . . . . . .
. . . . X . . . . . .
. . . . X . . . . . .
. . . . . . . . . . .
. . . . . . . . . . .
. . . . . . X . . . .
. . . . . . . . . . .
. . . X . . . . . . .
. * * * * * G . . . .
S * X . . . . . . . .
```

- ```S```: Start position
- ```G```: Goal position
- ```X```: Obstacles
- ```*```: Path taken

# Code Overview
- ```TurtleController``` Class: Manages the turtle's movement, obstacle creation, and agent interactions.
- ```AStarPlanner``` Class: Implements the A* path planning algorithm.
- ```FunctionTool```: Wraps functions to be used as tools by the NVIDIA LLM agent.

## Key Methods
- ```create_obstacle_turtles(num_obstacles: int)```: Creates obstacles in the simulation.
- ```move_to_goal(goal_position: tuple)```: Moves the turtle to the specified goal position.
- ```process_command(command: str)```: Processes user commands via the agent.
- ```print_grid(path=None)```: Displays the grid, obstacles, and path in the terminal.

# License
This project is licensed under the MIT License.
