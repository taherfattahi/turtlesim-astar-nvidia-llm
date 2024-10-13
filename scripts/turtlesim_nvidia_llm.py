#!/usr/bin/env python

import os
import sys
import getpass
import threading
import asyncio

from llama_index.llms.nvidia import NVIDIA
from llama_index.core.tools import FunctionTool
from llama_index.core.agent import FunctionCallingAgent

import rospy
import math
import random
import heapq
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import Kill, Spawn, TeleportAbsolute, SetPen

# del os.environ['NVIDIA_API_KEY']  ## delete key and reset
if os.environ.get("NVIDIA_API_KEY", "").startswith("nvapi-"):
    print("Valid NVIDIA_API_KEY already in environment. Delete to reset")
else:
    nvapi_key = getpass.getpass("NVAPI Key (starts with nvapi-): ")
    assert nvapi_key.startswith(
        "nvapi-"
    ), f"{nvapi_key[:5]}... is not a valid key"
    os.environ["NVIDIA_API_KEY"] = nvapi_key
    
# Grid dimensions (turtlesim default is 11x11 units)
GRID_WIDTH = 11
GRID_HEIGHT = 11
GRID_RESOLUTION = 1  # 1 unit per grid cell

class AStarPlanner:
    def __init__(self, start, goal, obstacles):
        self.start = self.grid_coord(start)
        self.goal = self.grid_coord(goal)
        self.obstacles = set(obstacles)
        self.open_set = []
        heapq.heappush(self.open_set, (0, self.start))
        self.came_from = {}
        self.g_score = {self.start: 0}
        self.f_score = {self.start: self.heuristic(self.start, self.goal)}

    def grid_coord(self, pos):
        x, y = pos
        return (int(x), int(y))

    def heuristic(self, a, b):
        # Euclidean distance
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def neighbors(self, node):
        neighbors = []
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            x2, y2 = node[0] + dx, node[1] + dy
            if 0 <= x2 < GRID_WIDTH and 0 <= y2 < GRID_HEIGHT:
                if (x2, y2) not in self.obstacles:
                    neighbors.append((x2, y2))
        return neighbors

    def reconstruct_path(self):
        node = self.goal
        path = [node]
        while node in self.came_from:
            node = self.came_from[node]
            path.append(node)
        path.reverse()
        return path

    def search(self):
        while self.open_set:
            _, current = heapq.heappop(self.open_set)
            if current == self.goal:
                return self.reconstruct_path()
            for neighbor in self.neighbors(current):
                tentative_g_score = self.g_score[current] + 1
                if neighbor not in self.g_score or tentative_g_score < self.g_score[neighbor]:
                    self.came_from[neighbor] = current
                    self.g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + self.heuristic(neighbor, self.goal)
                    self.f_score[neighbor] = f_score
                    heapq.heappush(self.open_set, (f_score, neighbor))
        return None
    
    def print_grid(self, path=None):
        grid = [['.' for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)]
        
        # Mark obstacles
        for (x, y) in self.obstacles:
            grid[GRID_HEIGHT - 1 - y][x] = 'X' 

        # Mark the path
        if path:
            for (x, y) in path:
                if (x, y) != self.start and (x, y) != self.goal:
                    grid[GRID_HEIGHT - 1 - y][x] = '*'

        # Mark start and goal positions
        sx, sy = self.start
        gx, gy = self.goal
        grid[GRID_HEIGHT - 1 - sy][sx] = 'S'
        grid[GRID_HEIGHT - 1 - gy][gx] = 'G'

        # Print the grid
        print("Grid ({}x{}):".format(GRID_WIDTH, GRID_HEIGHT))
        for row in grid:
            print(' '.join(row))

class TurtleController:
    def __init__(self):
        rospy.init_node('turtle_astar_controller', anonymous=True)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.spawn_service = rospy.ServiceProxy('/spawn', Spawn)
        self.kill_service = rospy.ServiceProxy('/kill', Kill)
        self.clear_service = rospy.ServiceProxy('/clear', Empty)
        self.pose = Pose()
        self.rate = rospy.Rate(10)
        self.obstacles = []
        self.path = []
        self.path_index = 0
        # Proportional control gains
        self.K_linear = 2.0
        self.K_angular = 10.0  
              
        # Parse command-line arguments for goal position
        if len(sys.argv) >= 3:
            try:
                self.goal = (float(sys.argv[1]), float(sys.argv[2]))
            except ValueError:
                rospy.logerr("Invalid goal coordinates. Using default goal (8.0, 8.0).")
                self.goal = (8.0, 8.0)
        else:
            self.goal = (0.0, 0.0)  # Default goal position
            
        self.llm = NVIDIA("meta/llama-3.1-405b-instruct")
        
        self.create_obstacle_turtles_tool = FunctionTool.from_defaults(
            fn=self.create_obstacle_turtles,
            name="create_obstacle_turtles",
            description=(
                "Creates a specified number of obstacle turtles in the simulation environment. "
                "Accepts an optional integer parameter `num_obstacles` (default is 4). "
                "Clears existing obstacles and spawns new ones at unique random positions within the grid."
                ),
        )
        self.move_to_goal_tool = FunctionTool.from_defaults(
            fn=self.move_to_goal,
            name="move_to_goal", 
            description=(
                "Moves the turtle to a specified goal position within the simulation environment. "
                "Accepts a parameter `goal_position`, which is a tuple of floats representing the (x, y) coordinates. "
                "Uses the A* algorithm to calculate the shortest path avoiding obstacles and moves the turtle along this path."
            ),
        )
        
        self.agent = FunctionCallingAgent.from_tools(
            [self.create_obstacle_turtles_tool, self.move_to_goal_tool],
            llm=self.llm,
            verbose=True,
        )
        
        # Create an event loop in a separate thread
        self.loop = asyncio.new_event_loop()
        self.loop_thread = threading.Thread(target=self.loop.run_forever)
        self.loop_thread.start()
        
    def process_command(self, command):
        # Schedule the agent's achat coroutine in the event loop running in a separate thread
        future = asyncio.run_coroutine_threadsafe(self.agent.achat(command), self.loop)
        try:
            # Wait for the result with a timeout to prevent indefinite blocking
            response = future.result(timeout=30)
            print(f"Agent response: {response}")
        except asyncio.TimeoutError:
            print("The agent took too long to respond.")
            
    def shutdown(self):
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.loop_thread.join()
        
    def update_pose(self, data):
        self.pose = data

    def create_obstacle_turtles(self, num_obstacles: int) -> None:
        # Clear existing obstacle turtles
        self.clear_obstacle_turtles()

        self.obstacles = []
        while len(self.obstacles) < num_obstacles:
            x = random.randint(0, GRID_WIDTH - 1)
            y = random.randint(0, GRID_WIDTH - 1)
            if (x, y) not in self.obstacles:
                self.obstacles.append((x, y))

        print("{num_obstacles} OBSTACLES = ", self.obstacles)

        # Spawn obstacle turtles on the screen
        self.spawn_obstacle_turtles()
        
    def move_to_goal(self, goal_position: tuple) -> None:
        
        self.goal = goal_position
        rospy.wait_for_message('/turtle1/pose', Pose)
        
        # Plan the path
        start = (self.pose.x, self.pose.y)
        planner = AStarPlanner(start, self.goal, self.obstacles)
        path = planner.search()

        if path is None:
            rospy.loginfo("No path found!")
            
            # Print grid with obstacles only
            planner.print_grid()
            return
        else:
            rospy.loginfo("Path found: {}".format(path))
            
            # Print grid with obstacles and path
            planner.print_grid(path=path)
            
            self.path = path
            self.follow_path()

    def follow_path(self):
        twist = Twist()
        for node in self.path[1:]:
            target_x = node[0] + 0.5  # Move to the center of the grid cell
            target_y = node[1] + 0.5

            while not rospy.is_shutdown():
                distance = math.hypot(target_x - self.pose.x, target_y - self.pose.y)
                angle_to_goal = math.atan2(target_y - self.pose.y, target_x - self.pose.x)
                angle_diff = self.normalize_angle(angle_to_goal - self.pose.theta)

                # Proportional Controller
                linear_speed = self.K_linear * distance
                angular_speed = self.K_angular * angle_diff

                twist.linear.x = linear_speed
                twist.angular.z = angular_speed
                self.velocity_publisher.publish(twist)

                if distance < 0.1:
                    break

                self.rate.sleep()

        # Stop the turtle after reaching the goal
        twist.linear.x = 0
        twist.angular.z = 0
        self.velocity_publisher.publish(twist)
        rospy.loginfo("Goal reached!")

    def normalize_angle(self, angle):
        if abs(angle) > math.pi:
            angle = angle - (2 * math.pi * angle) / abs(angle)
        return angle
    
    def spawn_obstacle_turtles(self):
        rospy.wait_for_service('/spawn')
        rospy.wait_for_service('/clear')

        # Clear the screen
        self.clear_service()

        # Spawn turtles at obstacle positions
        for idx, obstacle in enumerate(self.obstacles):
            x = obstacle[0] + 0.5  # Center of the grid cell
            y = obstacle[1] + 0.5
            name = 'obstacle{}'.format(idx)
            try:
                self.spawn_service(x, y, 0, name)
                rospy.loginfo("Spawned obstacle turtle '{}' at ({}, {})".format(name, x, y))
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: {}".format(e))
                
    def clear_obstacle_turtles(self):
        rospy.wait_for_service('/kill')
        try:
            # Get the list of all turtles
            turtles = rospy.get_param('/turtlesim_node/turtles', [])
            for idx, obstacle in enumerate(self.obstacles):
                turtle_name = 'obstacle{}'.format(idx)
                if turtle_name.startswith('obstacle'):
                    self.kill_service(turtle_name)
                    rospy.loginfo("Killed obstacle turtle '{}'".format(turtle_name))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))
        except KeyError:
            rospy.logwarn("No turtles found to kill.")

if __name__ == '__main__':
    try:
        controller = TurtleController()

        while not rospy.is_shutdown():
            command = input("Enter command for agent (or 'exit' to quit): ")
            if command.lower() == 'exit':
                rospy.signal_shutdown('User requested shutdown')
                break
            controller.process_command(command)
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.shutdown()
