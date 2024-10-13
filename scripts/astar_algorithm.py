#!/usr/bin/env python
import sys
import rospy
import math
import random
import heapq
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import Kill, Spawn, TeleportAbsolute, SetPen

# Grid dimensions (turtlesim default is 11x11 units)
GRID_WIDTH = 11
GRID_HEIGHT = 11
GRID_RESOLUTION = 1  # 1 unit per grid cell

num_obstacles = 4  # Choose how many obstacles you want
# Obstacles (list of (x, y) tuples)
OBSTACLES = []
while len(OBSTACLES) < num_obstacles:
    x = random.randint(0, GRID_WIDTH - 1)
    y = random.randint(0, GRID_WIDTH - 1)
    if (x, y) not in OBSTACLES:
        OBSTACLES.append((x, y))

print("OBSTACLES =", OBSTACLES)

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
        # self.goal = (8.0, 8.0)  # Goal position
        self.obstacles = OBSTACLES
        self.path = []
        self.path_index = 0
        # Proportional control gains
        self.K_linear = 1.5
        self.K_angular = 4.0  
              
           # Parse command-line arguments for goal position
        if len(sys.argv) >= 3:
            try:
                self.goal = (float(sys.argv[1]), float(sys.argv[2]))
            except ValueError:
                rospy.logerr("Invalid goal coordinates. Using default goal (8.0, 8.0).")
                self.goal = (8.0, 8.0)
        else:
            self.goal = (8.0, 8.0)  # Default goal position

    def update_pose(self, data):
        self.pose = data

    def move_to_goal(self):
        # Wait until the pose is received
        rospy.wait_for_message('/turtle1/pose', Pose)

        # Clear existing obstacle turtles
        self.clear_obstacle_turtles()
        
        # Spawn obstacle turtles on the screen
        self.spawn_obstacle_turtles()
        
        # Plan the path
        start = (self.pose.x, self.pose.y)
        planner = AStarPlanner(start, self.goal, self.obstacles)
        path = planner.search()

        if path is None:
            rospy.loginfo("No path found!")
            return
        else:
            rospy.loginfo("Path found: {}".format(path))
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
        controller.move_to_goal()
    except rospy.ROSInterruptException:
        pass
