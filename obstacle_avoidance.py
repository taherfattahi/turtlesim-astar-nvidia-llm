#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

# Global variable to store the turtle's current pose
current_pose = Pose()

# Obstacle position and radius (for a circular obstacle)
obstacle_x = 5.5
obstacle_y = 5.5
obstacle_radius = 1.0  # radius around the obstacle to avoid

# Callback function to update the turtle's pose
def pose_callback(data):
    global current_pose
    current_pose = data

# Function to check if the turtle is too close to the obstacle
def is_near_obstacle():
    distance_to_obstacle = math.sqrt((current_pose.x - obstacle_x) ** 2 + (current_pose.y - obstacle_y) ** 2)
    return distance_to_obstacle < obstacle_radius

# Function to move the turtle to a specific position, avoiding the obstacle
def move_to_goal(x_goal, y_goal):
    global current_pose

    # Create a publisher to send velocity commands
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    # Define the rate of the loop
    rate = rospy.Rate(10)  # 10 Hz

    # Proportional control gains
    K_linear = 1.5
    K_angular = 4.0

    # Twist message to store linear and angular velocity
    vel_msg = Twist()

    while not rospy.is_shutdown():
        # Compute the distance between current pose and goal
        distance_to_goal = math.sqrt((x_goal - current_pose.x) ** 2 + (y_goal - current_pose.y) ** 2)

        # Compute the desired angle to the goal
        desired_angle_goal = math.atan2(y_goal - current_pose.y, x_goal - current_pose.x)

        # Check if the turtle is near the obstacle
        if is_near_obstacle():
            rospy.loginfo("Near obstacle! Avoiding...")
            
            # Strategy: turn away from the obstacle and continue moving
            # Turn to move away from the obstacle by adjusting the desired angle
            angle_away_from_obstacle = math.atan2(current_pose.y - obstacle_y, current_pose.x - obstacle_x)
            angular_speed = K_angular * (angle_away_from_obstacle - current_pose.theta)
            
            # Move forward a bit to get away from the obstacle
            vel_msg.linear.x = K_linear * 0.5  # Slow down when avoiding obstacles
            vel_msg.angular.z = angular_speed
        else:
            # If far from obstacle, move towards the goal
            rospy.loginfo("Heading to goal...")

            # Linear velocity proportional to the distance to the goal
            linear_speed = K_linear * distance_to_goal

            # Angular velocity proportional to the angular error to the goal
            angular_speed = K_angular * (desired_angle_goal - current_pose.theta)

            # Set the velocities
            vel_msg.linear.x = linear_speed
            vel_msg.angular.z = angular_speed

        # Stop if we are close enough to the goal
        if distance_to_goal < 0.01:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            rospy.loginfo("Goal reached")
            break

        # Publish the velocity command
        velocity_publisher.publish(vel_msg)

        # Sleep for the defined loop rate
        rate.sleep()

if __name__ == "__main__":
    try:
        # Initialize the ROS node
        rospy.init_node('turtle_mover_with_obstacle_avoidance', anonymous=True)

        # Subscribe to the turtle's pose
        rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

        # Ask the user for the goal position
        x_goal = float(input("Enter x goal: "))
        y_goal = float(input("Enter y goal: "))

        # Call the function to move the turtle to the goal
        move_to_goal(x_goal, y_goal)

    except rospy.ROSInterruptException:
        pass
