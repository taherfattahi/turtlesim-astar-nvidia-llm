#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

current_pose = Pose()

def pose_callback(data):
    global current_pose
    current_pose = data

# Function to move the turtle to a specific position
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
        distance = math.sqrt((x_goal - current_pose.x) ** 2 + (y_goal - current_pose.y) ** 2)

        # Linear velocity proportional to the distance
        linear_speed = K_linear * distance

        # Compute the desired angle to the goal
        desired_angle_goal = math.atan2(y_goal - current_pose.y, x_goal - current_pose.x)
        angular_speed = K_angular * (desired_angle_goal - current_pose.theta)

        # Stop if we are close enough to the goal
        if distance < 0.01:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            rospy.loginfo("Goal reached")
            break

        # Set the velocities
        vel_msg.linear.x = linear_speed
        vel_msg.angular.z = angular_speed

        # Publish the velocity
        velocity_publisher.publish(vel_msg)

        # Sleep for the defined loop rate
        rate.sleep()

if __name__ == "__main__":
    try:
        # Initialize the ROS node
        rospy.init_node('turtle_mover', anonymous=True)

        # Subscribe to the turtle's pose
        rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

        # Ask the user for the goal position
        x_goal = float(input("Enter x goal: "))
        y_goal = float(input("Enter y goal: "))

        # Call the function to move the turtle to the goal
        move_to_goal(x_goal, y_goal)

    except rospy.ROSInterruptException:
        pass
