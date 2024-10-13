#!/usr/bin/env python
import rospy
from turtlesim.srv import Spawn, SpawnRequest, SpawnResponse

def spawn_turtle(x, y, theta, name):
    rospy.wait_for_service('/spawn')

    try:
        # Create a service proxy to interact with the /spawn service
        spawn_turtle_service = rospy.ServiceProxy('/spawn', Spawn)

        spawn_request = SpawnRequest()
        spawn_request.x = x
        spawn_request.y = y
        spawn_request.theta = theta
        spawn_request.name = name

        # Call the service with the request
        response = spawn_turtle_service(spawn_request)

        rospy.loginfo(f"Turtle spawned with name: {response.name}")
        return response.name

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None


if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('turtle_spawner')

    # Call the function to spawn a new turtle at specific coordinates
    spawn_turtle(5.0, 5.0, 0.0, "new_turtle")
