#!/usr/bin/env python

import rospy
from husky_gps_service.srv import GenerateWaypoints, GPSGoal, GPSGoalList
from sensor_msgs.msg import NavSatFix

def main():
    rospy.init_node('waypoint_generator_test_node')

    # Wait for the generate_waypoints service to become available
    rospy.wait_for_service('/generate_waypoints')
    generate_waypoints = rospy.ServiceProxy('/generate_waypoints', GenerateWaypoints)

    # Define a list of GPS goals
    goals = GPSGoalList()
    goals.goals = [
        GPSGoal(NavSatFix(latitude=37.7749, longitude=-122.4194), 1.0),
        GPSGoal(NavSatFix(latitude=37.7758, longitude=-122.4184), 1.0),
        GPSGoal(NavSatFix(latitude=37.7767, longitude=-122.4174), 1.0)
    ]

    # Call the generate_waypoints service with the GPS goals
    response = generate_waypoints(goals)

    # Print the list of generated waypoints
    print(response.waypoints)

if __name__ == '__main__':
    main()