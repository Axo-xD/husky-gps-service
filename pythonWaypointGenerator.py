#!/usr/bin/env pythonWaypointGenerator

import rospy
from your_package.msg import GPSGoalList
from your_package.srv import GenerateWaypoints, GenerateWaypointsResponse
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPointStamped
from geographic_msgs.srv import GetGeoPath
from std_msgs.msg import Float64

class WaypointGenerator:
    def __init__(self):
        rospy.init_node('waypoint_generator_node')

        # Wait for the required services to become available
        rospy.wait_for_service('/get_geo_path')
        rospy.wait_for_service('/projected_map_node/costmap/costmap/cell_size')

        # Get the cell size of the costmap
        cell_size_proxy = rospy.ServiceProxy('/projected_map_node/costmap/costmap/cell_size', Float64)
        self.cell_size = cell_size_proxy().data

        # Create a service for generating waypoints
        self.waypoint_service = rospy.Service('/generate_waypoints', GenerateWaypoints, self.generate_waypoints)

    def generate_waypoints(self, req):
        # Call the /get_geo_path service to get a path between the GPS goals
        geo_path_service = rospy.ServiceProxy('/get_geo_path', GetGeoPath)
        geo_path = geo_path_service(req.goals).path

        # Calculate the number of passes required to cover the work area with 1m between each pass
        num_passes = int(round(geo_path.length / self.cell_size))

        # Calculate the distance between each pass
        pass_distance = geo_path.length / num_passes

        # Create a list of waypoints for each pass
        waypoints = []
        for i in range(num_passes):
            # Calculate the offset from the start of the path
            offset = i * pass_distance

            # Get the point on the path at the offset
            path_point = geo_path.interpolate(offset)

            # Calculate the heading of the path at the point
            heading = geo_path.get_heading_at_offset(offset)

            # Calculate the waypoints for the pass
            pass_waypoints = self.calculate_pass_waypoints(path_point, heading)

            # Add the pass waypoints to the list of waypoints
            waypoints.extend(pass_waypoints)

        # Save the waypoints to a file
        with open('waypoints.txt', 'w') as f:
            for waypoint in waypoints:
                f.write('{},{},{}\n'.format(waypoint.latitude, waypoint.longitude, waypoint.altitude))

        # Return the list of waypoints as a response to the service request
        return GenerateWaypointsResponse(waypoints)

    def calculate_pass_waypoints(self, path_point, heading):
        # Calculate the points for one pass
        pass_width = 1.0
        pass_spacing = 1.0
        pass_heading = heading + 90.0
        pass_start_point = self.offset_point(path_point, pass_heading, pass_width / 2)
        pass_end_point = self.offset_point(path_point, pass_heading, -(pass_width / 2))

        # Calculate the distance between pass waypoints
        num_waypoints = int(round((pass_width / pass_spacing) + 1))
        distance_between_waypoints = pass_spacing / num_waypoints

        # Create a list of waypoints for the pass
        pass_waypoints = []
        for i in range(num_waypoints):
            # Calculate the offset from the start of the pass
            offset = i * distance_between_waypoints

            # Get the point on the pass at the offset
            pass_point = self.offset_point(pass_start_point, heading, offset)

            # Add the pass point to the list of waypoints
            pass_waypoints.append(pass_point)

        return pass_waypoints

    def offset_point(self, point, heading, distance):
        # Convert the point to a GeoPointStamped message
        geo_point = GeoPointStamped()
        geo_point.header.stamp = rospy.Time.now()
        geo_point.header.frame_id = 'map'
        geo_point.position = point

        # Convert the heading and distance to a geometry_msgs/Twist message
        twist = NavSatFix()
        twist.header.stamp = rospy.Time.now()
        twist.header.frame_id = 'map'
        twist.latitude = distance
        twist.longitude = 0.0
        twist.altitude = 0.0
        twist.position_covariance[0] = 1000.0
        twist.position_covariance[4] = 1000.0
        twist.position_covariance[8] = 1000.0

        # Rotate the twist message by the heading
        cos_heading = math.cos(math.radians(heading))
        sin_heading = math.sin(math.radians(heading))
        twist.latitude = cos_heading * twist.latitude - sin_heading * twist.longitude
        twist.longitude = sin_heading * twist.latitude + cos_heading * twist.longitude

        # Project the twisted point onto the map
        projected_point = rospy.wait_for_message('/projected_map_node/costmap/cell', NavSatFix)
        return GeoPointStamped(projected_point.header, projected_point.latitude + twist.latitude, projected_point.longitude + twist.longitude, projected_point.altitude)