# husky-gps-service

## test_generate_waypoints

Here are the steps you can follow to run the test file:

1. Start the ROS master:

```shell
$ roscore
```

2. Start the waypoint_generator_node that provides the generate_waypoints service:

``` shell
$ rosrun husky_gps_service waypoint_generator_node
```

3. Open a new terminal and navigate to the directory that contains the test file.

4. Make the test file executable:

```shell
$ chmod +x test_waypoint_generator.py
```

3. Run the test file:

```shell
$ ./test_waypoint_generator.py
```

This should call the generate_waypoints service with the predefined GPS goals and print the list of generated waypoints.