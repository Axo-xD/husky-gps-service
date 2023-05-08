# Service for Husky

## gps_user_service

This service communicates with the web app to save data points from user.

It saves the data to root folder. In a txt file where each line is a gps poins `lan,lat`.

Use following comand to start service:
```
rosrun gps_user_service gps_user_service_server.py
```

Can be tested by using test client when service are running:
```
rosrun gps_user_service gps_user_service_client.py
```

### Message type 

Uses SaveGPS.srv to defind request and response message.

```
# Request
sensor_msgs/NavSatFix[] gps_coordinates
---
# Response
bool success
```
