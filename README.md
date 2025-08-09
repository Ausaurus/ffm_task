## other packages that need to be downloaded
1. map_search
    * necessary for its waypoint service file
    * start_scripts.py and read_waypoints depends on it
    * it helps specifying which room number to go to and returns a true boolean
    value whenever timeout or the waypoint is reached

## How to run the code
1. roslaunch ffm_task start_fz.launch
right after launching the robot will start going to the first waypoint

## How to add waypoints
1. rosrun map_server map_server <your_map_path>
2. rviz
    * open the map topic in rviz
3. rosrun ffm_task append_waypoints.py
4. seleck the publish point tool at the top of rviz window
5. choose the points where you want to search for people
6. after choosing all the necessary points end everything
7. launch start_fz
