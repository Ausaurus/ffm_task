# FIND MY MATES TASK
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

## The purpose of every code
1. start_scripts.py
    * control when to start searching for people and go to waypoint
2. read_waypoints.py
    * reads and go to the waypoint according to the waypoints.txt that is
      created when running **append_waypoints.py**
    * can only run when **move_base** service is available, in other words, you
      need to run your navigation package when starting this node but this is
      already taken care of in **start_fz.launch** when you start the code
3. resumable_rotation_fz.py
    * this controls the execution between rotation and feature scanning
    * and automatically stops and send a service call to start_scripts to let it
      know the rotation operation has finished and go to the waypoint
4. aligned_depth_rgb.py aligned_depth_rgb_pyzed.py
    * gets the image and depth data, sends to _/zed2/zed_node/rgb_raw/image_raw_color_
      and _/zed2/zed_node/depth/depth_registered_ respectively
5. yolo_detection.py
    * detects whether a person is in the centre of the frame of the camera
    * sends to _/person_centered_ topic to let **resumable_rotation_fz** a new
      person has been detected and that feature scanning can start
6. take_photo.py
    * this code is redundant as photo is already taken by **characteristic.py**
      whenever feature scanning starts
    * it is only used for simplying the operation of feature scanning when I was
      just trying to make sure that the general flow of go to waypoint and
      rotation is correct
