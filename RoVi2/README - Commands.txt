for rosbag record:
----------------------------------------
Note - each command to be run in a seperate terminal

1. start "roslaunch openni2_launch openni2.launch "
2. run "rosbag record [topic_name] -o [name_of_file].bag" where [name_of_file] is the name of the file, and [topic_name] is the name of the topic to be recorded


for rosbag play:
----------------------------------------
Note - each command to be run in a seperate terminal

1. run roscore 
2. run command "rosrun tf static_transform_publisher 0 0 0 0 0 0 map camera_depth_optical_frame 100"
3. run "rosbag play --clock -l [name_of_file].bag" where [name_of_file] is the name of the file - duh
4. start rviz

for pose of marker:
----------------------------------------
Note - each command to be run in a seperate terminal

1. run "roslaunch jsk_pcl_ros openni2_local.launch"
2. run "roslaunch jsk_pcl_ros openni2_remote.launch"
3. run "roslaunch jsk_pcl_ros depth_error.launch"
4. run "rostopic echo /checkerboarddetector/objectdetection_pose" 