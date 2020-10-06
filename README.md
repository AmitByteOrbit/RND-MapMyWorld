# RND-Map My World

## Project submission

### File list
There are two launch files that need to my run under the `my_robot` package.
First run `roslaunch my_robot world.launch` then `roslaunch my_robot mapping.launch`
Lastly run `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` to launch the teleop interface.

Please note if you are running the `nouveau` graphics driver you will have to change to lines on the gazebo file as follows:
Open `my_robot.gazebo` located in `src/my_robot/urdf/`
Under `hokuyo` change `<sensor type="gpu_ray" name="head_hokuyo_sensor">` to `<sensor type="ray" name="head_hokuyo_sensor">` and <br>
` <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_gpu_laser.so">` to `<plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_laser.so">`

Link to DB file (850MB): https://drive.google.com/file/d/1SJG9JY3AdsrXNtcBuzyctamy2Mqxkkr8/view?usp=sharing

### Features
Initially there was a lack of unique features to gain sufficient looop closures so I introduced a bunch of random models into the world which made a substantial difference.

<p align="center"><img src="/images/my_world.png" width="800"></p>

### Launch Files
As instructed in the assignment I have created two launch files. `mapping.launch` and `localization.launch`. The teleop package is launched using the rosrun command as indicated in my intro. Here is a view of the mapping launch file:

``` XML
<?xml version="1.0" encoding="UTF-8"?>

<launch>
  <!-- Arguments for launch file with defaults provided -->
  <arg name="database_path"     default="rtabmap.db"/>
  <arg name="rgb_topic"   default="/camera/rgb/image_raw"/>
  <arg name="depth_topic" default="/camera/depth/image_raw"/>
  <arg name="camera_info_topic" default="/camera/rgb/camera_info"/>  


  <!-- Mapping Node -->
  <group ns="rtabmap">
    <node name="rtabmap" pkg="rtabmap_ros" type="rtabmap" output="screen" args="--delete_db_on_start">

      <!-- Basic RTAB-Map Parameters -->
      <param name="database_path"       type="string" value="$(arg database_path)"/>
      <param name="frame_id"            type="string" value="robot_footprint"/>
      <param name="odom_frame_id"       type="string" value="odom"/>
      <param name="subscribe_depth"     type="bool"   value="true"/>
      <param name="subscribe_scan"      type="bool"   value="true"/>
	
		<param name="Grid/Range"            type="string" value="10"/>


      <!-- RTAB-Map Inputs -->
      <remap from="scan" to="/my_robot/laser/scan"/>
      <remap from="rgb/image" to="$(arg rgb_topic)"/>
      <remap from="depth/image" to="$(arg depth_topic)"/>
      <remap from="rgb/camera_info" to="$(arg camera_info_topic)"/>

      <!-- RTAB-Map Output -->
      <remap from="grid_map" to="/map"/>

      <!-- Rate (Hz) at which new nodes are added to map -->
      <param name="Rtabmap/DetectionRate" type="string" value="3"/>

      <!-- 2D SLAM -->
      <param name="Reg/Force3DoF" type="string" value="true"/>

      <!-- Loop Closure Detection -->
      <!-- 0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE -->
      <param name="Kp/DetectorStrategy" type="string" value="0"/>

      <!-- Maximum visual words per image (bag-of-words) -->
      <param name="Kp/MaxFeatures" type="string" value="600"/>

      <!-- Used to extract more or less SURF features -->
      <param name="SURF/HessianThreshold" type="string" value="80"/>

      <!-- Loop Closure Constraint -->
      <!-- 0=Visual, 1=ICP (1 requires scan)-->
      <param name="Reg/Strategy" type="string" value="1"/>

      <!-- Minimum visual inliers to accept loop closure -->
      <param name="Vis/MinInliers" type="string" value="10"/>

      <!-- Set to false to avoid saving data when robot is not moving -->
      <param name="Mem/NotLinkedNodesKept" type="string" value="false"/>

	  <!-- Custom parameters -->
	  <param name="GFTT/MinDistance" type="string" value="10"/>
	  <param name="Optimizer/Slam2D" value="true" />
	  <param name="proj_max_ground_angle" value="45"/>
	  <param name="proj_max_ground_height" value="0.1"/>
	  <param name="proj_max_height" value="2.0"/>
      <param name="proj_min_cluster_size" value="20"/>

    </node>

	<!-- visualization with rtabmapviz 
    <node pkg="rtabmap_ros" type="rtabmapviz" name="rtabmapviz" args="-d $(find rtabmap_ros)/launch/config/rgbd_gui.ini" output="screen">
        <param name="subscribe_depth"             type="bool" value="true"/>
        <param name="subscribe_scan"              type="bool" value="true"/>
        <param name="frame_id"                    type="string" value="robot_footprint"/>

        <remap from="rgb/image"       to="$(arg rgb_topic)"/>
        <remap from="depth/image"     to="$(arg depth_topic)"/>
        <remap from="rgb/camera_info" to="$(arg camera_info_topic)"/>
        <remap from="scan"            to="/my_robot/laser/scan"/>
		<remap from="odom"            to="/odom"/>
    </node> -->
  </group>
</launch>
```




### Loop Closures
I managed to achieve 84 loop closures. Here is a screenshot of the graph view:

<p align="center"><img src="/images/Graph_view.png" width="800"></p>

The occupency grid map was also generated fairly accurately:

<p align="center"><img src="/images/occupancy_grid_map.png" width="800"></p>

Lastly out of curiosity I genderated the 3D render and after a bit of google time figured out I could use MeshLab to view the 3d file. The results were pretty impressive:

<p align="center"><img src="/images/meshlab_rend.png" width="800"></p>


###References </br>
http://wiki.ros.org/rtabmap_ros/Tutorials/Advanced%20Parameter%20Tuning</br>
https://dabit-industries.github.io/turtlebot2-tutorials/07b-RTABMAP.html</br>



