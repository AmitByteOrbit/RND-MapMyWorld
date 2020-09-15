# RND-Where Am I

## Project submission

### File checklist
All files submitted.
There are two launch files that need to my run under the `my_robot` package.
First run `roslaunch my_robot world.launch` then `roslaunch my_robot amcl.launch`

<img src="/images/directory_tree.png" width="300">

### Screenshots in Rviz
<p align="center"><img src="/images/Rviz1.png" width="800"></p>
<p align="center"><img src="/images/Rviz2.png" width="800"></p>
<p align="center"><img src="/images/Rviz3.png" width="800"></p>

### The Robot loaded in Gazebo
The robot from the previous project was utilized here. I did have to adjust the world so that the origin (0,0) was not on anything.
<p align="center"><img src="/images/robot_gz.png" width="600"></p>

### Landmarks and settings
Generated the PGM file and associated the corrected messages for localisation.
<p align="center"><img src="/images/map.png" width="600"></p>
<p align="center"><img src="/images/Rviz-tools.png" width="300"></p>


### Launch File Setup

I've setup my launch file to include a few of the additional laser settings. I've also elected to rather launch my RViz package from here along with the saved Rviz config. Here is the AMCL launch file.

``` XML
<?xml version="1.0" encoding="UTF-8"?>
<launch>

  <arg name="map_file" default="$(find my_robot)/maps/map.yaml"/>

  <!-- Run the map server -->
  <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />
  
  <!-- AMCL node -->
  <node pkg="amcl" type="amcl" name="amcl" output="screen">
  	<remap from="scan" to="my_robot/laser/scan"/>

	<!-- Overall Filter -->
	<param name="min_particles" value="50"/>
	<param name="max_particles" value="200"/>
	<param name="update_min_a" value="0.1"/>
	<param name="update_min_d" value="0.25"/>

  	<param name="initial_pose_x" value="0.0"/>
    	<param name="initial_pose_y" value="0.0"/>
	<param name="initial_pose_a" value="-1.57"/>

	<!-- Laser -->
	<param name="laser_max_beams" value="20"/>
	<param name="laser_z_rand" value="0.05"/>
	<param name="laser_z_hit" value="0.95"/>

	<!-- Odometry configs as per project outline -->
  	<param name="odom_frame_id" value="odom"/>
	<param name="odom_model_type" value="diff-corrected"/>
	<param name="base_frame_id" value="robot_footprint"/>
	<param name="global_frame_id" value="map"/>
  

    
  </node>

  <!-- Move_Base node  -->
  <node pkg="move_base" type="move_base" name="move_base" respawn="false" output="screen" >

	<remap from="scan" to="my_robot/laser/scan"/>  

	<param name="base_global_planner" value="navfn/NavfnROS" />
	<param name="base_local_planner" value="base_local_planner/TrajectoryPlannerROS"/>
	
	<rosparam file="$(find my_robot)/config/costmap_common_params.yaml" command="load" ns="global_costmap" />
	<rosparam file="$(find my_robot)/config/costmap_common_params.yaml" command="load" ns="local_costmap" />
	<rosparam file="$(find my_robot)/config/local_costmap_params.yaml" command="load" />
	<rosparam file="$(find my_robot)/config/global_costmap_params.yaml" command="load" />
	<rosparam file="$(find my_robot)/config/base_local_planner_params.yaml" command="load" />
  </node> 
  
  <!--launch rviz -->
  <node name="rviz" pkg="rviz" type="rviz" respawn="false" args="-d $(find my_robot)/launch/project.rviz"/>
  
</launch>
```

### Config file updates
In the `base_local_planner_params.yaml` I changed the min and max velocities as well as the pdist and gdist weightings to get more accuracy with the path planner.
```
max_vel_x: 0.6 #0.5
min_vel_x: 0.01 #0.01
...
pdist_scale: 2.5  #0.6
gdist_scale: 2.5  #0.8
occdist_scale: 0.1 #0.02
```
In the `costmap_common_params.yaml` I played with the laser scanning distance, the radius of the robot and the inflation radius for guidance, and obstacle avoidance.
```
obstacle_range: 2.5 # 2.0
raytrace_range: 3.0 # 3.0

transform_tolerance: 0.4 # 0.0

robot_radius: 0.5 # 0.0
inflation_radius: 0.4 # 0.0
```
In the `global_costmap_params.yaml` I changed the update and publish frequencies for better performance and to avoid the "missed loop" error.
```
update_frequency: 5.0
publish_frequency: 2.0
```
   
Lastly in the `local_costmap_params.yaml` I changes the update frequencies and the map size to optimize the short-term planning for the robot.
```
update_frequency: 5.0
publish_frequency: 2.0
width: 6.0
height: 6.0
```

### Obstacle avoidance
Utilizing good configurations for `radius, inflation radius, pdist and gdist` the robot avoid obstacles.
<p align="center"><img src="/images/avoid2.png" width="600"></p>


### It works!!! :)
![Robot](/images/demo.gif)


###References
ROS Wiki</br>
http://wiki.ros.org/amcl</br>
http://wiki.ros.org/navigation/Tutorials/RobotSetup</br>
http://wiki.ros.org/base_local_planner</br>


