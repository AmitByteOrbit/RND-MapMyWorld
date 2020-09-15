# RND-Where Am I

## Project submission

### File checklist
All files submitted.

![Directory Tree](/images/directory_tree.png | WIDTH=200) 

### Localized Robot In RVIZ


### Robot Design
![Robot](/images/robot.png)
#### Lidar and camera sensors.
I have used the Hokuyo Lidar with the Kinect Camera. I wanted to use point clouds for the ball detection but ran out of time. Luckily the camera worked well :)

#### Gazebo plugins for the robot’s differential drive, lidar, and camera.
I have used the `skid_steer_drive_controller` with four wheels.

#### Housed inside the world
I have used the world from my previous project. 

#### Significant changes from the sample taught in the project lesson.
I think it is much different from the sample taught in the project lesson :)

#### Robot is stable when moving
yes

### Gazebo World
The same as from my previous project with the white ball insterted. See picture below:
![World](/images/world.png)

### Ball Chasing
The following criteria is met by the code below:

* A ball_chaser/command_robot service.
* Service accepts linear x and angular z velocities.
* Service publishes to the the wheel joints.
* Service returns the requested velocities.

```C++
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
    ball_chaser::DriveToTarget::Response& res)
{

//    ROS_INFO("DriveToTarget received - linear_x:%1.2f, angular_z:%1.2f", (float)req.linear_x, (float)req.angular_z);

    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;
    // Set wheel velocities, forward
    motor_command.linear.x = req.linear_x;
    // Set angles to drive the robot
    motor_command.angular.z = req.angular_z;
    
    // Publish angle and velcoity to drive the robot
    motor_command_publisher.publish(motor_command);

    // Return a response message
    res.msg_feedback = "Drive velocities set - linear_x: " + std::to_string(req.linear_x) + " , angular_z: " + std::to_string(req.angular_z);
//    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}
```

The following criteria has been met by the `process_image` class:

Subscribes to the robot’s camera image.
A function to analyze the image and determine the presence and position of a white ball.
Requests a service to drive the robot towards a white ball (when present).

```C++

/ This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    int pixel_pos = -1;
    
    // Iterate throught the pixels looking for perfect white.
    for (int i = 0; i < img.height * img.step; i++) {
    	if (img.data[i] == white_pixel) {
    		pixel_pos = i % img.step;
    		break;
    	}
    }
    
    // Change angle or velocity as required. Dividing the width into three parts and using 45 degree turning angles.
    if (pixel_pos >= 0) {
    	if (pixel_pos < img.step/3) {
    		//go left
    		ROS_INFO("Moving left with pix pos: %d of %d", pixel_pos, img.step);
    		drive_robot(0.0, 0.78);
    	} else if (pixel_pos > 2*img.step/3) {
    		//go right
    		ROS_INFO("Moving right with pix pos: %d of %d", pixel_pos, img.step);
    		drive_robot(0.0, -0.78);
    	} else {
    		// go straight
    		drive_robot(0.5, 0.0);
    	}
    	
    } else {
    // stop moving
    drive_robot(0.0, 0.0);
    }
     
}
```

I also tried to get the point clouds operational from the Kinect camera. This was in an effort to detect any ball color. I came close but ran out of time. In the snippet below I get the coefficients but unfortunately need a bit more time to translate them into angular movements. (I have commented this out in the code and also the required PCL dependencies in the CMAKE file to make it easier to compile my code without installing PCL).

```C++
/ Find ball of any color using point clouds -- work in progress
void process_pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	    
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg (*cloud_msg, cloud);

	pcl::ModelCoefficients coefficients;
	pcl::PointIndices inliers;
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_SPHERE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (cloud.makeShared ());
	seg.segment (inliers, coefficients);

	// Publish the model coefficients
	pcl_msgs::ModelCoefficients ros_coefficients;
	pcl_conversions::fromPCL(coefficients, ros_coefficients);
	
	pcl::PointIndices::Ptr indx(new pcl::PointIndices());
	seg.segment(*indx, coefficients);	
	
	if (indx->indices.size() == 0)
     std::cout << ". RANSAC nothing found" << "\n";
    else
    {
      std::cout << ". RANSAC found shape with [%d] points:" << indx->indices.size() << "\n";
      std::cout << "Coefficients: " << ros_coefficients << "\n";
    }

}
```

### Launch Files
`world.launch`
``` XML
<?xml version="1.0" encoding="UTF-8"?>

<launch>

  <!-- Robot pose -->
  <arg name="x" default="4.8"/>
  <arg name="y" default="2.2"/>
  <arg name="z" default="0.1"/>
  <arg name="roll" default="0"/>
  <arg name="pitch" default="0"/>
  <arg name="yaw" default="0"/>

  <!-- Launch other relevant files-->
  <include file="$(find my_robot)/launch/robot_description.launch"/>

  <!-- World File -->
  <arg name="world_file" default="$(find my_robot)/worlds/my.world"/>

  <!-- Launch Gazebo World -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="use_sim_time" value="true"/>
    <arg name="debug" value="false"/>
    <arg name="gui" value="true" />
    <arg name="world_name" value="$(arg world_file)"/>
  </include>
  
  
  <!-- Find my robot Description-->
  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find my_robot)/urdf/my_robot.xacro'"/>

  <!-- Spawn My Robot -->
  <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen" 
        args="-urdf -param robot_description -model my_robot 
              -x $(arg x) -y $(arg y) -z $(arg z)
              -R $(arg roll) -P $(arg pitch) -Y $(arg yaw)"/>
  
  <!--launch rviz  -->
  <node name="rviz" pkg="rviz" type="rviz" respawn="false"/>

 
  
</launch>
```

`ball_chaser.launch`
``` XML
<launch>

 <!-- The drive_bot node -->
  <node name="drive_bot" type="drive_bot" pkg="ball_chaser" output="screen">
  </node>
  
  <node name="process_image" type="process_image" pkg="ball_chaser" output="screen">
  </node>

</launch>
```

## It works!!! :)
![Robot](/images/robot_demo.gif)



