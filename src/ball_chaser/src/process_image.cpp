#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

//#include "pcl_ros/point_cloud.h"

//#include <pcl/search/impl/search.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_clusters.h>


// #include <pcl/filters/extract_indices.h>
// #include <pcl/PointIndices.h>
// #include <pcl/io/ply_io.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/search/kdtree.h>

// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/radius_outlier_removal.h>


//#include <pcl/point_cloud.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{

    //ROS_INFO("Sending commands to the robot: linear_x:%1.2f, angular_z:%1.2f", lin_x, ang_z);

    ball_chaser::DriveToTarget srv;
    srv.request.angular_z = ang_z;
    srv.request.linear_x = lin_x;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    int pixel_pos = -1;
    
    for (int i = 0; i < img.height * img.step; i++) {
    	if (img.data[i] == white_pixel) {
    		pixel_pos = i % img.step;
    		break;
    	}
    }
    
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

void process_pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	//ROS_INFO("In point cloud callback");
	/**
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::fromROSMsg (*cloud_msg, *cloud);
	
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::SACSegmentationFromNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> segmentation;
	
	segmentation.setInputCloud(cloud);
	segmentation.setInputNormals(cloud);
	segmentation.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
	segmentation.setMethodType(pcl::SAC_RANSAC);
	segmentation.setDistanceThreshold(1.25);
	segmentation.setNormalDistanceWeight(0.1);
	segmentation.setOptimizeCoefficients(true);
	segmentation.setRadiusLimits(20,21);
	segmentation.setEpsAngle(1 / (180/3.141592654));
	segmentation.setMaxIterations(10000);
	
	pcl::PointIndices::Ptr indx(new pcl::PointIndices());
	segmentation.segment(*indx, *coefficients);	
	
	if (indx->indices.size() == 0)
     std::cout << ". RANSAC nothing found" << "\n";
    else
    {
      std::cout << ". RANSAC found shape with [%d] points:" << indx->indices.size() << "\n";
      double part = (double)indx->indices.size() / (double) cloud->points.size() * 100;
      std::cout << ". Model coefficient: " << *coefficients << std::endl << "Points: " << indx->indices.size() << " ("<< part <<"%)" << std::endl;
    }
    **/
    
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
	seg.setDistanceThreshold (0.1);

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
    }

}




int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
    
    // Create a ROS subscriber for the input point cloud
	ros::Subscriber sub2 = n.subscribe ("/camera/depth/points", 10, process_pointcloud_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
