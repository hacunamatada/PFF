#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/LaserScan.h>
#define PI 3.14159265

 
ros::Publisher scan_pub;
float robotheight, sensorheight, resolution;
int visionangle;

/**
 * This function recieve the Velodyne PointCloud. 
 * Dicard points with a vertical filter
 * Discar points with a horizontal filter
 * Detect stairs with the points with height less than the floor height
 * Project the points into 2D plane
 * Extract the nearest point for each angle
 * Convert to LaserScan
 * Publish
 *
 * @param input Velodyne PointCloud directly from the sensor
 */

void 
cloud_cb2 (const sensor_msgs::PointCloud2ConstPtr& input)
{ 
	pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
	pcl::fromROSMsg(*input, laserCloudIn);

	//Laser Transform variables
	unsigned int num_readings = 360/resolution;
  	double laser_frequency = 600;
  	double ranges[num_readings]={0};
  	double intensities[num_readings]={0};
	int addpos=0;
	int position=0;	

	for (int i = 0; i < laserCloudIn.size(); i++) {
		
		float hip = sqrt((laserCloudIn[i].x)*(laserCloudIn[i].x)+((laserCloudIn[i].y)*(laserCloudIn[i].y)));
		float hangle = (asin ((laserCloudIn[i].x)/hip))*180/PI;

		// Vertical filter : Points with a height greater than the height of the robot are discarded
		if (laserCloudIn[i].z > robotheight-sensorheight || (-sensorheight-0.1<=laserCloudIn[i].z && laserCloudIn[i].z < -sensorheight + 0.05)) {continue;}

		// Dicard points outside the area of interest (Horizontal Filter)
		if (visionangle==180 && laserCloudIn[i].x<=0){continue;}
		if ((visionangle<180 && laserCloudIn[i].x>0 && hangle<90-(visionangle/2)) || (visionangle<180 && laserCloudIn[i].x<=0)){continue;}
		if (visionangle>180 && visionangle<360 && laserCloudIn[i].x<0 && abs(hangle)>(visionangle-180)/2){continue;}

		//Stairs detection
		if (laserCloudIn[i].z < -sensorheight-0.1){
			float hip2 = sqrt(pow(hip,2)+pow(laserCloudIn[i].z,2));
			float hangle2= asin((laserCloudIn[i].z/hip2)*sin(1.5708))*180/PI;
			if (hangle2<-14)
			{
				hip=0.22;
				laserCloudIn[i].x = hip * sin(hangle*PI/180);
				if (laserCloudIn[i].y>0)
				{
					laserCloudIn[i].y = sqrt(pow(hip,2)-pow(laserCloudIn[i].x,2));
				}
				else
				{
					laserCloudIn[i].y = -sqrt(pow(hip,2)-pow(laserCloudIn[i].x,2));
				}
			}
			if (hangle2>-14)
			{
				//ROS_INFO("angle1 %f, %f, %f, %f, %f, %f",hangle2, hangle,laserCloudIn[i].x, laserCloudIn[i].y, hip, hip2);
				hip2=sensorheight*sin(PI/2)/sin((2-hangle2)*PI/180);
				hip = sqrt(pow(hip2,2)-pow(sensorheight,2));
				laserCloudIn[i].x = hip * sin (hangle*PI/180);
				if (laserCloudIn[i].y>0)
				{
					laserCloudIn[i].y = sqrt(pow(hip,2)-pow(laserCloudIn[i].x,2));
				}
				else
				{
					laserCloudIn[i].y = -sqrt(pow(hip,2)-pow(laserCloudIn[i].x,2));
				}
				//ROS_INFO("angle2 %f, %f, %f, %f, %f, %f",hangle2, hangle,laserCloudIn[i].x, laserCloudIn[i].y, hip, hip2);
				
			}
		}

		// Get the position in the arrays for get the nearest points
		if (laserCloudIn[i].y>0) {
			position=(180.0+hangle)/resolution;
		}
		else if (laserCloudIn[i].x>0 && laserCloudIn[i].y<=0) {
			position=(360.0-hangle)/resolution;
		}
		else {
			position=-hangle/resolution;			
		}	

		if (ranges[position]==0 || hip < ranges[position])
		{
			ranges[position]=hip;
			intensities[position]=0;
		}					
	}
	
	// Configuration of parameters for conversion to LaserScan
	ros::Time scan_time = ros::Time::now();
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "base_link";
    scan.angle_min = 3*PI/2;
    scan.angle_max = -PI/2;
    scan.angle_increment = -2*PI / num_readings;
    scan.time_increment = (1 / laser_frequency) / (num_readings);
    scan.range_min = 0.2;
    scan.range_max = 50.0;
    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);

	// assign values to the vector of ranges and intensities to publish the LaserScan
    for(unsigned int i = 0; i < num_readings; ++i){
      scan.ranges[i] = ranges[i];
      scan.intensities[i] = intensities[i];
    }  

	// Publishing of the LaserScan topic
    scan_pub.publish(scan);
}


int
main (int argc, char **argv)
{
	// Initialize ROS
	ros::init (argc, argv, "velodyne_pff");
	ros::NodeHandle nh;

	// Get launch parameters
	if(!nh.getParam("/velodyne_pff/robot_height",robotheight)){robotheight = 0.80;}
	if(!nh.getParam("/velodyne_pff/sensor_height",sensorheight)){sensorheight = 0.60;}
	if(!nh.getParam("/velodyne_pff/horizontal_fov",visionangle)){visionangle = 360;}
	if(!nh.getParam("/velodyne_pff/resolution",resolution)){resolution = 0.4;}

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/velodyne_points", 50, cloud_cb2);
	
	// Create a ROS publisher for the output LaserScan
	scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 1);
	
	// Spin
	ros::spin ();
}
