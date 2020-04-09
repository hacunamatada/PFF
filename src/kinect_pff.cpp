#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/LaserScan.h>
#define PI 3.14159265

 
ros::Publisher scan_pub,laserCloud_pub;
float robotheight, sensorheight, resolution;

//Struct to access for each point cordenates
struct Point3D {

  float x;
  float y;
  float z;
  Point3D(float x_, float y_, float z_)
    : x(x_), y(y_), z(z_) {

  }
};

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
 * @param input Astra Camera PointCloud directly from the sensor
 */

void 
cloud_cb2 (const sensor_msgs::PointCloud2ConstPtr& input) 
{

  int numPts = input->height * input->width;
  char* raw3DPtsData = (char*)(input->data.data());
  std::vector<Point3D> pc(numPts, Point3D(0.,0.,0.));

  pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
  pcl::PointCloud<pcl::PointXYZ> laserCloudOut;
  laserCloudOut.points.resize(numPts);
  pcl::fromROSMsg(*input, laserCloudIn);
  pcl::PointXYZI point;  

  //Laser Transform variables
	unsigned int num_readings = 360/resolution;
  double laser_frequency = 600;
  double ranges[num_readings]={0};
  double intensities[num_readings]={0};
	int addpos=0;
	int position=0;
    
    
	for (int i = 0; i < laserCloudIn.size(); i++) {   
           
    float* base = (float*)(raw3DPtsData + i * input->point_step);
    Point3D point3d(0.,0.,0.);       

    point.x=base[2];
    point.y=-base[0];
    point.z=-base[1];

    // Vertical filter : Points with a height greater than the height of the robot are discarded
    if (point.z < -sensorheight || point.z > robotheight-sensorheight || isnan(point.x)==1 || isnan(point.y)==1) {continue;}

    float hip = sqrt((point.x)*(point.x)+((point.y)*(point.y)));
		float hangle = (asin ((point.x)/hip))*180/PI;     

    //Fliter nearest point per angle resolution
    if (point.y>0) {
      position=(180.0+hangle)/resolution;
    }
    else if (point.y<=0) {
      position=(360.0-hangle)/resolution;
    }

    if (ranges[position]==0 || hip < ranges[position])
    {
      ranges[position]=hip;
      intensities[position]=50;
    }	

    laserCloudOut.points[i].x=point.x;
		laserCloudOut.points[i].y=point.y;
		laserCloudOut.points[i].z=point.z;	     

  }
  //Publish LegsCloud
	sensor_msgs::PointCloud2 laserCloudOut_output;
	pcl::toROSMsg(laserCloudOut, laserCloudOut_output);
  laserCloudOut_output.header.frame_id = "base_link";
	laserCloud_pub.publish (laserCloudOut_output);

  // Configuration of parameters for conversion to LaserScan
  ros::Time scan_time = ros::Time::now();
  sensor_msgs::LaserScan scan;
  scan.header.stamp = scan_time;
  scan.header.frame_id = "base_link";
  scan.angle_min = 3*PI/2;
  scan.angle_max = -PI/2;
  scan.angle_increment = -6.28 / num_readings;
  scan.time_increment = (1 /  num_readings);
  scan.range_min = 0.2;
  scan.range_max = 10.0;
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
    ros::init (argc, argv, "kinect_pff");
    ros::NodeHandle nh;

    // Get launch parameters
    if(!nh.getParam("/kinect_pff/robot_height",robotheight)){robotheight = 0.6;}
    if(!nh.getParam("/kinect_pff/sensor_height",sensorheight)){sensorheight = 0.3;}
    if(!nh.getParam("/kinect_pff/resolution",resolution)){resolution = 0.2;}
    
    // Create a ROS subscriber for the input PointCloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb2);
    
    // Create a ROS publisher for the output LaserScan
    scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 1);
    laserCloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/PeopleCloud", 1);
    
    // Spin
    ros::spin ();
}

