#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/LaserScan.h>
#define PI 3.14159265

class pff
{
public:
	pff();
	void filter(const sensor_msgs::PointCloud2ConstPtr &input);
	float robotheight, sensorheight, resolution;
	int visionangle, position;
	std::string topic_pub, topic_sub, frame_id;
	struct Point3D
	{

		float x;
		float y;
		float z;
		Point3D(float x_, float y_, float z_)
			: x(x_), y(y_), z(z_)
		{
		}
	};

private:
	ros::NodeHandle node_, ns_;
	ros::Publisher scan_pub;
	ros::Subscriber sub;
};

pff::pff() : node_("~"),
			 robotheight(1.0),
			 sensorheight(0.57),
			 visionangle(360),
			 resolution(0.2),
			 topic_pub("/scan"),
			 topic_sub("/velodyne_points"),
			 frame_id("base_scan")
{
	/** Get parameters from the launch file */
	node_.param("robot_height", robotheight, robotheight);
	node_.param("sensor_height", sensorheight, sensorheight);
	node_.param("horizontal_fov", visionangle, visionangle);
	node_.param("resolution", resolution, resolution);
	node_.param("topic_pub", topic_pub, topic_pub);
	node_.param("topic_sub", topic_sub, topic_sub);
	node_.param("frame_id", frame_id, frame_id);

	/** Define Subscriber */
	sub = ns_.subscribe(topic_sub, 50, &pff::filter, this);

	/** Define Publisher */
	scan_pub = ns_.advertise<sensor_msgs::LaserScan>(topic_pub, 1, false);
}

void pff::filter(const sensor_msgs::PointCloud2ConstPtr &input)
{

	int numPts = input->height * input->width;
	char* raw3DPtsData = (char*)(input->data.data());
	std::vector<Point3D> pc(numPts, Point3D(0.,0.,0.));

	pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
	pcl::fromROSMsg(*input, laserCloudIn);

	/** Set scan parameters */
	ros::Time scan_time = ros::Time::now();
	sensor_msgs::LaserScan scan;
	scan.header.stamp = scan_time;
	scan.header.frame_id = frame_id;
	scan.angle_min = PI;
	scan.angle_max = -PI;
	scan.angle_increment = -2 * PI / (360 / resolution);
	scan.time_increment = 1 / (600 * (360 / resolution));
	scan.range_min = 0.2;
	scan.range_max = 4.0;
	scan.ranges.resize(360 / resolution);
	scan.intensities.resize(360 / resolution);

	for (int i = 0; i < laserCloudIn.size(); i++)
	{
		float* base = (float*)(raw3DPtsData + i * input->point_step);
		Point3D point3d(0.,0.,0.);       

		laserCloudIn[i].x=base[2];
		laserCloudIn[i].y=-base[0];
		laserCloudIn[i].z=-base[1];

		float hip = sqrt(pow(laserCloudIn[i].x, 2) + pow(laserCloudIn[i].y, 2));
		float hangle = (asin((laserCloudIn[i].x) / hip)) * 180 / PI;

		/** Discard points */
		if (laserCloudIn[i].z > robotheight - sensorheight ||
			(-sensorheight - 0.1 <= laserCloudIn[i].z && laserCloudIn[i].z < -sensorheight + 0.05) ||
			(visionangle <= 180 && laserCloudIn[i].x <= 0) ||
			(visionangle < 180 && laserCloudIn[i].x > 0 && hangle < 90 - (visionangle / 2)) ||
			(visionangle > 180 && visionangle < 360 && laserCloudIn[i].x < 0 && abs(hangle) > (visionangle - 180) / 2) ||
			(isnan(laserCloudIn[i].x)==1 || isnan(laserCloudIn[i].y)==1) || hip>4.0)
		{
			continue;
		}

		/** Stairs/holes detection */
		if (laserCloudIn[i].z < -sensorheight - 0.1)
		{
			float hip2 = sqrt(pow(hip, 2) + pow(laserCloudIn[i].z, 2));
			float hangle2 = asin((laserCloudIn[i].z / hip2) * sin(1.5708)) * 180 / PI;
			if (hangle2 < -14)
			{
				hip = 0.22;
				laserCloudIn[i].x = hip * sin(hangle * PI / 180);
				if (laserCloudIn[i].y > 0)
				{
					laserCloudIn[i].y = sqrt(pow(hip, 2) - pow(laserCloudIn[i].x, 2));
				}
				else
				{
					laserCloudIn[i].y = -sqrt(pow(hip, 2) - pow(laserCloudIn[i].x, 2));
				}
			}
			if (hangle2 > -14)
			{
				hip2 = sensorheight * sin(PI / 2) / sin((2 - hangle2) * PI / 180);
				hip = sqrt(pow(hip2, 2) - pow(sensorheight, 2));
				laserCloudIn[i].x = hip * sin(hangle * PI / 180);
				if (laserCloudIn[i].y > 0)
				{
					laserCloudIn[i].y = sqrt(pow(hip, 2) - pow(laserCloudIn[i].x, 2));
				}
				else
				{
					laserCloudIn[i].y = -sqrt(pow(hip, 2) - pow(laserCloudIn[i].x, 2));
				}
			}
		}

		/** Get the position in the array */
		if (laserCloudIn[i].y > 0)
		{
			position = (180.0 + hangle) / resolution;
		}
		else if (laserCloudIn[i].x > 0 && laserCloudIn[i].y <= 0)
		{
			position = (360.0 - hangle) / resolution;
		}
		else
		{
			position = -hangle / resolution;
		}

		/** Add near point to the scan array */
		if (scan.ranges[position] == 0 || hip < scan.ranges[position])
		{
			scan.ranges[position] = hip;
			scan.intensities[position] = 100;
		}
	}

	/** Publish Scan */
	scan_pub.publish(scan);
}

/** Main function*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "astra_pff");
	pff filter;
	ros::spin();
	return 0;
}
