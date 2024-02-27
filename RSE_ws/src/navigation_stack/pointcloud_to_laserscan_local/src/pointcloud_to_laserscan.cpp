#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudToLaserScan {
public:
    PointCloudToLaserScan() {
        // Initialize ROS node handle
        nh = ros::NodeHandle("~");

        // Subscribe to the PointCloud topic
        pc_sub = nh.subscribe("/steer_bot/points", 1, &PointCloudToLaserScan::pointCloudCallback, this);

        // Advertise the LaserScan topic
        laser_pub = nh.advertise<sensor_msgs::LaserScan>("/output_laserscan", 1);

        // Set the sensor height
        nh.param<double>("sensor_height", sensor_height, 0.0); // default to 1.0 meter
    }

    // Callback function to process PointCloud messages
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pc_msg) {
        // Convert PointCloud2 message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pc_msg, *cloud);

        // Create LaserScan message
        sensor_msgs::LaserScan laser_scan;
        laser_scan.header = pc_msg->header;
        laser_scan.angle_min = 0;
        laser_scan.angle_max = 2*M_PI;
        laser_scan.angle_increment = 2.0 * M_PI / cloud->width; // assuming 360-degree field of view
        laser_scan.time_increment = 0.0;
        laser_scan.scan_time = 0.1;
        laser_scan.range_min = 0.0; // minimum range
        laser_scan.range_max = 100.0; // maximum range

        // Fill LaserScan ranges
        laser_scan.ranges.resize(cloud->width);
        for (size_t i = 0; i < cloud->width; ++i) {
            // Get the point
            pcl::PointXYZ point = cloud->points[i];

            // Calculate the range from the sensor to the point
            double range = sqrt(point.x * point.x + point.y * point.y);

            // Check if the point is within an acceptable height range
            if (std::abs(point.z - sensor_height) < 0.5) { // Adjust the threshold as needed
                // Set the range in the LaserScan message
                laser_scan.ranges[i] = range;
            } else {
                // If the point's height is outside the acceptable range, set range to maximum
                laser_scan.ranges[i] = laser_scan.range_max;
            }
        }

        // Publish LaserScan message
        laser_pub.publish(laser_scan);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber pc_sub;
    ros::Publisher laser_pub;
    double sensor_height; // Height of the sensor
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_to_laserscan");
    PointCloudToLaserScan pcl_to_laserscan;
    ros::spin();
    return 0;
}

