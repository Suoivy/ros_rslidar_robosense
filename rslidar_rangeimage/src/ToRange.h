/* 
*
*   Copyright (C) 2018 Southeast University, Suo_ivy
*   License: Modified BSD Software License Agreement
*
*/

/*  
   This ROS node converts PointCloud2 to PCL:rangeimage

*/

#ifndef _TORANGE_H_
#define _TORANGE_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace rslidar_rangeimage
{
    class RangeConvert 
    {
    public:
        RangeConvert(ros::NodeHandle node, int argc, char** argv);

        ~RangeConvert(){}

    private:

        void printUsage(const char* Nodename);
        void Conversion(const sensor_msgs::PointCloud2::ConstPtr &pointMsg);

        ros::Subscriber rslidar_pointcloud_;
        ros::Publisher range_image_;
        pcl::visualization::RangeImageVisualizer rangevisual;

        // Rangeimage Config Parameters
        float angular_resolution_x = 0.1f,
              angular_resolution_y = 0.1f;
        float max_angle_width = 360.0f;
        float max_angle_height = 180.0f;
        Eigen::Affine3f sensor_pose (Eigen::Affine3f::Identity());
        pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
        float nosie_level = 0.0f;
        float min_range = 0.0f;
        int border_size = 0;

    };
}
#endif