/* 
*
*   Copyright (C) 2018 Southeast University, Suo_ivy
*   License: Modified BSD Software License Agreement
*
*/

/*  
   This ROS node converts PointCloud2 to PCL:rangeimage

*/

#include "ToRange.h"

namespace rslidar_rangeimage
{
    RangeConvert::RangeConvert(ros::NodeHandle node, int argc, char** argv) 
    {
        // subscribe to rslidar_pointcloud node
        // Type: sensor_msgs::PointCloud2 
        rslidar_pointcloud_ = 
                    node.subscribe("rslidar_points",10,
                    &RangeConvert::Conversion,(RangeConvert *) this, 
                    ros::TransportHints().tcpNoDelay(true));
        
        // advertise rangeimage convert from pcl::rangeimage.creatFromPointCloud()
        // Type: Image
        range_image_ = 
                    node.advertise<sensor_msgs::Image>("range_image",10);

        // pcl::visualization::RangeImageVisualizer range_image_widget ("RangeImage");

    }

    void RangeConvert::printUsage(const char* NodeName)
    {
        std::cout << "\n\nUsage:"<<NodeName<<"[options] <Rangeimage_config>\n\n"
                  << "OptionsL\n"
                  << "----------------------------------\n"
                  << "-rx <float>   angular_resolution_x in degrees (default "<<angular_resolution_x<<")\n"
                  << "-ry <float>   angular_resolution_y in degrees (default "<<angular_resolution_y<<")\n"
                  << "-aw <float>   max_angle_width  in degrees (default "<<max_angle_width<<")\n"
                  << "-ah <float>   max_angle_height in degrees (default "<<max_angle_height<<")\n"
                  
    }

    void RangeConvert::Conversion(const sensor_msgs::PointCloud2::ConstPtr &pointMsg)
    {
        // Define PointCloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>& pointcloud = *pointcloudPtr;

        // Define RangeImage
        pcl::RangeImage::Ptr rangeimagePtr(new pcl::RangeImage);
        pcl::RangeImage& rangeimage = *rangeimagePtr;

        // Convert ros sensor_msgs::PointCloud2 to pcl::PointCloud
        pcl::fromROSMsg(*pointMsg, pointcloud);

        // Set rangeimage convertion parameters
        float angular_resolution_x = pcl::deg2rad(0.1f),
              angular_resolution_y = angular_resolution_x;
        float max_angle_width = pcl::deg2rad(180.0f);
        float max_angle_height = pcl::deg2rad(180.0f);
        Eigen::Affine3f sensor_pose (Eigen::Affine3f::Identity());
        sensor_pose = Eigen::Affine3f (Eigen::Translation3f (pointcloud.sensor_origin_[0],
                                                            pointcloud.sensor_origin_[1],
                                                            pointcloud.sensor_origin_[2]))
                                                            *Eigen::Affine3f(pointcloud.sensor_orientation_);
        pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
        float nosie_level = 0.0f;
        float min_range = 0.0f;
        int border_size = 0;

        // creat rangeimage form pointcloud according to the parameters
        rangeimage.createFromPointCloud(pointcloud, angular_resolution_x,angular_resolution_y,
                                        max_angle_width,max_angle_height,sensor_pose,
                                        coordinate_frame,nosie_level, min_range,border_size);

        // rangeimage visualization using pcl::visualization::RangeImageVisualizer
  
        // range_image_widget.showRangeImage(rangeimage);
        rangevisual.showRangeImage(rangeimage);


    }
}