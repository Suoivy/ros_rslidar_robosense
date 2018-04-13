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

        // rangeimage parameter initailize
        angular_resolution_x = 0.5f;
        angular_resolution_y = 0.5f;
        max_angle_width = 360.0f;
        max_angle_height = 180.0f;
        
        // Parse the commandline arguments
        if (pcl::console::parse (argc, argv, "-rx", angular_resolution_x) >= 0)
            std::cout << "Setting angular resolution in x-direction to "<<angular_resolution_x<<"deg.\n";
        if (pcl::console::parse (argc, argv, "-ry", angular_resolution_y) >= 0)
            std::cout << "Setting angular resolution in y-direction to "<<angular_resolution_y<<"deg.\n";
        if (pcl::console::parse (argc, argv, "-aw", max_angle_width) >= 0)
            std::cout << "Setting maximum angle width range to "<<max_angle_width<<"deg.\n";
        if (pcl::console::parse (argc, argv, "-ah", max_angle_height) >= 0)
            std::cout << "Setting maximum angle height range to "<<max_angle_height<<"deg.\n";
        if (pcl::console::find_argument (argc, argv, "-h") >= 0)
        {
            printUsage (argv[0]);
        }

        // Set rangeimage convertion parameters
        sensor_pose=Eigen::Affine3f::Identity();
        angular_resolution_x = pcl::deg2rad(angular_resolution_x);
        angular_resolution_y = pcl::deg2rad(angular_resolution_y);
        max_angle_width = pcl::deg2rad(max_angle_width);
        max_angle_height = pcl::deg2rad(max_angle_height);
        
        // std::cout <<pointcloud.sensor_origin_[0]<<endl;
        // std::cout <<pointcloud.sensor_orientation_[0]<<endl;
        coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
        nosie_level = 0.0f;
        min_range = 0.0f;
        border_size = 0;

        // pcl::visualization::RangeImageVisualizer range_image_widget ("RangeImage");

    }

    void RangeConvert::printUsage(const char* NodeName)
    {
        std::cout << "\n\nUsage:"<<NodeName<<"[options] <Rangeimage_config>\n\n"
                  << "Options\n"
                  << "----------------------------------\n"
                  << "-rx <float>   angular_resolution_x in degrees (default "<<angular_resolution_x<<")\n"
                  << "-ry <float>   angular_resolution_y in degrees (default "<<angular_resolution_y<<")\n"
                  << "-aw <float>   max_angle_width  in degrees (default "<<max_angle_width<<")\n"
                  << "-ah <float>   max_angle_height in degrees (default "<<max_angle_height<<")\n"
                  << "-sp <>        sensor_pose in \n"
                  << "-h            this help\n"
                  << "\n";
                  
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

        sensor_pose = Eigen::Affine3f (Eigen::Translation3f (pointcloud.sensor_origin_[0],
                                                            pointcloud.sensor_origin_[1],
                                                            pointcloud.sensor_origin_[2]))
                                                            *Eigen::Affine3f(pointcloud.sensor_orientation_);

        // creat rangeimage form pointcloud according to the parameters
        rangeimage.createFromPointCloud(pointcloud, angular_resolution_x,angular_resolution_y,
                                        max_angle_width,max_angle_height,sensor_pose,
                                        coordinate_frame,nosie_level, min_range,border_size);

        // rangeimage visualization using pcl::visualization::RangeImageVisualizer
        rangevisual.showRangeImage(rangeimage);


    }
}