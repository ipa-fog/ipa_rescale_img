/*
 * Copyright (c) 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 * All rights reserved.
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential.
 */

#ifndef IPA_RESCALE_IMG_NODE_H
#define IPA_RESCALE_IMG_NODE_H

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <compressed_image_transport/compressed_publisher.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>


class IPARescaleImgNode
{
public:
    IPARescaleImgNode(ros::NodeHandle node_handle);

private:
    ros::NodeHandle node_;

    int counter = 0;
    ros::Subscriber sub_img_one;
    ros::Subscriber sub_img_two;

    ros::Publisher pub_img_one;
    ros::Publisher pub_img_two;

    void img1Callback(const sensor_msgs::CompressedImageConstPtr& msg);
    void img2Callback(const sensor_msgs::Image::ConstPtr& msg);

};

#endif // IPA_RESCALE_IMG_NODE_H
