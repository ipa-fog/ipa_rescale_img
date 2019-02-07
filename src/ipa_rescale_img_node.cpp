/*
 * Copyright (c) 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 * All rights reserved.
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential.
 */

/**
 * /author
 *    Author: Florenz Graf, email: florenz.graf@gmx.de
 *
 * /Filedescription
 *
 */

#include "ipa_rescale_img_node.h"


//  === CONSTRUCTOR ===
IPARescaleImgNode::IPARescaleImgNode(ros::NodeHandle node_handle):
    node_(node_handle)
{
    sub_img_one = node_.subscribe("/camera/rgb/image_raw/compressed", 10, &IPARescaleImgNode::img1Callback, this);
    sub_img_two = node_.subscribe("/camera/rgb/image_raw/compressed2", 10, &IPARescaleImgNode::img2Callback, this);
    sub_img_three = node_.subscribe("img_bgr1_out", 10, &IPARescaleImgNode::img2Callback, this);

    pub_img_one = node_.advertise<sensor_msgs::CompressedImage>("img_bgr1_out",10);
    pub_img_two = node_.advertise<sensor_msgs::CompressedImage>("img_bgr2_out",10);
}


void IPARescaleImgNode::img1Callback(const sensor_msgs::CompressedImageConstPtr& msg)
{

	// convert ros msgs to cv 
    cv::Mat image;
	try
	{
         image = cv::imdecode(cv::Mat(msg->data),CV_LOAD_IMAGE_COLOR);//convert compressed image data to cv::Mat	catch (cv_bridge::Exception& e)
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert to image!");
    }


             // get params from launch file
             double rescale_factor = 0.2;
             int reduce_fps = 2;
             ros::param::get("~rescale_factor", rescale_factor);
             ros::param::get("~reduce_fps", reduce_fps);

             int height = (int)(image.rows * rescale_factor);
             int width = (int)(image.cols * rescale_factor);




     	static int counter;
        if(++counter == reduce_fps) //reduce FPS
        {

         // resize img
            cv::Mat img_small;
            cv::resize(image, img_small, cv::Size(width, height)); //  set factor for resize
            cv::imshow("img_small", img_small);
            cv::waitKey(1);

            std::vector<int> params;
            params.resize(3, 0);
            params[0] = CV_IMWRITE_JPEG_QUALITY;
            params[1] = 80; // jpg quality [0-100]

            sensor_msgs::CompressedImage compressed;
            compressed.header = msg->header;
            compressed.format = msg->format;
            cv::imencode(".jpg", img_small, compressed.data, params);

            pub_img_one.publish(compressed);

            counter = 0;
        }

}

void IPARescaleImgNode::img2Callback(const sensor_msgs::CompressedImageConstPtr& msg)
{

    // convert ros msgs to cv
    cv::Mat image;
    try
    {
         image = cv::imdecode(cv::Mat(msg->data),CV_LOAD_IMAGE_COLOR);//convert compressed image data to cv::Mat	catch (cv_bridge::Exception& e)
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert to image!");
    }


             // get params from launch file
             double rescale_factor = 10;
             int reduce_fps = 1;
             ros::param::get("~rescale_factor", rescale_factor);
             ros::param::get("~reduce_fps", reduce_fps);

             int height = (int)(image.rows * rescale_factor);
             int width = (int)(image.cols * rescale_factor);




        static int counter;
        if(++counter == reduce_fps) //reduce FPS
        {

         // resize img
            cv::Mat img_small;
            cv::resize(image, img_small, cv::Size(width, height)); //  set factor for resize
            cv::imshow("img_small2", img_small);
            cv::waitKey(1);

            std::vector<int> params;
            params.resize(3, 0);
            params[0] = CV_IMWRITE_JPEG_QUALITY;
            params[1] = 80; // jpg quality [0-100]

            sensor_msgs::CompressedImage compressed;
            compressed.header = msg->header;
            compressed.format = msg->format;
            cv::imencode(".jpg", img_small, compressed.data, params);

            pub_img_two.publish(compressed);

            counter = 0;
        }

}




void IPARescaleImgNode::img3Callback(const sensor_msgs::CompressedImageConstPtr& msg)
{


    // TEST



    // convert ros msgs to cv
    cv::Mat image;
    try
    {
         image = cv::imdecode(cv::Mat(msg->data),CV_LOAD_IMAGE_COLOR);//convert compressed image data to cv::Mat	catch (cv_bridge::Exception& e)
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert to image!");
    }


    // get params from launch file
    double rescale_factor = 5;
    int reduce_fps = 1;
    ros::param::get("~rescale_factor", rescale_factor);
    ros::param::get("~reduce_fps", reduce_fps);

    int height = (int)(image.rows * rescale_factor);
    int width = (int)(image.cols * rescale_factor);

    // resize img
    cv::Mat img_small;
    cv::resize(image, img_small, cv::Size(width, height)); //  set factor for resize
    cv::imshow("img_small2", img_small);
    cv::waitKey(1);




}





//  MAIN
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ipa_rescale_img_node");

    ros::NodeHandle node_handle;
    IPARescaleImgNode ipa_rescale_img_node(node_handle);

    ROS_INFO("Node is spinning...");
    ros::spin();
    return 0;
}
