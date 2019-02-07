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
    sub_img_two = node_.subscribe("img_bgr2_in", 10, &IPARescaleImgNode::img2Callback, this);

    pub_img_one = node_.advertise<sensor_msgs::CompressedImage>("img_bgr1_out",10);
    pub_img_two = node_.advertise<sensor_msgs::Image>("img_bgr2_out",10);
}


void IPARescaleImgNode::img1Callback(const sensor_msgs::CompressedImageConstPtr& msg)
{

	// convert ros msgs to cv 
	cv_bridge::CvImagePtr cv_ptr;
    cv::Mat image;
	try
	{
         image = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat	catch (cv_bridge::Exception& e)
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

             int width = (int)(image.rows * rescale_factor);
             int height = (int)(image.cols * rescale_factor);




     	static int counter;
        if(++counter == reduce_fps) //reduce FPS
	{

		// resize img
		cv::Mat img_small;
        cv::resize(image, img_small, cv::Size(width, height)); //  set factor for resize


        std::vector<int> params;
        params.resize(3, 0);
        params[0] = CV_IMWRITE_JPEG_QUALITY;
        params[1] = 80; // jpg quality [0-100]

        sensor_msgs::CompressedImage compressed;
        compressed.header = msg->header;
        compressed.format = msg->format;
        cv::imencode(compressed.format, img_small, compressed.data, params);

        pub_img_one.publish(compressed);

        counter = 0;
	}

}


void IPARescaleImgNode::img2Callback(const sensor_msgs::Image::ConstPtr& msg)
{
    // get params from launch file
    double rescale_factor = 1.0;
    int reduce_fps = 1;
    ros::param::get("~rescale_factor", rescale_factor);
    ros::param::get("~reduce_fps", reduce_fps);

    int width = (int)(msg->width * rescale_factor);
    int height = (int)(msg->height * rescale_factor);


	// convert ros msgs to cv
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
	}

     	static int counter;
        if(++counter == reduce_fps) //reduce FPS
	{

		// resize img
		cv::Mat img_small;
		img_small = cv_ptr->image; 
        cv::resize(cv_ptr->image, img_small, cv::Size(width, height)); //  set factor for resize


		// convert back to ros msgs
		cv_bridge::CvImage img_bridge;
		sensor_msgs::Image img_msg; // >> message to be sent

		img_bridge = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::RGB8, img_small);
		img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
		pub_img_two.publish(img_msg); 

		counter = 0;
	}

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
