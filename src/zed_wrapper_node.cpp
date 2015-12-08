///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////




/****************************************************************************************************
 ** This sample is a wrapper for the ZED library in order to use the ZED Camera with ROS.          **
 ** You can publish Left+Depth or Left+Right images and camera info on the topics of your choice.  **
 **                                                                                                **
 ** A set of parameters can be specified in the launch file.                                       **
 ****************************************************************************************************/

//standard includes
#include <csignal>
#include <cstdio>
#include <math.h>
#include <limits>
#include <thread>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

//opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;

int computeDepth;

// Function to publish left and depth/right images

void publishImages(cv::Mat left, cv::Mat second, image_transport::Publisher &pub_left, image_transport::Publisher &pub_second,
        string left_frame_id, string second_frame_id, ros::Time t) {

    // Publish left image
    cv_bridge::CvImage left_im;
    left_im.image = left;
    left_im.encoding = sensor_msgs::image_encodings::BGR8;
    left_im.header.frame_id = left_frame_id;
    left_im.header.stamp = t;
    pub_left.publish(left_im.toImageMsg());

    // Publish second image
    cv_bridge::CvImage second_im;
    second_im.image = second;
    second_im.encoding = (computeDepth) ? sensor_msgs::image_encodings::TYPE_16UC1 : sensor_msgs::image_encodings::BGR8;
    second_im.header.frame_id = second_frame_id;
    second_im.header.stamp = t;
    pub_second.publish(second_im.toImageMsg());

}

// Function to publish camera info messages for both images

void publishCamInfo(sensor_msgs::CameraInfoPtr left_cam_info_msg, sensor_msgs::CameraInfoPtr second_cam_info_msg,
        ros::Publisher pub_left_cam_info, ros::Publisher pub_second_cam_info, ros::Time t) {
    static int seq = 0;

    left_cam_info_msg->header.stamp = second_cam_info_msg->header.stamp = t;
    left_cam_info_msg->header.seq = second_cam_info_msg->header.seq = seq;

    pub_left_cam_info.publish(left_cam_info_msg);
    pub_second_cam_info.publish(second_cam_info_msg);

    seq++;
}

// Function to fill both camera info messages with camera's parameters

void fillCamInfo(cv::VideoCapture &zedCv,
		 ros::NodeHandle &nh_ns,
		 sensor_msgs::CameraInfoPtr left_cam_info_msg,
		 sensor_msgs::CameraInfoPtr second_cam_info_msg,
		 string left_frame_id,
		 string second_frame_id) {

    int width = (int) zedCv.get(CV_CAP_PROP_FRAME_WIDTH) / 2;
    int height = (int) zedCv.get(CV_CAP_PROP_FRAME_HEIGHT);

    float baseline = 0.12;

    float fx = 0;
    float fy = 0;
    float cx = width * 2;
    float cy = height / 2 ;
    

    // There is no distorsions since the images are rectified
    double k1 = 0;
    double k2 = 0;
    double k3 = 0;
    double p1 = 0;
    double p2 = 0;

    nh_ns.getParam("baseline", baseline);
    nh_ns.getParam("fx", fx);
    nh_ns.getParam("fy", fy);
    nh_ns.getParam("cx", cx);
    nh_ns.getParam("cy", cy);
    nh_ns.getParam("k1", k1);
    nh_ns.getParam("k2", k2);
    nh_ns.getParam("k3", k3);
    nh_ns.getParam("p1", p1);
    nh_ns.getParam("p2", p2);

    left_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    second_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    left_cam_info_msg->D.resize(5);
    second_cam_info_msg->D.resize(5);
    left_cam_info_msg->D[0] = second_cam_info_msg->D[0] = k1;
    left_cam_info_msg->D[1] = second_cam_info_msg->D[1] = k2;
    left_cam_info_msg->D[2] = second_cam_info_msg->D[2] = k3;
    left_cam_info_msg->D[3] = second_cam_info_msg->D[3] = p1;
    left_cam_info_msg->D[4] = second_cam_info_msg->D[4] = p2;

    left_cam_info_msg->K.fill(0.0);
    second_cam_info_msg->K.fill(0.0);
    left_cam_info_msg->K[0] = second_cam_info_msg->K[0] = fx;
    left_cam_info_msg->K[2] = second_cam_info_msg->K[2] = cx;
    left_cam_info_msg->K[4] = second_cam_info_msg->K[4] = fy;
    left_cam_info_msg->K[5] = second_cam_info_msg->K[5] = cy;
    left_cam_info_msg->K[8] = second_cam_info_msg->K[8] = 1.0;

    left_cam_info_msg->R.fill(0.0);
    second_cam_info_msg->R.fill(0.0);

    left_cam_info_msg->P.fill(0.0);
    second_cam_info_msg->P.fill(0.0);
    left_cam_info_msg->P[0] = second_cam_info_msg->P[0] = fx;
    left_cam_info_msg->P[2] = second_cam_info_msg->P[2] = cx;
    left_cam_info_msg->P[5] = second_cam_info_msg->P[5] = fy;
    left_cam_info_msg->P[6] = second_cam_info_msg->P[6] = cy;
    left_cam_info_msg->P[10] = second_cam_info_msg->P[10] = 1.0;
    second_cam_info_msg->P[3] = (computeDepth) ? 0.0 : (-1 * fx * (baseline / 1000));

    left_cam_info_msg->width = second_cam_info_msg->width = width;
    left_cam_info_msg->height = second_cam_info_msg->height = height;

    left_cam_info_msg->header.frame_id = left_frame_id;
    second_cam_info_msg->header.frame_id = second_frame_id;
}

// Main function

int main(int argc, char **argv) {
    computeDepth = (argc >= 2) ? atoi(argv[1]) : 1; // Argument "computeDepth" in launch file, "0": publish left+right, "1": publish left+depth
    int capture_width, capture_height;

    // Launch file parameters
    int rate = 25;
    int device_number;
    int resolution;

    std::string img_topic = "image_rect";
#if 0
    // Used in some package
    img_topic = "image_raw";
#endif

    string left_topic = (computeDepth) ? "rgb/" + img_topic : "left/" + img_topic;
    string second_topic = (computeDepth) ? "depth/" + img_topic : "right/" + img_topic;
    string left_cam_info_topic = (computeDepth) ? "rgb/camera_info" : "left/camera_info";
    string second_cam_info_topic = (computeDepth) ? "depth/camera_info" : "right/camera_info";
    string left_frame_id = (computeDepth) ? "/zed_rgb_optical_frame" : "/zed_left_optical_frame";
    string second_frame_id = (computeDepth) ? "/zed_depth_optical_frame" : "/zed_right_optical_frame";

    ros::init(argc, argv, "zed_depth_stereo_wrapper_node");
    ROS_INFO("ZED_WRAPPER Node initialized");

    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");

    // Get parameters from launch file
    nh_ns.getParam("resolution", resolution);
    nh_ns.getParam("device_number", device_number);
    nh_ns.getParam("frame_rate", rate);
    nh_ns.getParam("left_topic", left_topic);
    nh_ns.getParam("second_topic", second_topic);
    nh_ns.getParam("left_cam_info_topic", left_cam_info_topic);
    nh_ns.getParam("second_cam_info_topic", second_cam_info_topic);
    nh_ns.getParam("left_frame_id", left_frame_id);
    nh_ns.getParam("second_frame_id", second_frame_id);

    cv::VideoCapture zedCv(device_number);
    switch (resolution) {
    case 0:
      capture_width = 4416;
      capture_height = 1242;
      break;
    case 1:
      capture_width = 3840;
      capture_height = 1080;
      break;
    case 2:
      capture_width = 2560;
      capture_height = 720;
      break;
    case 3:
      capture_width = 640;
      capture_height = 480;
      break;
    }

    zedCv.set(CV_CAP_PROP_FRAME_WIDTH, capture_width);
    zedCv.set(CV_CAP_PROP_FRAME_HEIGHT, capture_height);
    int width = capture_width /2;
    int height = capture_height;

    ROS_DEBUG_STREAM("Image size : " << width << "x" << height);

    cv::Size frameSize(capture_width, capture_height);
    cv::Size Taille(width, height);

    cv::Mat frameRGB(frameSize, CV_8UC3);
    cv::Mat frameLeft;
    cv::Mat frameRight;

    image_transport::ImageTransport it_zed(nh);

    // Image publishers
    image_transport::Publisher pub_left = it_zed.advertise(left_topic, 1);
    ROS_INFO_STREAM("Advertized on topic " << left_topic);
    image_transport::Publisher pub_second = it_zed.advertise(second_topic, 1);
    ROS_INFO_STREAM("Advertized on topic " << second_topic);

    // Camera info publishers
    ros::Publisher pub_left_cam_info = nh.advertise<sensor_msgs::CameraInfo>(left_cam_info_topic, 1);
    ROS_INFO_STREAM("Advertized on topic " << left_cam_info_topic);
    ros::Publisher pub_second_cam_info = nh.advertise<sensor_msgs::CameraInfo>(second_cam_info_topic, 1);
    ROS_INFO_STREAM("Advertized on topic " << second_cam_info_topic);

    // Camera info messages
    sensor_msgs::CameraInfoPtr left_cam_info_msg(new sensor_msgs::CameraInfo());
    sensor_msgs::CameraInfoPtr second_cam_info_msg(new sensor_msgs::CameraInfo());

    fillCamInfo(zedCv, nh_ns, left_cam_info_msg, second_cam_info_msg, left_frame_id, second_frame_id);

    ros::Rate loop_rate(rate);
    bool got_frame;

    try {
      // Main loop
      while (ros::ok()) {
	// Check for subscribers
	if (pub_left.getNumSubscribers() > 0 || pub_second.getNumSubscribers() > 0) {

	  // Get current time
	  ros::Time t = ros::Time::now();

	  got_frame = zedCv.read(frameRGB);
	  
	  frameLeft = frameRGB.colRange(0, width);
	  frameRight = frameRGB.colRange(width, capture_width);

	  if (got_frame) {
	    publishCamInfo(left_cam_info_msg, second_cam_info_msg, pub_left_cam_info, pub_second_cam_info, t);
	    publishImages(frameLeft, frameRight, pub_left, pub_second, left_frame_id, second_frame_id, t);
	  }
	  ros::spinOnce();
	  loop_rate.sleep();
	} else std::this_thread::sleep_for(std::chrono::milliseconds(10)); // No subscribers, we just wait

      }
    } catch (...) {
      ROS_ERROR("Unknown error.");
      return 1;
    }

    ROS_INFO("Quitting zed_depth_stereo_wrapper_node ...\n");
    return 0;
}
