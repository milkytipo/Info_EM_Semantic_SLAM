/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>
#include<opencv2/core/core.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include"../../../../include/System.h"
#include <boost/thread.hpp>
#include <thread>
using namespace std;

class ImageGrabber
{
private: 
    ros::NodeHandle nh2;

    ros::Publisher pub_tf; 
    cv_bridge::CvImageConstPtr mRoi;
    cv_bridge::CvImageConstPtr mMask;

    float depth_object;

public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){  
  
    pub_tf = nh2.advertise<geometry_msgs::TransformStamped>("/slam/tf",10);

}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD); //original 
    void GrabROI(const sensor_msgs::ImageConstPtr& msgRoi,const sensor_msgs::ImageConstPtr& msgMask); 
    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r,Twc;
    float q[4];
    geometry_msgs::PoseStamped msg;
    geometry_msgs::TransformStamped tf1;
    bool isPre = false; //flag to judge whether the current mask frame is genereated by mask_rcnn.
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> roi_sub(nh, "/roi_image", 1);
    message_filters::Subscriber<sensor_msgs::Image> mask_sub(nh, "/mask_image", 1);


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub); // no synchronize the roi and image since the frequency is too low 
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2)); 

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> sync_pol2;
    message_filters::Synchronizer<sync_pol2> sync2(sync_pol2(10),roi_sub,mask_sub);
    sync2.registerCallback(boost::bind(&ImageGrabber::GrabROI,&igb,_1,_2)); 

 //   ros::AsyncSpinner spinner(2);
 //   spinner.start();
 //   ros::waitForShutdown();

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
 //   ros::spin();
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }   
    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
  //  cv::Mat imDepth = cv_ptrD->image;

  //  imDepth.convertTo(imDepth,CV_32F,0.001);
  //  depth_object = imDepth.ptr<float>(100)[100];
    if(mRoi == NULL){
       mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec()); //original
       std::cout << " No mask was received "<<endl;
    }
    // else{
    //    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,mRoi->image,mMask->image,cv_ptrRGB->header.stamp.toSec(),isPre);
    // }
    if(mpSLAM->GetFramePose(Twc, q)){

        tf1.header.stamp = msgRGB->header.stamp;
        tf1.header.frame_id = "world" ;
        tf1.child_frame_id = "slam" ;
        tf1.transform.translation.x = Twc.at<float>(2);//Twc.at<float>(0);
        tf1.transform.translation.y = -Twc.at<float>(0);//Twc.at<float>(1);
        tf1.transform.translation.z = -Twc.at<float>(1);//Twc.at<float>(2);
        tf1.transform.rotation.x = q[2];//q[0];
        tf1.transform.rotation.y = -q[0];//q[1];
        tf1.transform.rotation.z = -q[1];//q[2];
        tf1.transform.rotation.w = q[3];
        pub_tf.publish(tf1);
    }
    isPre == true;
}

//mask
void ImageGrabber::GrabROI(const sensor_msgs::ImageConstPtr& msgRoi,const sensor_msgs::ImageConstPtr& msgMask)
{

    cv_bridge::CvImageConstPtr cv_ptrRoi;
    try
    {
        cv_ptrRoi = cv_bridge::toCvShare(msgRoi);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrMask;
    try
    {
        cv_ptrMask = cv_bridge::toCvShare(msgMask);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mRoi = cv_ptrRoi; 
    mMask = cv_ptrMask;
    isPre == false;
}


