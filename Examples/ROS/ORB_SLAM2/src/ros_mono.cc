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

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

//patriksc: mod
#include <Optimizer.h>
#include <jslam_msgs/orbslam_optimization_data.h>

//jslam_msgs::orbslam_optimization_data* p_msg_opt;
//bool data_flag;
//-----------------

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

//patriksc: mod
class data_publisher
{
public:
    data_publisher(ros::NodeHandle nh) : nh_(nh){
        pub_ = nh_.advertise<jslam_msgs::orbslam_optimization_data>("orb_opt_data",1);
    }

    void publish_data(){
        if (data_flag){
            std::cout << "*** publishing opt msg - num of KFs: " << p_msg_opt->keyframes.size() << " ***" << std::endl;
            pub_.publish(*p_msg_opt);
            p_msg_opt = new jslam_msgs::orbslam_optimization_data;
            std::cout << "global msg RESET" << std::endl;
            data_flag = false;
            std::cout << "data flag FALSE" << std::endl;
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
};

//-----------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    //patriksc: mod
    p_msg_opt = new jslam_msgs::orbslam_optimization_data;
    std::cout << "global msg INITIALIZED (RESET)" << std::endl;
    data_flag = false;
    std::cout << "data flag INITIALIZED (FALSE)" << std::endl;

    data_publisher dpub(nodeHandler);

    ros::Rate r(100);
    while(ros::ok())
    {
        dpub.publish_data();
        ros::spinOnce();
        r.sleep();
    }

    //-----------------

    //ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}


