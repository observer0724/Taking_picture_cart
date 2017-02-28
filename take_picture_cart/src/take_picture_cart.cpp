//
// Created by observer0724 on 2/24/17.
//

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <cwru_davinci_traj_streamer/trajAction.h>
#include <actionlib/client/simple_action_client.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <time.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include <vector>
#include <iostream>
#include <string>
#include <highgui.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <stdlib.h>
#include <stdio.h>
#include "opencv/cv.hpp"
#include <cwru_davinci_kinematics/davinci_kinematics.h>
#include <fstream>
#include <sstream>
using namespace cv;
bool freshImage;
bool freshCameraInfo;

using namespace std;
double weight_data = 0.0;
bool g_server_goal_completed= false;




void doneCb(const actionlib::SimpleClientGoalState& state,
            const cwru_davinci_traj_streamer::trajResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d; traj_id = %d",result->return_val,result->traj_id);
    g_server_goal_completed = true;
}

void scalerCallback(const std_msgs::Float64& weight)
{
    ROS_INFO("received value is: %f",weight.data);

    weight_data = weight.data;
}

void newImageCallback(const sensor_msgs::ImageConstPtr& msg, cv::Mat* outputImage)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg);
        outputImage[0] = cv_ptr->image;
        freshImage = true;

    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg ->encoding.c_str());
    }

}






int main(int argc, char **argv) {
    //Set up our node.
    ros::init(argc, argv, "take_picture");
    ros::NodeHandle nh;



    Mat rawImage_left = cv::Mat::zeros(640, 920, CV_8UC3);
    Mat rawImage_right = cv::Mat::zeros(640, 920, CV_8UC3);
    Mat save_image = cv::Mat::zeros(640, 920, CV_8UC3);


    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub_l = it.subscribe("/davinci_endo/left/image_raw", 1,
    boost::function< void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_left)));
    image_transport::Subscriber img_sub_r = it.subscribe("/davinci_endo/right/image_raw", 1,
    boost::function< void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_right)));

    Mat seg_left;
    Mat seg_right;



    Eigen::Vector3d x_vec1, y_vec1,z_vec1, tip_origin1;
    Eigen::Vector3d x_vec2, y_vec2,z_vec2, tip_origin2;
    double gripper_ang1, gripper_ang2;
    double arrival_time;

    x_vec1<< 0.545431, 0.366506, -0.753776;
    z_vec1<<-0.585144, -0.477382, -0.655525;
    y_vec1 = z_vec1.cross(x_vec1);
    tip_origin1<< -0.139252 ,0.00492328,  -0.151131;
    gripper_ang1 = 0.0;


    x_vec2<<0.72405 , 0.031784 , 0.689014;
    z_vec2<<0.673484,  0.183066 ,-0.716175;
    tip_origin2<<0.1300, -0.0150,  -0.1259117;
    y_vec2 = z_vec2.cross(x_vec2);
    gripper_ang2 = -10.0;

    arrival_time = 3;
    Eigen::Matrix3d R;
    Eigen::Affine3d des_gripper_affine1,des_gripper_affine2,des_moving;
    vector<Eigen::Affine3d> gripper1_affines,gripper2_affines;
    R.col(0) = x_vec1;
    R.col(1) = y_vec1;
    R.col(2) = z_vec1;
    des_gripper_affine1.linear() = R;
    des_gripper_affine1.translation() = tip_origin1;

    R.col(0) = x_vec2;
    R.col(1) = y_vec2;
    R.col(2) = z_vec2;
    des_gripper_affine2.linear() = R;
    des_gripper_affine2.translation() = tip_origin2;
    des_moving = des_gripper_affine2;

    Vectorq7x1 q_vec1,q_vec2;
    q_vec1.resize(7);
    q_vec2.resize(7);

    Davinci_IK_solver ik_solver1;
    Davinci_IK_solver ik_solver2;

    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions.resize(14);

    trajectory_msgs::JointTrajectory des_trajectory;



    ROS_INFO("All cartspace coordinates received");


    ik_solver1.ik_solve(des_gripper_affine1); //convert desired pose into equiv joint displacements
    q_vec1 = ik_solver1.get_soln();
    q_vec1(6) = gripper_ang1;

    ik_solver2.ik_solve(des_gripper_affine2); //convert desired pose into equiv joint displacements
    q_vec2 = ik_solver2.get_soln();
    q_vec2(6) = gripper_ang2;
    //repackage q's into a trajectory;
    ROS_INFO_STREAM("q_vec2:"<<q_vec2);
    for (int i=0;i<7;i++) {
        trajectory_point.positions[i] = q_vec1(i);
        trajectory_point.positions[i+7] = q_vec2(i);
        //should fix up jaw-opening values...do this later
    }
    double height = trajectory_point.positions[9];
    trajectory_point.time_from_start = ros::Duration(arrival_time);
    des_trajectory.points.push_back(trajectory_point);


    ROS_INFO("All cart coordinates has been converetd into Joint coordinates");

    des_trajectory.header.stamp = ros::Time::now();

    cwru_davinci_traj_streamer::trajGoal tgoal;
    tgoal.trajectory = des_trajectory;
    cwru_davinci_traj_streamer::trajGoal tstart;
    tstart.trajectory = des_trajectory;


    srand(time(NULL));
    tgoal.traj_id = rand();

    //Locate and lock the action server
    actionlib::SimpleActionClient<
    cwru_davinci_traj_streamer::trajAction
    > action_client("trajActionServer", true);
    bool server_exists = action_client.waitForServer(ros::Duration(5.0));
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
    ROS_INFO("Waiting for server: ");
    while (!server_exists && ros::ok()) {
        server_exists = action_client.waitForServer(ros::Duration(5.0));
        ROS_WARN("Could not connect to server; retrying...");
    }
    ROS_INFO("SERVER LINK LATCHED");

    //Send our message:
    ROS_INFO("Sending trajectory with ID %u", tgoal.traj_id);


    ros::Subscriber scaler_sub= nh.subscribe("scale", 1, scalerCallback);

    int pic_num = 0;
    char temp[16];
    string folder = "/home/dvrk/Desktop/pictures/";
    string name;
    //picture name and route
    srand(time(0));
    double rand_x;
    double rand_y;
    double rand_r;
    double rand_theta;
    for (int j=0; j<10; j++){
    	rand_r = (double)(rand()%400)/10000.00;
    	rand_theta = (rand()%6282)/1000.00;
    	rand_x = rand_r*cos(rand_theta);
    	rand_y = rand_r*sin(rand_theta);
    	ROS_INFO_STREAM("rand_r:"<<rand_r);
	    ROS_INFO_STREAM("rand_theta:"<<rand_theta);
        ROS_INFO_STREAM("rand_x:"<<rand_x);
        ROS_INFO_STREAM("rand_y:"<<rand_y);
        des_moving = des_gripper_affine2;
        des_moving.translation()[0] = des_gripper_affine2.translation()[0]+rand_x;
        des_moving.translation()[1] = des_gripper_affine2.translation()[1]+rand_y;
        ik_solver2.ik_solve(des_moving);
        q_vec2 = ik_solver2.get_soln();
        q_vec2(6) = gripper_ang2;
	    for (int i=0;i<7;i++) {
	        tgoal.trajectory.points[0].positions[i+7] = q_vec2(i);
	    }
        tgoal.trajectory.points[0].positions[9] = height;



	        ROS_INFO_STREAM("CART coordinates:"<<des_moving.translation()); 
	        ROS_INFO_STREAM("LOOP i is: " << j);
	        ros::spinOnce();
	        while (weight_data <= 2.5){
	            g_server_goal_completed= false;
	            action_client.sendGoal(tgoal,&doneCb);

	            while(!g_server_goal_completed){
	                doneCb;
	                ros::Duration(2).sleep();
	                ROS_INFO("STILL MOVING");
	            }
	            ros::spinOnce();
	            ROS_INFO("Taking a picture");
	            ros::Duration(2).sleep();
	            if (freshImage){
	                sprintf(temp,"%d",pic_num);
	                string file(temp);
	                if (weight_data>0.1){
	                    name = folder+"touch/"+file+".png";
	                }
	                else{
	                    name = folder+"not_touch/"+file+".png";
	                }
	                imwrite(name,rawImage_left);
	                freshImage = false;
	                pic_num ++;
	            }
	            ROS_INFO("Picture taken");
	            des_moving.translation()[2] -= 0.002;
	            ik_solver2.ik_solve(des_moving);
	            q_vec2 = ik_solver2.get_soln();
	            q_vec2(6) = gripper_ang2;
	            for (int i=0;i<7;i++) {
	                tgoal.trajectory.points[0].positions[i+7] = q_vec2(i);
	            }
	            tgoal.trajectory.points[0].time_from_start = ros::Duration(arrival_time);
	            tstart.trajectory.points[0].time_from_start = ros::Duration(arrival_time+2.0);
	            tgoal.traj_id = rand();
	            ROS_INFO("%f",weight_data);
	        }
	        action_client.sendGoal(tstart,&doneCb);
	        //return to the start point
	        g_server_goal_completed= false;
	        while(!g_server_goal_completed){
	            doneCb;
	            ros::Duration(2).sleep();
	            ROS_INFO("STILL MOVING");
	            //waiting till it reaches the start point
	        }
	    }






    //Wait for it to finish.
    while(!action_client.waitForResult(ros::Duration(arrival_time + 4.0)) && ros::ok()){
        ROS_WARN("CLIENT TIMED OUT- LET'S TRY AGAIN...");
        //Could add logic here to resend the request or take other actions if we conclude that
        //the original is NOT going to get served.
    }
    //Report back what happened.
    ROS_INFO(
            "Server state is %s, goal state for trajectory %u is %i",
            action_client.getState().toString().c_str(),
            action_client.getResult()->traj_id,
            action_client.getResult()->return_val
    );

    //This has to do with the intermittent "Total Recall" bug that breaks the trajectory interpolator
    //If you see this appear in your execution, you are running the program too soon after starting Gazebo.
    if(action_client.getState() ==  actionlib::SimpleClientGoalState::RECALLED){
        ROS_WARN("Server glitch. You may panic now.");
    }

    return 0;
}
