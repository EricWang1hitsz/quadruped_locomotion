/*
 *  kinematicsTest.cpp
 *  Descriotion:
 *
 *  Created on: date, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

#include "quadruped_model/quadrupedkinematics.h"
#include "quadruped_model/QuadrupedModel.hpp"

#include "sensor_msgs/JointState.h"
#include <std_msgs/Float64MultiArray.h>
#include "ros/ros.h"

#include "iostream"
#include "fstream"
#include "sstream"

using namespace std;
using namespace quadruped_model;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_joint_angle");
    ros::NodeHandle nh;
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1000);
    ros::Publisher pos_command_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/all_joints_position_effort_group_controller/command",1);
    ros::Rate loop_rate(400);
    sensor_msgs::JointState joint_state;
    std_msgs::Float64MultiArray joint_group_positions_;

    joint_state.header.frame_id = "base_link";
    joint_state.name.resize(12);
    joint_state.position.resize(12);
    joint_state.name[0] = "front_left_1_joint";
    joint_state.name[1] = "front_left_2_joint";
    joint_state.name[2] = "front_left_3_joint";
    joint_state.name[3] = "front_right_1_joint";
    joint_state.name[4] = "front_right_2_joint";
    joint_state.name[5] = "front_right_3_joint";
    joint_state.name[6] = "rear_right_1_joint";
    joint_state.name[7] = "rear_right_2_joint";
    joint_state.name[8] = "rear_right_3_joint";
    joint_state.name[9] = "rear_left_1_joint";
    joint_state.name[10] = "rear_left_2_joint";
    joint_state.name[11] = "rear_left_3_joint";

    joint_group_positions_.data.resize(12);

    std::ifstream readfile;
    readfile.open("/home/eric/catkin_ws/data.txt");
    quadruped_model::JointPositions joint_position_file;
    std::vector<quadruped_model::JointPositions> joint_position_collection;
    double time;
    for(int i = 0; !readfile.eof(); i++)
    {
//        readfile >> time >> joint_position_file(0) >> joint_position_file(1)>> joint_position_file(2)
//            >> joint_position_file(3)>> joint_position_file(4)
//            >> joint_position_file(5)>> joint_position_file(6)
//            >> joint_position_file(7)>> joint_position_file(8)
//            >> joint_position_file(9)>> joint_position_file(10)
//            >> joint_position_file(11);
        readfile >> time >> joint_position_file(0) >> joint_position_file(1)>> joint_position_file(2)
            >> joint_position_file(3)>> joint_position_file(4)
            >> joint_position_file(5)>> joint_position_file(9)
            >> joint_position_file(10)>> joint_position_file(11)
            >> joint_position_file(6)>> joint_position_file(7)
            >> joint_position_file(8);
        joint_position_collection.push_back(joint_position_file);
    }
    readfile.close();
    joint_position_collection.pop_back();
    unsigned long i = 0;
    std::cout << "right here" << std::endl;
    std::cout << joint_position_collection.size() << std::endl;
//    for (unsigned int i = 0; i < joint_position_collection.size(); i++) {
//        for (unsigned int j = 0; j < 12; j++) {
//            std::cout << joint_position_collection[i](j)<<" ";
//        }
//        std::cout << std::endl;
//    }
//    cout<<joint_position_collection<<endl;
    while(ros::ok())
    {

        if(i < joint_position_collection.size())
        {
            for (unsigned int j = 0; j < 12; j++) {
                joint_state.position[j] = joint_position_collection[i](j);
                joint_group_positions_.data[j] = joint_position_collection[i](j);
            }
            joint_pub.publish(joint_state);
            pos_command_pub_.publish(joint_group_positions_);

        }
        i = i + 1;
        std::cout << i << std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 1;

}

