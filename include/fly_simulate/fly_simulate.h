#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>

#include "uwb_multi_robot_sim/FlyTaskAction.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

class fly_sim{
    public:
        fly_sim();

        ~fly_sim(void);

    private:
        ros::NodeHandle nh_;

        actionlib::SimpleActionServer<uwb_multi_robot_sim::FlyTaskAction> fly_server_;

        uwb_multi_robot_sim::FlyTaskActionGoal goal_;
        uwb_multi_robot_sim::FlyTaskActionResult result_;
        uwb_multi_robot_sim::FlyTaskFeedback feedback_;

        // mavros related
        ros::Subscriber state_sub_;
        
        ros::Subscriber current_pos_sub_;

        ros::Subscriber mavros_pos_sub_;

        ros::Publisher waypoint_pub_;
        
        ros::Publisher vel_pub_;
        
        ros::ServiceClient arming_client_;

        ros::ServiceClient set_mode_client_;


    private:
        mavros_msgs::State current_state_;

        geometry_msgs::PoseStamped current_pose_;

        geometry_msgs::PoseStamped mavros_pose_;

        geometry_msgs::PoseStamped target_pose_;

        float reach_tolerance_;

        ros::Timer fly_timer_;

        int up_down_cnt_{1};

        int up_count_{50};

        int down_count_{40};

    public:

        geometry_msgs::PoseStamped local_pose;

    public:

        void run(); 
        // const ros::TimerEvent& event

        void up_fly();

        void down_fly();

        void up_down_fly(const ros::TimerEvent& event);

        void execute_cb(const actionlib::SimpleActionServer<uwb_multi_robot_sim::FlyTaskAction>::GoalConstPtr& goal);

        void state_cb(const mavros_msgs::State::ConstPtr& msg){
            current_state_ = *msg;
        }

        void current_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
            current_pose_ = *msg;
        }

        void mavros_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
        
};