#include "fly_simulate/fly_simulate.h"

#include <sensor_msgs/Range.h>


static ros::Time begin_;

fly_sim::fly_sim() :
    fly_server_(nh_, "fly_action", boost::bind(&fly_sim::execute_cb, this, _1), false)
{
    fly_server_.start();
    state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &fly_sim::state_cb, this);
    current_pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &fly_sim::current_pos_cb, this); // "/dwm1001/tago/pose"
    // current_pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &fly_sim::current_pos_cb, this); // "/dwm1001/tago/pose"
    // mavros_pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &fly_sim::mavros_pos_cb, this);
    waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel", 10);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    

}

fly_sim::~fly_sim(){

}

void fly_sim::run(){
     //the setpoint publishing rate MUST be faster than 2Hz required by offboard mode
    ros::Rate rate(10.0);
    
    // wait for FCU connection
    // ROS_DEBUG_STREAM();
    while(ros::ok() && !current_state_.connected){
        ros::spinOnce();
        rate.sleep();
    }
    for (size_t i = 0; i < 80; i++)
    {
        if(current_state_.connected)
            break;
        else
            rate.sleep();
    }
    if(!current_state_.connected){
        ROS_ERROR_STREAM("Error! Could not connect to FCU... Exiting now.");
    }
    
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    // Send a few waypoints before starting (just the current position
    ROS_DEBUG_STREAM("Sending a few waypoints before starting (2 seconds)...");
    for(int i = 40; ros::ok() && i > 0; --i){
        waypoint_pub_.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode_;
    offb_set_mode_.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd_;
    arm_cmd_.request.value = true;

    ros::Time last_request = ros::Time::now();
    bool set_flag =  true;

    while(ros::ok()){
		if( current_state_.mode != "OFFBOARD" &&
				(ros::Time::now() - last_request > ros::Duration(2.0))){
				if( set_mode_client_.call(offb_set_mode_) &&
						offb_set_mode_.response.mode_sent){
						ROS_INFO_STREAM("Offboard Enabled");
				}
				last_request = ros::Time::now();
		} else {
				if( !current_state_.armed &&
						(ros::Time::now() - last_request > ros::Duration(2.0))){
						if( arming_client_.call(arm_cmd_) &&
								arm_cmd_.response.success){
								ROS_INFO_STREAM("Hahaha, UAV Armed");
						}
						last_request = ros::Time::now();
				}
		}
        ros::spinOnce();
		rate.sleep();
    }
}


void fly_sim::up_fly(){
    geometry_msgs::PoseStamped next_waypoint;
    next_waypoint.pose.position.x = - current_pose_.pose.position.x;
    next_waypoint.pose.position.y = - current_pose_.pose.position.y;
    next_waypoint.pose.position.z = current_pose_.pose.position.z + 0.5;

    next_waypoint.pose.orientation.x = current_pose_.pose.orientation.x;
    next_waypoint.pose.orientation.y = current_pose_.pose.orientation.y;
    next_waypoint.pose.orientation.z = current_pose_.pose.orientation.z;
    next_waypoint.pose.orientation.w = current_pose_.pose.orientation.w;
    next_waypoint.header.stamp = ros::Time::now();  
    waypoint_pub_.publish(next_waypoint);
}


void fly_sim::down_fly(){
    geometry_msgs::PoseStamped next_waypoint;
    next_waypoint.pose.position.x = - current_pose_.pose.position.x;
    next_waypoint.pose.position.y = - current_pose_.pose.position.y;
    next_waypoint.pose.position.z = current_pose_.pose.position.z - 0.5;

    next_waypoint.pose.orientation.x = current_pose_.pose.orientation.x;
    next_waypoint.pose.orientation.y = current_pose_.pose.orientation.y;
    next_waypoint.pose.orientation.z = current_pose_.pose.orientation.z;
    next_waypoint.pose.orientation.w = current_pose_.pose.orientation.w;
    next_waypoint.header.stamp = ros::Time::now();  
    waypoint_pub_.publish(next_waypoint);
}


void fly_sim::up_down_fly(const ros::TimerEvent& event){
    float x_diff = fabs(target_pose_.pose.position.x - current_pose_.pose.position.x);
    float y_diff = fabs(target_pose_.pose.position.y - current_pose_.pose.position.y);
    float z_diff = fabs(target_pose_.pose.position.z - current_pose_.pose.position.z);
    ROS_INFO_STREAM(x_diff << " ,"<< y_diff << " ," << z_diff);
    if(x_diff > reach_tolerance_ || y_diff > reach_tolerance_ || z_diff > reach_tolerance_)
    {
        geometry_msgs::PoseStamped next_waypoint;
        next_waypoint = target_pose_;
        next_waypoint.pose.orientation.x = current_pose_.pose.orientation.x;
        next_waypoint.pose.orientation.y = current_pose_.pose.orientation.y;
        next_waypoint.pose.orientation.z = current_pose_.pose.orientation.z;
        next_waypoint.pose.orientation.w = current_pose_.pose.orientation.w;
        next_waypoint.header.stamp = ros::Time::now();  

        waypoint_pub_.publish(next_waypoint);
    }
    else
    {
        ROS_INFO_STREAM(">>>>>>>> Congrat. Reach the Target <<<<<<<<");
        bool success = true;
        if(fly_server_.isPreemptRequested() || !ros::ok()){
            fly_server_.setPreempted();
        }
        else{
            uwb_multi_robot_sim::FlyTaskResult result;
            result.is_reached = true;
            fly_server_.setSucceeded(result);
        }

        uwb_multi_robot_sim::FlyTaskFeedback feedback;
        feedback.err_x = x_diff;
        feedback.err_y = y_diff;
        feedback.err_z = z_diff;
        fly_server_.publishFeedback(feedback);

        fly_timer_.stop();
    }
}

void fly_sim::execute_cb(const actionlib::SimpleActionServer<uwb_multi_robot_sim::FlyTaskAction>::GoalConstPtr& goal){
    target_pose_.pose.position.x = current_pose_.pose.position.x + goal->x;
    target_pose_.pose.position.y = current_pose_.pose.position.y + goal->y;
    target_pose_.pose.position.z = current_pose_.pose.position.z + goal->z;
    target_pose_.pose.orientation.w = current_pose_.pose.orientation.w + goal->o_w;
    target_pose_.pose.orientation.x = current_pose_.pose.orientation.x + goal->o_x;
    target_pose_.pose.orientation.y = current_pose_.pose.orientation.y + goal->o_y;
    target_pose_.pose.orientation.z = current_pose_.pose.orientation.z + goal->o_w;
    reach_tolerance_ = goal->err_t;
    if(goal->x > goal->err_t || goal->y > goal->err_t || goal->z > goal->err_t){
        fly_timer_ = nh_.createTimer(ros::Duration(0.1), &fly_sim::up_down_fly, this);
    }

}


int main(int argc, char **argv){
    ros::init(argc, argv, "fly_simulate");

    ros::NodeHandle nh;

    fly_sim fly_s;
    
    // begin_ = ros::Time::now();
    // ros::Timer timer = nh.createTimer(ros::Duration(0.1), &fly_sim::up_down_fly, &fly_s);

    fly_s.run();

    return 0;
}