#ifndef UWB_SIMULATOR_H
#define UWB_SIMULATOR_H

#include <vector>
#include <algorithm>

#include <boost/geometry.hpp>
#include <boost/geometry/arithmetic/dot_product.hpp>
#include <boost/geometry/arithmetic/cross_product.hpp>

#include <ros/ros.h>

#include <sensor_msgs/Range.h>

#include <geometry_msgs/Pose.h>

#include <gazebo_msgs/ModelStates.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


/*
    Struct to store a range to be calculated
    including two models (nodes) and the ID of
    the UWB node in each of them.
*/
struct range_publisher
{
    std::string ori_node;
    std::string end_node;
    int ori_id;
    int end_id;

    range_publisher(std::string n1, std::string n2, int id1, int id2)
    {
        ori_node = n1;
        end_node = n2;
        ori_id = id1;
        end_id = id2;
    }

    bool const operator==(const range_publisher &o) const {
        return ori_node + std::to_string(ori_id) + end_node + std::to_string(end_id) == o.ori_node + std::to_string(o.ori_id) + o.end_node + std::to_string(o.end_id);
    }

    bool const operator<(const range_publisher &o) const {
        return ori_node + std::to_string(ori_id) + end_node + std::to_string(end_id) < o.ori_node + std::to_string(o.ori_id) + o.end_node + std::to_string(o.end_id);
    }

};

std::ostream &operator<<(std::ostream &out, const range_publisher &o)
{
    out << "/from/" + o.ori_node + "/" + std::to_string(o.ori_id) + "/to/" + o.end_node + "/" + std::to_string(o.end_id);
    return out;
}

/*
    UWB Simulator class.
    Subscribes to Gazebo model states (ground truth).
    Publishes UWB ranges between nodes specified in the config.
*/
class UWBSimulator 
{

    public:
        // Constructors
        UWBSimulator();
        UWBSimulator(ros::NodeHandle& nh);

        // Timers
        void publish_ranges(const ros::TimerEvent& event);

    private:
        // UWB
        double duty_cycle_;
        double max_twr_freq_;
        
        // Topic names
        std::string ground_truth_topic_;
        std::string pub_topic_prefix_;

        // Configuration
        std::vector<std::string> model_names_;
        std::map<std::string, std::vector<std::vector<double>>> uwb_nodes_;
        std::map<std::string, std::vector<std::string>> uwb_node_names_;
        std::vector<std::string> uwb_ranges_param_;
        std::map<std::string, std::string> uwb_ranges_;

        // Model states
        std::map<std::string, geometry_msgs::Pose> models_gt_;       // Ground truth for the models

        // ROS subscribers
        ros::Subscriber grount_truth_sub_;

        // ROS publishers
        std::map<range_publisher, ros::Publisher> uwb_publishers_;

        // Topic callbacks
        void ground_truth_callback(const gazebo_msgs::ModelStates::ConstPtr& msg);

        // Utils
        boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> calc_uwb_node_pose(
            geometry_msgs::Pose vehicle_pose, std::vector<double> uwb_relative_pos);
};

#endif