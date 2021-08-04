#include "uwbsimulator.h"

UWBSimulator::UWBSimulator(ros::NodeHandle& nh)
{

    ROS_INFO_STREAM("Test");
    
    // Topics
    nh.param<std::string>("ground_truth_topic", ground_truth_topic_, "/gazebo/model_states");
    nh.param<std::string>("pub_topic_prefix", pub_topic_prefix_, "/uwb");

    // UWB global params
    nh.param<double>("duty_cycle", duty_cycle_, 1);
    nh.param<double>("max_twr_freq", max_twr_freq_, 400);

    // Models
    nh.param<std::vector<std::string>>("/uwbsimulator/model_names", model_names_, std::vector<std::string>());
    std::cout << "Models to read GT from: " << std::endl;
    for (std::string &model: model_names_)
    {
        std::cout << "  --> " << model << std::endl;
        // Initialize the model ground truth pose
        models_gt_.insert(
            std::pair<std::string, geometry_msgs::Pose>(
                model,
                geometry_msgs::Pose()
            )
        );
    }

    // UWB nodes
    std::cout << "Reading UWB nodes: " << std::endl;
    for (std::string &model: model_names_)
    {
        int num_of_nodes;
        std::vector<std::string> node_names;
        std::vector<double> node_poses_flat;
        nh.param<int>("/uwbsimulator/uwb_nodes/"+model+"/num", num_of_nodes, 0);
        nh.param<std::vector<double>>("/uwbsimulator/uwb_nodes/"+model+"/poses", node_poses_flat, std::vector<double>(0));
        nh.param<std::vector<std::string>>("/uwbsimulator/uwb_nodes/"+model+"/names", node_names, std::vector<std::string>());
        std::cout << " --> " + model + " has " << num_of_nodes << " UWB nodes." << std::endl;

        // Now fill in the map with node positions
        std::vector<std::vector<double>> node_poses(num_of_nodes, std::vector<double>(3, 0));
        for (int i=0; i < num_of_nodes*3; i++)
        {
            node_poses[int(i/3)][i%3] = node_poses_flat[i];
        }

        // Add the position of the node
        uwb_nodes_.insert(
            std::pair<std::string, std::vector<std::vector<double>>>(
                model,
                node_poses
            )
        );

        // Print poses
        std::cout << "   [ " << std::endl;
        for (int i = 0; i < node_poses.size(); i++)
        {
            std::cout << "     [ ";
            for (int j = 0; j < node_poses[i].size(); j++)
            {
                std::cout << node_poses[i][j] << ", ";
            }
            std::cout << "]" << std::endl;
        }
        std::cout << "   ] " << std::endl;

        // Save node names
        uwb_node_names_.insert(
            std::pair<std::string, std::vector<std::string>>(
                model,
                node_names
            )
        );
    }
    
    // Ranges
    nh.param<std::vector<std::string>>("/uwbsimulator/uwb_ranges", uwb_ranges_param_, std::vector<std::string>());
    std::cout << "Reading ranges: " << std::endl;
    for (std::string &range: uwb_ranges_param_)
    {
        std::string ori = range.substr(0, range.find("+"));
        std::string end = range.substr(range.find("+") + 1);
        std::cout << "  " << ori << " --> " << end << std::endl;

        // Add a range to the map
        uwb_ranges_.insert(
            std::pair<std::string, std::string>(ori, end)
        );

        for (int i=0; i<uwb_nodes_[ori].size(); i++)
        {
            for (int j=0; j<uwb_nodes_[end].size(); j++)
            {
                // Create range publisher struct
                range_publisher rpub = range_publisher(
                    ori,
                    end,
                    i,
                    j
                );

                // Create publisher object
                uwb_publishers_.insert(
                    std::pair<range_publisher, ros::Publisher>(
                        rpub,
                        nh.advertise<sensor_msgs::Range>(
                            pub_topic_prefix_ + "/from/" + ori + "/" + uwb_node_names_[ori][i] + "/to/" + end + "/" +  uwb_node_names_[end][j], 
                            10
                        )
                    )
                );
            }
        }
    }

    grount_truth_sub_ = nh.subscribe<gazebo_msgs::ModelStates>(ground_truth_topic_, 1, &UWBSimulator::ground_truth_callback, this); 

}




void UWBSimulator::ground_truth_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    std::vector<std::string> all_models = msg->name;
    std::vector<geometry_msgs::Pose> all_poses = msg->pose;

    for (std::string &model: model_names_)
    {
        // Find position of model in Gazebo model list
        auto model_idx = std::find(all_models.begin(), all_models.end(), model);
        if (model_idx != all_models.end())
        {  
            // Update model pose
            int idx = std::distance(all_models.begin(), model_idx); 
            models_gt_[model] = all_poses[idx];

            // Broadcast transform
            // tf::Transform transform;
            // transform.setOrigin( tf::Vector3(all_poses[idx].position.x, all_poses[idx].position.y, all_poses[idx].position.z) );
            // tf::Quaternion q;
            // tf2::convert(all_poses[idx].orientation , q);
            // transform.setRotation(q);
            // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", model));
        }
    }
    
}

boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> UWBSimulator::calc_uwb_node_pose(
    geometry_msgs::Pose vehicle_pose, std::vector<double> uwb_relative_pos)
{
    /*
        We use here "efficient" quaternion multiplication.
        Objective is to rotate a vector v (uwb node relative pose)
        by a unit-length quaternion q, rewritten as:

            q=(ux,uy,uz,s)⇔q=(u,s)

        The result can be calculated as:

            v' = 2(u⋅v)u+(s2−u⋅u)v+2s(u×v)
        
        with just two dot products and one cross product.
    */
    boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> v_base(
        vehicle_pose.position.x,
        vehicle_pose.position.y,
        vehicle_pose.position.z
    );
    boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> v(
        uwb_relative_pos[0],
        uwb_relative_pos[1],
        uwb_relative_pos[2]
    );
    boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> u(
        vehicle_pose.orientation.x,
        vehicle_pose.orientation.y,
        vehicle_pose.orientation.z
    );
    double s = vehicle_pose.orientation.w;

    boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> v_final, part2, part3;
    v_final = u; 
    boost::geometry::multiply_value(v_final, 2 * boost::geometry::dot_product(u, v));
    part2 = v;
    boost::geometry::multiply_value(part2, (s * s - boost::geometry::dot_product(u, u)));
    part3 = boost::geometry::cross_product(u, v);
    boost::geometry::multiply_value(part3, 2 * s);
    boost::geometry::add_point(v_final, part2);
    boost::geometry::add_point(v_final, part3);
    boost::geometry::add_point(v_final, v_base);

    return v_final;
}

void UWBSimulator::publish_ranges(const ros::TimerEvent& event)
{
    // Calculate position of all UWB nodes relative to the models
    for(const auto& elem : uwb_publishers_)
    {
        
        boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> v_ori, v_end;
        v_ori = calc_uwb_node_pose(models_gt_[elem.first.ori_node],
            uwb_nodes_[elem.first.ori_node][elem.first.ori_id]);
        v_end = calc_uwb_node_pose(models_gt_[elem.first.end_node],
            uwb_nodes_[elem.first.end_node][elem.first.end_id]);
    
        double real_range = boost::geometry::distance(v_ori, v_end);

        sensor_msgs::Range r = sensor_msgs::Range();
        r.header.stamp = ros::Time::now();
        r.radiation_type = 2;
        r.field_of_view = 0;
        r.min_range = 0.2;
        r.max_range = 42;
        r.range = real_range;

        elem.second.publish(r);

        // tf::TransformListener listener;
        // tf::StampedTransform transform;
        // try{
        //     listener.lookupTransform("/world", "/" + elem.first.node_ori, ros::Time(0), transform);
        // }
        // catch (tf::TransformException ex){
        //     ROS_ERROR("%s",ex.what());
        //     ros::Duration(1.0).sleep();
        //     std::cout << "Could not find TF !" << std::endl;
        // }
        // tf2::doTransform()

    }
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "mrs_uwb_sim_node");
    ros::NodeHandle nh;
    UWBSimulator uwb_ranging_sim = UWBSimulator(nh);

    ros::Timer timer = nh.createTimer(ros::Duration(0.2), &UWBSimulator::publish_ranges, &uwb_ranging_sim);
    ros::spin();

    return 0;
}
