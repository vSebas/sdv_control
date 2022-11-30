/** ----------------------------------------------------------------------------
 * @file: car_tf2_broadcast_node.cpp
 * @date: November 29, 2022
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * @author: Montserrat Cruz
 * 
 * @brief: ROS tf broadcast node for the car. Uses car_simulation library.
 * -----------------------------------------------------------------------------
 **/

#include "6dof_tf2_broadcaster.hpp"

#include <ros/ros.h>
#include <string.h>

int main(int argc, char **argv){
    int frequency = 100;
    std::string parent_frame;
    std::string child_frame;

    ros::init(argc, argv, "tf2_broadcaster_node");
    ros::NodeHandle nh("~");
    
    nh.param<std::string>("parent_frame", parent_frame, "world");
    nh.param<std::string>("child_frame", child_frame, "car");
    nh.getParam("node_frequency",frequency);

    ros::Rate cycle_rate(frequency);
    TF2Broadcaster tf_broadcaster(parent_frame, child_frame);
    
    ros::Publisher  car_path    = nh.advertise<nav_msgs::Path>("/car_simulation/car_tf_broadcast/car_path", 1000);
    
    ros::Subscriber car_pose    = nh.subscribe("/car_simulation/dynamic_model/eta_pose", 
                                               10, 
                                               &TF2Broadcaster::BroadcastTransform, 
                                               &tf_broadcaster);
    
    while(ros::ok())
    {
        /* Run Queued Callbacks */
        ros::spinOnce();

        /* Publish Path */
        car_path.publish(tf_broadcaster.path);
        
        /* Sleep for N ms */
        cycle_rate.sleep();
    }

    return 0;
}