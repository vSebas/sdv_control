/** ----------------------------------------------------------------------------
 * @file: car_simulation_node.cpp
 * @date: November 29, 2022
 * @author: Sebas Martinez
 * @email: sebas.martp@gmail.com
 * @author: Montserrat Cruz
 * @brief: ROS simulation node for the el rasho car class.
 * -----------------------------------------------------------------------------
 **/

#include "el_rasho.hpp"
#include "vanttec_msgs/EtaPose.h"
#include "vanttec_msgs/SystemDynamics.h"
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32.h>

#include <ros/ros.h>
#include <stdio.h>

int main(int argc, char **argv){
    int frequency = 100;
    ros::init(argc, argv, "el_rasho_simulation_node");
    ros::NodeHandle nh("~");

    nh.getParam("node_frequency",frequency);    
    ros::Rate cycle_rate(frequency);
    Cafe car_model(float(1.0/frequency));
    vanttec_msgs::SystemDynamics  car_functions;
    
    ros::Publisher  car_accel     = nh.advertise<geometry_msgs::Vector3>("/vectornav/ins_3d/ins_acc", 10);
    ros::Publisher  car_vel       = nh.advertise<geometry_msgs::Twist>("/car_simulation/dynamic_model/vel", 10);
    ros::Publisher  car_eta_pose  = nh.advertise<vanttec_msgs::EtaPose>("/car_simulation/dynamic_model/eta_pose", 10);
    ros::Publisher  car_dynamics  = nh.advertise<vanttec_msgs::SystemDynamics>("/car_simulation/dynamic_model/non_linear_functions", 10);

    ros::Subscriber car_force_input = nh.subscribe("/car_control/car_control_node/force", 
                                                    10, 
                                                    &CarDynamicModel::setForceInput,
                                                    dynamic_cast<CarDynamicModel*> (&car_model));

    ros::Subscriber car_steering_input = nh.subscribe("/car_control/car_control_node/steering", 
                                                    10, 
                                                    &CarDynamicModel::setSteeringInput,
                                                    dynamic_cast<CarDynamicModel*> (&car_model));
            
    car_functions.g.layout.dim.push_back(std_msgs::MultiArrayDimension());
    car_functions.g.layout.dim.push_back(std_msgs::MultiArrayDimension());
    car_functions.g.layout.dim[0].label = "rows";
    car_functions.g.layout.dim[1].label = "cols";
    car_functions.g.layout.dim[0].size = 3;
    car_functions.g.layout.dim[1].size = 3;
    car_functions.g.layout.dim[0].stride = 3;
    car_functions.g.layout.data_offset = 0;

    while(ros::ok())
    {
        /* Run Queued Callbacks */
        ros::spinOnce();

        /* calculate Model States */
        car_model.calculateStates();

        /* Publish Odometry */
        car_accel.publish(car_model.accelerations_);
        car_vel.publish(car_model.velocities_);
        car_eta_pose.publish(car_model.eta_pose_);
        
        /* Publish nonlinear functions */

        car_functions.f = {car_model.f_(0), car_model.f_(1), car_model.f_(2)};
        car_functions.g.data = { car_model.g_(0,0), car_model.g_(0,1), car_model.g_(0,2),
                                 car_model.g_(1,0), car_model.g_(1,1), car_model.g_(1,2),
                                 car_model.g_(2,0), car_model.g_(2,1), car_model.g_(2,2)};
        
        car_dynamics.publish(car_functions);

        /* Sleep for 10ms */
        cycle_rate.sleep();
    }

    return 0;
}