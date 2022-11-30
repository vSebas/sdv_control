/** ----------------------------------------------------------------------------
 * @file: car_guidance_control.cpp
 * @date: November 29, 2022
 * @author: Sebastian Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: control node for a car casded guidance-control scheme.
 *         The guidance law is an implementation of a simple Stanley controller
 *         that provides a desired steering angle.
 *         The low level controller takes as inputs the steering command and a 
 *         user defined velocity, and generates a force to move the vehicle.
 * -----------------------------------------------------------------------------
 **/

#include <stdio.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <vanttec_msgs/SystemDynamics.h>
#include <vanttec_msgs/ThrustControl.h>

#include "stanley_controller.hpp"
#include "pid.hpp"

geometry_msgs::Twist car_velocity;
// vanttec_msgs::SystemDynamics functions;
float f = 0;
float g = 0;

void velCallback(const geometry_msgs::Twist& vel){
    car_velocity = vel;
}

void dynamicsCallback(const vanttec_msgs::SystemDynamics& non_linear_functions){
    // functions = non_linear_functions;
    f = non_linear_functions.f[0];
    g = non_linear_functions.g.data[0];
}

int main(int argc, char **argv){
    int frequency = 100;
    float v_norm;
    
    std_msgs::Float32 delta;
    vanttec_msgs::ThrustControl u;
    nav_msgs::Path path;

    // Stanley controller params
    float delta_max = 1;
    float k = 0.3;
    float k_soft = 1;

    // PD controller gains
    float kp = 1;
    float kd = 1;

    // Path
    float x0 = 0;
    float y0 = -10;
    float x1 = 0;
    float y1 = 10;

    ros::init(argc, argv, "stanley_controller");
    ros::NodeHandle nh("~");

    nh.getParam("node_frequency",frequency);    
    ros::Rate cycle_rate(frequency);

    StanleyController stanley(delta_max, k, k_soft);
    PID pid(float(1.0/frequency), kp, 0, kd, 340, LINEAR_DOF);
    
    ros::Subscriber  stanley_heading = nh.subscribe("/car_simulation/dynamic_model/eta_pose", 
                                                    10, 
                                                    &StanleyController::setHeading,
                                                    &stanley);
    ros::Subscriber car_vel = nh.subscribe("/car_simulation/dynamic_model/vel", 1, velCallback);
    ros::Subscriber car_dynamics    = nh.subscribe("/car_simulation/dynamic_model/non_linear_functions", 1, 
                                                    dynamicsCallback);

    ros::Publisher  car_steering = nh.advertise<std_msgs::Float32>("/car_control/car_control_node/steering", 1);
    ros::Publisher  car_force = nh.advertise<vanttec_msgs::ThrustControl>("/car_control/car_control_node/force", 1);
    ros::Publisher  follow_path    = nh.advertise<nav_msgs::Path>("/car_path_to_follow", 1);

    while(ros::ok())
    {
        /* Run Queued Callbacks */
        ros::spinOnce();

        /* Stanley controller */
        stanley.calculateCrosstrackError(x0,y0,x1,y1);
        // v_norm = std::sqrt(std::pow(car_velocity.linear.x,2) + std::pow(car_velocity.linear.y,2));
        stanley.calculateSteering(car_velocity.linear.x);
        delta.data = stanley.delta_;
        car_steering.publish(delta);

        /* PD controller */
        pid.updateFunctions(f, g);
        pid.updateSetpoint(4,0);
        pid.calculateManipulation(car_velocity.linear.x);
        u.tau_x = pid.u_;
        car_force.publish(u);

        geometry_msgs::PoseStamped pose;

        pose.header.stamp       = ros::Time::now();
        pose.header.frame_id    = "world";
        pose.pose.position.x    = x0;
        pose.pose.position.y    = y0;

        path.header.stamp     = ros::Time::now();
        path.header.frame_id  = "world";
        path.poses.push_back(pose);
        
        pose.header.stamp       = ros::Time::now();
        pose.header.frame_id    = "world";
        pose.pose.position.x    = x1;
        pose.pose.position.y    = y1;

        path.header.stamp     = ros::Time::now();
        path.header.frame_id  = "world";
        path.poses.push_back(pose);

        follow_path.publish(path);

        /* Sleep for 10ms */
        cycle_rate.sleep();
    }

    return 0;
}