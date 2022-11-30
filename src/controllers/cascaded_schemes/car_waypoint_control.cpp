/** ----------------------------------------------------------------------------
 * @file: car_guidance_control.cpp
 * @date: November 29, 2022
 * @author: Sebastian Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: control node for a car casded guidance-control scheme.
 *         The guidance law is an implementation of a simple los controller
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
#include <vanttec_msgs/EtaPose.h>

#include "pid.hpp"

geometry_msgs::Twist car_velocity;
vanttec_msgs::EtaPose car_pose;
float f = 0;
float g = 0;

float calculatePathAngle(float x0, float y0, float x1, float y1){
    float ex = x1 - x0;
    float ey = y1 - y0;

    // Angle of path frame
    return std::atan2(ey,ex);
}

void poseCallback(const vanttec_msgs::EtaPose& pose){
    car_pose = pose;
}

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

    // PD controller gains
    float kp = 1;
    float kd = 1;

    // Path
    float x0 = 0;
    float y0 = -10;
    float x1 = 0;
    float y1 = 10;
    float beta;

    float error = 0;

    float DELTA_MAX = 100;

    ros::init(argc, argv, "sdv_controller");
    ros::NodeHandle nh("~");

    nh.getParam("node_frequency",frequency);    
    ros::Rate cycle_rate(frequency);

    PID pid(float(1.0/frequency), kp, 0, kd, 340, LINEAR_DOF);
    
    ros::Subscriber  car_eta = nh.subscribe("/car_simulation/dynamic_model/eta_pose", 
                                                    1, 
                                                    poseCallback);
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

        beta = calculatePathAngle(car_pose.x,car_pose.y,x0,y0);
        delta.data = -(car_pose.psi - beta);
        ROS_INFO_STREAM("beta "<<beta);
        ROS_INFO_STREAM("delta "<<delta.data);
        ROS_INFO_STREAM("psi "<<car_pose.psi);
        
        if (delta.data >= DELTA_MAX)
            delta.data = DELTA_MAX;
        else if (delta.data <= -DELTA_MAX)
            delta.data = -DELTA_MAX;

        car_steering.publish(delta);

        /* PD controller */
        // error = std::sqrt(std::pow(x0 - car_pose.x,2) + std::pow(y0 - car_pose.y,2));
        pid.updateFunctions(f, g);
        pid.updateSetpoint(3,0);
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