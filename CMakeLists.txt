cmake_minimum_required(VERSION 3.0.2)
project(sdv_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    tf2_ros
    roscpp
    vanttec_msgs
    # nodelet
    # pluginlib
)
catkin_package()

include_directories(
    vanttec_controllers/common_lib/include
    vanttec_controllers/dynamic_models/ground_vehicles/include
    vanttec_controllers/simulation/include
    vanttec_controllers/controllers/guidance_laws/include
    # vanttec_controllers/controllers/feedback_linearization/PID_based/include
    # vanttec_controllers/controllers/feedback_linearization/SMC_based/ASMC/include
    # vanttec_controllers/controllers/feedback_linearization/SMC_based/ANTSMC/include
    ${catkin_INCLUDE_DIRS}
)

add_executable(tf2_broadcaster_node
    src/model/car_tf2_broadcast_node.cpp
    vanttec_controllers/simulation/src/6dof_tf2_broadcaster.cpp)
target_link_libraries(tf2_broadcaster_node ${catkin_LIBRARIES})

add_executable(car_simulation_node
    src/model/car_simulation_node.cpp
    vanttec_controllers/dynamic_models/ground_vehicles/src/el_rasho.cpp
    vanttec_controllers/dynamic_models/ground_vehicles/src/car_3dof_dynamic_model.cpp)
target_link_libraries(car_simulation_node ${catkin_LIBRARIES})

add_executable(stanley_controller_node
    src/controllers/cascaded_schemes/car_guidance_control.cpp
    vanttec_controllers/controllers/guidance_laws/src/stanley_controller.cpp)
target_link_libraries(stanley_controller_node ${catkin_LIBRARIES})

# add_executable(vtec_u4_pid_node
#     src/controllers/first_order/vtec_u4_pid_control_node.cpp
#     vanttec_controllers/controllers/feedback_linearization/PID_based/src/6dof_pid.cpp
#     vanttec_controllers/controllers/feedback_linearization/PID_based/src/pid.cpp
#     vanttec_controllers/common_lib/src/common.cpp)
# target_link_libraries(vtec_u4_pid_node ${catkin_LIBRARIES})