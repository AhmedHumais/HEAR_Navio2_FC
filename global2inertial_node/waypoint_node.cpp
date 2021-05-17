// TODO: remove this node altogether

#include <iostream>
#include <vector>
#include <pthread.h>
#include <sched.h>

#include "ros/ros.h"
#include "HEAR_core/std_logger.hpp"
#include "HEAR_core/Mux3D.hpp"
#include "HEAR_core/Demux3D.hpp"
#include "HEAR_ROS_BRIDGE//ROSUnit_Factory.hpp"


int main(int argc, char** argv) {
    std::cout << "*** Waypoint Node Started ***" << std::endl;
    // //*****************************LOGGER********************************** 
    Logger::assignLogger(new StdLogger());
    // //****************************ROS UNITS*******************************
    ros::init(argc, argv, "waypoint_node");
    ros::NodeHandle nh;
    ros::Rate rate(200);

    ROSUnit_Factory ROSUnit_Factory_main{nh};

    ROSUnit* rosunit_waypoint_x = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "waypoint_reference/x");//0
    ROSUnit* rosunit_waypoint_y = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "waypoint_reference/y");//1
    ROSUnit* rosunit_waypoint_z = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "waypoint_reference/z");//2
    ROSUnit* rosunit_waypoint_yaw = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "waypoint_reference/yaw");//3 

    ROSUnit* ref_pos_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "pos_ref_h");                                                                                                                     
    ROSUnit* ref_yaw_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "yaw_ref");

    auto mux_pos_ref = new Mux3D();
    rosunit_waypoint_x->getPorts()[(int)ROSUnit_FloatSub::ports_id::OP_0]->connect(mux_pos_ref->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    rosunit_waypoint_y->getPorts()[(int)ROSUnit_FloatSub::ports_id::OP_1]->connect(mux_pos_ref->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    rosunit_waypoint_z->getPorts()[(int)ROSUnit_FloatSub::ports_id::OP_2]->connect(mux_pos_ref->getPorts()[(int)Mux3D::ports_id::IP_2_DATA]);
    rosunit_waypoint_yaw->getPorts()[(int)ROSUnit_FloatSub::ports_id::OP_3]->connect(ref_yaw_pub->getPorts()[(int)ROSUnit_FloatPub::ports_id::IP_0]);

    mux_pos_ref->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(ref_pos_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    
    // Timer tempo;
    while(ros::ok()){
        // tempo.tick();

        // ros::spinOnce();

        // int gone = tempo.tockMicroSeconds();
        // if(gone > 5000) {
        //     // std::cout  << "FC over 5000: " << gone << "\n";
        // }
        // rate.sleep();
        ros::spin();

    }
    return 0;

}