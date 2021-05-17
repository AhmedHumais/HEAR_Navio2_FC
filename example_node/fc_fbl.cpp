// TODO: where to put x and y saturations

#include <iostream>
#include <vector>
#include <pthread.h>
#include <sched.h>

#include "ros/ros.h"
#include "HEAR_core/std_logger.hpp"
#include "HEAR_core/Switch.hpp"
#include "HEAR_core/Mux3D.hpp"
#include "HEAR_core/Demux3D.hpp"
#include "HEAR_core/InvertedSwitch.hpp"
#include "HEAR_math/Differentiator.hpp"
#include "HEAR_math/Sum.hpp"
#include "HEAR_math/Saturation.hpp"
#include "HEAR_math/HoldVal.hpp"
#include "HEAR_math/KalmanFilter.hpp"
#include "HEAR_math/AvgFilter.hpp"
#include "HEAR_math/NegateFloat.hpp"
#include "HEAR_math/FbLinearizer.hpp"
#include "HEAR_control/PIDController.hpp"
#include "HEAR_control/MRFTController.hpp"
#include "HEAR_control/BoundingBoxController.hpp"
#include "HEAR_actuation/HexaActuationSystem.hpp"
#include "HEAR_nav/WrapAroundFunction.hpp"
#include "HEAR_nav/Global2Inertial.hpp"
#include "HEAR_nav/RestrictedNormWaypointRefGenerator.hpp"
#include "HEAR_nav/Transform_InertialToBody.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_Optitrack.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_UpdateControllerSrv.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_BroadcastData.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_IMU.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_RestNormSettings.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_Factory.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_PoseProvider.hpp"
#include "HEAR_NAVIO_Interface/ESCMotor.hpp"
#include "HEAR_NAVIO_Interface/BatteryMonitor.hpp"

#define XSENS_OVER_ROS
#define OPTITRACK
#define BIG_HEXA
#undef BATTERY_MONITOR
//#define MRFT_SLAM

const int OPTI_FREQUENCY = 120;
const int PWM_FREQUENCY = 200;
const float SATURATION_VALUE_XY = 0.2617; 
const float SATURATION_VALUE_YAW = 0.2617;
const float SATURATION_VALUE_YAWRATE = 0.3;

int main(int argc, char** argv) {
    std::cout << "Hello Flight Controller!" << std::endl;
    // //*****************************LOGGER********************************** 
    Logger::assignLogger(new StdLogger());
    // //****************************ROS UNITS*******************************
    ros::init(argc, argv, "flight_controller_node");
    ros::NodeHandle nh;
    ros::Rate rate(200);
    ROSUnit_Factory ROSUnit_Factory_main{nh};
    ROSUnit* myROSUpdateController = new ROSUnit_UpdateControllerSrv(nh);
    ROSUnit* myROSBroadcastData = new ROSUnit_BroadcastData(nh);

    ROSUnit* myROSArm = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                            ROSUnit_msg_type::ROSUnit_Bool,
                                                            "arm");
    ROSUnit* myROSResetController = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                      ROSUnit_msg_type::ROSUnit_Int8,
                                                                      "reset_controller");
                                                       
    ROSUnit* xh_ref = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "pos_ref_h");//0

    ROSUnit* yaw_ref = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "yaw_ref");//0

    ROSUnit* pos_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/pos");
    ROSUnit* ori_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/ori");                                                                
    ROSUnit* xh_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/pos_horizon");                                                                
    ROSUnit* rot_err_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/axis_angle_ref");                                                                


    // ROSUnit* mrft_pub_x = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Float,
    //                                                                 "/mrft_output/x");
    // ROSUnit* mrft_pub_y = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Float,
    //                                                                 "/mrft_output/y");
    // ROSUnit* mrft_pub_z = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Float,
    //                                                                 "/mrft_output/z");

    //**************************SETTING BLOCKS**********************************
    PIDController* PID_x = new PIDController(block_id::PID_X);
    PIDController* PID_x_slam = new PIDController(block_id::PID_SLAM_X);
    PIDController* PID_pitch = new PIDController(block_id::PID_PITCH);
    PIDController* PID_y = new PIDController(block_id::PID_Y);
    PIDController* PID_y_slam = new PIDController(block_id::PID_SLAM_Y);
    PIDController* PID_roll = new PIDController(block_id::PID_ROLL);
    PIDController* PID_z = new PIDController(block_id::PID_Z);
    PIDController* PID_z_slam = new PIDController(block_id::PID_SLAM_Z);
    PIDController* PID_yaw = new PIDController(block_id::PID_YAW);
    PIDController* PID_yaw_rate = new PIDController(block_id::PID_YAW_RATE);

    MRFTController* MRFT_x = new MRFTController(block_id::MRFT_X);
    MRFTController* MRFT_y = new MRFTController(block_id::MRFT_Y);
    MRFTController* MRFT_z = new MRFTController(block_id::MRFT_Z);
 
    Transform_InertialToBody* inertialToBody_RotMat = new Transform_InertialToBody();

    Saturation* X_Saturation = new Saturation(SATURATION_VALUE_XY);
    Saturation* Y_Saturation = new Saturation(SATURATION_VALUE_XY);
    Saturation* Yaw_Saturation = new Saturation(SATURATION_VALUE_YAW);
    Saturation* YawRate_Saturation = new Saturation(SATURATION_VALUE_YAWRATE);

    //*********************SETTING ACTUATION SYSTEMS************************
    
    Actuator* M1 = new ESCMotor(0, PWM_FREQUENCY);
    Actuator* M2 = new ESCMotor(1, PWM_FREQUENCY);
    Actuator* M3 = new ESCMotor(2, PWM_FREQUENCY);
    Actuator* M4 = new ESCMotor(3, PWM_FREQUENCY);
    Actuator* M5 = new ESCMotor(4, PWM_FREQUENCY);
    Actuator* M6 = new ESCMotor(5, PWM_FREQUENCY);

    std::vector<Actuator*> actuators{M1, M2, M3, M4, M5, M6};

    ActuationSystem* myActuationSystem = new HexaActuationSystem(actuators);
    #ifdef BIG_HEXA
    myActuationSystem->setESCValues(1165 ,1000, 2000);
    #else
    myActuationSystem->setESCValues(1140 ,1000, 2000);
    #endif
    
    // ActuationSystem* myActuationSystem = new QuadActuationSystem(actuators);

    // //***********************************SETTING CONNECTIONS***********************************
    // //========                                                                             =============
    // //|      |-------------->X_Control_System-->RM_X-->Saturation-->Roll_Control_System--->|           |
    // //| USER |-------------->Y_Control_System-->RM_Y-->Saturation-->Pitch_Control_System-->| Actuation |
    // //|      |-------------->Z_Control_System--------------------------------------------->|  System   |
    // //|      |-------------->Yaw_Control_System-->Saturation--->YawRate_Control_System---->|           |
    // //========                                                                             =============
    
    //*******************************************************************************************************************

    auto pose_provider = new ROSUnit_PoseProvider(nh);
    auto x_dot = new Differentiator(1./OPTI_FREQUENCY);
    auto y_dot = new Differentiator(1./OPTI_FREQUENCY);
    auto z_dot = new Differentiator(1./OPTI_FREQUENCY);
    
    auto demux_opti_ori = new Demux3D();
    auto demux_imu_ori = new Demux3D();
    auto demux_xh = new Demux3D();
    auto demux_x_ref = new Demux3D();
    auto demux_rot_err = new Demux3D();
    auto demux_body_rate = new Demux3D();
    
    
    auto err_sum_x = new Sum(std::minus<float>());
    auto err_sum_y = new Sum(std::minus<float>());
    auto err_sum_z = new Sum(std::minus<float>());
    auto err_sum_yaw_rate = new Sum(std::minus<float>());
    
    auto mux_FH_des = new Mux3D();
    auto mux_e_xh = new Mux3D();
    auto mux_e_yh = new Mux3D();
    auto mux_e_zh = new Mux3D();

    auto mux_e_roll = new Mux3D();
    auto mux_e_pitch = new Mux3D();
    auto mux_e_yaw = new Mux3D();
    auto mux_e_yaw_rate = new Mux3D();
    
    auto fb_linearizer = new FbLinearizer();
    
    pose_provider->getPorts()[(int)ROSUnit_PoseProvider::OP_1_ORI_OPTI]->connect(demux_opti_ori->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);
    pose_provider->getPorts()[(int)ROSUnit_PoseProvider::OP_2_ORI_IMU]->connect(demux_imu_ori->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);

    // connecting inputs of feedback linearizer
    pose_provider->getPorts()[(int)ROSUnit_PoseProvider::ports_id::OP_0_POS]->connect(fb_linearizer->getPorts()[(int)FbLinearizer::ports_id::IP_X]);
    demux_opti_ori->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(fb_linearizer->getPorts()[(int)FbLinearizer::ports_id::IP_YAW]);
    demux_imu_ori->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(fb_linearizer->getPorts()[(int)FbLinearizer::ports_id::IP_ROLL]);
    demux_imu_ori->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(fb_linearizer->getPorts()[(int)FbLinearizer::ports_id::IP_PITCH]);
    yaw_ref->getPorts()[(int)ROSUnit_FloatSub::ports_id::OP_0]->connect(fb_linearizer->getPorts()[(int)FbLinearizer::ports_id::IP_YAW_REF]);
    mux_FH_des->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(fb_linearizer->getPorts()[(int)FbLinearizer::ports_id::IP_FHX_DES]);

    //connecting outputs of feedback linearizer
    fb_linearizer->getPorts()[(int)FbLinearizer::ports_id::OP_XH]->connect(demux_xh->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);

    demux_xh->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(x_dot->getPorts()[(int)Differentiator::ports_id::IP_0_DATA]);
    demux_xh->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(y_dot->getPorts()[(int)Differentiator::ports_id::IP_0_DATA]);
    demux_xh->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(z_dot->getPorts()[(int)Differentiator::ports_id::IP_0_DATA]);

    xh_ref->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_0]->connect(demux_x_ref->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);
    demux_x_ref->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(err_sum_x->getPorts()[(int)Sum::ports_id::IP_0_DATA]);
    demux_x_ref->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(err_sum_y->getPorts()[(int)Sum::ports_id::IP_0_DATA]);
    demux_x_ref->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(err_sum_z->getPorts()[(int)Sum::ports_id::IP_0_DATA]);
    demux_xh->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(err_sum_x->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    demux_xh->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(err_sum_y->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    demux_xh->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(err_sum_z->getPorts()[(int)Sum::ports_id::IP_1_DATA]);

    err_sum_x->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(mux_e_xh->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    x_dot->getPorts()[(int)Differentiator::ports_id::OP_0_DATA]->connect(mux_e_xh->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    err_sum_y->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(mux_e_yh->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    y_dot->getPorts()[(int)Differentiator::ports_id::OP_0_DATA]->connect(mux_e_yh->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    err_sum_z->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(mux_e_zh->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    z_dot->getPorts()[(int)Differentiator::ports_id::OP_0_DATA]->connect(mux_e_zh->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    
    mux_e_xh->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(PID_x->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);
    mux_e_yh->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(PID_y->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);
    mux_e_zh->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(PID_z->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);

    PID_x->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(mux_FH_des->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]); // TODO: Where to put saturation
    PID_y->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(mux_FH_des->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    PID_z->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(mux_FH_des->getPorts()[(int)Mux3D::ports_id::IP_2_DATA]);

    fb_linearizer->getPorts()[(int)FbLinearizer::ports_id::OP_Z_COMMAND]->connect(myActuationSystem->getPorts()[(int)HexaActuationSystem::ports_id::IP_3_DATA_Z]);
    fb_linearizer->getPorts()[(int)FbLinearizer::ports_id::OP_ROT_ERROR]->connect(demux_rot_err->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);

    // connecting inner loop controllers
    demux_rot_err->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(mux_e_roll->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    demux_rot_err->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(mux_e_pitch->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    demux_rot_err->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(mux_e_yaw->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);

    pose_provider->getPorts()[(int)ROSUnit_PoseProvider::ports_id::OP_3_BODY_RATE]->connect(demux_body_rate->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);
    demux_body_rate->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(mux_e_roll->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);  
    demux_body_rate->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(mux_e_pitch->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);  
    demux_body_rate->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(err_sum_yaw_rate->getPorts()[(int)Sum::ports_id::IP_1_DATA]);

    mux_e_roll->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(PID_roll->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);
    mux_e_pitch->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(PID_pitch->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);
    mux_e_yaw->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(PID_yaw->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);

    PID_yaw->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(Yaw_Saturation->getPorts()[(int)Saturation::ports_id::IP_0_DATA]);
    Yaw_Saturation->getPorts()[(int)Saturation::ports_id::OP_0_DATA]->connect(err_sum_yaw_rate->getPorts()[(int)Sum::ports_id::IP_0_DATA]);
    err_sum_yaw_rate->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(mux_e_yaw_rate->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    mux_e_yaw_rate->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(PID_yaw_rate->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);
    
    PID_roll->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(((Block*)myActuationSystem)->getPorts()[(int)HexaActuationSystem::ports_id::IP_0_DATA_ROLL]);
    PID_pitch->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(((Block*)myActuationSystem)->getPorts()[(int)HexaActuationSystem::ports_id::IP_1_DATA_PITCH]);
    PID_yaw_rate->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(((Block*)myActuationSystem)->getPorts()[(int)HexaActuationSystem::ports_id::IP_2_DATA_YAW]);
 
    // adding publishers
    auto mux_prov_ori = new Mux3D();

    demux_imu_ori->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(mux_prov_ori->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    demux_imu_ori->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(mux_prov_ori->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    demux_opti_ori->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(mux_prov_ori->getPorts()[(int)Mux3D::ports_id::IP_2_DATA]);

    pose_provider->getPorts()[(int)ROSUnit_PoseProvider::ports_id::OP_0_POS]->connect(pos_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_prov_ori->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(ori_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);    

    fb_linearizer->getPorts()[(int)FbLinearizer::ports_id::OP_XH]->connect(xh_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    fb_linearizer->getPorts()[(int)FbLinearizer::ports_id::OP_ROT_ERROR]->connect(rot_err_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    //*******************************************************************************************************************
    
    // ROS CONTROL OUTPUTS
    PID_x->getPorts()[(int)Saturation::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_0_X_OUTPUT]);
    PID_y->getPorts()[(int)Saturation::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_1_Y_OUTPUT]);
    PID_z->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_2_Z_OUTPUT]);
    PID_roll->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_3_ROLL_OUTPUT]);
    PID_pitch->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_4_PITCH_OUTPUT]);
    Yaw_Saturation->getPorts()[(int)Saturation::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_5_YAW_OUTPUT]);
    PID_yaw_rate->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_6_YAWRATE_OUTPUT]);   

    //***********************SETTING FLIGHT SCENARIO INPUTS****************************
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateControllerSrv::ports_id::OP_0_PID]->connect(PID_x->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateControllerSrv::ports_id::OP_0_PID]->connect(PID_y->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateControllerSrv::ports_id::OP_0_PID]->connect(PID_z->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateControllerSrv::ports_id::OP_0_PID]->connect(PID_roll->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateControllerSrv::ports_id::OP_0_PID]->connect(PID_pitch->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateControllerSrv::ports_id::OP_0_PID]->connect(PID_yaw->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateControllerSrv::ports_id::OP_0_PID]->connect(PID_yaw_rate->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);

    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateControllerSrv::ports_id::OP_0_PID]->connect(PID_x_slam->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateControllerSrv::ports_id::OP_0_PID]->connect(PID_y_slam->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateControllerSrv::ports_id::OP_0_PID]->connect(PID_z_slam->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);

    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateControllerSrv::ports_id::OP_1_MRFT]->connect(MRFT_x->getPorts()[(int)MRFTController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateControllerSrv::ports_id::OP_1_MRFT]->connect(MRFT_y->getPorts()[(int)MRFTController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateControllerSrv::ports_id::OP_1_MRFT]->connect(MRFT_z->getPorts()[(int)MRFTController::ports_id::IP_1_UPDATE]);
    
    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(PID_x->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);
    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(PID_y->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);
    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(PID_z->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);

    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(PID_x_slam->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);
    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(PID_y_slam->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);
    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(PID_z_slam->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);

    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(MRFT_x->getPorts()[(int)MRFTController::ports_id::IP_2_RESET]);
    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(MRFT_y->getPorts()[(int)MRFTController::ports_id::IP_2_RESET]);
    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(MRFT_z->getPorts()[(int)MRFTController::ports_id::IP_2_RESET]);

//    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(PID_z_slam->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);

    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(PID_roll->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);
    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(PID_pitch->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);
    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(PID_yaw->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);
    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(PID_yaw_rate->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);

    myROSArm->getPorts()[(int)ROSUnit_SetBoolSrv::ports_id::OP_0]->connect(((Block*)myActuationSystem)->getPorts()[(int)HexaActuationSystem::ports_id::IP_4_ARM]);
    
    //********************SETTING FLIGHT SCENARIO OUTPUTS***************************

    ((Block*)myActuationSystem)->getPorts()[(int)HexaActuationSystem::ports_id::OP_0_CMD]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_14_MOTORS]);
    ((Block*)myActuationSystem)->getPorts()[(int)HexaActuationSystem::ports_id::OP_1_ARM]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_15_ARMED]);

    Timer tempo;
    while(ros::ok()){
        tempo.tick();

        ros::spinOnce();

        int gone = tempo.tockMicroSeconds();
        if(gone > 5000) {
            std::cout  << "FC over 5000: " << gone << "\n";
        }
        rate.sleep();

    }
    return 0;
}
