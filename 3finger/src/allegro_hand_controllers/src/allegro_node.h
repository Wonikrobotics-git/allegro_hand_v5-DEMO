//
// Created by felixd on 10/1/15.
//
// Modified by Hibo on 2024/12/03.

#ifndef PROJECT_ALLEGRO_NODE_COMMON_H
#define PROJECT_ALLEGRO_NODE_COMMON_H

// Defines DOF_JOINTS.
#include "allegro_hand_driver/AllegroHandDrv.h"
using namespace allegro;

#include <string>
#include <boost/thread/thread.hpp>

#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Int32MultiArray.h>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "fstream"
#include "bhand/BHand.h"
#include <vector>
// Forward declaration.
class AllegroHandDrv;

#define ALLEGRO_CONTROL_TIME_INTERVAL 0.002

#define Fout 1
std::ofstream data; //sensor data

// Topic names: current & desired JointState, named grasp to command.
const std::string JOINT_STATE_TOPIC = "allegroHand/joint_states";
const std::string DESIRED_STATE_TOPIC = "allegroHand/joint_cmd";
const std::string LIB_CMD_TOPIC = "allegroHand/lib_cmd";
class AllegroNode {
 public:

  AllegroNode(bool sim = false);

  virtual ~AllegroNode();

  void publishData();

  void desiredStateCallback(const sensor_msgs::JointState &desired);

  void ControltimeCallback(const std_msgs::Float32::ConstPtr& msg);

  void GraspforceCallback(const std_msgs::Float32::ConstPtr& msg);

  virtual void updateController();

  // This is the main method that must be implemented by the various
  // controller nodes.
  virtual void computeDesiredTorque() {
    ROS_ERROR("Called virtual function!");
  };

  ros::Timer startTimerCallback();

  void timerCallback(const ros::TimerEvent &event);

  void Rviz_Arrow();

  double desired_position[DOF_JOINTS] = {0.0};
  
 protected:

  double current_position[DOF_JOINTS] = {0.0};
  double previous_position[DOF_JOINTS] = {0.0};

  double current_position_filtered[DOF_JOINTS] = {0.0};
  double previous_position_filtered[DOF_JOINTS] = {0.0};

  double current_velocity[DOF_JOINTS] = {0.0};
  double previous_velocity[DOF_JOINTS] = {0.0};
  double current_velocity_filtered[DOF_JOINTS] = {0.0};

  double desired_torque[DOF_JOINTS] = {0.0};

  // ROS stuff
  ros::NodeHandle nh;
  ros::Publisher  joint_state_pub;
  ros::Subscriber joint_cmd_sub;
  ros::Subscriber joint_state_sub;
  ros::Subscriber time_sub;
  ros::Subscriber force_sub;
  ros::Publisher  marker_pub;
  ros::Publisher  fingertip_pub;

  // Store the current and desired joint states.
  sensor_msgs::JointState current_joint_state;
  sensor_msgs::JointState desired_joint_state;

  // ROS Time
  ros::Time tstart;
  ros::Time tnow;
  double dt;

  // CAN device
  allegro::AllegroHandDrv *canDevice;
  boost::mutex *mutex;

  // Flags
  int lEmergencyStop = 0;
  long frame = 0;

  visualization_msgs::Marker marker1, marker2, marker3;

  BHand *pBHand = NULL;

  bool CANMode = true;

};

double motion_time = 0.6;//
double force_get = 1.0;//



#endif //PROJECT_ALLEGRO_NODE_COMMON_H
