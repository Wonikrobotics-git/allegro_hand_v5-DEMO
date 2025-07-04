#ifndef PROJECT_ALLEGRO_NODE_GRASP_H
#define PROJECT_ALLEGRO_NODE_GRASP_H

#include "allegro_node.h"

#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "candrv/candrv.h"
#include "bhand/BHand.h"
#include <std_msgs/String.h>
#include <iostream>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

// Forward class declaration.
class BHand;

// Grasping controller that uses the BHand library for commanding various
// pre-defined grasp (e.g., three-finger ping, envelop, etc...).
//
// This node is most useful when run with the keyboard node (the keyboard node
// sends the correct String to this node). A map from String command -> Grasp
// type is defined in the implementation (cpp) file.
//
// This node can also save & hold a position, but in constrast to the PD node
// you do not have any control over the controller gains.
//
// Author: Felix Duvallet
//
class AllegroNodeGrasp : public AllegroNode {

 public:

    AllegroNodeGrasp();

    ~AllegroNodeGrasp();

    void initController(const std::string &whichHand, const std::string &whichType);

    void computeDesiredTorque();

    void libCmdCallback(const std_msgs::String::ConstPtr &msg);

    void setJointCallback(const sensor_msgs::JointState &msg);

    void envelopTorqueCallback(const std_msgs::Float32 &msg);

    void setGraspoffsetCallback(const std_msgs::Float64MultiArray &msg);

    void setGainCallback(const std_msgs::Float64MultiArray &msg);

    void doIt(bool polling);

    double desired_position[DOF_JOINTS] = {0.0};

 protected:

    // Handles external joint command (sensor_msgs/JointState).
    ros::Subscriber joint_cmd_sub;

    // Handles defined grasp commands (std_msgs/String).
    ros::Subscriber lib_cmd_sub;

    ros::Subscriber gain_sub;

    ros::Subscriber offset_sub;

    ros::Subscriber envelop_torque_sub;
    

    // Initialize BHand
    BHand *pBHand = NULL;


};

#endif //PROJECT_ALLEGRO_NODE_GRASP_H
