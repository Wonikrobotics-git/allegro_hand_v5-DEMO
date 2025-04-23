#ifndef PROJECT_ALLEGRO_NODE_485_H
#define PROJECT_ALLEGRO_NODE_485_H


#include "allegro_hand_driver/AllegroHandRS485Drv.h"
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>

// Grasping controller that uses the BHand library for commanding various
// pre-defined grasp through RS-485 communication(e.g., three-finger pick, home position, etc...).
//
// This node is most useful when run with the keyboard node (the keyboard node
// sends the correct String to this node). A map from String command -> Grasp
// type is defined in the implementation (cpp) file.
//
// This node can also save & hold a position.
//
// Author: Soohoon Yang(Hibo)
//
class AllegroNode485 {

 public:

    AllegroNode485();

    ~AllegroNode485();

    void initController(const std::string &whichPort);

    void libCmdCallback(const std_msgs::String::ConstPtr &msg);

    void doIt(bool polling);

protected:

    ros::NodeHandle nh;
    // Handles defined grasp commands (std_msgs/String).
    ros::Subscriber lib_cmd_sub;

   std::string whichPort;  // Right or left hand.
   int fd = 0;
};


#endif //PROJECT_ALLEGRO_NODE_485_H
