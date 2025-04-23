#include "allegro_node_grasp.h"
#include "candrv/candrv.h"
#include <std_msgs/String.h>
#include "allegro_hand_driver/AllegroHandDrv.h"

// The only topic specific to the 'grasp' controller is the envelop torque.
const std::string ENVELOP_TORQUE_TOPIC = "allegroHand/envelop_torque";

std::string pkg1_path;
std::string data_path;

std::string prev_lib_cmd = "";

// Define a map from string (received message) to eMotionType (Bhand controller grasp).
std::map<std::string, eMotionType> bhand_grasps = {
        {"home",     eMotionType_HOME}, // home pose (box object)
        {"home2",    eMotionType_HOME2}, // home pose (box object)
        {"sphere",   eMotionType_SPHERE}, // home pose (spherical object)
        {"grasp_3",  eMotionType_GRASP_3},  // grasp with 3 fingers
        {"envelop",  eMotionType_ENVELOP},  // envelop grasp (power-grasp)
        {"off",      eMotionType_NONE},  // turn joints off
        {"gravcomp", eMotionType_GRAVITY_COMP},  // gravity 
        {"assemble",eMotionType_ASSEMBLE},
        {"grasp_1",  eMotionType_GRASP_12},  // grasp with index finger & Thumb
        {"grasp_2",  eMotionType_GRASP_23},  // grasp with pinky finger & Thumb
        {"grasp12_3",  eMotionType_GRASP_12_3}, 
};


AllegroNodeGrasp::AllegroNodeGrasp()
        : AllegroNode() {

  initController();

  joint_cmd_sub = nh.subscribe(
          DESIRED_STATE_TOPIC, 3, &AllegroNodeGrasp::setJointCallback, this);
  lib_cmd_sub = nh.subscribe(
          LIB_CMD_TOPIC, 1, &AllegroNodeGrasp::libCmdCallback, this);
  envelop_torque_sub = nh.subscribe(
          ENVELOP_TORQUE_TOPIC, 1, &AllegroNodeGrasp::envelopTorqueCallback,
          this);
  
}

AllegroNodeGrasp::~AllegroNodeGrasp() {
  data.close();
  delete pBHand;
}


void AllegroNodeGrasp::libCmdCallback(const std_msgs::String::ConstPtr &msg) {
  
  ROS_INFO("CTRL: Heard: [%s]", msg->data.c_str());
  const std::string lib_cmd = msg->data;

  // Main behavior: apply the grasp directly from the map. Secondary behaviors can still be handled
  // normally (case-by-case basis), note these should *not* be in the map.
  auto itr = bhand_grasps.find(msg->data);
  if (itr != bhand_grasps.end()) {
    pBHand->SetMotionType(itr->second);
    ROS_INFO("motion type = %d", itr->second);

    bool prev_was_home_off_sphere_assemble =
        prev_lib_cmd.find("home") == 0 || prev_lib_cmd.find("off") == 0 ||
        prev_lib_cmd.find("sphere") == 0 || prev_lib_cmd.find("assemble") == 0;

    bool current_is_home_off_sphere_assemble =
        lib_cmd.find("home") == 0 || lib_cmd.find("off") == 0 ||
        lib_cmd.find("sphere") == 0 || lib_cmd.find("assemble") == 0;

    if(lib_cmd.find("home") == 0 || lib_cmd.find("off") == 0 || lib_cmd.find("sphere") == 0 || lib_cmd.find("assemble") == 0)
    {
      command_place(_can_handle);
    }
    else{
      command_pick(_can_handle);
    }

    // if (prev_was_home_off_sphere_assemble && !current_is_home_off_sphere_assemble) {
    //     ROS_INFO("Command_pick");
    //     command_pick(_can_handle);
    // } else if (current_is_home_off_sphere_assemble) {
    //     command_place(_can_handle);
    // }
    // prev_lib_cmd = lib_cmd;
  }  else if (lib_cmd.find("GoPos") == 0) {
    // New behavior for V5.
    // Can be any pose that saved in specified number of yaml pose file.
    // Desired position only necessary if in PD Control mode.
    
    std::string num_str = lib_cmd.substr(5);

    int pose_num = std::stoi(num_str);
    ROS_INFO("PDControl Mode with pose number %d", pose_num);

    std::string pkg_path = ros::package::getPath("allegro_hand_controllers");
    std::string file_path = pkg_path + "/pose/pose" + std::to_string(pose_num) + ".yaml";

    std::ifstream infile(file_path);
    if (!infile.good()) {
      ROS_WARN("Pose file does not exist. Please select a different command.");
      return;
    }

    YAML::Node node = YAML::LoadFile(file_path);
    std::vector<double> positions = node["position"].as<std::vector<double>>();

    for (int i = 0; i < 9 && i < positions.size(); i++) {
      desired_position[i] = positions[i];
    }

    pBHand->SetJointDesiredPosition(desired_position);
    pBHand->SetMotionType(eMotionType_POSE_PD);

  } else if (lib_cmd.compare("moveit") == 0) {
    std::string pkg_path = ros::package::getPath("allegro_hand_controllers");
    std::string file_path = pkg_path + "/pose/pose_moveit.yaml";

    std::ifstream infile(file_path);
    if (!infile.good()) {
      ROS_WARN("Pose file does not exist. Please select a different command.");
      return;
    }

    YAML::Node node = YAML::LoadFile(file_path);
    std::vector<double> positions = node["position"].as<std::vector<double>>();

    for (int i = 0; i < 9 && i < positions.size(); i++) {
      desired_position[i] = positions[i];
    }
    

    pBHand->SetJointDesiredPosition(desired_position);
    pBHand->SetMotionType(eMotionType_POSE_PD);

  }
  else if(lib_cmd.find("Operating") == 0){
    // if(CANMode == false)
    //   CANMode = true;
    // else
    //   CANMode = false;

    if(OperatingMode == 0)
      OperatingMode = 1;
    else
      OperatingMode = 0;

    ROS_INFO("Operating Mode: %s", (OperatingMode == 0) ? "InnerGrasp" : (OperatingMode == 1) ? "OuterGrasp" : "unknown");
  }
  else {
       std::string pkg_path = ros::package::getPath("allegro_hand_controllers");
        std::string file_path = pkg_path + "/pose/" + lib_cmd + ".yaml";

        std::ifstream infile(file_path);
        if (!infile.good()) {
            ROS_WARN("Unknown commanded grasp: %s.", lib_cmd.c_str());
            return;
        }
        YAML::Node node = YAML::LoadFile(file_path);
        std::vector<double> positions = node["position"].as<std::vector<double>>();

        for (int i = 0; i < 9 && i < positions.size(); i++) {
            desired_position[i] = positions[i];
        }
        pBHand->SetJointDesiredPosition(desired_position);
        pBHand->SetMotionType(eMotionType_POSE_PD);
    //ROS_WARN("Unknown commanded grasp: %s.", lib_cmd.c_str());
  }


}

// Called when a desired joint position message is received
void AllegroNodeGrasp::setJointCallback(const sensor_msgs::JointState &msg) {
  mutex->lock();

  for (int i = 0; i < DOF_JOINTS; i++)
    desired_position[i] = msg.position[i];
  mutex->unlock();

  pBHand->SetJointDesiredPosition(desired_position);
  pBHand->SetMotionType(eMotionType_JOINT_PD);
}

// The grasp controller can set the desired envelop grasp torque by listening to
// Float32 messages on ENVELOP_TORQUE_TOPIC ("allegroHand/envelop_torque").
void AllegroNodeGrasp::envelopTorqueCallback(const std_msgs::Float32 &msg) {
  const double torque = msg.data;
  ROS_INFO("Setting envelop torque to %.3f.", torque);
  pBHand->SetEnvelopTorqueScalar(torque);
}

void AllegroNodeGrasp::computeDesiredTorque() {
  // compute control torque using Bhand library
  pBHand->SetJointPosition(current_position);

  // BHand lib control updated with time stamp
  pBHand->UpdateControl((double) frame * ALLEGRO_CONTROL_TIME_INTERVAL);//((double) frame * ALLEGRO_CONTROL_TIME_INTERVAL);

  /// V5
	pBHand->SetGraspingForce(f);

	pBHand->GetFKResult(x,y,z);
  // Necessary torque obtained from Bhand lib
  pBHand->GetJointTorque(desired_torque);

}

void AllegroNodeGrasp::initController() {

  pBHand = new BHand(eHandType_Right);
  ROS_WARN("CTRL: Allegro Hand 3F controller initialized.");
  
  pBHand->SetTimeInterval(ALLEGRO_CONTROL_TIME_INTERVAL);
  pBHand->SetMotionType(eMotionType_NONE);

  // sets initial desired pos at start pos for PD control
  for (int i = 0; i < DOF_JOINTS; i++)
    desired_position[i] = current_position[i];

  printf("*************************************\n");
  printf("         Grasp (BHand) Method        \n");
  printf("-------------------------------------\n");
  printf("         Every command works.        \n");
  printf("*************************************\n");
}

void AllegroNodeGrasp::doIt(bool polling) {

  if (polling) {
    ROS_INFO("Polling = true.");
    while (ros::ok()) {
      updateController();
      ros::spinOnce();
     if(CANMode == false)
     ros::Duration(0.05).sleep();
    }
  } else {
    ROS_INFO("Polling = false.");

    // Timer callback (not recommended).
    ros::Timer timer = startTimerCallback();
    ros::spin();
  }
}

int main(int argc, char **argv) {

  pkg1_path = ros::package::getPath("allegro_hand_controllers");
  data_path = pkg1_path + "/pose/sensor.txt";  
  data.open(data_path);

  ros::init(argc, argv, "allegro_hand_core_grasp");
  AllegroNodeGrasp grasping;

  bool polling = false;
  if (argv[1] == std::string("true")) {
    polling = true;
  }

  grasping.doIt(polling);
}
