#include "allegro_node_grasp.h"
#include "allegro_hand_driver/AllegroHandDrv.h"

// The only topic specific to the 'grasp' controller is the envelop torque.
const std::string ENVELOP_TORQUE_TOPIC = "allegroHand/envelop_torque";

// Define a map from string (received message) to eMotionType (Bhand controller grasp).
std::map<std::string, eMotionType> bhand_grasps = {
        {"home",     eMotionType_HOME}, // home pose (direct)
        {"grasp_3",  eMotionType_GRASP_3},  // grasp with 3 fingers
        {"grasp_4",  eMotionType_GRASP_4},  // grasp with 4 fingers
        {"pinch_it", eMotionType_PINCH_IT},  // pinch, index & thumb
        {"pinch_mt", eMotionType_PINCH_MT},  // pinch, middle & thumb
        {"envelop",  eMotionType_ENVELOP},  // envelop grasp (power-y)
        {"off",      eMotionType_NONE},  // turn joints off
        {"gravcomp", eMotionType_GRAVITY_COMP},  // gravity compensation
};

//Define PD gain for eMotionType_JOINT_PD
double kp[DOF_JOINTS] = { /// default p_gain
                          1.0,1.0,1.0,1.0, // index
                          1.0,1.0,1.0,1.0, // middle
                          1.0,1.0,1.0,1.0, // pinky
                          1.0,1.0,1.0,1.0  // thumb
                        };

double kd[DOF_JOINTS] = { /// default d_gain
                          0.03,0.03,0.03,0.03, // index
                          0.03,0.03,0.03,0.03, // middle
                          0.03,0.03,0.03,0.03, // pinky
                          0.03,0.03,0.03,0.03  // thumb
                        };

         
AllegroNodeGrasp::AllegroNodeGrasp() 
        : AllegroNode() {

  initController(whichHand,whichType);

  joint_cmd_sub = nh.subscribe(
          DESIRED_STATE_TOPIC, 3, &AllegroNodeGrasp::setJointCallback, this);
  lib_cmd_sub = nh.subscribe(
          LIB_CMD_TOPIC, 1, &AllegroNodeGrasp::libCmdCallback, this);

  gain_sub = nh.subscribe("gainchange",1,&AllegroNodeGrasp::setGainCallback, this);

  offset_sub = nh.subscribe("offsetchange",1,&AllegroNodeGrasp::setGraspoffsetCallback, this);

  envelop_torque_sub = nh.subscribe(
          ENVELOP_TORQUE_TOPIC, 1, &AllegroNodeGrasp::envelopTorqueCallback,
          this);
}

AllegroNodeGrasp::~AllegroNodeGrasp() {
  delete pBHand;
}

void AllegroNodeGrasp::libCmdCallback(const std_msgs::String::ConstPtr &msg) {
   // Main behavior: apply the grasp directly from the map. Secondary behaviors can still be handled
  // normally (case-by-case basis), note these should *not* be in the map. 
  controlTorque = false;
  
  ROS_INFO("CTRL: Heard: [%s]", msg->data.c_str());
  const std::string lib_cmd = msg->data;

  auto itr = bhand_grasps.find(msg->data);
  if (itr != bhand_grasps.end()) {
    // Behavior pre-defined by Wonik Robotics.
    if(lib_cmd.find("home") == 0 || lib_cmd.find("off") == 0)
    {
      command_place(_can_handle);
    }
    else{
      command_pick(_can_handle);
    }
    pBHand->SetMotionType(itr->second);
    ROS_INFO("motion type = %d", itr->second);
   
  } else if (lib_cmd.find("pdControl") == 0) {
    // New behavior for V5.
    // Can be any pose that saved in specified number of yaml pose file.
    // Desired position only necessary if in PD Control mode.
    
    std::string num_str = lib_cmd.substr(9);

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

    for (int i = 0; i < DOF_JOINTS && i < positions.size(); i++) {
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

    for (int i = 0; i < DOF_JOINTS && i < positions.size(); i++) {
      desired_position[i] = positions[i];
    }

    pBHand->SetJointDesiredPosition(desired_position);
    pBHand->SetMotionType(eMotionType_POSE_PD);

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

        for (int i = 0; i < DOF_JOINTS && i < positions.size(); i++) {
            desired_position[i] = positions[i];
        }
        pBHand->SetJointDesiredPosition(desired_position);
        pBHand->SetMotionType(eMotionType_POSE_PD);
    //ROS_WARN("Unknown commanded grasp: %s.", lib_cmd.c_str());
  }

}

// Change Joint_PD gain
void AllegroNodeGrasp::setGraspoffsetCallback(const std_msgs::Float64MultiArray &msg)
{
    double x[4], y[4], z[4]; // 4개의 값을 각각 저장할 배열

    for (int i = 0; i < 4; i++) {
        x[i] = msg.data[i * 3];     // 3개씩 끊은 첫 번째 값
        y[i] = msg.data[i * 3 + 1]; // 3개씩 끊은 두 번째 값
        z[i] = msg.data[i * 3 + 2]; // 3개씩 끊은 세 번째 값
    }

    for (int i = 0; i < 4; i++) {
        ROS_INFO("x[%d] = %f, y[%d] = %f, z[%d] = %f", i, x[i], i, y[i], i, z[i]);
    }

    pBHand->SetGraspingPose(x,y,z);
}

// Change Joint_PD gain
void AllegroNodeGrasp::setGainCallback(const std_msgs::Float64MultiArray &msg)
{
  for(int i = 0; i<16; i++){
     kp[i] = msg.data[i];
     kd[i] = msg.data[i + 16];
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
  //// Set PDgains
  pBHand->SetGainsEx(kp,kd);

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
  pBHand->UpdateControl((double) frame * ALLEGRO_CONTROL_TIME_INTERVAL);

  // BHand lib set grasping force
	pBHand->SetGraspingForce(f);

	pBHand->GetFKResult(x,y,z);
  
  // Necessary torque obtained from Bhand lib
  pBHand->GetJointTorque(desired_torque);

}

void AllegroNodeGrasp::initController(const std::string &whichHand,const std::string &whichType) {
  // Initialize BHand controller

  if (whichHand.compare("left") == 0) {
    pBHand = new BHand(eHandType_Left);
    ROS_WARN("CTRL: Left Allegro Hand controller initialized.");

  }
  else {
    pBHand = new BHand(eHandType_Right);
    ROS_WARN("CTRL: Right Allegro Hand controller initialized.");
  }



  if (whichType.compare("A") == 0){
    //pBHand->SetMotionType(eHardwareType_A);
    pBHand->GetType(eHardwareType_A);
    ROS_WARN("CTRL: A-Type Allegro Hand controller initialized.");
  }

  else{
    //pBHand->SetMotionType(eHardwareType_B);
    pBHand->GetType(eHardwareType_B);
    ROS_WARN("CTRL: B-Type Allegro Hand controller initialized.");
  } 



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
    }
  } else {
    ROS_INFO("Polling = false.");

    // Timer callback (not recommended).
    ros::Timer timer = startTimerCallback();
    ros::spin();
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "allegro_hand_core_grasp");
  AllegroNodeGrasp grasping;

  bool polling = true;

  ROS_INFO("Start controller with polling = %d", polling);

  grasping.doIt(polling);

  return 0;
}
