#include "allegro_node_485.h"

// The only topic specific to the 'grasp' controller is the envelop torque.

// Define a map from string (received message) to eMotionType (Bhand controller grasp).
std::map<std::string, std::vector<uint8_t>> bhand_grasps = {
    {"home",    {CMD_PLACE, CMD_PLACE + sizeof(CMD_PLACE)}},                  // home position
    {"grasp_3", {CMD_PICK_3F, CMD_PICK_3F + sizeof(CMD_PICK_3F)}},            // pinch, 3fingers
    {"gravcomp", {CMD_GRAVITY, CMD_GRAVITY + sizeof(CMD_GRAVITY)}},           // gravity compensation
    {"off",      {CMD_TORQUE_OFF, CMD_TORQUE_OFF + sizeof(CMD_TORQUE_OFF)}},  // turn joints off
    {"PosRead1",{CMD_POS_READ_1, CMD_POS_READ_1 + sizeof(CMD_POS_READ_1)}},   // read positions of index finger(mm)
    {"PosRead2",{CMD_POS_READ_2, CMD_POS_READ_2 + sizeof(CMD_POS_READ_2)}},   // read positions of middle finger(mm)
    {"PosRead3",{CMD_POS_READ_3, CMD_POS_READ_3 + sizeof(CMD_POS_READ_3)}},   // read positions of ring finger(mm)
};


AllegroNode485::AllegroNode485() {

    ros::param::get("~port_info/which_port", whichPort);

    initController(whichPort);

    lib_cmd_sub = nh.subscribe(
        "allegroHand/lib_cmd", 1, &AllegroNode485::libCmdCallback, this);
}

AllegroNode485::~AllegroNode485() {

}

void AllegroNode485::libCmdCallback(const std_msgs::String::ConstPtr &msg) {
    ROS_INFO("CTRL: Heard: [%s]", msg->data.c_str());
    const std::string command = msg->data;

    CommandLength cmdLength;
    std::vector<uint8_t> cmd;

    auto itr = bhand_grasps.find(msg->data);
    if (itr != bhand_grasps.end()) {
        if (command.find("PosRead") == 0) 
            cmdLength = POSREAD;
        else
            cmdLength = COMMANDREAD;
         const auto& cmd = itr->second;
         sendData(fd, cmd.data(), cmd.size(), cmdLength);

    } else if (command.find("SavPos") == 0) {
        cmdLength = WRITEPOS;

        if (command.size() > 6 && isdigit(command[6])) {
            int index = command[6] - '0'; 
            if (index >= 1 && index <= 9) {
                cmd = {SAV_POS[0], static_cast<uint8_t>(index)}; 

                sendData(fd, cmd.data(), cmd.size(), cmdLength);
            } 
        }
    } else if (command.find("GoPos") == 0) {
        cmdLength = WRITEPOS;

        if (command.size() > 5 && isdigit(command[5])) {
            int index = command[5] - '0'; 
            if (index >= 1 && index <= 9) {
                cmd = {GO_POS[0], static_cast<uint8_t>(index)}; 

                sendData(fd, cmd.data(), cmd.size(), cmdLength);
            } 
        }
    }
     else {
        ROS_WARN("Unknown command type: [%s]", command.c_str());
        return; 
    }
}

void AllegroNode485::initController(const std::string &whichPort) {
  // Initialize RS-485 controller

  const char* portName = whichPort.c_str();//"/dev/ttyUSB0";
  int baudrate = getBaudrate(115200);
  fd = RS485init(portName, baudrate);

  ROS_INFO("Allegro Hand RS-485 node start");

  printf("*************************************\n");
  printf("         RS-485 Grasp Method         \n");
  printf("-------------------------------------\n");
  printf("         Every command works.        \n");
  printf("*************************************\n");
}

void AllegroNode485::doIt(bool polling) {
  if (polling) {
    ROS_INFO("Polling = true.");
    while (ros::ok()) {
      ros::spinOnce();
    }
    running = false;

    close(fd);
    ROS_INFO("RS-485 port closed.");
    
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "allegro_hand_core_485");
  AllegroNode485 RS485;

  bool polling = true;

  ROS_INFO("Start RS-485 controller with polling = %d", polling);

  RS485.doIt(polling);

  return 0;
}
