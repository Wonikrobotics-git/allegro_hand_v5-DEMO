// Common allegro node code used by any node. Each node that implements an
// AllegroNode must define the computeDesiredTorque() method.
//
// Editor: Hibo (sh-yang @ wonik.com)

#include "allegro_node.h"

std::string jointNames[DOF_JOINTS] =
        {
                "joint_0_0", "joint_1_0", "joint_2_0", "joint_3_0",
                "joint_4_0", "joint_5_0", "joint_6_0", "joint_7_0",
                "joint_8_0", "joint_9_0", "joint_10_0", "joint_11_0",
                "joint_12_0", "joint_13_0", "joint_14_0", "joint_15_0",
        };

std::vector<std::string> frame_ids = {"link_3_0_tip", "link_7_0_tip", "link_11_0_tip", "link_15_0_tip"};
std::vector<std::string> namespaces = {"fingertip_[0]", "fingertip_[1]", "fingertip_[2]", "fingertip_[3]"};
std::vector<visualization_msgs::Marker> markers(4);

AllegroNode::AllegroNode(bool sim /* = false */) {
  mutex = new boost::mutex();
  
  // Create arrays 16 long for each of the four joint state components
  current_joint_state.position.resize(DOF_JOINTS);
  current_joint_state.velocity.resize(DOF_JOINTS);
  current_joint_state.effort.resize(DOF_JOINTS);
  current_joint_state.name.resize(DOF_JOINTS);

  // Initialize values: joint names should match URDF, desired torque and
  // velocity are both zero.
  for (int i = 0; i < DOF_JOINTS; i++) {
    current_joint_state.name[i] = jointNames[i];
    desired_torque[i] = 0.0;
    current_velocity[i] = 0.0;
    current_position_filtered[i] = 0.0;
    current_velocity_filtered[i] = 0.0;
  }

  
  // Get Allegro Hand information from parameter server
  // This information is found in the Hand-specific "zero.yaml" file from the allegro_hand_description package
  std::string robot_name, manufacturer, origin, serial;
  double version;
  ros::param::get("~hand_info/robot_name", robot_name);
  ros::param::get("~hand_info/which_hand", whichHand);
  ros::param::get("~hand_info/which_type", whichType);
  ros::param::get("~hand_info/manufacturer", manufacturer);
  ros::param::get("~hand_info/origin", origin);
  ros::param::get("~hand_info/serial", serial);
  ros::param::get("~hand_info/version", version);

  // Initialize CAN device
  canDevice = 0;
  if(!sim) {
    canDevice = new allegro::AllegroHandDrv();
    if (canDevice->init()) {
        usleep(3000);
    }
    else {
        delete canDevice;
        canDevice = 0;
    }
  }

  // Start ROS time
  tstart = ros::Time::now();
  
  // Advertise current joint state publisher and subscribe to desired joint
  // states.
  joint_state_pub = nh.advertise<sensor_msgs::JointState>(JOINT_STATE_TOPIC, 3);
  fingertip_pub = nh.advertise<std_msgs::Int32MultiArray>("fingertip_sensors", 1);
  joint_cmd_sub = nh.subscribe(DESIRED_STATE_TOPIC, 1, // queue size
                                &AllegroNode::desiredStateCallback, this);
  torque_cmd_sub = nh.subscribe(
          "allegroHand/torque_cmd", 3, &AllegroNode::setTorqueCallback, this);   
 
  time_sub = nh.subscribe("timechange", 10, &AllegroNode::ControltimeCallback,this);
  force_sub = nh.subscribe("forcechange", 10, &AllegroNode::GraspforceCallback,this);
 
  marker_pub = nh.advertise<visualization_msgs::Marker>("fingertip_arrow_markers", 1);
}

AllegroNode::~AllegroNode() {
  if (canDevice) delete canDevice;
  delete mutex;
  nh.shutdown();
}

// Get Allegro Hand desired joint position
void AllegroNode::desiredStateCallback(const sensor_msgs::JointState &msg) {
  mutex->lock();
  desired_joint_state = msg;
  mutex->unlock();
}

// Get Allegro Hand motion control time from gui
void AllegroNode::ControltimeCallback(const std_msgs::Float32::ConstPtr& msg)
{
    motion_time = msg->data;
    pBHand->SetMotiontime(motion_time);
}

void AllegroNode::setTorqueCallback(const sensor_msgs::JointState &msg) {

  mutex->lock();

  for (int i = 0; i < DOF_JOINTS; i++)
    desired_torque[i] = msg.effort[i];
  mutex->unlock();

  controlTorque = true;

}

// Get Allegro Hand grasping force from gui
void AllegroNode::GraspforceCallback(const std_msgs::Float32::ConstPtr& msg)
{
    force_get = msg->data;
}

// Main publisher
void AllegroNode::publishData() {
  // current position, velocity and effort (torque), fingertip_sensor (Rviz) published
  current_joint_state.header.stamp = tnow;
  
  for (int i = 0; i < DOF_JOINTS; i++) {
    current_joint_state.position[i] = current_position[i]; /// current_position_filtered[i];
    current_joint_state.velocity[i] = current_velocity[i]; /// current_velocity_filtered[i];
    current_joint_state.effort[i] = desired_torque[i];


  }
  joint_state_pub.publish(current_joint_state);
      std_msgs::Int32MultiArray fingertip_msg;
    // fingertip_sensor 배열을 메시지의 data에 추가
    fingertip_msg.data.push_back(fingertip_sensor[0]);
    fingertip_msg.data.push_back(fingertip_sensor[1]);
    fingertip_msg.data.push_back(fingertip_sensor[2]);
    fingertip_msg.data.push_back(fingertip_sensor[3]);
    // 메시지 publish
    fingertip_pub.publish(fingertip_msg);
  // fingertip_sensor_Rviz
  for (const auto& marker : markers) {
    marker_pub.publish(marker);
    ros::Duration(1e-6).sleep();
    }

}


void AllegroNode::Rviz_Arrow(){ 
  //  This is the function to visualize fingertip sensors on Rviz.

    tf2::Quaternion orientation;
    orientation.setRPY(0, -M_PI/4, 0); 

  for (int i = 0; i < 4; ++i) {

    float value = fingertip_sensor[i] * (255.0f / 500.0f);
    if (value >= 500) value = 500;  // 최대값 제한

    float r = 0.0, g = 0.0, b = 0.0;

    if (value <= 64) {
        float green_value = value / 64.0f;
        r = 0.0;
        g = green_value;
        b = 1.0;
    } else if (value <= 128) {
        float blue_value = 1.0f - ((value - 64.0f) / 64.0f);
        r = 0.0;
        g = 1.0;
        b = blue_value;
    } else if (value <= 192) {
        float red_value = (value - 128.0f) / 64.0f;
        r = red_value;
        g = 1.0;
        b = 0.0;
    } else if (value <= 255) {
        float green_value = 1.0f - ((value - 192.0f) / 64.0f);
        r = 1.0;
        g = green_value;
        b = 0.0;
    } else {
        r = 1.0;
        g = 0.0;
        b = 0.0;
    }

    float value_limit = std::min(fingertip_sensor[i],500);

    markers[i].header.frame_id = frame_ids[i];
    markers[i].ns = namespaces[i];
    markers[i].type = visualization_msgs::Marker::SPHERE;
    markers[i].action = visualization_msgs::Marker::ADD;
    //markers[i].pose.orientation = tf2::toMsg(orientation);
    markers[i].scale.x = 0.03 + value_limit/20000.0;//fingertip_sensor[i] / 10000.0;
    markers[i].scale.y = 0.03 + value_limit/20000.0;//fingertip_sensor[i] / 10000.0;//0.005;
    markers[i].scale.z = 0.03 + value_limit/20000.0;//fingertip_sensor[i] / 10000.0;//0.005;
    markers[i].color.r = r;
    markers[i].color.g = g;
    markers[i].color.b = b;
    markers[i].color.a = 0.4;
  }
}

void AllegroNode::updateController() {

  // Calculate loop time;
  tnow = ros::Time::now();
  dt = ALLEGRO_CONTROL_TIME_INTERVAL;

  if(dt <= 0) {
    ROS_DEBUG_STREAM_THROTTLE(1, "AllegroNode::updateController dt is zero.");
    return;
  }

  tstart = tnow;


  if (canDevice)
  {
    // try to update joint positions through CAN comm:
    lEmergencyStop = canDevice->readCANFrames();
 
    // check if all positions are updated:
    if (lEmergencyStop == 0 && canDevice->isJointInfoReady())
    {
      // back-up previous joint positions:
      for (int i = 0; i < DOF_JOINTS; i++) {
        previous_position[i] = current_position[i];
        previous_position_filtered[i] = current_position_filtered[i];
        previous_velocity[i] = current_velocity[i];
      }
     
      // update joint positions:
      canDevice->getJointInfo(current_position);

      // low-pass filtering:
      for (int i = 0; i < DOF_JOINTS; i++) {
        current_position_filtered[i] =  current_position[i];
        current_velocity_filtered[i] = (current_position[i] - previous_position[i]) / dt;
        current_velocity[i] = (current_position[i] - previous_position[i]) / dt;
      }

      //printf(" 1: %d 2: %d 3: %d 4: %d \n", fingertip_sensor[0],fingertip_sensor[1],fingertip_sensor[2],fingertip_sensor[3]);

      // update grasping forces based on fingertip_sensors:
      if (OperatingMode == 0) {

					if ((fingertip_sensor[0] + fingertip_sensor[1] + fingertip_sensor[3]) > 200)
							f[0] = f[1] = f[2] = force_get; ///default = 2.0
					else
							f[0] = f[1] = f[2] = 0.5f;

			}     

      ///make an Arrow on Rviz
      Rviz_Arrow();
      // calculate control torque:
      if(!controlTorque)
        computeDesiredTorque(); /// calculate desired torque using BHand Library

      // set & write torque to each joint:
      canDevice->setTorque(desired_torque);
      lEmergencyStop = canDevice->writeJointTorque();
   
      // reset joint position update flag:
      canDevice->resetJointInfoReady();
    

      // publish joint positions to ROS topic:
      publishData();
   
      frame++;

    }

    ///check if handedness & hand type are correctly entered:
    if(frame == 1)
    {

      if(whichHand.compare("left") == 0)
      {
        if(canDevice->RIGHT_HAND){
        ROS_ERROR("WRONG HANDEDNESS DETECTED!");
        canDevice = 0;
        return;
        }
      }
      else
      {
        if(!canDevice->RIGHT_HAND){
        ROS_ERROR("WRONG HANDEDNESS DETECTED!");
        canDevice = 0;
        return;
        }
      }

      if(whichType.compare("A") == 0)
      {
        if(!canDevice->HAND_TYPE_A){
        ROS_ERROR("WRONG TYPE DETECTED!");
        canDevice = 0;
        return;
        }
      }
      else
      {
        if(canDevice->HAND_TYPE_A){
        ROS_ERROR("WRONG TYPE DETECTED!");
        canDevice = 0;
        return;
        }
      }
      
    }

  }

  if (lEmergencyStop < 0) {
    // Stop program when Allegro Hand is switched off
    ROS_ERROR("Allegro Hand Node is Shutting Down! (Emergency Stop)");
    ros::shutdown();
  }
}

// Interrupt-based control is not recommended by Wonik Robotics. I have not tested it.
void AllegroNode::timerCallback(const ros::TimerEvent &event) {
  updateController();
}

ros::Timer AllegroNode::startTimerCallback() {
  ros::Timer timer = nh.createTimer(ros::Duration(0.001),
                                    &AllegroNode::timerCallback, this);
  return timer;
}
