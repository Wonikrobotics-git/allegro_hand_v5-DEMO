#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <vector>
#include <mutex>

class CommandPublisher {
private:
    ros::NodeHandle nh_;
    ros::Publisher lib_cmd_pub_;
    ros::Publisher force_pub_;
    std::vector<std::string> commands_{"home", "pinch_it", "home", "pinch_mt", "home", "grasp_3", "forcechange", "home", "forcereset", "grasp_4", "forcechange","home","forcereset"};
    int command_index_ = 0;
    double interval_ = 1.5; // 기본 간격 (초)
    double interval_grasp = 4.0;//
    std::mutex mutex_;

public:
    CommandPublisher() {
        lib_cmd_pub_ = nh_.advertise<std_msgs::String>("allegroHand/lib_cmd", 10);
        force_pub_ = nh_.advertise<std_msgs::Float32>("allegroHand/forcechange", 1);
    }

    void publishCommands() {
        ros::Rate rate(1.0 / interval_);
        rate.sleep();
        while (ros::ok()) {
            std_msgs::String msg;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                msg.data = commands_[command_index_];
                command_index_ = (command_index_ + 1) % commands_.size();
            }

            if (msg.data == "forcechange") {
                double start_time = ros::Time::now().toSec();
                double end_time = start_time + interval_grasp;
                double force_value = 0.5;
                while (ros::Time::now().toSec() < end_time && ros::ok()) {
                    std_msgs::Float32 force_msg;
                    force_msg.data = force_value;
                    force_pub_.publish(force_msg);
                    //ROS_INFO("Published force: %.2f", force_value);
                    force_value += (3.0 - 0.5) / (interval_grasp * 10); // 선형 증가
                    ros::Duration(interval_grasp / 10).sleep();
                }
            }else if (msg.data == "forcereset") {
                std_msgs::Float32 force_msg;
                force_msg.data = 0.5;
                force_pub_.publish(force_msg);
                ros::Duration(0.1).sleep();
            } else {
                lib_cmd_pub_.publish(msg);
            // ROS_INFO("Published command: %s", msg.data.c_str());
            }

            ros::spinOnce();
            if (msg.data != "forcereset" && msg.data != "grasp_3" && msg.data != "grasp_4") {
                rate.sleep();
            }
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "mission_node");
    CommandPublisher node;
    node.publishCommands();
    return 0;
}
