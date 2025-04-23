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
    std::vector<std::string> commands_{"home", "grasp_1", "home", "grasp_2", "home", "grasp_3", "forcechange", "home","forcereset","sphere","grasp_3", "forcechange","sphere","forcereset"};
    int command_index_ = 0;
    double interval_ = 1.5; // 기본 간격 (초)
    std::mutex mutex_;

public:
    CommandPublisher() {
        lib_cmd_pub_ = nh_.advertise<std_msgs::String>("allegroHand/lib_cmd", 10);
        force_pub_ = nh_.advertise<std_msgs::Float32>("allegroHand/forcechange", 1);
    }

    void publishCommands() {
    ros::Rate rate(1.0 / interval_);
    // 초기 3초 대기
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
            double mid_time = start_time + (interval_ / 5.0);  // 절반 시간
            double end_time = start_time + interval_;
            double force_value = 2.0;

            while (ros::Time::now().toSec() < end_time && ros::ok()) {
                std_msgs::Float32 force_msg;
                
                // 절반 시간 이후에는 force_value를 8.0으로 설정
                if (ros::Time::now().toSec() >= mid_time) {
                    force_value = 8.0;
                }

                force_msg.data = force_value;
                force_pub_.publish(force_msg);

                ros::Duration(interval_ / 10).sleep();
            }
        }
        else if (msg.data == "forcereset") {
            std_msgs::Float32 force_msg;
            force_msg.data = 2.0;
            force_pub_.publish(force_msg);
            ros::Duration(0.1).sleep();
        } else {
            lib_cmd_pub_.publish(msg);
           // ROS_INFO("Published command: %s", msg.data.c_str());
        }

        ros::spinOnce();
        if (msg.data != "forcereset") {
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
