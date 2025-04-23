#include <QApplication>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QWidget>
#include <QProgressBar>
#include <QLabel>

#include <ros/ros.h>
#include <ros/package.h>
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/yaml_config_reader.h>
#include <rviz/config.h>
#include <rviz/visualization_frame.h>
#include <std_msgs/Int32MultiArray.h>
#include <visualization_msgs/Marker.h>

int fingertip_sensor[3] = {0, 0, 0};  // Fingertip 센서 값 (정수형)

QProgressBar* gauge1;
QProgressBar* gauge2;
QProgressBar* gauge3;
QLabel* sensor1_label;
QLabel* sensor2_label;
QLabel* sensor3_label;

ros::Publisher marker_pub;


// Rviz 손가락 끝에 'INDEX', 'PINKY', 'THUMB' 문구 추가
void publishTextMarker(const std::string& text, const std::string& frame_id, int marker_id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;  // 손가락 끝 TF 프레임
    marker.header.stamp = ros::Time::now();
    marker.ns = "finger_labels";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;

    // 상대 위치 설정 (손가락 끝 기준)
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.03;  // 살짝 위로 띄움

    // 텍스트 설정
    marker.text = text;

    // 크기 설정
    marker.scale.z = 0.02;

    // 색상 설정 (검은색)
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker_pub.publish(marker);
}

void updateMarkers() {
    publishTextMarker("1", "link_2_0_tip", 0);
    publishTextMarker("2", "link_5_0_tip", 1);
    publishTextMarker("3", "link_8_0_tip", 2);
}


void updateProgressBarColor(QProgressBar* gauge, float value) {

    if (value >=500)
        value = 500;
    // 값에 따라 ProgressBar 색상 변경 (0 ~ 1000 범위)
    if (value <= 64) {
        // 0에서 125까지는 파란색에서 초록색으로 점차 변함
        float green_value = (float)value / 64.0f;
        gauge->setStyleSheet(QString("QProgressBar::chunk { background: rgb(0, %1, 255); }").arg(int(green_value * 255)));
    } else if (value <= 128) {
        // 125에서 250까지는 초록색에서 빨간색으로 점차 변함
        float blue_value = 1.0f - ((float)(value - 64.0f) / 64.0f);
        gauge->setStyleSheet(QString("QProgressBar::chunk { background: rgb(0, 255, %1); }").arg(int(blue_value * 255)));
    } else if (value <= 192){
        // 250에서 375까지는 초록색에서 빨간색으로 점차 변함
        float red_value = (float)(value - 128.0f) / 64.0f;
        gauge->setStyleSheet(QString("QProgressBar::chunk { background: rgb(%1, 255, 0); }").arg(int(red_value * 255))); 
    } else if (value <= 255){
        // 375에서 500까지는 초록색에서 빨간색으로 점차 변함
        float green_value = 1.0f - ((float)(value - 192.0f) / 64.0f);
        gauge->setStyleSheet(QString("QProgressBar::chunk { background: rgb(255, %1, 0); }").arg(int(green_value * 255))); 
    } else
        gauge->setStyleSheet(QString("QProgressBar::chunk { background: rgb(255, 0, 0); }")); 

}

void fingertipSensorCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if (msg->data.size() >= 3) {
        fingertip_sensor[0] = msg->data[0];
        fingertip_sensor[1] = msg->data[1];
        fingertip_sensor[2] = msg->data[2];

        sensor1_label->setText("Fingertip Sensor 1: " + QString::number(fingertip_sensor[0]) + "  Pa");
        sensor2_label->setText("Fingertip Sensor 2: " + QString::number(fingertip_sensor[1]) + "  Pa");
        sensor3_label->setText("Fingertip Sensor 3: " + QString::number(fingertip_sensor[2]) + "  Pa");
   
        // fingertip_sensor 값에 따라 QProgressBar 업데이트
        float scaled_value1 = fingertip_sensor[0] * (255.0f / 500.0f);  // 값의 범위를 0~1000으로 변환
        float scaled_value2 = fingertip_sensor[1] * (255.0f / 500.0f);
        float scaled_value3 = fingertip_sensor[2] * (255.0f / 500.0f);

        gauge1->setValue(std::min(fingertip_sensor[0],500));
        gauge2->setValue(std::min(fingertip_sensor[1],500));
        gauge3->setValue(std::min(fingertip_sensor[2],500));

        updateProgressBarColor(gauge1, scaled_value1);
       // ros::Duration(1e-6).sleep();
        updateProgressBarColor(gauge2, scaled_value2);
        ros::Duration(1e-6).sleep();
        updateProgressBarColor(gauge3, scaled_value3);
      //  ros::Duration(1e-6).sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rviz_embedded_node");
    ros::NodeHandle nh;

    marker_pub = nh.advertise<visualization_msgs::Marker>("/finger_labels", 10);

    QApplication app(argc, argv);
    QWidget window;
    QHBoxLayout* mainLayout = new QHBoxLayout(&window);

    rviz::VisualizationFrame* frame = new rviz::VisualizationFrame();
    frame->initialize();

    std::string rviz_config_path = ros::package::getPath("allegro_hand_description") + "/allegro_hand_qt_config.rviz";
    if (!rviz_config_path.empty()) {
        rviz::Config config;
        rviz::YamlConfigReader reader;
        reader.readFile(config, QString::fromStdString(rviz_config_path));

        if (config.isValid()) {
            frame->load(config);
        } else {
            ROS_ERROR("Failed to load RViz config file: %s", rviz_config_path.c_str());
        }
    } else {
        ROS_ERROR("Failed to find RViz config file!");
    }

    frame->setFixedSize(800, 1000);

    gauge1 = new QProgressBar();
    gauge2 = new QProgressBar();
    gauge3 = new QProgressBar();

    gauge1->setRange(0, 500);
    gauge2->setRange(0, 500);
    gauge3->setRange(0, 500);

    gauge1->setTextVisible(false);
    gauge2->setTextVisible(false);
    gauge3->setTextVisible(false);

    gauge1->setFixedHeight(30);
    gauge2->setFixedHeight(30);
    gauge3->setFixedHeight(30);

    gauge1->setFixedWidth(300);
    gauge2->setFixedWidth(300);
    gauge3->setFixedWidth(300);

    sensor1_label = new QLabel("Fingertip Sensor 1: 0  Pa", &window);
    sensor2_label = new QLabel("Fingertip Sensor 2: 0  Pa", &window);
    sensor3_label = new QLabel("Fingertip Sensor 3: 0  Pa", &window);

    QFont font = sensor1_label->font();
    font.setPointSize(18);
    sensor1_label->setFont(font);
    sensor2_label->setFont(font);
    sensor3_label->setFont(font);

    QVBoxLayout* gaugeLayout = new QVBoxLayout();
    gaugeLayout->addWidget(sensor1_label);
    gaugeLayout->addWidget(gauge1);
    gaugeLayout->addWidget(sensor2_label);
    gaugeLayout->addWidget(gauge2);
    gaugeLayout->addWidget(sensor3_label);
    gaugeLayout->addWidget(gauge3);

    mainLayout->addWidget(frame, 6);
    mainLayout->addLayout(gaugeLayout, 1);

    window.setLayout(mainLayout);
    window.show();

    ros::Subscriber sensor_sub = nh.subscribe<std_msgs::Int32MultiArray>("fingertip_sensors", 1, fingertipSensorCallback);

    ros::Rate loop_rate(1000);
    while (ros::ok()) {
        ros::spinOnce();
        updateMarkers();  // 손가락 라벨 업데이트
        app.processEvents();
        loop_rate.sleep();
    }
    
    return 0;
    //return app.exec();
}
