#ifndef WITMOTION_ROS
#define WITMOTION_ROS

#include <QFile>
#include <QSerialPort>
#include <QIODevice>
#include <QThread>
#include <QCoreApplication>
#include <QDateTime>

#include "witmotion/types.h"
#include "witmotion/serial.h"

#include <boost/array.hpp>
#include <vector>
#include <algorithm>
#include <boost/range/algorithm.hpp>
#include <ctime>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_srvs/srv/empty.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

using namespace witmotion;

class ROSWitmotionSensorController: public QObject, public rclcpp::Node
{
    Q_OBJECT
private:
    /* QT FIELDS */
    std::string port_name;
    QSerialPort::BaudRate port_rate;
    uint32_t interval;
    QThread reader_thread;
    QBaseSerialWitmotionSensorReader* reader;
    bool suspended;
   
    /* ROS FIELDS*/
  
    //std::shared_ptr<rclcpp::Node> node; 
    bool Restart(std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response);
    std::string _restart_service_name;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr restart_service;
 

    /* IMU */
    std::string _imu_topic;
    std::string imu_frame_id;
    double imu_yaw_correction_angle;
    bool imu_enable_accel;
    bool imu_have_accel;
    std::vector<double> imu_accel_covariance;
    bool imu_enable_velocities;
    bool imu_have_velocities;
    std::vector<double> imu_velocity_covariance;
    bool imu_enable_orientation;
    bool imu_have_orientation;
    bool imu_native_orientation;
    std::vector<double> imu_orientation_covariance;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
    void imu_process(const witmotion_datapacket& packet);

    /* TEMPERATURE */
    std::string _temp_topic;
    witmotion_packet_id temp_from;
    std::string temp_frame_id;
    bool temp_enable;
    float temp_variance;
    float temp_coeff;
    float temp_addition;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_publisher;
    void temp_process(const witmotion_datapacket& packet);

    /* MAGNETOMETER */
    std::string _magnetometer_topic;
    std::string magnetometer_frame_id;
    bool magnetometer_enable;
    std::vector<double> magnetometer_covariance;
    void magnetometer_process(const witmotion_datapacket& packet);
    float magnetometer_coeff;
    float magnetometer_addition;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetometer_publisher;

    /* BAROMETER */
    std::string _barometer_topic;
    std::string barometer_frame_id;
    bool barometer_enable;
    double barometer_variance;
    double barometer_coeff;
    double barometer_addition;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr barometer_publisher;

    /* ALTIMETER */
    std::string _altimeter_topic;
    bool altimeter_enable;
    bool have_altitude;
    double last_altitude;
    double altimeter_coeff;
    double altimeter_addition;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr altimeter_publisher;
    void altimeter_process(const witmotion_datapacket& packet);

    /* ORIENTATION */
    std::string _orientation_topic;
    bool orientation_enable;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr orientation_publisher;
    void orientation_process(const witmotion_datapacket& packet);

    /* GPS */
    bool gps_enable;
    bool have_gps;
    bool have_ground_speed;
    bool have_accuracy;
    uint32_t satellites;
    std::vector<double> gps_covariance;
    std::string gps_frame_id;
    float gps_altitude;

    std::string _gps_topic;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher;
    void gps_process(const witmotion_datapacket& packet);

    std::string _ground_speed_topic;
    std::string _gps_altitude_topic;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ground_speed_publisher; 
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gps_altitude_publisher;
    void ground_speed_process(const witmotion_datapacket& packet);

    std::string _accuracy_topic;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr accuracy_publisher;  
    std::string _satellites_topic;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr satellites_publisher;
    void accuracy_process(const witmotion_datapacket& packet);

    /* REALTIME CLOCK */
    bool rtc_enable;
    std::string _rtc_topic;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr  rtc_publisher;
    bool rtc_presync;
    void rtc_process(const witmotion_datapacket& packet);
public:
    ROSWitmotionSensorController();
    virtual ~ROSWitmotionSensorController();
    void Start();
public slots:
    void Packet(const witmotion_datapacket& packet);
    void Error(const QString& description);
signals:
    void RunReader();
    void ConfigureSensor(const witmotion_config_packet& config_packet);
};

#endif
