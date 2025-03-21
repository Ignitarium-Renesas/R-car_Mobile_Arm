#ifndef MYAGV_ODOMETRY_NODE_HPP
#define MYAGV_ODOMETRY_NODE_HPP

#include <cstdio>
#include <vector>
#include <iostream>
#include <iomanip>
#include <time.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "std_msgs/msg/float32.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/asio.hpp>

#define twoKpDef	1.0f				// (2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	0.0f				// (2.0f * 0.0f)	// 2 * integral gain
#define TOTAL_RECEIVE_SIZE 27         	// 27 //The length of the data sent by the esp32 //esp32
#define OFFSET_COUNT 	200
#define DEG_TO_RAD (M_PI / 180.0)
#define CORRECTION_FACTOR_FX 1.19047
#define CORRECTION_FACTOR_RX 1.07526

class MyAGVOdometry : public rclcpp::Node
{
    public:
        MyAGVOdometry();
        ~MyAGVOdometry();
        bool init();
        float invSqrt(float number);
        void execute(double linearX, double linearY, double angularZ);
        //void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
        void accelerometerOffset(float gx, float gy, float gz);
        void publisherOdom();
        void publisherImuSensor();
        void publisherImuSensorRaw();
        void Publish_Voltage();

    private:

        bool readSpeed();
        void writeSpeed(double movex, double movey, double rot);
        void restore();
        void restoreRun();
        void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void checkCmdVelTimeout();
        void setZeroVelocity();
        void ask_for_input();
        geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);
        void run();

        rclcpp::Time currentTime, lastTime;

        double linearX;
        double linearY;
        double angularZ;

        double x;
        double y;
        double theta;

        double vx;
        double vy;
        double vtheta;

        double ax;
        double ay;
        double az;

        double wx;
        double wy;
        double wz;
        
        float Gyroscope_Xdata_Offset;
        float Gyroscope_Ydata_Offset;
        float Gyroscope_Zdata_Offset;
        float sampleFreq;
        float Battery_voltage,Backup_Battery_voltage; 
        unsigned short Offest_Count;
        sensor_msgs::msg::Imu imu_data;
        // Publishers
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_voltage;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_raw;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
        rclcpp::TimerBase::SharedPtr timer_;

        // TransformBroadcaster
        std::shared_ptr<tf2_ros::TransformBroadcaster> odomBroadcaster;
        bool cmd_vel_received_;
        rclcpp::Time last_cmd_time_;
        double timeout_duration_;
        bool direction_flag;

        float x_max_speed_;
        float y_max_speed_;
        float angular_max_speed_;

};

#endif // MYAGV_ODOMETRY_NODE_HPP