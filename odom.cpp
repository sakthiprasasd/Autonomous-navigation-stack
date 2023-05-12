#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

class OdometryCalculator {
public:
    OdometryCalculator() {
        // Initialize the ROS node handle
        nh_ = ros::NodeHandle("~");

        // Subscribe to the IMU and encoder topics
        imu_sub_ = nh_.subscribe("/imu", 1, &OdometryCalculator::imuCallback, this);
        left_encoder_sub_ = nh_.subscribe("/left_encoder", 1, &OdometryCalculator::leftEncoderCallback, this);
        right_encoder_sub_ = nh_.subscribe("/right_encoder", 1, &OdometryCalculator::rightEncoderCallback, this);

        // Advertise the odometry topic
        odom_pub_ = nh_.advertise<std_msgs::String>("/odometry", 1);

        // Set the initial encoder values
        left_encoder_value_ = 0.0;
        right_encoder_value_ = 0.0;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // Store the IMU data
        imu_timestamp_ = msg->header.stamp;
        imu_linear_acceleration_ = msg->linear_acceleration;
        imu_angular_velocity_ = msg->angular_velocity;
        imu_orientation_ = msg->orientation;
    }

    void leftEncoderCallback(const std_msgs::Float32::ConstPtr& msg) {
        // Store the left encoder data
        left_encoder_value_ = msg->data;
    }

    void rightEncoderCallback(const std_msgs::Float32::ConstPtr& msg) {
        // Store the right encoder data
        right_encoder_value_ = msg->data;
    }

    void calculateOdometry() {
        // Calculate the distance traveled by the left and right wheels
        double left_distance = (left_encoder_value_ / encoder_ticks_per_revolution_) * wheel_circumference_;
        double right_distance = (right_encoder_value_ / encoder_ticks_per_revolution_) * wheel_circumference_;

        // Calculate the average distance traveled
        double distance = (left_distance + right_distance) / 2.0;

        // Calculate the change in orientation
        double orientation_change = (right_distance - left_distance) / wheelbase_ * imu_angular_velocity_.z * (ros::Time::now() - imu_timestamp_).toSec();

        // Calculate the new orientation
        imu_orientation_radians_ += orientation_change;

        // Create the odometry message
        std_msgs::String odom_msg;
        std::stringstream ss;
        ss << "Distance: " << distance << " Orientation: " << imu_orientation_radians_;
        odom_msg.data = ss.str();

        // Publish the odometry message
        odom_pub_.publish(odom_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber left_encoder_sub_;
    ros::Subscriber right_encoder_sub_;
    ros::Publisher odom_pub_;

    ros::Time imu_timestamp_;
    geometry_msgs::Vector3 imu_linear_acceleration_;
    geometry_msgs::Vector3 imu_angular_velocity_;
    geometry_msgs::Quaternion imu_orientation_;
    double imu_orientation_radians_;

    float left_encoder_value_;
    float right_encoder_value_;
    const float encoder_ticks_per_revolution_ = 1000.0;
    const float wheel_circumference_ = 2.0 * M_PI * 0.1; // 10 cm wheel diameter
    const float wheelbase_ = 0.5; // Distance between the two wheels
};

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "odometry_calculator");

  // Create the OdometryCalculator object
  OdometryCalculator odom_calculator;

  // Set the loop rate
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    // Calculate the odometry
    odom_calculator.calculateOdometry();

    // Spin once to check for incoming messages
    ros::spinOnce();

    // Sleep to maintain the loop rate
    loop_rate.sleep();
  }

  return 0;
}

