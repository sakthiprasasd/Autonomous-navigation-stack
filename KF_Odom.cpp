#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <eigen3/Eigen/Dense>
#include <stack_msgs/Odometry.h>

using namespace Eigen;

// Kalman filter variables
VectorXd x; // state vector
MatrixXd P; // state covariance matrix
VectorXd u; // control input vector
MatrixXd F; // state transition matrix
MatrixXd H; // measurement matrix
MatrixXd R; // measurement covariance matrix
MatrixXd I; // identity matrix

ros::Publisher odom_pub;

// Function to initialize Kalman filter variables
void initKalmanFilter()
{
    x.resize(9);
    x << 0, 0, 0, 0, 0, 0, 0, 0, 0; // initial state [x, y, z, orientation_x, orientation_y, orientation_z, orientation_w, twist_linear_x, twist_angular_z]

    P.resize(9, 9);
    P << 0.001, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0.001, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0.001, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0.001, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0.001, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0.001, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0.001, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0.001, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0.001; // initial state covariance matrix

    u.resize(5);
    u << 0, 0, 0, 0, 0; // control input [linear_x, linear_y, linear_z, angular_z]

    F.resize(9, 9);
    F << 1, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 1, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 1; // state transition matrix

    H.resize(9, 9);
    H.setIdentity(); // measurement matrix

    R.resize(9, 9);
    R << 0.001, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0.001, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0.001, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0.001, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0.001, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0.001, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0.001, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0.001, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0.001; // measurement covariance matrix

    I.resize(9, 9);
    I.setIdentity(); // identity matrix
}

// Prediction step of the Kalman filter
void predictKalmanFilter(double dt)
{
    // Update the state and state covariance matrix based on the control input
    x.head(6) = F.topLeftCorner(6, 6) * x.head(6) + u.head(6) * dt;
    x.tail(3) = u.tail(3); // Assuming twist values are measured directly, not predicted
    P.topLeftCorner(6, 6) = F.topLeftCorner(6, 6) * P.topLeftCorner(6, 6) * F.topLeftCorner(6, 6).transpose();
}

// Update/Correction step of the Kalman filter
void updateKalmanFilter(const VectorXd& z)
{
    // Calculate the Kalman gain
    MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    // Calculate the measurement residual
    VectorXd y = z - H * x;

    // Update the state and state covariance matrix based on the measurement residual
    x = x + K * y;
    P = (I - K * H) * P;
}

// Callback function for raw odometry data
void rawOdomCallback(const stack_msgs::Odometry::ConstPtr& msg)
{
    // Process raw odometry data and extract position, orientation, and twist
    double raw_odom_x = msg->pose.pose.position.x;
    double raw_odom_y = msg->pose.pose.position.y;
    double raw_odom_z = msg->pose.pose.position.z;
    double raw_odom_orientation_x = msg->pose.pose.orientation.x;
    double raw_odom_orientation_y = msg->pose.pose.orientation.y;
    double raw_odom_orientation_z = msg->pose.pose.orientation.z;
    double raw_odom_orientation_w = msg->pose.pose.orientation.w;
    double raw_odom_twist_linear_x = msg->twist.twist.linear.x;
    double raw_odom_twist_angular_z = msg->twist.twist.angular.z;

    ROS_INFO("Raw Odometry - X: %f, Y: %f", raw_odom_x, raw_odom_y);

    // Create a measurement vector
    VectorXd z(9);
    z << raw_odom_x, raw_odom_y, raw_odom_z, raw_odom_orientation_x, raw_odom_orientation_y, raw_odom_orientation_z, raw_odom_orientation_w, raw_odom_twist_linear_x, raw_odom_twist_angular_z;

    // Update the Kalman filter with the raw odometry sensor inputs
    updateKalmanFilter(z);

    ROS_INFO("Estimated Position - X: %f, Y: %f, Z: %f", x[0], x[1], x[2]);

    // Publish the estimated odometry
    stack_msgs::Odometry odom;
    odom.pose.pose.position.x = x[0];
    odom.pose.pose.position.y = x[1];
    odom.pose.pose.position.z = x[2];
    odom.pose.pose.orientation.x = x[3];
    odom.pose.pose.orientation.y = x[4];
    odom.pose.pose.orientation.z = x[5];
    odom.pose.pose.orientation.w = x[6];
    odom.twist.twist.linear.x = x[7];
    odom.twist.twist.angular.z = x[8];
    odom_pub.publish(odom);
}

// Main function
int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_node");
    ros::NodeHandle nh;

    initKalmanFilter();

    ros::Subscriber raw_odom_sub = nh.subscribe<stack_msgs::Odometry>("/raw_odom", 10, rawOdomCallback);
    odom_pub = nh.advertise<stack_msgs::Odometry>("/odom", 10);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

