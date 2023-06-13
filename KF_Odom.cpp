#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>

using namespace Eigen;

// Kalman filter variables
VectorXd x; // state vector
MatrixXd P; // state covariance matrix
VectorXd u; // control input vector
MatrixXd F; // state transition matrix
MatrixXd H; // measurement matrix
MatrixXd R; // measurement covariance matrix
MatrixXd Q; // process covariance matrix
MatrixXd I; // identity matrix

ros::Publisher odom_pub;

// Function to initialize Kalman filter variables
void initKalmanFilter()
{
    x.resize(3);
    x << 0, 0, 0; // initial state [x, y, theta]

    P.resize(3, 3);
    P << 0.001, 0, 0,
         0, 0.001, 0,
         0, 0, 0.001; // initial state covariance matrix

    u.resize(3);
    u << 0, 0, 0; // control input [linear_x, linear_y, angular_z]

    F.resize(3, 3);
    F.setIdentity(); // state transition matrix

    H.resize(3, 3);
    H.setIdentity(); // measurement matrix

    R.resize(3, 3);
    R << 0.001, 0, 0,
         0, 0.001, 0,
         0, 0, 0.001; // measurement covariance matrix

    Q.resize(3, 3);
    Q << 0.001, 0, 0,
         0, 0.001, 0,
         0, 0, 0.001; // process covariance matrix

    I.resize(3, 3);
    I.setIdentity(); // identity matrix
}

// Prediction step: Predict the next odometry based on control input
void predictOdometry(double dt)
{
    double linear_x = u[0];
    double linear_y = u[1];
    double angular_z = u[2];

    // Calculate the predicted odometry values
    double predicted_x = x[0] + linear_x * dt;
    double predicted_y = x[1] + linear_y * dt;
    double predicted_theta = x[2] + angular_z * dt;

    // Update the state vector with the predicted values
    x << predicted_x, predicted_y, predicted_theta;
}

// Prediction step of the Kalman filter
void predictKalmanFilter(double dt)
{
    // Predict the next odometry based on control input
    predictOdometry(dt);

    // Update the state transition matrix based on the control input
    F(0, 2) = -u[1] * dt;
    F(1, 2) = u[0] * dt;

    // Update the state and state covariance matrix based on the predicted odometry
    x = F * x;
    P = F * P * F.transpose() + Q;
}

// Update/Correction step of the Kalman filter
void updateKalmanFilter(const VectorXd& z)
{
    // Calculate the Kalman gain
    MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    // Update the state vector and covariance matrix based on the measured odometry
    x = x + K * (z - H * x);
    P = (I - K * H) * P;
}

// Callback function for IMU data
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Extract angular velocity around Z-axis from IMU data
    double angular_z = msg->angular_velocity.z;

    // Update the control input vector
    u[2] = angular_z;
}

// Callback function for control input (velocity command)
void controlInputCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // Extract control input values (linear velocities)
    double linear_x = msg->linear.x;
    double linear_y = msg->linear.y;

    // Update the control input vector
    u[0] = linear_x;
    u[1] = linear_y;
}

// Callback function for raw odometry data
void rawOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Process raw odometry data and extract position
    double raw_odom_x = msg->pose.pose.position.x;
    double raw_odom_y = msg->pose.pose.position.y;
    double raw_odom_theta = msg->pose.pose.orientation.z;

    ROS_INFO("Raw Odometry - X: %f, Y: %f, Theta: %f", raw_odom_x, raw_odom_y, raw_odom_theta);

    // Create a measurement vector
    VectorXd z(3);
    z << raw_odom_x, raw_odom_y, raw_odom_theta;

    // Update the Kalman filter with the measured odometry
    updateKalmanFilter(z);

    ROS_INFO("Estimated Position - X: %f, Y: %f, Theta: %f", x[0], x[1], x[2]);

    // Publish the corrected odometry
    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = x[0];
    odom.pose.pose.position.y = x[1];
    odom.pose.pose.orientation.z = x[2];
    odom_pub.publish(odom);
}

// Main function
int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_node");
    ros::NodeHandle nh;

    initKalmanFilter();

    ros::Subscriber control_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, controlInputCallback);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu", 10, imuCallback);
    ros::Subscriber raw_odom_sub = nh.subscribe<nav_msgs::Odometry>("/raw_odom", 10, rawOdomCallback);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);

    double dt = 0.1; // time step for prediction (adjust as needed)
    ros::Rate loop_rate(1 / dt);

    while (ros::ok())
    {
        predictKalmanFilter(dt);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
