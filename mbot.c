/**
 * This file is the main executable for the MBot firmware.
 */

#include <pico/stdlib.h>
#include <pico/mutex.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>
#include <rc/motor/motor.h>
#include <rc/mpu/mpu.h>
#include <comms/common.h>
#include <comms/protocol.h>
#include <comms/listener.h>
#include <comms/topic_data.h>
#include <comms/mbot_messages.h>

#include <math.h>
#include <inttypes.h>
#include "mbot.h"

#define LED_PIN 25
#define MAIN_LOOP_HZ 20.0 // 20 hz loop
#define MAIN_LOOP_PERIOD (1.0f / MAIN_LOOP_HZ)
// #define ENCODER_RESOLUTION 48.0
#define GEAR_RATIO 78
#define TIMESTEP_S 1.0f
#define TIME_CONSTANT 0.1
#define CAP_RPM 0.0f
const float RPM_conversion_factor = 60.0 / (GEAR_RATIO * TIMESTEP_S * ENCODER_RES);

// data to hold current mpu state
static rc_mpu_data_t mpu_data;

uint64_t timestamp_offset = 0;
uint64_t current_pico_time = 0;

float enc2meters = ((2.0 * PI * WHEEL_RADIUS) / (GEAR_RATIO * ENCODER_RES));
float rpm2meters = (2 * PI);

void timestamp_cb(timestamp_t *received_timestamp)
{
    // if we havent set the offset yet
    if (timestamp_offset == 0)
    {
        uint64_t cur_pico_time = to_us_since_boot(get_absolute_time());
        timestamp_offset = received_timestamp->utime - cur_pico_time;
    }
}

void reset_encoders_cb(mbot_encoder_t *received_encoder_vals)
{
    rc_encoder_write(LEFT_MOTOR_CHANNEL, received_encoder_vals->leftticks);
    rc_encoder_write(RIGHT_MOTOR_CHANNEL, received_encoder_vals->rightticks);
}

void reset_odometry_cb(odometry_t *received_odom)
{
    current_odom.utime = received_odom->utime;
    current_odom.x = received_odom->x;
    current_odom.y = received_odom->y;
    current_odom.theta = received_odom->theta;
}

int write_pid_coefficients(i2c_inst_t *i2c)
{
    uint8_t pid_bytes[PID_VALUES_LEN];
    memcpy(pid_bytes, &mbot_pid_gains, PID_VALUES_LEN);
    return rc_write_fram(i2c, PID_VALUES_ADDR, PID_VALUES_LEN, &pid_bytes[0]);
}

void pid_values_cb(mbot_pid_gains_t *received_pid_gains)
{
    memcpy(&mbot_pid_gains, received_pid_gains, sizeof(mbot_pid_gains_t));
    write_pid_coefficients(i2c);
}

void register_topics()
{
    // timesync topic
    comms_register_topic(MBOT_TIMESYNC, sizeof(timestamp_t), (Deserialize)&timestamp_t_deserialize, (Serialize)&timestamp_t_serialize, (MsgCb)&timestamp_cb);
    // odometry topic
    comms_register_topic(ODOMETRY, sizeof(odometry_t), (Deserialize)&odometry_t_deserialize, (Serialize)&odometry_t_serialize, NULL);
    // reset odometry topic
    comms_register_topic(RESET_ODOMETRY, sizeof(odometry_t), (Deserialize)&odometry_t_deserialize, (Serialize)&odometry_t_serialize, (MsgCb)&reset_odometry_cb);
    // IMU topic
    comms_register_topic(MBOT_IMU, sizeof(mbot_imu_t), (Deserialize)&mbot_imu_t_deserialize, (Serialize)&mbot_imu_t_serialize, NULL);
    // encoders topic
    comms_register_topic(MBOT_ENCODERS, sizeof(mbot_encoder_t), (Deserialize)&mbot_encoder_t_deserialize, (Serialize)&mbot_encoder_t_serialize, NULL);
    // reset encoders topic
    comms_register_topic(RESET_ENCODERS, sizeof(mbot_encoder_t), (Deserialize)&mbot_encoder_t_deserialize, (Serialize)&mbot_encoder_t_serialize, (MsgCb)&reset_encoders_cb);
    // motor commands topic
    comms_register_topic(MBOT_MOTOR_COMMAND, sizeof(mbot_motor_command_t), (Deserialize)&mbot_motor_command_t_deserialize, (Serialize)&mbot_motor_command_t_serialize, NULL);
    // PID values topic
    comms_register_topic(MBOT_PIDS, sizeof(mbot_pid_gains_t), (Deserialize)&mbot_pid_gains_t_deserialize, (Serialize)&mbot_pid_gains_t_serialize, (MsgCb)&pid_values_cb);
}

void read_pid_coefficients(i2c_inst_t *i2c)
{
    uint8_t pid_bytes[PID_VALUES_LEN];

    if (rc_read_fram(i2c, PID_VALUES_ADDR, PID_VALUES_LEN, pid_bytes) > 0)
    {
        printf("reading fram success.\n");
        memcpy(&mbot_pid_gains, pid_bytes, PID_VALUES_LEN);
        printf("read gains from fram!\r\n");
    }
    else
    {
        printf("reading PID gains from fram failure.\n");
    }
}

// Define a function to return the open loop control command
float open_loop_control(int MOTOR_CHANNEL, float sp)
{
    if (MOTOR_CHANNEL == LEFT_MOTOR_CHANNEL)
    {
            int16_t l_cmd;                 // left motor commands
            float l_duty;                 // duty cycle in range [-1, 1]
            float left_sp = sp;           // left speed
            // left_sp *= LEFT_ENC_POL;
            if (left_sp < -CAP_RPM)
                {
                    l_duty = left_sp * LEFT_ENC_POL * motor_1_recover.motor_back.slope + motor_1_recover.motor_back.intercept;
                }
            else if (left_sp > CAP_RPM)
                {
                    l_duty = left_sp * LEFT_ENC_POL * motor_1_recover.motor_front.slope + motor_1_recover.motor_front.intercept;
                }
            else
                {
                    l_duty = 0;
                }
            // Clamp duty cycle to [-1, 1]
            l_duty = clamp_duty(l_duty);

            return l_duty;
    }
    else if (MOTOR_CHANNEL == RIGHT_MOTOR_CHANNEL)
    {
        int16_t r_cmd;                 //right motor commands
        float r_duty;                 // duty cycle in range [-1, 1]
        float right_sp = sp;           // right speed
        // right_sp *= RIGHT_ENC_POL;
        if (right_sp < -CAP_RPM)
            {
                r_duty = right_sp * RIGHT_ENC_POL * motor_3_recover.motor_back.slope + motor_3_recover.motor_back.intercept;
            }
        else if (right_sp > CAP_RPM)
            {
                r_duty = right_sp * RIGHT_ENC_POL * motor_3_recover.motor_front.slope + motor_3_recover.motor_front.intercept;
            }
        else
            {
                r_duty = 0;
            }
        // Clamp duty cycle to [-1, 1]
        r_duty = clamp_duty(r_duty);

        return r_duty;
    }
}


bool timer_cb(repeating_timer_t *rt)
{
    // Read the PID values
    if (comms_get_topic_data(MBOT_PIDS, &mbot_pid_gains))
    {
        uint64_t msg_time = current_pico_time;
        // Print the PID values
        printf("Left: %f, %f, %f, %f", mbot_pid_gains.motor_a_kp,
               mbot_pid_gains.motor_a_ki,
               mbot_pid_gains.motor_a_kd,
               1.0 / mbot_pid_gains.motor_a_Tf);

        // update the PID gains of the left motor PID controller
        rc_filter_pid(&left_pid,
                      mbot_pid_gains.motor_a_kp,
                      mbot_pid_gains.motor_a_ki,
                      mbot_pid_gains.motor_a_kd,
                      1.0 / mbot_pid_gains.motor_a_Tf,
                      1.0 / MAIN_LOOP_HZ);

        // update the PID gains of the right motor PID controller
        rc_filter_pid(&right_pid,
                      mbot_pid_gains.motor_c_kp,
                      mbot_pid_gains.motor_c_ki,
                      mbot_pid_gains.motor_c_kd,
                      1.0 / mbot_pid_gains.motor_c_Tf,
                      1.0 / MAIN_LOOP_HZ);

        // update the PID gains of the body frame controller
        rc_filter_pid(&fwd_vel_pid,
                      mbot_pid_gains.bf_trans_kp,
                      mbot_pid_gains.bf_trans_ki,
                      mbot_pid_gains.bf_trans_kd,
                      1.0 / mbot_pid_gains.bf_trans_Tf,
                      1.0 / MAIN_LOOP_HZ);

        // update the PID gains of the body frame rotation controller
        rc_filter_pid(&turn_vel_pid,
                      mbot_pid_gains.bf_rot_kp,
                      mbot_pid_gains.bf_rot_ki,
                      mbot_pid_gains.bf_rot_kd,
                      1.0 / mbot_pid_gains.bf_rot_Tf,
                      1.0 / MAIN_LOOP_HZ);
    }
    // only run if we've received a timesync message...
    if (comms_get_topic_data(MBOT_TIMESYNC, &received_time))
    {
        uint64_t cur_pico_time = to_us_since_boot(get_absolute_time()) + timestamp_offset;
        uint64_t latency_time = cur_pico_time - current_pico_time;
        current_pico_time = cur_pico_time;
        // first, get the IMU data and send across the wire
        current_imu.utime = cur_pico_time; // received_time.utime;

        // read the encoders
        int enc_cnt_l = LEFT_ENC_POL * rc_encoder_read_count(LEFT_MOTOR_CHANNEL);
        int enc_delta_l = LEFT_ENC_POL * rc_encoder_read_delta(LEFT_MOTOR_CHANNEL);
        int enc_cnt_r = RIGHT_ENC_POL * rc_encoder_read_count(RIGHT_MOTOR_CHANNEL);
        int enc_delta_r = RIGHT_ENC_POL * rc_encoder_read_delta(RIGHT_MOTOR_CHANNEL);
        current_encoders.utime = cur_pico_time; // received_time.utime;
        current_encoders.right_delta = enc_delta_r;
        current_encoders.rightticks = enc_cnt_r;
        current_encoders.left_delta = enc_delta_l;
        current_encoders.leftticks = enc_cnt_l;

        // compute new odometry
        /*************************************************************
         * TODO:
         *      - Populate the odometry messages.
         *          -The struct for odometry_t is defined in comms/include/comms/messages_mb.h (DO NOT EDIT THIS FILE)
         *      - Note that the way we compute the displacement of the motor is from encoder readings and
         *        that we convert encoder readings to meters using the enc2meters variable defined above.
         *      - Use the equations provided in the document to compute the odometry components
         *      - Remember to clamp the orientation between [0, 2pi]!
         *************************************************************/
        float delta_d, delta_theta; // displacement in meters and rotation in radians
        float left_distance_delta = -enc2meters * enc_delta_l;
        float right_distance_delta = -enc2meters * enc_delta_r;
        delta_theta = (right_distance_delta - left_distance_delta)/WHEEL_BASE;
        delta_d = (right_distance_delta + left_distance_delta)/2;

        float delta_gyro = mpu_data.gyro[2]/180*M_PI * MAIN_LOOP_PERIOD;
        float delta_gyro_odom = delta_gyro - delta_theta;
        float prev_theta = current_odom.theta;
        if ((delta_gyro_odom > 0.01) || (delta_gyro_odom < 0.01)) {
            current_odom.theta = current_odom.theta + delta_gyro;
            delta_theta = delta_gyro;
        } else {
            current_odom.theta = current_odom.theta + delta_theta;
        }
        current_odom.utime = current_pico_time;
        current_odom.x = current_odom.x + delta_d * (cos(clamp_orientation(prev_theta + (delta_theta/2))));
        current_odom.y = current_odom.y + delta_d * (sin(clamp_orientation(prev_theta + (delta_theta/2))));
        current_odom.theta = clamp_orientation(current_odom.theta);
        
        /*************************************************************
         * End of TODO
         *************************************************************/

        // get the current motor command state (if we have one)
        if (comms_get_topic_data(MBOT_MOTOR_COMMAND, &current_cmd))
        {
            int16_t l_cmd, r_cmd;                 // left and right motor commands
            float left_sp, right_sp;              // speed in m/s
            float measured_vel_l, measured_vel_r; // measured velocity in m/s
            float l_duty, r_duty;                 // duty cycle in range [-1, 1]
            float dt = MAIN_LOOP_PERIOD;          // time since last update in seconds
            // printf("got motor command\r\n");
            if (OPEN_LOOP)
            {
                /*************************************************************
                 * TODO:
                 *      - Implement the open loop motor controller to compute the left
                 *          and right wheel commands
                 *      - Determine the setpoint velocities for left and right motor using the wheel velocity model
                 *      - To compute the measured velocities, use dt as the timestep (∆t)
                 ************************************************************/
                left_sp = current_cmd.trans_v - current_cmd.angular_v * (WHEEL_BASE / 2);
                left_sp = (((left_sp / WHEEL_RADIUS) / (2 * 3.14159)) * 60);
                right_sp = current_cmd.trans_v + current_cmd.angular_v * (WHEEL_BASE / 2);
                right_sp = (((right_sp / WHEEL_RADIUS) / (2 * 3.14159)) * 60);
                // if (current_cmd.trans_v != 0 || current_cmd.angular_v != 0)
                // {
                //     printf("current command trans %f, trun %f\n", current_cmd.trans_v, current_cmd.angular_v);
                // }
                // printf("Command received: %f, %f\n", current_cmd.trans_v, current_cmd.angular_v);
                // if (current_cmd.angular_v != 0){
                //     printf("left_sp: %f, right_sp: %f\n", left_sp, right_sp);
                // }
                
                // printf("motor_3_recover.motor_front.slope: %f", motor_3_recover.motor_front.slope);
                // printf("motor_3_recover.motor_front.intercept: %f", motor_3_recover.motor_front.intercept);
                // printf("motor_1_recover.motor_back.slope: %f", motor_1_recover.motor_back.slope);
                // printf("motor_1_recover.motor_front.intercept: %f", motor_1_recover.motor_front.intercept);
                // Use the calibration function to calculate the l_duty and r_duty when ready
                // left motor is 3, right motor is 1
                // left_sp *= LEFT_ENC_POL;
                // right_sp *= RIGHT_ENC_POL;
                if (left_sp < 0)
                {
                    l_duty = left_sp * LEFT_ENC_POL * motor_1_recover.motor_back.slope + motor_1_recover.motor_back.intercept;
                    // printf("l_duty: %f\n", l_duty);
                    // l_duty *= LEFT_MOTOR_POL;
                }
                else if (left_sp > 0)
                {
                    l_duty = left_sp * LEFT_ENC_POL * motor_1_recover.motor_front.slope + motor_1_recover.motor_front.intercept;
                    // l_duty = -0.6;
                    // printf("l_duty: %f\n", l_duty);
                    // l_duty *= LEFT_MOTOR_POL;
                }
                else
                {
                    l_duty = 0;
                }
                if (right_sp < 0)
                {
                    r_duty = right_sp * RIGHT_ENC_POL * motor_3_recover.motor_back.slope + motor_3_recover.motor_back.intercept;
                    // printf("r_duty: %f\n", r_duty);
                    // r_duty *= RIGHT_MOTOR_POL;
                }
                else if (right_sp > 0)
                {
                    r_duty = right_sp * RIGHT_ENC_POL * motor_3_recover.motor_front.slope + motor_3_recover.motor_front.intercept;
                    // r_duty = 0.6;
                    // printf("r_duty: %f\n", r_duty);
                    // r_duty *= RIGHT_MOTOR_POL;
                }
                else
                {
                    r_duty = 0;
                }
                
                /*************************************************************
                 * End of TODO
                 *************************************************************/
            }
            else
            {
                /*************************************************************
                 * TODO:
                 *      - Implement the closed loop motor controller to compute the left
                 *          and right wheel commands
                 *      - To calculate the measured velocity, use MAIN_LOOP_PERIOD or latency_time
                 *          as the timestep
                 *      - Compute the error between the setpoint velocity and the measured velocity
                 *      - We recommend to use the open loop controller as part of the closed loop to improve
                 *          performance.
                 *          Example: open_loop_control(LEFT_MOTOR_CHANNEL, left_sp)
                 *      - Use the PID filters defined in mbot.h and main() function to calculate desired
                 *          duty, use rc_filter_march() function.
                 *          Example: rc_filter_march(&left_pid, left_error)
                 * TODO:
                 *      - Compute the error between the target and measured translation+rotation velocity
                 *      - Compute the new forward and rotation setpoints that will be used in
                 *          the wheel speed PID (these should be written in lines above
                 *          the previous controller)
                 *      - Update and the fwd_sp and turn_sp variables for this.
                 *
                 ************************************************************/

                // if (current_cmd.trans_v == 0 && current_cmd.angular_v == 0)
                // {
                //     // printf("left encoder reading is %f, right encoder reading is %f\n", left_encoder_reading, right_encoder_reading);
                //     // printf("measured_vel_l: %f, measured_vel_r: %f\n", measured_vel_l, measured_vel_r);
                //     // // printf("measured_vel_l_rpm: %f, measured_vel_r_rpm: %f\n", measured_vel_l_rpm, measured_vel_r_rpm);
                //     // printf("fwd_error: %f, turn_error: %f\n", fwd_error, turn_error);
                //     // printf("new_fwd_vel: %f, new_turn_vel: %f", new_fwd_vel, new_turn_vel);
                //     // printf("left_error: %f, right_error: %f\n", left_error, right_error);
                //     // // printf("left_sp: %f, right_sp: %f\n", left_sp, right_sp);
                //     // printf("----------------------------------------------\n");
                //     l_duty = 0.0;
                //     r_duty = 0.0;
                // }
                // else{
                float fwd_sp, turn_sp;                     // forward and turn setpoints in m/s and rad/s
                float measured_vel_fwd, measured_vel_turn; // measured forward and turn velocities in m/s and rad/s

                // Compute current measured wheel speed
                float left_encoder_reading, right_encoder_reading, measured_vel_l_rpm, measured_vel_r_rpm;
                left_encoder_reading = -current_encoders.left_delta;
                right_encoder_reading = -current_encoders.right_delta; //our polarity was defined with -delta

                // Encoder reading go through the LPF
                left_encoder_reading = rc_filter_march(&low_pass_filter_mea_fwd_vel, left_encoder_reading);
                right_encoder_reading = rc_filter_march(&low_pass_filter_mea_turn_vel, right_encoder_reading);

                measured_vel_l = enc2meters * left_encoder_reading / dt;
                measured_vel_l_rpm = RPM_conversion_factor * left_encoder_reading /dt;
                measured_vel_r = enc2meters * right_encoder_reading / dt;
                measured_vel_r_rpm = RPM_conversion_factor * right_encoder_reading / dt;
                
                // Compute current measured body speed
                measured_vel_fwd = (measured_vel_l + measured_vel_r) / 2; //m/s
                measured_vel_turn = ((measured_vel_r - measured_vel_l) / 2) / (WHEEL_BASE / 2); //rad/s

                // Go through the LPF
                // measured_vel_fwd = rc_filter_march(&low_pass_filter_mea_fwd_vel, measured_vel_fwd);
                // measured_vel_turn = rc_filter_march(&low_pass_filter_mea_turn_vel, measured_vel_turn);

                // Compute the error between the measured vel and the targeted vel
                float fwd_error, turn_error;
                fwd_sp = current_cmd.trans_v; //m/s
                turn_sp = current_cmd.angular_v; //rad/s
                fwd_error = fwd_sp - measured_vel_fwd; //m/s
                turn_error = turn_sp - measured_vel_turn; //rad/s

                // fwd and turn PID
                float new_fwd_vel, new_turn_vel;
                // printf("forward error: %f\n", fwd_error);
                new_fwd_vel = rc_filter_march(&fwd_vel_pid, fwd_error); //m/s
                new_turn_vel = rc_filter_march(&turn_vel_pid, turn_error); //rad/s
                
                // Go through the LPF
                // new_fwd_vel = rc_filter_march(&low_pass_filter_new_fwd_vel, new_fwd_vel);
                // new_turn_vel = rc_filter_march(&low_pass_filter_new_turn_vel, new_turn_vel);

                // Calculate left and right wheel speed
                // left_sp = current_cmd.trans_v - current_cmd.angular_v * (WHEEL_BASE / 2); //rpm
                left_sp = new_fwd_vel - new_turn_vel * (WHEEL_BASE / 2); //rpm
                left_sp = (((left_sp / WHEEL_RADIUS) / (2 * 3.14159)) * 60);
                // right_sp = current_cmd.trans_v + current_cmd.angular_v * (WHEEL_BASE / 2); //rpm
                right_sp = new_fwd_vel + new_turn_vel * (WHEEL_BASE / 2); //rpm
                right_sp = (((right_sp / WHEEL_RADIUS) / (2 * 3.14159)) * 60);

                
                
                // Compute the calibration cmd
                float calibration_duty_l, calibration_duty_r;
                // printf("right sp: %f, left sp: %f \n", right_sp, left_sp);
                calibration_duty_l = open_loop_control(LEFT_MOTOR_CHANNEL, left_sp);
                calibration_duty_r = open_loop_control(RIGHT_MOTOR_CHANNEL, right_sp);

                // Compute the error between new wheel sp and measured wheel sp
                float left_error, right_error;
                left_error = left_sp - measured_vel_l_rpm;
                right_error = right_sp - measured_vel_r_rpm;

                if (abs(left_sp) < CAP_RPM && abs(right_sp) < CAP_RPM){//TODO: how to make sure the robot stop
                    l_duty = 0;
                    r_duty = 0;
                    left_sp = 0;
                    right_sp = 0;
                    left_error = 0;
                    right_error = 0;
                }
                
                // print statements
                // if (current_cmd.trans_v ==0 || current_cmd.angular_v == 0)
                // {
                //     // printf("left encoder reading is %f, right encoder reading is %f\n", left_encoder_reading, right_encoder_reading);
                //     // printf("measured_vel_l: %f, measured_vel_r: %f\n", measured_vel_l, measured_vel_r);
                //     // // printf("measured_vel_l_rpm: %f, measured_vel_r_rpm: %f\n", measured_vel_l_rpm, measured_vel_r_rpm);
                //     // printf("fwd_error: %f, turn_error: %f\n", fwd_error, turn_error);
                //     // printf("new_fwd_vel: %f, new_turn_vel: %f", new_fwd_vel, new_turn_vel);
                //     // printf("left_error: %f, right_error: %f\n", left_error, right_error);
                //     // // printf("left_sp: %f, right_sp: %f\n", left_sp, right_sp);
                //     // printf("----------------------------------------------\n");
                // }
                    
                

                // Compute the wheel cmd
                float left_pid_duty, right_pid_duty;
                left_pid_duty = rc_filter_march(&left_pid, left_error);
                right_pid_duty = rc_filter_march(&right_pid, right_error);

                // printf("left_pid_duty: %f, right_pid_duty: %f\n", left_pid_duty, right_pid_duty);
                l_duty = left_pid_duty * LEFT_MOTOR_POL + calibration_duty_l;
                r_duty = right_pid_duty * RIGHT_MOTOR_POL + calibration_duty_r;
                
                if (fabs(left_sp) < 3 && fabs(right_sp) < 3){//TODO: check this value!!! Jingyu
                    l_duty = 0;
                    r_duty = 0;
                }

                //TODO: check this option
                // if (current_cmd.trans_v == 0.0 && current_cmd.angular_v == 0.0)
                // {
                //     l_duty = left_pid_duty * LEFT_MOTOR_POL;
                //     r_duty = right_pid_duty * RIGHT_MOTOR_POL;
                // }
                // else
                // {
                    // l_duty = left_pid_duty * LEFT_MOTOR_POL + calibration_duty_l;
                    // r_duty = right_pid_duty * RIGHT_MOTOR_POL + calibration_duty_r;
                // }


                /**
                 *  Example closed loop controller
                 *      (assuming the error between the target and measured is computed)
                 *
                 * float pid_delta_vel = rc_filter_march(&pid_filter, error);
                 * float desired_vel = commanded_val + pid_delta_vel;
                 */

                /*************************************************************
                 * End of TODO
                 *************************************************************/
            // }
            }
            // Clamp duty cycle to [-1, 1]

            // if (fabs(current_cmd.trans_v) < 0.01 && fabs(current_cmd.angular_v) < 0.01)
            //     {
            //         // printf("left encoder reading is %f, right encoder reading is %f\n", left_encoder_reading, right_encoder_reading);
            //         // printf("measured_vel_l: %f, measured_vel_r: %f\n", measured_vel_l, measured_vel_r);
            //         // // printf("measured_vel_l_rpm: %f, measured_vel_r_rpm: %f\n", measured_vel_l_rpm, measured_vel_r_rpm);
            //         // printf("fwd_error: %f, turn_error: %f\n", fwd_error, turn_error);
            //         // printf("new_fwd_vel: %f, new_turn_vel: %f", new_fwd_vel, new_turn_vel);
            //         // printf("left_error: %f, right_error: %f\n", left_error, right_error);
            //         // // printf("left_sp: %f, right_sp: %f\n", left_sp, right_sp);
            //         // printf("----------------------------------------------\n");
            //         l_duty = 0.0;
            //         r_duty = 0.0;
            //     }
            
            l_duty = clamp_duty(l_duty);
            r_duty = clamp_duty(r_duty);

            // duty to motor command
            // l_cmd = LEFT_MOTOR_POL * (int)(l_duty * 0.95 * pow(2, 15));
            // r_cmd = RIGHT_MOTOR_POL * (int)(r_duty * 0.95 * pow(2, 15));
            l_cmd = (int)(l_duty * 0.95 * pow(2, 15));
            r_cmd = (int)(r_duty * 0.95 * pow(2, 15));

            // set left and right motor command
            rc_motor_set(LEFT_MOTOR_CHANNEL, l_cmd);
            rc_motor_set(RIGHT_MOTOR_CHANNEL, r_cmd);
        }

        // write the encoders to serial
        comms_write_topic(MBOT_ENCODERS, &current_encoders);
        // send odom on wire
        comms_write_topic(ODOMETRY, &current_odom);
        // write the IMU to serial
        comms_write_topic(MBOT_IMU, &current_imu);
        uint64_t fn_run_len = to_us_since_boot(get_absolute_time()) + timestamp_offset - cur_pico_time;
    }

    return true;
}


int main()
{
    bi_decl(bi_program_description("Firmware for the MBot Robot Control Board"));
    
    set_sys_clock_khz(250000, true); // set master clock to 250MHz (if problematic try 125Mhz)
    stdio_init_all(); // enable USB serial terminal
    sleep_ms(1500); // quick sleep so we can catch the bootup process in terminal
    printf("\nMBot Booting Up!\n");

    printf("initializinging motors...\n");
    rc_motor_init();
    printf("initializinging encoders...\n");
    rc_encoder_init();

    // Pins
    // for the i2c to the IMU
    const uint sda_pin = 4;
    const uint scl_pin = 5;

    // Ports
    i2c = i2c0;
    // Initialize I2C port at 400 kHz
    i2c_init(i2c, 400 * 1000);
    // Initialize I2C pins
    printf("setting i2c functions...\n");
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    printf("setting i2c pullups...\n");
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(sda_pin, scl_pin, GPIO_FUNC_I2C));

    printf("setting heartbeat LED GPIOs...\n");
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    if(rc_initialize_fram(i2c)){
         printf("ERROR: FRAM chip failed to initialize\n");
         return -1;
    }

    uint8_t pid_bytes_1_recover[MOTOR_PARAMS_SIZE], pid_bytes_3_recover[MOTOR_PARAMS_SIZE];
    rc_read_fram(i2c, MOTOR1_CALIB_PARAM_ADD, MOTOR_PARAMS_SIZE, &pid_bytes_1_recover[0]);
    memcpy(&motor_1_recover, pid_bytes_1_recover, MOTOR_PARAMS_SIZE);

    rc_read_fram(i2c, MOTOR3_CALIB_PARAM_ADD, MOTOR_PARAMS_SIZE, &pid_bytes_3_recover[0]);
    memcpy(&motor_3_recover, pid_bytes_3_recover, MOTOR_PARAMS_SIZE);


    // printf("motor_1_recover.motor_back.slope: %f\n", motor_1_recover.motor_back.slope);
    // printf("motor_1_recover.motor_front.intercept: %f\n", motor_1_recover.motor_front.intercept);

    // Initialize the IMU using the Digital Motion Processor
    printf("initializing DMP...\n");
    rc_mpu_config_t mpu_config = rc_mpu_default_config();
    mpu_config.i2c_bus = i2c;
    mpu_config.dmp_fetch_accel_gyro = 1;
    mpu_config.enable_magnetometer = 1;
    mpu_config.read_mag_after_callback = 0;
    mpu_config.orient = ORIENTATION_Z_UP;
    mpu_config.dmp_sample_rate = 200;
    
    // Calibrate the gyro to eliminate bias, Mbot must be still for this
    rc_mpu_calibrate_gyro_routine(mpu_config);
    sleep_ms(500);
    rc_mpu_initialize_dmp(&mpu_data, mpu_config);
    gpio_set_irq_enabled_with_callback(rc_MPU_INTERRUPT_GPIO, GPIO_IRQ_EDGE_FALL, true, &rc_dmp_callback);
    printf("MPU Initialized!\n");

    // create topics and register the serialize/deserialize functions
    printf("init comms...\r\n");
    comms_init_protocol();
    comms_init_topic_data();
    register_topics();

    // launch the other core's comm loop
    printf("starting comms on core 1...\r\n");
    multicore_launch_core1(comms_listener_loop);

    // wait for other core to get rolling
    sleep_ms(500);
    int running = 1;

    printf("motor_3_recover.motor_front.slope: %f\n", motor_3_recover.motor_front.slope);
    printf("motor_3_recover.motor_front.intercept: %f\n", motor_3_recover.motor_front.intercept);
    printf("motor_3_recover.motor_back.slope: %f\n", motor_3_recover.motor_back.slope);
    printf("motor_3_recover.motor_back.intercept: %f\n", motor_3_recover.motor_back.intercept);
    printf("motor_1_recover.motor_front.slope: %f\n", motor_1_recover.motor_front.slope);
    printf("motor_1_recover.motor_front.intercept: %f\n", motor_1_recover.motor_front.intercept);
    printf("motor_1_recover.motor_back.slope: %f\n", motor_1_recover.motor_back.slope);
    printf("motor_1_recover.motor_back.intercept: %f\n", motor_1_recover.motor_back.intercept);

    // run the main loop as a timed interrupt
    printf("starting the timed interrupt...\r\n");
    repeating_timer_t loop_timer;
    add_repeating_timer_ms(MAIN_LOOP_PERIOD * 1000, timer_cb, NULL, &loop_timer); // 1000x to convert to ms

    printf("Done Booting Up!\n\n");



    /*************************************************************
     * TODO:
     *  - wheel speed PID
     *      - Initilize the PID Filters rc_filter_empty()
     *      - Set the PID gains rc_filter_pid()
     * TODO:
     *  - body frame velocity PID
     *      - Initialize the PID filters for translation and rotation vel
     *      - Set the PID gains for translation and rotation vel
     *
     *************************************************************/

    // Example initialization of a PID filter defined in mbot.h
    // my_filter = rc_filter_empty();

    left_pid = rc_filter_empty();
    right_pid = rc_filter_empty();
    fwd_vel_pid = rc_filter_empty();
    turn_vel_pid = rc_filter_empty();

    // Example of assigning PID parameters (using pid_parameters_t from mbot.h)
    // rc_filter_pid(&my_filter,
    //             pid_params.kp,
    //             pid_params.ki,
    //             pid_params.kd,
    //             1.0 / pid_params.dFilterHz,
    //             1.0 / MAIN_LOOP_HZ);

    rc_filter_pid(&left_pid,
                left_pid_params.kp,
                left_pid_params.ki,
                left_pid_params.kd,
                1.0 / left_pid_params.dFilterHz,
                1.0 / MAIN_LOOP_HZ);
    rc_filter_pid(&right_pid,
                right_pid_params.kp,
                right_pid_params.ki,
                right_pid_params.kd,
                1.0 / right_pid_params.dFilterHz,
                1.0 / MAIN_LOOP_HZ);   
    rc_filter_pid(&fwd_vel_pid,
                fwd_vel_pid_params.kp,
                fwd_vel_pid_params.ki,
                fwd_vel_pid_params.kd,
                1.0 / fwd_vel_pid_params.dFilterHz,
                1.0 / MAIN_LOOP_HZ); 
    rc_filter_pid(&turn_vel_pid,
                turn_vel_pid_params.kp,
                turn_vel_pid_params.ki,
                turn_vel_pid_params.kd,
                1.0 / turn_vel_pid_params.dFilterHz,
                1.0 / MAIN_LOOP_HZ);   

    // Example of setting limits to the output of the filter
    // rc_filter_enable_saturation(&my_filter, min_val, max_val);

    rc_filter_enable_saturation(&fwd_vel_pid, -MAX_FWD_VEL, MAX_FWD_VEL);
    rc_filter_enable_saturation(&turn_vel_pid, -MAX_TURN_VEL, MAX_TURN_VEL);

    // Initialize the LPF
    low_pass_filter_new_fwd_vel = rc_filter_empty();
    low_pass_filter_new_turn_vel = rc_filter_empty();
    rc_filter_first_order_lowpass(&low_pass_filter_new_fwd_vel, 2 * MAIN_LOOP_PERIOD, TIME_CONSTANT);
    rc_filter_first_order_lowpass(&low_pass_filter_new_turn_vel, 2 * MAIN_LOOP_PERIOD, TIME_CONSTANT);

    low_pass_filter_mea_fwd_vel = rc_filter_empty();
    low_pass_filter_mea_turn_vel = rc_filter_empty();
    rc_filter_first_order_lowpass(&low_pass_filter_mea_fwd_vel, 2 * MAIN_LOOP_PERIOD, TIME_CONSTANT);
    rc_filter_first_order_lowpass(&low_pass_filter_mea_turn_vel, 2 * MAIN_LOOP_PERIOD, TIME_CONSTANT);

    /*************************************************************
     * End of TODO
     *************************************************************/

    if (OPEN_LOOP)
    {
        printf("Running in open loop mode\n");
    }
    else
    {
        printf("Running in closed loop mode\n");
    }

    while (running)
    {
        // printf("\033[2A\r|      SENSORS      |           ODOMETRY          |     SETPOINTS     |\n\r|  L_ENC  |  R_ENC  |    X    |    Y    |    θ    |   FWD   |   ANG   \n\r|%7lld  |%7lld  |%7.3f  |%7.3f  |%7.3f  |%7.3f  |%7.3f  |", current_encoders.leftticks, current_encoders.rightticks, current_odom.x, current_odom.y, current_odom.theta, current_cmd.trans_v, current_cmd.angular_v);
    }
}

/**
 * @brief Clamp duty cycle between -1 and 1. If not applied, robot drives in reverse only
 *
 * @param duty
 */
float clamp_duty(float duty)
{
    if (duty > 1.0)
    {
        return 1.0;
    }
    else if (duty < -1.0)
    {
        return -1.0;
    }
    return duty;
}

/**
 * @brief Clamp angles between (0, 2*pi]
 *
 * @param angle
 */
float clamp_orientation(float angle) {
    while (angle <   0) angle += 2*M_PI;
    while (angle >= (2*M_PI)) angle -= 2*M_PI;
    return angle;
}
