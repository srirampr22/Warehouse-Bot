/**
 * This file is the main executable for the MBot firmware.
 */
#include <pico/stdlib.h>
#include <stdio.h>
#include <hardware/gpio.h>
#include "mbot.h"
#include "odometry.h"
#include <mbot/utils/utils.h>
#include "print_tables.h"
#include <mbot/defs/mbot_params.h>

//servo setup

#include <stdint.h>

#include "hardware/pio.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include <mbot/servo/servo.h>
#include <mbot/motor/motor.h>
//

#define THETA "\u0398"
#pragma pack(1)

#ifndef MBOT_DRIVE_TYPE
#error "Please define a drive type for the bot"
#endif

// Global
static uint64_t timestamp_offset = 0;
static uint64_t global_utime = 0;
static uint64_t global_pico_time = 0;
static bool global_comms_status = COMMS_ERROR; 
static int drive_mode = 0;
static int gyro_bias = -0.0002830835317231931;
static bool running = false;
static float previous_servo = 0.0;
static mbot_params_t params;
mbot_ctlr_cfg_t pid_gains;

static int servo_freq = 100;
static int servo_chan = 2;

// static float last_filtered_yaw = 0.0;
// static float last_gyro_rate = 0.0;
// static float last_yaw_angle = 0.0;
// #define ALPHA 0.1 // filter constant between 0 and 1

mbot_bhy_data_t mbot_imu_data;
mbot_bhy_config_t mbot_imu_config;



void print_mbot_params(const mbot_params_t* params) {
    printf("Robot Type: %d\n", params->robot_type);
    printf("Wheel Radius: %f\n", params->wheel_radius);
    printf("Wheel Base Radius: %f\n", params->wheel_base_radius);
    printf("Gear Ratio: %f\n", params->gear_ratio);
    printf("Encoder Resolution: %f\n", params->encoder_resolution);
    printf("Motor Left: %d\n", params->mot_left);
    printf("Motor Right: %d\n", params->mot_right);
    printf("Motor Back: %d\n", params->mot_back);
    printf("Motor Polarity: %d %d %d\n", params->motor_polarity[0], params->motor_polarity[1], params->motor_polarity[2]);
    printf("Encoder Polarity: %d %d %d\n", params->encoder_polarity[0], params->encoder_polarity[1], params->encoder_polarity[2]);
    printf("Positive Slope: %f %f %f\n", params->slope_pos[0], params->slope_pos[1], params->slope_pos[2]);
    printf("Positive Intercept: %f %f %f\n", params->itrcpt_pos[0], params->itrcpt_pos[1], params->itrcpt_pos[2]);
    printf("Negative Slope: %f %f %f\n", params->slope_neg[0], params->slope_neg[1], params->slope_neg[2]);
    printf("Negative Intercept: %f %f %f\n", params->itrcpt_neg[0], params->itrcpt_neg[1], params->itrcpt_neg[2]);
}

void register_topics()
{
    // Subscriptions
    comms_register_topic(MBOT_TIMESYNC, sizeof(serial_timestamp_t), (Deserialize)&timestamp_t_deserialize, (Serialize)&timestamp_t_serialize, (MsgCb)&timestamp_cb);
    comms_register_topic(MBOT_ODOMETRY_RESET,  sizeof(serial_pose2D_t), (Deserialize)&pose2D_t_deserialize, (Serialize)&pose2D_t_serialize, (MsgCb)&reset_odometry_cb);
    comms_register_topic(MBOT_ENCODERS_RESET, sizeof(serial_mbot_encoders_t), (Deserialize)&mbot_encoders_t_deserialize, (Serialize)&mbot_encoders_t_serialize, (MsgCb)&reset_encoders_cb);
    comms_register_topic(MBOT_MOTOR_PWM_CMD, sizeof(serial_mbot_motor_pwm_t), (Deserialize)&mbot_motor_pwm_t_deserialize, (Serialize)&mbot_motor_pwm_t_serialize, (MsgCb)mbot_motor_pwm_cmd_cb);
    comms_register_topic(MBOT_MOTOR_VEL_CMD, sizeof(serial_mbot_motor_vel_t), (Deserialize)&mbot_motor_vel_t_deserialize, (Serialize)&mbot_motor_vel_t_serialize, (MsgCb)mbot_motor_vel_cmd_cb);
    comms_register_topic(MBOT_VEL_CMD, sizeof(serial_twist2D_t), (Deserialize)&twist2D_t_deserialize, (Serialize)&twist2D_t_serialize, (MsgCb)mbot_vel_cmd_cb);
    
    //servo setup
    comms_register_topic(MBOT_SERVO_PWM_CMD, sizeof(serial_mbot_servo_pwm_t), (Deserialize)&mbot_servo_pwm_t_deserialize, (Serialize)&mbot_servo_pwm_t_serialize, (MsgCb)mbot_servo_pwm_cmd_cb);


    // Published Topics
    comms_register_topic(MBOT_ODOMETRY, sizeof(serial_pose2D_t), (Deserialize)&pose2D_t_deserialize, (Serialize)&pose2D_t_serialize, NULL);
    comms_register_topic(MBOT_IMU, sizeof(serial_mbot_imu_t), (Deserialize)&mbot_imu_t_deserialize, (Serialize)&mbot_imu_t_serialize, NULL);
    comms_register_topic(MBOT_ENCODERS, sizeof(serial_mbot_encoders_t), (Deserialize)&mbot_encoders_t_deserialize, (Serialize)&mbot_encoders_t_serialize, NULL);
    comms_register_topic(MBOT_VEL, sizeof(serial_twist2D_t), (Deserialize)&twist2D_t_deserialize, (Serialize)&twist2D_t_serialize, NULL);
    comms_register_topic(MBOT_MOTOR_VEL, sizeof(serial_mbot_motor_vel_t), (Deserialize)&mbot_motor_vel_t_deserialize, (Serialize)&mbot_motor_vel_t_serialize, NULL);
    comms_register_topic(MBOT_MOTOR_PWM, sizeof(serial_mbot_motor_pwm_t), (Deserialize)&mbot_motor_pwm_t_deserialize, (Serialize)&mbot_motor_pwm_t_serialize, NULL);

    //servo setup
    comms_register_topic(MBOT_SERVO_PWM, sizeof(serial_mbot_servo_pwm_t), (Deserialize)&mbot_servo_pwm_t_deserialize, (Serialize)&mbot_servo_pwm_t_serialize, NULL);
    //
}

void timestamp_cb(serial_timestamp_t *msg)
{
    global_pico_time = to_us_since_boot(get_absolute_time());
    timestamp_offset = msg->utime - global_pico_time;
    global_comms_status = COMMS_OK;
}

void reset_encoders_cb(serial_mbot_encoders_t *msg)
{
    //memcpy(&encoders, msg, sizeof(serial_mbot_encoders_t));
    for(int i=0; i<3; i++){
        mbot_encoder_write(i, msg->ticks[i]);
    }
}

void reset_odometry_cb(serial_pose2D_t *msg)
{
    mbot_odometry.x = msg->x;
    mbot_odometry.y = msg->y;
    mbot_odometry.theta = msg->theta;
}

void mbot_vel_cmd_cb(serial_twist2D_t *msg)
{
    memcpy(&mbot_vel_cmd, msg, sizeof(serial_twist2D_t));
    drive_mode = MODE_MBOT_VEL;
}

void mbot_motor_vel_cmd_cb(serial_mbot_motor_vel_t *msg)
{
    memcpy(&mbot_motor_vel_cmd, msg, sizeof(serial_mbot_motor_vel_t));
    drive_mode = MODE_MOTOR_VEL_PID;
}

void mbot_motor_pwm_cmd_cb(serial_mbot_motor_pwm_t *msg)
{
    memcpy(&mbot_motor_pwm_cmd, msg, sizeof(serial_mbot_motor_pwm_t));
    drive_mode = MODE_MOTOR_PWM;
}

void mbot_servo_pwm_cmd_cb(serial_mbot_servo_pwm_t *msg)
{
    memcpy(&mbot_servo_pwm_cmd, msg, sizeof(serial_mbot_servo_pwm_t));
}

void mbot_calculate_motor_vel(serial_mbot_encoders_t encoders, serial_mbot_motor_vel_t *motor_vel){
    float conversion = (1.0 / params.gear_ratio) * (1.0 / params.encoder_resolution) * 1E6f * 2.0 * M_PI;
    motor_vel->velocity[params.mot_left] = params.encoder_polarity[params.mot_left] * (conversion / encoders.delta_time) * encoders.delta_ticks[params.mot_left];
    if(MBOT_DRIVE_TYPE == OMNI_120_DRIVE){
        motor_vel->velocity[params.mot_back] = params.encoder_polarity[params.mot_back] * (conversion / encoders.delta_time) * encoders.delta_ticks[params.mot_back];
    }
    motor_vel->velocity[params.mot_right] = params.encoder_polarity[params.mot_right] * (conversion / encoders.delta_time) * encoders.delta_ticks[params.mot_right];
}

void mbot_read_imu(serial_mbot_imu_t *imu){
    imu->utime = global_utime;
    imu->gyro[0] = mbot_imu_data.gyro[0];
    imu->gyro[1] = mbot_imu_data.gyro[1];
    imu->gyro[2] = mbot_imu_data.gyro[2];
    imu->accel[0] = mbot_imu_data.accel[0];
    imu->accel[1] = mbot_imu_data.accel[1];
    imu->accel[2] = mbot_imu_data.accel[2];
    imu->mag[0] = mbot_imu_data.mag[0];
    imu->mag[1] = mbot_imu_data.mag[1];
    imu->mag[2] = mbot_imu_data.mag[2];
    imu->angles_rpy[0] = mbot_imu_data.rpy[0];
    imu->angles_rpy[1] = mbot_imu_data.rpy[1];
    imu->angles_rpy[2] = mbot_imu_data.rpy[2];
    imu->angles_quat[0] = mbot_imu_data.quat[0];
    imu->angles_quat[1] = mbot_imu_data.quat[1];
    imu->angles_quat[2] = mbot_imu_data.quat[2];
    imu->angles_quat[3] = mbot_imu_data.quat[3];   
}

void mbot_read_encoders(serial_mbot_encoders_t* encoders){
    int64_t delta_time = global_utime - encoders->utime;
    encoders->utime = global_utime;
    encoders->delta_time = delta_time;

    encoders->ticks[params.mot_right] = mbot_encoder_read_count(params.mot_right);
    encoders->delta_ticks[params.mot_right] = mbot_encoder_read_delta(params.mot_right);
    encoders->ticks[params.mot_left] = mbot_encoder_read_count(params.mot_left);
    encoders->delta_ticks[params.mot_left] = mbot_encoder_read_delta(params.mot_left);

    if(MBOT_DRIVE_TYPE == OMNI_120_DRIVE){
        encoders->ticks[params.mot_back] = mbot_encoder_read_count(params.mot_back);
        encoders->delta_ticks[params.mot_back] = mbot_encoder_read_delta(params.mot_back);
    }
}

// float update_yaw_with_complementary_filter(float gyro_rate, float magnetometer_yaw) {
//     // Integrate the gyroscope rate over time to get a change in angle
//     float gyro_angle_change = last_gyro_rate * MAIN_LOOP_PERIOD;
    
//     // Predict the new yaw angle based on the change from the gyroscope
//     float predicted_yaw = last_yaw_angle + gyro_angle_change;

//     // Fuse the predicted yaw with the magnetometer measurement
//     float fused_yaw = ALPHA * predicted_yaw + (1 - ALPHA) * magnetometer_yaw;

//     // Update the last state
//     last_yaw_angle = fused_yaw;
//     last_gyro_rate = gyro_rate;

//     return fused_yaw;
// }

int mbot_init_pico(void){
    bi_decl(bi_program_description("Firmware for the MBot Robot Control Board"));
    
    // set master clock to 250MHz (if unstable set SYS_CLOCK to 125Mhz)
    if(!set_sys_clock_khz(125000, true)){
        printf("ERROR mbot_init_pico: cannot set system clock\n");
        return MBOT_ERROR;
    }; 
    
    stdio_init_all(); // enable USB serial terminal
    sleep_ms(500);
    printf("\nMBot Booting Up!\n");
    return MBOT_OK;
}

int mbot_init_hardware(void){
    sleep_ms(1000);
    // Initialize Motors
    printf("initializinging motors...\n");
    mbot_motor_init(DIFF_MOTOR_LEFT_SLOT);
    if(MBOT_DRIVE_TYPE == OMNI_120_DRIVE){
        mbot_motor_init(1);
    }
    mbot_motor_init(DIFF_MOTOR_RIGHT_SLOT);
    printf("initializinging encoders...\n");
    mbot_encoder_init();

    // Initialize LED
    printf("Starting heartbeat LED...\n");
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    mbot_imu_config = mbot_imu_default_config();
    mbot_imu_config.sample_rate = 200;
    // Initialize the IMU using the Digital Motion Processor
    printf("Initializing IMU...\n");
    mbot_imu_init(&mbot_imu_data, mbot_imu_config);
    mbot_init_fram();
    return MBOT_OK;
}

int mbot_init_comms(void){
    printf("Initializing LCM serial communication...\r\n");
    comms_init_protocol();
    comms_init_topic_data();
    register_topics();

    // launch the other core's comm loop
    printf("starting comms on core 1...\r\n");
    multicore_launch_core1(comms_listener_loop);

    // wait for other core to get rolling
    sleep_ms(500);
    return MBOT_OK;
}

void mbot_init_pid(mbot_ctlr_cfg_t* pid_gains, const mbot_params_t* params)
{
    // Initialize PID gains

    // Right
    pid_gains->right.kp = 0.11;
    pid_gains->right.ki = 0.0;
    pid_gains->right.kd = 0.0001;
    pid_gains->right.Tf = 0.05;

    // Left
    pid_gains->left.kp = 0.11;
    pid_gains->left.ki = 0.0;
    pid_gains->left.kd = 0.0001;
    pid_gains->left.Tf = 0.05;

    // Back
    pid_gains->back.kp = 1.0;
    pid_gains->back.ki = 0.0;
    pid_gains->back.kd = 0.0;
    pid_gains->back.Tf = 1.0;

    // VX
    pid_gains->vx.kp = 1.0;
    pid_gains->vx.ki = 0.0;
    pid_gains->vx.kd = 0.0;
    pid_gains->vx.Tf = 1.0;

    // VY
    pid_gains->vy.kp = 0.0;
    pid_gains->vy.ki = 0.0;
    pid_gains->vy.kd = 0.0;
    pid_gains->vy.Tf = 1.0;

    // WZ
    pid_gains->wz.kp = 0.5;
    pid_gains->wz.ki = 0.05;
    pid_gains->wz.kd = 0.1;
    pid_gains->wz.Tf = 0.05;

    //GYRO
    pid_gains->gyro.dt = MAIN_LOOP_PERIOD;
    pid_gains->gyro.time_con = MAIN_LOOP_PERIOD*500;

    mbot_init_ctlr(*pid_gains, params);
}

void mbot_print_state(serial_mbot_imu_t imu, serial_mbot_encoders_t encoders, serial_pose2D_t odometry, serial_mbot_motor_vel_t motor_vel, serial_mbot_servo_pwm_t servo_pwm){
    printf("\033[2J\r");
    if(global_comms_status == COMMS_OK){
        printf("| \033[32m COMMS OK \033[0m TIME: %lld |\n", global_utime);
    }
    else{
        printf("| \033[31m SERIAL COMMUNICATION FAILURE\033[0m     |\n");
    }
    const char* imu_headings[] = {"ROLL", "PITCH", "YAW"};
    const char* enc_headings[] = {"ENC 0", "ENC 1", "ENC 2"};
    const char* odom_headings[] = {"X", "Y", "THETA"};
    const char* motor_vel_headings[] = {"MOT 0", "MOT 1", "MOT 2"};
    // we shouldn't need to do this, need to update generateTable to handle different datatypes
    int encs[3] = {(int)encoders.ticks[0], (int)encoders.ticks[1], (int)encoders.ticks[2]};
    char buf[1024] = {0};
    generateTableInt(buf, 1, 3, "ENCODERS", enc_headings, encs);
    printf("\r%s", buf);
    
    buf[0] = '\0';
    generateTableFloat(buf, 1, 3, "IMU", imu_headings, imu.angles_rpy);
    printf("\r%s", buf);
    
    buf[0] = '\0';
    generateTableFloat(buf, 1, 3, "MOTOR", motor_vel_headings, motor_vel.velocity);
    printf("\r%s", buf);
    
    buf[0] = '\0';
    float odom_array[3] = {odometry.x, odometry.y, odometry.theta};
    generateTableFloat(buf, 1, 3, "ODOMETRY", odom_headings, odom_array);
    printf("\r%s", buf);

    buf[0] = '\0';
    generateBottomLine(buf, 3);
    printf("\r%s\n", buf);

    printf("| \033[32m  SERVO: %.2f \033[30m |\n", servo_pwm.pwm);
}

//Helper function to use slope + intercept from calibration to generate a PWM command.
float _calibrated_pwm_from_vel_cmd(float vel_cmd, int motor_idx){
    if (vel_cmd > 0.0)
    {
        return (vel_cmd * params.slope_pos[motor_idx]) + params.itrcpt_pos[motor_idx];
    }
    else if (vel_cmd < 0.0)
    {
        return (vel_cmd * params.slope_neg[motor_idx]) + params.itrcpt_neg[motor_idx];
    }
    return 0.0;
}

void mbot_init_ctlr(mbot_ctlr_cfg_t ctlr_cfg, const mbot_params_t* params){
    
    rc_filter_pid(&left_wheel_pid, ctlr_cfg.left.kp, ctlr_cfg.left.ki, ctlr_cfg.left.kd, ctlr_cfg.left.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&right_wheel_pid, ctlr_cfg.right.kp, ctlr_cfg.right.ki, ctlr_cfg.right.kd, ctlr_cfg.right.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&mbot_vx_pid, ctlr_cfg.vx.kp, ctlr_cfg.vx.ki, ctlr_cfg.vx.kd, ctlr_cfg.vx.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&mbot_vy_pid, ctlr_cfg.vy.kp, ctlr_cfg.vy.ki, ctlr_cfg.vy.kd, ctlr_cfg.vy.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&mbot_wz_pid, ctlr_cfg.wz.kp, ctlr_cfg.wz.ki, ctlr_cfg.wz.kd, ctlr_cfg.wz.Tf, MAIN_LOOP_PERIOD);
    rc_filter_first_order_lowpass(&gyro_low, ctlr_cfg.gyro.dt, ctlr_cfg.gyro.time_con);
    if(params->robot_type == OMNI_120_DRIVE)
    {
        rc_filter_pid(&back_wheel_pid, ctlr_cfg.back.kp, ctlr_cfg.back.ki, ctlr_cfg.back.kd, ctlr_cfg.back.Tf, MAIN_LOOP_PERIOD);
    }
}

void mbot_motor_vel_ctlr(serial_mbot_motor_vel_t vel_cmd, serial_mbot_motor_vel_t vel, const mbot_params_t* params, serial_mbot_motor_pwm_t *mbot_motor_pwm){
    float right_error = vel_cmd.velocity[params->mot_right] - vel.velocity[params->mot_right];
    float left_error = vel_cmd.velocity[params->mot_left] - vel.velocity[params->mot_left];
    float right_cmd = rc_filter_march(&right_wheel_pid ,right_error);
    float left_cmd = rc_filter_march(&left_wheel_pid, left_error);

    mbot_motor_pwm->pwm[params->mot_right] = right_cmd;
    mbot_motor_pwm->pwm[params->mot_left] = left_cmd;
    if(params->robot_type == OMNI_120_DRIVE)
    {
        float back_error = vel_cmd.velocity[params->mot_back] - vel.velocity[params->mot_back];
        float back_cmd = rc_filter_march(&back_wheel_pid, back_error);
        mbot_motor_pwm->pwm[params->mot_back] = left_cmd;
    }
}


// TODO: this could be tied to the IMU interrupt
bool mbot_loop(repeating_timer_t *rt)
{
    global_utime = to_us_since_boot(get_absolute_time()) + timestamp_offset;
    mbot_vel.utime = global_utime;
    mbot_read_encoders(&mbot_encoders);
    mbot_read_imu(&mbot_imu);
    mbot_calculate_motor_vel(mbot_encoders, &mbot_motor_vel);
    mbot_motor_vel.utime = global_utime;
    
    if(MBOT_DRIVE_TYPE == DIFFERENTIAL_DRIVE){
        mbot_calculate_diff_body_vel(   mbot_motor_vel.velocity[params.mot_left], 
                                        mbot_motor_vel.velocity[params.mot_right], 
                                        &mbot_vel
                                    );
    }
    
    else if(MBOT_DRIVE_TYPE == OMNI_120_DRIVE){
        mbot_calculate_omni_body_vel(   mbot_motor_vel.velocity[params.mot_left], 
                                        mbot_motor_vel.velocity[params.mot_right], 
                                        mbot_motor_vel.velocity[params.mot_back], 
                                        &mbot_vel
                                    );
    }

    //
    if (abs(mbot_vel.wz) != 0.0) {          
        float gyro_reading_current = mbot_imu.gyro[2];
        mbot_vel.wz = gyro_reading_current;
    }
    // SR
    
    mbot_calculate_odometry(mbot_vel, MAIN_LOOP_PERIOD, &mbot_odometry);
    mbot_odometry.utime = global_utime;
    // only run if we've got 2 way communication...
    if (global_comms_status == COMMS_OK)
    {
        if(drive_mode == MODE_MOTOR_VEL_OL){
            mbot_motor_pwm.utime = global_utime;
            mbot_motor_pwm_cmd.pwm[params.mot_right] = _calibrated_pwm_from_vel_cmd(mbot_motor_vel_cmd.velocity[params.mot_right], params.mot_right);
            mbot_motor_pwm_cmd.pwm[params.mot_left] = _calibrated_pwm_from_vel_cmd(mbot_motor_vel_cmd.velocity[params.mot_left], params.mot_left);
            if(MBOT_DRIVE_TYPE == OMNI_120_DRIVE){
                mbot_motor_pwm_cmd.pwm[params.mot_back] = _calibrated_pwm_from_vel_cmd(mbot_motor_vel_cmd.velocity[params.mot_back], params.mot_back);
            }
        }else if(drive_mode == MODE_MBOT_VEL){
            //TODO: open loop for now - implement closed loop controller
            if(OPEN_LOOP == 1){
                if(MBOT_DRIVE_TYPE == OMNI_120_DRIVE){
                    mbot_motor_vel_cmd.velocity[params.mot_left] = (SQRT3 / 2.0 * mbot_vel_cmd.vx - 0.5 * mbot_vel_cmd.vy - params.wheel_base_radius * mbot_vel_cmd.wz) / params.wheel_radius;
                    mbot_motor_vel_cmd.velocity[params.mot_right] = (-SQRT3 / 2.0 * mbot_vel_cmd.vx - 0.5 * mbot_vel_cmd.vy - params.wheel_base_radius * mbot_vel_cmd.wz) / params.wheel_radius;
                    mbot_motor_vel_cmd.velocity[params.mot_back] = (mbot_vel_cmd.vy - params.wheel_base_radius * mbot_vel_cmd.wz) / params.wheel_radius;
                    float vel_left_comp = params.motor_polarity[params.mot_left] * mbot_motor_vel_cmd.velocity[params.mot_left];
                    float vel_right_comp = params.motor_polarity[params.mot_right] * mbot_motor_vel_cmd.velocity[params.mot_right];
                    float vel_back_comp = params.motor_polarity[params.mot_back] * mbot_motor_vel_cmd.velocity[params.mot_back];
                    
                    mbot_motor_pwm.utime = global_utime;
                    mbot_motor_pwm_cmd.pwm[params.mot_right] = _calibrated_pwm_from_vel_cmd(vel_right_comp, params.mot_right);
                    mbot_motor_pwm_cmd.pwm[params.mot_back] = _calibrated_pwm_from_vel_cmd(vel_back_comp, params.mot_back);
                    mbot_motor_pwm_cmd.pwm[params.mot_left] = _calibrated_pwm_from_vel_cmd(vel_left_comp, params.mot_left);
                }else if(MBOT_DRIVE_TYPE == DIFFERENTIAL_DRIVE){
                    mbot_motor_vel_cmd.velocity[params.mot_left] = (mbot_vel_cmd.vx - DIFF_BASE_RADIUS * mbot_vel_cmd.wz) / DIFF_WHEEL_RADIUS;
                    mbot_motor_vel_cmd.velocity[params.mot_right] = (-mbot_vel_cmd.vx - DIFF_BASE_RADIUS * mbot_vel_cmd.wz) / DIFF_WHEEL_RADIUS;
                    
                    float vel_left_comp = params.motor_polarity[params.mot_left] * mbot_motor_vel_cmd.velocity[params.mot_left];
                    float vel_right_comp = params.motor_polarity[params.mot_right] * mbot_motor_vel_cmd.velocity[params.mot_right];

                    mbot_motor_pwm.utime = global_utime;
                    mbot_motor_pwm_cmd.pwm[params.mot_right] = _calibrated_pwm_from_vel_cmd(vel_right_comp, params.mot_right);
                    mbot_motor_pwm_cmd.pwm[params.mot_left] = _calibrated_pwm_from_vel_cmd(vel_left_comp, params.mot_left);
                }
            } else{
                if(MBOT_DRIVE_TYPE == OMNI_120_DRIVE){
                    mbot_motor_vel_cmd.velocity[params.mot_left] = (SQRT3 / 2.0 * mbot_vel_cmd.vx - 0.5 * mbot_vel_cmd.vy - params.wheel_base_radius * mbot_vel_cmd.wz) / params.wheel_radius;
                    mbot_motor_vel_cmd.velocity[params.mot_right] = (-SQRT3 / 2.0 * mbot_vel_cmd.vx - 0.5 * mbot_vel_cmd.vy - params.wheel_base_radius * mbot_vel_cmd.wz) / params.wheel_radius;
                    mbot_motor_vel_cmd.velocity[params.mot_back] = (mbot_vel_cmd.vy - params.wheel_base_radius * mbot_vel_cmd.wz) / params.wheel_radius;
                    mbot_motor_vel_cmd.velocity[params.mot_left] = params.motor_polarity[params.mot_left] * mbot_motor_vel_cmd.velocity[params.mot_left];
                    mbot_motor_vel_cmd.velocity[params.mot_right] = params.motor_polarity[params.mot_right] * mbot_motor_vel_cmd.velocity[params.mot_right];
                    mbot_motor_vel_cmd.velocity[params.mot_back] = params.motor_polarity[params.mot_back] * mbot_motor_vel_cmd.velocity[params.mot_back];
                    
                    serial_mbot_motor_pwm_t pid_motor_pwm;
                    mbot_motor_vel_ctlr(mbot_motor_vel_cmd, mbot_motor_vel, &params, &pid_motor_pwm);

                    mbot_motor_pwm.utime = global_utime;
                    mbot_motor_pwm_cmd.pwm[params.mot_right] = _calibrated_pwm_from_vel_cmd(mbot_motor_vel_cmd.velocity[params.mot_right], params.mot_right) + (pid_motor_pwm.pwm[params.mot_right]/100.0);
                    mbot_motor_pwm_cmd.pwm[params.mot_back] = _calibrated_pwm_from_vel_cmd(mbot_motor_vel_cmd.velocity[params.mot_back], params.mot_back) + (pid_motor_pwm.pwm[params.mot_back]/100.0);
                    mbot_motor_pwm_cmd.pwm[params.mot_left] = _calibrated_pwm_from_vel_cmd(mbot_motor_vel_cmd.velocity[params.mot_left], params.mot_left) + (pid_motor_pwm.pwm[params.mot_left]/100.0);
                }else if(MBOT_DRIVE_TYPE == DIFFERENTIAL_DRIVE){
                    mbot_motor_vel_cmd.velocity[params.mot_left] = params.motor_polarity[params.mot_left] * (mbot_vel_cmd.vx - DIFF_BASE_RADIUS * mbot_vel_cmd.wz) / DIFF_WHEEL_RADIUS;
                    mbot_motor_vel_cmd.velocity[params.mot_right] = params.motor_polarity[params.mot_right] * (-mbot_vel_cmd.vx - DIFF_BASE_RADIUS * mbot_vel_cmd.wz) / DIFF_WHEEL_RADIUS;

                    mbot_motor_pwm.utime = global_utime;
                    serial_mbot_motor_pwm_t pid_motor_pwm;
                    mbot_motor_vel_ctlr(mbot_motor_vel_cmd, mbot_motor_vel, &params, &pid_motor_pwm);
                    mbot_motor_pwm_cmd.pwm[params.mot_right] = _calibrated_pwm_from_vel_cmd(mbot_motor_vel_cmd.velocity[params.mot_right], params.mot_right) + (pid_motor_pwm.pwm[params.mot_right]/100.0);
                    mbot_motor_pwm_cmd.pwm[params.mot_left] = _calibrated_pwm_from_vel_cmd(mbot_motor_vel_cmd.velocity[params.mot_left], params.mot_left) + (pid_motor_pwm.pwm[params.mot_left]/100.0);
                }
            }
        }else if (drive_mode == MODE_MOTOR_VEL_PID){
            mbot_motor_pwm.utime = global_utime;
            serial_mbot_motor_pwm_t pid_motor_pwm;

            mbot_motor_vel_ctlr(mbot_motor_vel_cmd, mbot_motor_vel, &params, &pid_motor_pwm);

            // printf("PID gains: %.4f, %.4f\n", pid_motor_pwm.pwm[params.mot_left]/100.0, pid_motor_pwm.pwm[params.mot_right]/100.0);
            // printf("Right wheel PID: %.4f, %.4f\n", right_wheel_pid.num.d[0],right_wheel_pid.den.d[0]);
            // printf("Left wheel PID: %.4f, %.4f\n", left_wheel_pid.num.d[0],left_wheel_pid.den.d[0]);
            mbot_motor_pwm_cmd.pwm[params.mot_right] = _calibrated_pwm_from_vel_cmd(mbot_motor_vel_cmd.velocity[params.mot_right], params.mot_right) + (pid_motor_pwm.pwm[params.mot_right]/100.0);
            mbot_motor_pwm_cmd.pwm[params.mot_left] = _calibrated_pwm_from_vel_cmd(mbot_motor_vel_cmd.velocity[params.mot_left], params.mot_left) + (pid_motor_pwm.pwm[params.mot_left]/100.0);
            if(MBOT_DRIVE_TYPE == OMNI_120_DRIVE){
                mbot_motor_pwm_cmd.pwm[params.mot_back] = _calibrated_pwm_from_vel_cmd(mbot_motor_vel_cmd.velocity[params.mot_back], params.mot_back) + (pid_motor_pwm.pwm[params.mot_back]/100.0);
            }
        }else {
            drive_mode = MODE_MOTOR_PWM;
            mbot_motor_pwm.utime = global_utime;
        }

        // Set motors
        mbot_motor_set_duty(params.mot_right, mbot_motor_pwm_cmd.pwm[params.mot_right]);
        mbot_motor_pwm.pwm[params.mot_right] = mbot_motor_pwm_cmd.pwm[params.mot_right];
        mbot_motor_set_duty(params.mot_left, mbot_motor_pwm_cmd.pwm[params.mot_left]);
        mbot_motor_pwm.pwm[params.mot_left] = mbot_motor_pwm_cmd.pwm[params.mot_left];
        // printf("DRIVE MODE: %d", drive_mode);
        // printf(", PWM LEFT: %.4f, PWM RIGHT: %.4f\n", mbot_motor_pwm.pwm[params.mot_left], mbot_motor_pwm.pwm[params.mot_right]);

        //Set servo
        // printf("SERVO MOTOR PWM: %.4f\n", mbot_servo_pwm_cmd.pwm);
        if(mbot_servo_pwm_cmd.pwm != previous_servo)
        {
            previous_servo = mbot_servo_pwm_cmd.pwm;
            mbot_servo_send_pulse_normalized(servo_chan, mbot_servo_pwm_cmd.pwm);
        }

        // previous_servo = mbot_servo_pwm_cmd.pwm;
        // mbot_servo_send_pulse_normalized(servo_chan, mbot_servo_pwm_cmd.pwm);

        if(MBOT_DRIVE_TYPE == OMNI_120_DRIVE){
            mbot_motor_set_duty(params.mot_back, mbot_motor_pwm_cmd.pwm[params.mot_back]);
            mbot_motor_pwm.pwm[params.mot_back] = mbot_motor_pwm_cmd.pwm[params.mot_back];
        }
        mbot_servo_pwm.utime = global_utime;
        mbot_servo_pwm.pwm = mbot_servo_pwm_cmd.pwm;

        // write the encoders to serial
        comms_write_topic(MBOT_ENCODERS, &mbot_encoders);
        // send odom on wire
        comms_write_topic(MBOT_ODOMETRY, &mbot_odometry);
        // write the IMU to serial
        comms_write_topic(MBOT_IMU, &mbot_imu);
        // write the Body velocity to serial
        comms_write_topic(MBOT_VEL, &mbot_vel);
        // write the Motor velocity to serial
        comms_write_topic(MBOT_MOTOR_VEL, &mbot_motor_vel);
        // write the PWMs to serial
        comms_write_topic(MBOT_MOTOR_PWM, &mbot_motor_pwm);
        //uint64_t fn_run_len = to_us_since_boot(get_absolute_time()) + timestamp_offset - cur_pico_time;
        comms_write_topic(MBOT_SERVO_PWM, &mbot_servo_pwm);

    }
    //check comms and kill motors if its been too long
    uint64_t timeout = to_us_since_boot(get_absolute_time()) - global_pico_time;
    if(timeout > MBOT_TIMEOUT_US){
        mbot_motor_set_duty(DIFF_MOTOR_LEFT_SLOT, 0.0);
        if(MBOT_DRIVE_TYPE == OMNI_120_DRIVE){
            mbot_motor_set_duty(1, 0.0);
        }
        mbot_motor_set_duty(DIFF_MOTOR_RIGHT_SLOT, 0.0);
        global_comms_status = COMMS_ERROR;
    }

    return true;
}


int main()
{   
    running = false;
    mbot_init_pico();
    mbot_servo_init_freq(servo_chan, servo_freq);
    mbot_init_hardware();
    mbot_init_comms();
    mbot_read_fram(0, sizeof(params), &params);
    mbot_init_pid(&pid_gains, &params);

    // Servo Setup

    // mbot_servo_cleanup(servo_chan);

    // printf("Successful Setup\n");
    // in_file = fopen("/home/mbot/mbot_ws/mbot_firmware/src/pwm.txt", "w");

    // printf("File Created");
    // // fprintf(in_file, "Time, Motor_Left_Velocty, Motor_Right_Velocity /n");

    // if (in_file == NULL) {
    // printf("Error opening file");
    // }

    // const char *message = "Time, Motor_Left_Velocty, Motor_Right_Velocity /n";
    // size_t len = strlen(message);
    // fwrite(message, 1, len, in_file);

    // printf("Success File Creation\n");
    
    //Check also that define drive type is same as FRAM drive type
    int validate_status = validate_FRAM_data(&params);
    if (validate_status < 0)
    {
        printf("Failed to validate FRAM Data! Error code: %d\n", validate_status);
        return -1;
    }

    if(params.robot_type != MBOT_DRIVE_TYPE){
        printf("#define type is not equal to calibration type!\n");
        return -1;
    }

    sleep_ms(3000);
    print_mbot_params(&params);
    printf("Starting MBot Loop...\n");
    repeating_timer_t loop_timer;
    add_repeating_timer_ms(MAIN_LOOP_PERIOD * 1000, mbot_loop, NULL, &loop_timer); // 1000x to convert to ms
    printf("Done Booting Up!\n");
    running = true;
    uint16_t counter = 0;
    
    while(running){
        // Heartbeat
        if(!(counter % 5)){
            gpio_put(LED_PIN, 1);
        }
        else if(!(counter % 7)){
            gpio_put(LED_PIN, 1);
            counter = 0;
        }
        else{
            gpio_put(LED_PIN, 0);
        }
        // Print State
        mbot_print_state(mbot_imu, mbot_encoders, mbot_odometry, mbot_motor_vel, mbot_servo_pwm_cmd);
        sleep_ms(200);  
        counter++;
    }
}