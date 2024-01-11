#include "controller.h"

void mbot_init_ctlr(mbot_ctlr_cfg_t ctlr_cfg, const mbot_params_t* params){
    rc_filter_pid(&left_wheel_pid, ctlr_cfg.left.kp, ctlr_cfg.left.ki, ctlr_cfg.left.kd, ctlr_cfg.left.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&right_wheel_pid, ctlr_cfg.right.kp, ctlr_cfg.right.ki, ctlr_cfg.right.kd, ctlr_cfg.right.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&mbot_vx_pid, ctlr_cfg.vx.kp, ctlr_cfg.vx.ki, ctlr_cfg.vx.kd, ctlr_cfg.vx.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&mbot_vy_pid, ctlr_cfg.vy.kp, ctlr_cfg.vy.ki, ctlr_cfg.vy.kd, ctlr_cfg.vy.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&mbot_wz_pid, ctlr_cfg.wz.kp, ctlr_cfg.wz.ki, ctlr_cfg.wz.kd, ctlr_cfg.wz.Tf, MAIN_LOOP_PERIOD);
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
