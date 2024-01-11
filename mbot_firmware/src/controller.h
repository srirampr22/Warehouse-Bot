#include <mbot_lcm_msgs_serial.h>
#include <mbot/defs/mbot_params.h>
#include <rc/math/filter.h>

typedef struct mbot_pid_cfg_t{
    float kp;
    float ki;
    float kd;
    float Tf;
} mbot_pid_cfg_t;

typedef struct mbot_low_cfg_t{
    float dt;
    float time_con;
} mbot_low_cfg_t;

typedef struct mbot_ctlr_cfg_t{
    mbot_pid_cfg_t right;
    mbot_pid_cfg_t left;
    mbot_pid_cfg_t back;
    mbot_pid_cfg_t vx;
    mbot_pid_cfg_t vy;
    mbot_pid_cfg_t wz;
    mbot_low_cfg_t gyro;
} mbot_ctlr_cfg_t;

rc_filter_t left_wheel_pid = RC_FILTER_INITIALIZER;
rc_filter_t right_wheel_pid = RC_FILTER_INITIALIZER;
rc_filter_t mbot_vx_pid = RC_FILTER_INITIALIZER;
rc_filter_t mbot_vy_pid = RC_FILTER_INITIALIZER;
rc_filter_t mbot_wz_pid = RC_FILTER_INITIALIZER;
rc_filter_t back_wheel_pid = RC_FILTER_INITIALIZER;
rc_filter_t gyro_low = RC_FILTER_INITIALIZER;

void mbot_init_ctlr(mbot_ctlr_cfg_t ctlr_cfg, const mbot_params_t* params);
void mbot_motor_vel_ctlr(serial_mbot_motor_vel_t vel_cmd, serial_mbot_motor_vel_t vel, const mbot_params_t* params, serial_mbot_motor_pwm_t *mbot_motor_pwm);
void mbot_ctlr(serial_twist2D_t vel_cmd, serial_twist2D_t vel, const mbot_params_t* params, serial_mbot_motor_vel_t *mbot_motor_vel);
