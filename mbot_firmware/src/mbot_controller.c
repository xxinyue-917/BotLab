#include "mbot_controller.h"

rc_filter_t left_wheel_pid;
rc_filter_t right_wheel_pid;
rc_filter_t back_wheel_pid;
rc_filter_t mbot_vx_pid;
rc_filter_t mbot_vy_pid;
rc_filter_t mbot_wz_pid;
rc_filter_t low_pass;

int mbot_init_ctlr(mbot_ctlr_cfg_t ctlr_cfg) {
    left_wheel_pid = rc_filter_empty();
    right_wheel_pid = rc_filter_empty();
    // back_wheel_pid = rc_filter_empty();
    // mbot_vx_pid = rc_filter_empty();
    // mbot_vy_pid = rc_filter_empty();
    mbot_wz_pid = rc_filter_empty();
    // low_pass = rc_filter_empty();

    rc_filter_pid(&left_wheel_pid, ctlr_cfg.left.kp, ctlr_cfg.left.ki, ctlr_cfg.left.kd, ctlr_cfg.left.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&right_wheel_pid, ctlr_cfg.right.kp, ctlr_cfg.right.ki, ctlr_cfg.right.kd, ctlr_cfg.right.Tf, MAIN_LOOP_PERIOD);
    // rc_filter_pid(&back_wheel_pid, ctlr_cfg.back.kp, ctlr_cfg.back.ki, ctlr_cfg.back.kd, ctlr_cfg.back.Tf, MAIN_LOOP_PERIOD);
    // rc_filter_pid(&mbot_vx_pid, ctlr_cfg.vx.kp, ctlr_cfg.vx.ki, ctlr_cfg.vx.kd, ctlr_cfg.vx.Tf, MAIN_LOOP_PERIOD);
    // rc_filter_pid(&mbot_vy_pid, ctlr_cfg.vy.kp, ctlr_cfg.vy.ki, ctlr_cfg.vy.kd, ctlr_cfg.vy.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&mbot_wz_pid, ctlr_cfg.wz.kp, ctlr_cfg.wz.ki, ctlr_cfg.wz.kd, ctlr_cfg.wz.Tf, MAIN_LOOP_PERIOD);

    // Checkpoint 1.3: Add first order lowpass

    // float fc = 20.0;
    // rc_filter_first_order_lowpass(&low_pass, 1.0 / MAIN_LOOP_PERIOD, fc);

    return 0;
}

int mbot_motor_vel_ctlr(serial_mbot_motor_vel_t vel_cmd, serial_mbot_motor_vel_t vel, serial_mbot_motor_pwm_t *mbot_motor_pwm) {
    float left_error = vel_cmd.velocity[MOT_L] - vel.velocity[MOT_L];
    float right_error = vel_cmd.velocity[MOT_R] - vel.velocity[MOT_R];
    // printf("right vel is: %f", vel.velocity[MOT_R]);
    // printf("left vel is: %f \n", vel.velocity[MOT_L]);

    // Let the mbot stop when we want it stop
    // if (vel_cmd.velocity[MOT_R] == 0){
    //     right_error = 0;
    // }
    // if (vel_cmd.velocity[MOT_L] == 0){
    //     left_error = 0;
    // }
    printf("right err is: %f", right_error);
    printf("left err is: %f \n", left_error);

    // Use PID to calculate the output (combine fb and ff)
    float left_cmd = rc_filter_march(&left_wheel_pid, left_error);
    float right_cmd = rc_filter_march(&right_wheel_pid, right_error);

    // Limit the speed in [-10, 10]
    left_cmd = fmaxf(fminf(left_cmd, 10.0f), -10.0f);
    right_cmd = fmaxf(fminf(right_cmd, 10.0f), -10.0f);
    // printf("right err cmd is: %f", right_cmd);
    // printf("left err cmd is: %f \n", left_cmd);

    // Configure the speed to pwm
    mbot_motor_pwm->pwm[MOT_L] = left_cmd;
    mbot_motor_pwm->pwm[MOT_R] = right_cmd;

    #ifdef MBOT_TYPE_OMNI
    float back_error = vel_cmd.velocity[MOT_B] - vel.velocity[MOT_B];
    float back_cmd = rc_filter_march(&back_wheel_pid, back_error);
    mbot_motor_pwm->pwm[MOT_B] = back_cmd;
    #endif

    return 0;
}

int mbot_ctlr(serial_twist2D_t vel_cmd, serial_twist2D_t vel, serial_mbot_motor_vel_t *mbot_motor_vel_cmd) {
    // float vx_cmd_filter = rc_filter_march(&right_wheel_pid, vel_cmd.vx);
    // float wz_cmd_filter = rc_filter_march(&right_wheel_pid, vel_cmd.wz);
    float wz_error = vel_cmd.wz - vel.wz;
    // printf("The angular error is: %f \n", wz_error);
    float az_cmd = rc_filter_march(&mbot_wz_pid, wz_error);
    float wz_cmd = vel_cmd.wz + az_cmd;
    // float wz_cmd = vel_cmd.wz;

    if (fabs(wz_cmd) < 0.01) {
        wz_cmd = 0.0;
    }

    mbot_motor_vel_cmd -> velocity[MOT_L] = (vel_cmd.vx - DIFF_BASE_RADIUS * wz_cmd) / DIFF_BASE_RADIUS;
    mbot_motor_vel_cmd -> velocity[MOT_R] = (-vel_cmd.vx - DIFF_BASE_RADIUS * wz_cmd) / DIFF_BASE_RADIUS;
    return 0;
}