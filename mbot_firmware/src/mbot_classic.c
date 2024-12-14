/**
 * This file is the main executable for the MBot Classic firmware.
 */
#include "mbot_classic.h"
#pragma pack(1)

// Global Variables and Forward Declarations
static bool running = false;
mbot_params_t params;
uint64_t timestamp_offset = 0;
uint64_t global_utime = 0;
uint64_t global_pico_time = 0;
bool global_comms_status = COMMS_ERROR;
int drive_mode = 0;
mbot_bhy_config_t mbot_imu_config;
mbot_bhy_data_t mbot_imu_data;
float DELTA_THETA_THRESHOLD = 0.3 * (M_PI / 180.0f);    // The minimun value between gyro and odo

// Forward declaration for internal helper function
int mbot_init_pico(void);
int mbot_init_hardware(void);
void mbot_read_encoders(serial_mbot_encoders_t* encoders);
void mbot_read_imu(serial_mbot_imu_t *imu);
void mbot_calculate_motor_vel(serial_mbot_encoders_t encoders, serial_mbot_motor_vel_t *motor_vel);
static float _calibrated_pwm_from_vel_cmd(float vel_cmd, int motor_idx);
void print_mbot_params(const mbot_params_t* params);

/*********************************************************************
 * Main Control Functions
 * ----------------------------------------------------
 * These functions are critical for MBot's operation and include the
 * main loop and initial setup. Students may be asked to review or
 * modify parts of this section depending on their assignment.
 *********************************************************************/

bool mbot_loop(repeating_timer_t *rt)
{
    // Update mbot_vel
    global_utime = to_us_since_boot(get_absolute_time()) + timestamp_offset;
    mbot_vel.utime = global_utime;
    mbot_read_encoders(&mbot_encoders);
    mbot_read_imu(&mbot_imu);
    mbot_calculate_motor_vel(mbot_encoders, &mbot_motor_vel);

    // mbot_calculate_diff_body_vel(   mbot_motor_vel.velocity[MOT_L],
    //                                 mbot_motor_vel.velocity[MOT_R],
    //                                 &mbot_vel
    //                             );

    mbot_calculate_diff_body_vel_imu(   mbot_motor_vel.velocity[MOT_L],
                                        mbot_motor_vel.velocity[MOT_R],
                                        mbot_imu,
                                        &mbot_vel
                                );

    // Update mbot_odometry
    mbot_odometry.utime = global_utime;
    mbot_calculate_odometry(mbot_vel, MAIN_LOOP_PERIOD, &mbot_odometry);

    // only run if we've got 2 way communication...
    if (global_comms_status == COMMS_OK)
    {
        if(drive_mode == MODE_MOTOR_VEL_OL){
            mbot_motor_pwm.utime = global_utime;
            mbot_motor_pwm_cmd.pwm[MOT_R] = _calibrated_pwm_from_vel_cmd(mbot_motor_vel_cmd.velocity[MOT_R], MOT_R);
            mbot_motor_pwm_cmd.pwm[MOT_L] = _calibrated_pwm_from_vel_cmd(mbot_motor_vel_cmd.velocity[MOT_L], MOT_L);
        }else if(drive_mode == MODE_MBOT_VEL){
            //TODO: open loop for now - implement closed loop controller
            mbot_ctlr(mbot_vel_cmd, mbot_vel, &mbot_motor_vel_cmd); // Calculate the mbot_motor_vel_cmd after 
            // printf("The left motor vel is: %f  \n", mbot_motor_vel_cmd.velocity[MOT_L]);
            // printf("The right motor vel is: %f  \n", mbot_motor_vel_cmd.velocity[MOT_R]);

            // Calculate the pwm of feadback control (I know the code is mix here, the pwm_cmd should be vel_cmd, but the data is right)
            mbot_motor_vel_ctlr(mbot_motor_vel_cmd, mbot_motor_vel, &mbot_motor_pwm_cmd); 
            // printf("righ fb cmd is: %f", mbot_motor_pwm_cmd.pwm[MOT_R]);
            // printf("left fb cmd is: %f \n", mbot_motor_pwm_cmd.pwm[MOT_L]);
            float left_pwm_cmd_fb = mbot_motor_pwm_cmd.pwm[MOT_L];
            float right_pwm_cmd_fb = mbot_motor_pwm_cmd.pwm[MOT_R];

            // Calculate the pwm of forward calibration
            // float vel_left_comp = params.motor_polarity[MOT_L] * mbot_motor_vel_cmd.velocity[MOT_L];
            // float vel_right_comp = params.motor_polarity[MOT_R] * mbot_motor_vel_cmd.velocity[MOT_R];
            // mbot_motor_vel_cmd.velocity[MOT_L] = vel_left_comp;
            // mbot_motor_vel_cmd.velocity[MOT_R] = vel_right_comp;
            // printf("righ ff cmd is: %f", mbot_motor_vel_cmd.velocity[MOT_R]);
            // printf("left ff cmd is: %f \n", mbot_motor_vel_cmd.velocity[MOT_L]);

            float left_pwm_cmd_ff = _calibrated_pwm_from_vel_cmd(mbot_motor_vel_cmd.velocity[MOT_L], MOT_L);
            float right_pwm_cmd_ff = _calibrated_pwm_from_vel_cmd(mbot_motor_vel_cmd.velocity[MOT_R], MOT_R);

            mbot_motor_pwm_cmd.pwm[MOT_L] = params.motor_polarity[MOT_L] * (left_pwm_cmd_ff + left_pwm_cmd_fb);
            // printf("The left pwm is: %f \n", mbot_motor_pwm_cmd.pwm[MOT_L]);
            mbot_motor_pwm_cmd.pwm[MOT_R] = params.motor_polarity[MOT_R] * (right_pwm_cmd_ff + right_pwm_cmd_fb);
            // printf("The right pwm is: %f \n", mbot_motor_pwm_cmd.pwm[MOT_R]);

            // Apply acceleration and deceleration limits
            static float prev_left_pwm_cmd = 0.0;
            static float prev_right_pwm_cmd = 0.0;
            float max_accel = 0.05; // Maximum change in PWM per loop iteration
            float max_decel = 0.05; // Maximum change in PWM per loop iteration

            // Calculate the change in PWM command
            float delta_left_pwm = mbot_motor_pwm_cmd.pwm[MOT_L] - prev_left_pwm_cmd;
            float delta_right_pwm = mbot_motor_pwm_cmd.pwm[MOT_R] - prev_right_pwm_cmd;

            // Limit the change in PWM command to prevent abrupt movements
            if (delta_left_pwm > max_accel) {
                delta_left_pwm = max_accel;
            } else if (delta_left_pwm < -max_decel) {
                delta_left_pwm = -max_decel;
            }

            if (delta_right_pwm > max_accel) {
                delta_right_pwm = max_accel;
            } else if (delta_right_pwm < -max_decel) {
                delta_right_pwm = -max_decel;
            }

            // Update the PWM command with the limited change
            mbot_motor_pwm_cmd.pwm[MOT_L] = prev_left_pwm_cmd + delta_left_pwm;
            mbot_motor_pwm_cmd.pwm[MOT_R] = prev_right_pwm_cmd + delta_right_pwm;

            // Store the current PWM command for the next iteration
            prev_left_pwm_cmd = mbot_motor_pwm_cmd.pwm[MOT_L];
            prev_right_pwm_cmd = mbot_motor_pwm_cmd.pwm[MOT_R];
            
        }else {
            drive_mode = MODE_MOTOR_PWM;
            mbot_motor_pwm.utime = global_utime;
        }

        // Set motors
        mbot_motor_set_duty(MOT_R, mbot_motor_pwm_cmd.pwm[MOT_R]);
        mbot_motor_pwm.pwm[MOT_R] = mbot_motor_pwm_cmd.pwm[MOT_R];
        mbot_motor_set_duty(MOT_L, mbot_motor_pwm_cmd.pwm[MOT_L]);
        mbot_motor_pwm.pwm[MOT_L] = mbot_motor_pwm_cmd.pwm[MOT_L];

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
    }
    // comparing current pico time against the last successful communication timestamp(global_pico_time)
    uint64_t timeout = to_us_since_boot(get_absolute_time()) - global_pico_time;
    if(timeout > MBOT_TIMEOUT_US){
        mbot_motor_set_duty(MOT_L, 0.0);
        mbot_motor_set_duty(MOT_R, 0.0);
        global_comms_status = COMMS_ERROR;
    }

    return true;
}

int main()
{
    printf("********************************\n");
    printf("* MBot Classic Firmware v%s *\n", VERSION);
    printf("********************************\n");

    mbot_init_pico();
    mbot_init_hardware();
    mbot_init_comms();
    // Checkpoint 1.3 - Initialize PID parameters/ctlr
    mbot_ctlr_cfg_t ctlr_cfg;

    ctlr_cfg.left.kp = 0.11f;
    ctlr_cfg.left.ki = 0.1f;
    ctlr_cfg.left.kd = 0.001f;
    ctlr_cfg.left.Tf = 2 * MAIN_LOOP_PERIOD;

    ctlr_cfg.right.kp = 0.12f;
    ctlr_cfg.right.ki = 0.09f;
    ctlr_cfg.right.kd = 0.001f;
    ctlr_cfg.right.Tf = 2 * MAIN_LOOP_PERIOD;

    ctlr_cfg.wz.kp = 0.12f;
    ctlr_cfg.wz.ki = 0.00f;
    ctlr_cfg.wz.kd = 0.000f;
    ctlr_cfg.wz.Tf = 2 * MAIN_LOOP_PERIOD;

    mbot_init_ctlr(ctlr_cfg);

    mbot_read_fram(0, sizeof(params), &params);

    //Check also that define drive type is same as FRAM drive type
    int validate_status = validate_mbot_classic_FRAM_data(&params, MOT_L, MOT_R, MOT_UNUSED);
    if (validate_status < 0)
    {
        printf("Failed to validate FRAM Data! Error code: %d\n", validate_status);
        return -1;
    }

    sleep_ms(3000);
    print_mbot_params(&params);
    printf("Starting MBot Loop...\n");
    repeating_timer_t loop_timer;
    add_repeating_timer_ms(MAIN_LOOP_PERIOD * 1000, mbot_loop, NULL, &loop_timer); // 1000x to convert to ms
    printf("Done Booting Up!\n");
    running = true;

    while(running){
        // Print State
        mbot_print_state(mbot_imu, mbot_encoders, mbot_odometry, mbot_motor_vel);
        sleep_ms(200);
    }
}


/******************************************************
 * Helper Functions
 * ----------------------------------------------------
 * These functions are used internally by the main control functions.
 * They are not intended for modification by students. These functions
 * provide lower-level control and utility support.
 ******************************************************/
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
    mbot_motor_init(MOT_L);
    mbot_motor_init(MOT_R);
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

void mbot_read_encoders(serial_mbot_encoders_t* encoders){
    int64_t delta_time = global_utime - encoders->utime;
    encoders->utime = global_utime;
    encoders->delta_time = delta_time;

    encoders->ticks[MOT_R] = mbot_encoder_read_count(MOT_R);
    encoders->delta_ticks[MOT_R] = mbot_encoder_read_delta(MOT_R);
    encoders->ticks[MOT_L] = mbot_encoder_read_count(MOT_L);
    encoders->delta_ticks[MOT_L] = mbot_encoder_read_delta(MOT_L);
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

// Converting the raw encoder ticks into actual rotational velocities in radians per second
void mbot_calculate_motor_vel(serial_mbot_encoders_t encoders, serial_mbot_motor_vel_t *motor_vel){
    float conversion = (1.0 / GEAR_RATIO) * (1.0 / ENCODER_RES) * 1E6f * 2.0 * M_PI;
    motor_vel->velocity[MOT_L] = params.encoder_polarity[MOT_L] * (conversion / encoders.delta_time) * encoders.delta_ticks[MOT_L];
    motor_vel->velocity[MOT_R] = params.encoder_polarity[MOT_R] * (conversion / encoders.delta_time) * encoders.delta_ticks[MOT_R];
}

int mbot_calculate_diff_body_vel(float wheel_left_vel, float wheel_right_vel, serial_twist2D_t *mbot_vel){
    mbot_vel->vx =  DIFF_WHEEL_RADIUS * (wheel_left_vel - wheel_right_vel) / 2.0f;
    mbot_vel->vy = 0;
    mbot_vel->wz =  DIFF_WHEEL_RADIUS * (-wheel_left_vel - wheel_right_vel) / (2.0f * DIFF_BASE_RADIUS);
    return 0; // Return 0 to indicate success
}

int mbot_calculate_diff_body_vel_imu(float wheel_left_vel, float wheel_right_vel, serial_mbot_imu_t imu, serial_twist2D_t *mbot_vel){
    mbot_vel->vx = DIFF_WHEEL_RADIUS * (wheel_left_vel - wheel_right_vel) / 2.0f;
    mbot_vel->vy = 0;
    mbot_vel->wz = DIFF_WHEEL_RADIUS * (-wheel_left_vel - wheel_right_vel) / (2.0f * DIFF_BASE_RADIUS);

    // gyrodometry
    float delta_theta_gyro = imu.gyro[2]; // Angular velocity
    float delta_theta_odo = mbot_vel->wz; 
    float delta_gyro_odo = delta_theta_gyro - delta_theta_odo;

    if(fabs(delta_gyro_odo) > DELTA_THETA_THRESHOLD){
        mbot_vel->wz = delta_theta_gyro; 
    }

    return 0; // Return 0 to indicate success
}

// Use slope + intercept from calibration to generate a PWM command.
static float _calibrated_pwm_from_vel_cmd(float vel_cmd, int motor_idx){
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

void print_mbot_params(const mbot_params_t* params) {
    printf("Motor Polarity: %d %d %d\n", params->motor_polarity[0], params->motor_polarity[1], params->motor_polarity[2]);
    printf("Encoder Polarity: %d %d %d\n", params->encoder_polarity[0], params->encoder_polarity[1], params->encoder_polarity[2]);
    printf("Positive Slope: %f %f %f\n", params->slope_pos[0], params->slope_pos[1], params->slope_pos[2]);
    printf("Positive Intercept: %f %f %f\n", params->itrcpt_pos[0], params->itrcpt_pos[1], params->itrcpt_pos[2]);
    printf("Negative Slope: %f %f %f\n", params->slope_neg[0], params->slope_neg[1], params->slope_neg[2]);
    printf("Negative Intercept: %f %f %f\n", params->itrcpt_neg[0], params->itrcpt_neg[1], params->itrcpt_neg[2]);
}
