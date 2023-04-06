#ifndef _leg_message
#define _leg_message

#include <stdint.h>

// 116 bytes
// 58 16-bit words
struct spi_data_t
{
    float q_abad[4];
    float q_hip[4];
    float q_knee[4];
    float qd_abad[4];
    float qd_hip[4];
    float qd_knee[4];
    int32_t flags[4];
    int32_t checksum;
};

// 260 bytes
// 130 16-bit words
struct spi_command_t
{
    float q_des_abad[4];
    float q_des_hip[4];
    float q_des_knee[4];
    float qd_des_abad[4];
    float qd_des_hip[4];
    float qd_des_knee[4];
    float kp_abad[4];
    float kp_hip[4];
    float kp_knee[4];
    float kd_abad[4];
    float kd_hip[4];
    float kd_knee[4];
    float tau_abad_ff[4];
    float tau_hip_ff[4];
    float tau_knee_ff[4];
    int32_t flags[4];
    int32_t checksum;
};



struct joint_control{
    float p_des, v_des, kp, kd, t_ff;
    };
    
struct joint_state{
    float p, v, t;
    };
    
struct leg_state{
    joint_state a, h, k;
    };
struct leg_control{
    joint_control a, h, k;
    }
    ;

struct torque_t{
    float tau_abad[4];
    float tau_hip[4];
    float tau_knee[4];
};


struct current_t{
    float tau_abad[4];
    float tau_hip[4];
    float tau_knee[4];
};
#endif
