#ifndef PID_H
#define PID_H

enum pid_command_type {
    PID_COMMAND_SET_GAINS,
};

struct pid_command {
    enum pid_command_type type;
    float kp;
    float ki;
    float kd;
};

int pid_submit_command(const struct pid_command *command);

#endif
