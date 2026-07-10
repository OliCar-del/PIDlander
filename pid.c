#include "pid.h"

void pid_init(PID *pid, float kp, float ki, float kd, float tau,
              float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->tau = tau;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid_reset(pid);
}

void pid_reset(PID *pid)
{
    pid->i_term = 0.0f;
    pid->prev_meas = 0.0f;
    pid->d_filt = 0.0f;
    pid->sat = 0;
    pid->primed = 0;
    pid->p_term = 0.0f;
    pid->d_term = 0.0f;
}

float pid_update(PID *pid, float setpoint, float measurement, float dt)
{
    float e = setpoint - measurement;

    // Anti-windup: freeze the integrator when the last output was saturated
    // and this error would only push it further into the limit.
    // Ki is folded in here so a live Ki change rescales only future
    // accumulation instead of stepping the whole term (bumpless tuning).
    if (!((pid->sat > 0 && e > 0.0f) || (pid->sat < 0 && e < 0.0f)))
        pid->i_term += pid->ki * e * dt;

    // Derivative on measurement (d/dt of -y): immune to setpoint steps.
    float d_raw = pid->primed ? -(measurement - pid->prev_meas) / dt : 0.0f;
    pid->prev_meas = measurement;
    pid->primed = 1;

    // First-order low-pass; alpha = dt/(tau+dt) is the discrete pole mapping.
    pid->d_filt += (d_raw - pid->d_filt) * dt / (pid->tau + dt);

    pid->p_term = pid->kp * e;
    pid->d_term = pid->kd * pid->d_filt;
    float u = pid->p_term + pid->i_term + pid->d_term;

    pid->sat = 0;
    if (u > pid->out_max) { u = pid->out_max; pid->sat = +1; }
    if (u < pid->out_min) { u = pid->out_min; pid->sat = -1; }
    return u;
}
