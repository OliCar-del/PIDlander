// pid.h — self-contained PID controller module.
//
// Phase 4 version — instrumented and live-tunable:
//   - anti-windup: conditional integration (no windup while saturated)
//   - derivative on measurement, not error (no kick on setpoint steps)
//   - first-order low-pass on the derivative (tames noise amplification)
//   - the integrator accumulates Ki*e*dt (the term itself, not raw ∫e dt),
//     so gains can be changed at runtime without bumping the output
//   - per-term contributions are kept in the struct for on-screen display
// Feedforward stays outside this module: the caller adds u_ff and passes the
// remaining actuator range as [out_min, out_max] so saturation is still honest.

#ifndef PID_H
#define PID_H

typedef struct PID {
    // gains (safe to change between updates)
    float kp;
    float ki;
    float kd;
    float tau;          // derivative low-pass time constant [s]
    // output limits (actuator range available to the PID)
    float out_min;
    float out_max;
    // state
    float i_term;       // accumulated integral contribution [output units]
    float prev_meas;    // measurement from the previous update
    float d_filt;       // filtered derivative of -measurement
    int   sat;          // last output: +1 clamped high, -1 clamped low, 0 free
    int   primed;       // 0 until first update; suppresses the initial d spike
    // diagnostics (written every update, for telemetry display)
    float p_term;
    float d_term;
} PID;

// Initialize gains/limits and clear state.
void pid_init(PID *pid, float kp, float ki, float kd, float tau,
              float out_min, float out_max);

// Clear integrator and derivative history (e.g. when (re)engaging control).
void pid_reset(PID *pid);

// One controller step at fixed dt. Returns the clamped actuator command.
float pid_update(PID *pid, float setpoint, float measurement, float dt);

#endif // PID_H
