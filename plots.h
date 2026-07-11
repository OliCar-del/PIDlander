// plots.h — the analysis plots: root locus, Bode with margins and an exact
// Routh stability verdict, phase portrait, and the predicted linear step
// response for the current gains.

#ifndef PLOTS_H
#define PLOTS_H

#include "chart.h"
#include "sim.h"

// Predicted linear closed-loop unit-step response for the current gains.
typedef struct StepPred {
    float curve[STEP_N];
    float ov_pct;       // peak overshoot [%]
    float settle;       // 2% settling time [s], -1 if none inside STEP_T
    bool  diverged;
} StepPred;

void locus_draw(int lx, int ly, int lw, int lh,
                float m, float kp, float ki, float kd);

void bode_draw(int lx, int ly, int lw, int lh, float m,
               float kp, float ki, float kd, ThrustProf prof);

void phase_draw(const Chart *c, int lx, int ly, int lw, int lh);

// Simulate the LINEAR closed loop (deviation coordinates: feedforward cancels
// gravity, no saturation or ground) for a unit setpoint step with the current
// gains, filter, and (for LAG) actuator.
void step_predict(StepPred *sp, float m, float kp, float ki, float kd,
                  ThrustProf prof);

void step_draw(const StepPred *sp, int lx, int ly, int lw, int lh);

#endif // PLOTS_H
