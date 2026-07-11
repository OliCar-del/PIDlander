// sim.h — the plant: rigid-body vertical dynamics, actuator response, and
// the wind gust generator.

#ifndef SIM_H
#define SIM_H

#include <stdbool.h>

typedef enum { TP_INSTANT, TP_LAG, TP_SLEW } ThrustProf;
extern const char *PROF_LABEL[3];

typedef struct SimState {
    float y;        // altitude [m], up is positive, 0 = ground
    float v;        // vertical velocity [m/s]
    float u;        // applied thrust [N] (actuator output)
    float wind;     // current gust force [N], set by the gust model
    bool  landed;   // resting on the ground (gently)
    bool  crashed;  // touched down above the crash speed
    float impact;   // touchdown speed of the crash [m/s]
} SimState;

// Randomized smooth gust generator: quiet gap, then a (1-cos)/2 pulse.
typedef struct Gust {
    float t_next;   // sim time when the next gust starts [s]
    float t_start;  // start time of the active gust [s]
    float dur;      // duration of the active gust [s]
    float amp;      // signed peak force of the active gust [N]
    bool  active;
} Gust;

void  sim_reset(SimState *s);

// One fixed physics step: semi-implicit Euler (velocity first, then position).
// Touching down faster than vcrash marks the craft crashed.
void  sim_step(SimState *s, float m, float g, float vcrash, float dt);

// Actuator response: applied thrust chases the command per the profile.
float actuate(float u_act, float u_cmd, ThrustProf prof, float dt);

// Gust force at sim time t; amp_scale is the peak-force slider value.
float gust_force(Gust *g, float t, float amp_scale);

#endif // SIM_H
