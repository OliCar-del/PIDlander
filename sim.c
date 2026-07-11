#include "sim.h"

#include <math.h>

#include "raylib.h"

#include "config.h"
#include "util.h"

const char *PROF_LABEL[3] = { "INSTANT", "LAG", "SLEW" };

void sim_reset(SimState *s)
{
    s->y = START_Y;
    s->v = 0.0f;
    s->u = 0.0f;
    s->wind = 0.0f;
    s->landed = false;
    s->crashed = false;
    s->grounded = false;
    s->impact = -1.0f;
}

void sim_step(SimState *s, float m, float g, float vcrash, float dt)
{
    // Newton's second law on the vertical axis:
    //   thrust (up) - weight (down) - linear drag (opposes v) + wind
    float a = (s->u - m * g - DRAG * s->v + s->wind) / m;

    // Semi-implicit Euler: update velocity FIRST, then advance position
    // with the NEW velocity. This ordering keeps the integrator stable and
    // energy-consistent at a fixed timestep (plain explicit Euler slowly
    // injects energy into oscillating systems).
    s->v += a * dt;
    s->y += s->v * dt;

    if (s->y <= 0.0f) {
        // Ground contact. On the TRANSITION step only (airborne last step),
        // record the touchdown speed and decide whether it was survivable.
        // Resting on the pad keeps re-entering this branch, so the guard
        // prevents the recorded impact from being overwritten with zeros.
        if (!s->grounded) {
            s->impact = (s->v < 0.0f) ? -s->v : 0.0f;
            if (!s->crashed && s->impact > vcrash) s->crashed = true;
        }
        s->grounded = true;
        s->y = 0.0f;                       // the ground is rigid
        if (s->v < 0.0f) s->v = 0.0f;      // and perfectly inelastic
        // "landed" means resting gently: intact, and thrust is not about
        // to lift the craft off again (u below its own weight)
        s->landed = !s->crashed && (s->u <= m * g);
    } else {
        s->grounded = false;
        s->landed = false;
    }
}

float actuate(float u_act, float u_cmd, ThrustProf prof, float dt)
{
    switch (prof) {
    case TP_LAG:
        return u_act + (u_cmd - u_act) * dt / (ACT_TAU + dt);
    case TP_SLEW: {
        float du = u_cmd - u_act;
        float lim = ACT_SLEW * dt;
        if (du > lim)  du = lim;
        if (du < -lim) du = -lim;
        return u_act + du;
    }
    default:
        return u_cmd;
    }
}

float gust_force(Gust *g, float t, float amp_scale)
{
    // Between gusts: wait quietly until the scheduled start time, then
    // roll a random duration and a random signed peak (50-100% of the
    // slider value, either direction).
    if (!g->active) {
        if (t < g->t_next) return 0.0f;
        g->active = true;
        g->t_start = t;
        g->dur = frand(GUST_DUR_MIN, GUST_DUR_MAX);
        g->amp = frand(0.5f, 1.0f) * amp_scale
               * (GetRandomValue(0, 1) ? 1.0f : -1.0f);
    }

    // During a gust: phase runs 0..1 over its duration. When it completes,
    // schedule the next gust after a random quiet gap.
    float ph = (t - g->t_start) / g->dur;
    if (ph >= 1.0f) {
        g->active = false;
        g->t_next = t + frand(GUST_GAP_MIN, GUST_GAP_MAX);
        return 0.0f;
    }

    // Smooth (1 - cos)/2 envelope: force ramps up from zero, peaks at the
    // gust midpoint, and ramps back down — no step disturbances.
    return g->amp * 0.5f * (1.0f - cosf(2.0f * PI * ph));
}
