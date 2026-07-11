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
    s->impact = 0.0f;
}

void sim_step(SimState *s, float m, float g, float vcrash, float dt)
{
    float a = (s->u - m * g - DRAG * s->v + s->wind) / m;
    s->v += a * dt;
    s->y += s->v * dt;

    if (s->y <= 0.0f) {              // ground contact
        if (!s->crashed && s->v < -vcrash) {
            s->crashed = true;
            s->impact = -s->v;
        }
        s->y = 0.0f;
        if (s->v < 0.0f) s->v = 0.0f;
        s->landed = !s->crashed && (s->u <= m * g);
    } else {
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
    if (!g->active) {
        if (t < g->t_next) return 0.0f;
        g->active = true;
        g->t_start = t;
        g->dur = frand(GUST_DUR_MIN, GUST_DUR_MAX);
        g->amp = frand(0.5f, 1.0f) * amp_scale
               * (GetRandomValue(0, 1) ? 1.0f : -1.0f);
    }
    float ph = (t - g->t_start) / g->dur;
    if (ph >= 1.0f) {
        g->active = false;
        g->t_next = t + frand(GUST_GAP_MIN, GUST_GAP_MAX);
        return 0.0f;
    }
    return g->amp * 0.5f * (1.0f - cosf(2.0f * PI * ph));
}
