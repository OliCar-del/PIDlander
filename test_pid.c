// Headless closed-loop test: same plant dynamics as main.c + the real pid.c.
// Phase 3 wiring: u = U_FF + PID(e), PID limits = remaining actuator range.
// Phase 2 baselines to beat: engage sag 2.12 m; step 50->80 overshoot 15.8%,
// 2%-settling 19.5 s.
#include <stdio.h>
#include <math.h>
#include "pid.h"

#define DT    (1.0f/120.0f)
#define MASS  1.0f
#define G     9.81f
#define DRAG  0.15f
#define UMAX  25.0f

#define M_EST 0.9f
#define U_FF  (M_EST * G)

typedef struct { float y, v; } Sim;

static void step(Sim *s, float u, float wind)
{
    float a = (u - MASS*G - DRAG*s->v + wind) / MASS;
    s->v += a * DT;
    s->y += s->v * DT;
    if (s->y <= 0.0f) { s->y = 0.0f; if (s->v < 0.0f) s->v = 0.0f; }
}

int main(void)
{
    PID p;
    pid_init(&p, 4.0f, 0.5f, 4.0f, 0.05f, -U_FF, UMAX - U_FF);
    Sim s = { 50.0f, 0.0f };

    // --- Scenario 1: engage at 50 m, hold 30 s ---
    float sp = 50.0f, min_y = s.y;
    for (int i = 0; i < 30*120; i++) {
        step(&s, U_FF + pid_update(&p, sp, s.y, DT), 0.0f);
        if (s.y < min_y) min_y = s.y;
    }
    printf("hold 50m: after 30s  y=%.3f  v=%.4f  sag=%.2f m  i_term=%.2f N (expect %.2f)\n",
           s.y, s.v, 50.0f - min_y, p.i_term, (MASS - M_EST)*G);

    // --- Scenario 2: step 50 -> 80, run 20 s ---
    sp = 80.0f;
    float max_y = s.y; int settled_at = -1;
    for (int i = 0; i < 20*120; i++) {
        step(&s, U_FF + pid_update(&p, sp, s.y, DT), 0.0f);
        if (s.y > max_y) max_y = s.y;
        if (fabsf(sp - s.y) > 0.02f*30.0f) settled_at = -1;      // outside 2% of step
        else if (settled_at < 0) settled_at = i;
    }
    printf("step 50->80: final y=%.3f  peak=%.2f  overshoot=%.1f%%  ",
           s.y, max_y, (max_y - 80.0f)/30.0f*100.0f);
    if (settled_at >= 0) printf("2%%-settled at %.1fs\n", settled_at/120.0f);
    else                 printf("not settled within 20s\n");

    // --- Scenario 3: gust rejection at 80 m ---
    // 5 N (1-cos)/2 pulse over 1.5 s, then 5 s of calm; worst-case amplitude.
    float dev_max = 0.0f;
    for (int i = 0; i < (int)(6.5f*120); i++) {
        float t = i * DT;
        float wind = (t < 1.5f) ? 5.0f * 0.5f * (1.0f - cosf(2.0f*3.14159265f*t/1.5f))
                                : 0.0f;
        step(&s, U_FF + pid_update(&p, sp, s.y, DT), wind);
        float dev = fabsf(s.y - sp);
        if (dev > dev_max) dev_max = dev;
    }
    printf("gust +5N/1.5s at 80m: max deviation=%.2f m  residual after 5s calm=%.3f m\n",
           dev_max, fabsf(s.y - sp));

    // --- Scenario 4: analytic agreement check ---
    // The 2nd-order model  wn=sqrt(Kp/m), zeta=(Kd+c)/(2*sqrt(Kp*m))  describes
    // PD + feedforward only, so test exactly that: Ki=0, exact-mass FF, small
    // unsaturated step. zeta≈1.04 predicts no overshoot, 2%-settle ~2.9 s
    // (wn*t≈5.8 for critical damping).
    float wn = sqrtf(4.0f / MASS), z = (4.0f + DRAG) / (2.0f * sqrtf(4.0f * MASS));
    float uff_exact = MASS * G;
    PID pd;
    pid_init(&pd, 4.0f, 0.0f, 4.0f, 0.05f, -uff_exact, UMAX - uff_exact);
    s.y = 80.0f; s.v = 0.0f;
    sp = 82.0f;
    max_y = s.y; settled_at = -1;
    for (int i = 0; i < 10*120; i++) {
        step(&s, uff_exact + pid_update(&pd, sp, s.y, DT), 0.0f);
        if (s.y > max_y) max_y = s.y;
        if (fabsf(sp - s.y) > 0.02f*2.0f) settled_at = -1;       // 2% of the 2 m step
        else if (settled_at < 0) settled_at = i;
    }
    printf("PD-only step 80->82: overshoot=%.1f%%  ", (max_y - 82.0f)/2.0f*100.0f);
    if (settled_at >= 0) printf("2%%-settled at %.1fs  ", settled_at/120.0f);
    else                 printf("not settled in 10s  ");
    printf("(analytic: wn=%.2f, zeta=%.2f -> 0%% overshoot, ~2.9s)\n", wn, z);

    return 0;
}
