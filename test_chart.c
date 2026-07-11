// test_chart.c — headless verification that the history ring buffer is
// sample-faithful: every value drawn at position k-from-the-right must be
// exactly the k-th most recent push, across crashes, resets, frame bursts,
// pauses, and ring wraparound. Uses the real chart.c and sim.c modules and
// replicates main.c's accumulator/push structure.
//
// Build/run via `make test`.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "chart.h"
#include "config.h"
#include "sim.h"

#define SHADOW_MAX 200000

static Chart chart;
static SimState sim;
static float acc = 0.0f;
static float t_sim = 0.0f;

static float shadow_y[SHADOW_MAX];   // every push, in order
static int   shadow_ground[SHADOW_MAX];
static int   n_shadow = 0;

static int fails = 0;

// One render frame exactly as main.c runs it (manual thrust only).
static void frame(float dt, int space, int paused)
{
    if (dt > 0.25f) dt = 0.25f;
    if (paused) return;
    acc += dt;
    while (acc >= SIM_DT) {
        float u_cmd = sim.crashed ? 0.0f : (space ? UMAX_DEF : 0.0f);
        sim.u = actuate(sim.u, u_cmd, TP_INSTANT, SIM_DT);
        sim_step(&sim, MASS_DEF, G_DEF, VCRASH_DEF, SIM_DT);
        t_sim += SIM_DT;
        acc -= SIM_DT;
        if (++chart.div >= 120 / CHART_HZ) {
            chart.div = 0;
            chart_push(&chart, sim.y, 50.0f, sim.u, 0.0f, sim.v, 0);
            if (n_shadow < SHADOW_MAX) {
                shadow_y[n_shadow] = sim.y;
                shadow_ground[n_shadow] = (sim.y <= 0.0f);
                n_shadow++;
            }
        }
    }
}

// Walk the ring with the exact index arithmetic chart_draw uses and compare
// every sample with the shadow history tail.
static void verify(const char *phase)
{
    for (int i = 0; i < chart.count; i++) {
        int ring = (chart.head - chart.count + i + CHART_N) % CHART_N;
        float expect = shadow_y[n_shadow - chart.count + i];
        if (chart.y[ring] != expect) {
            if (fails < 10)
                printf("MISMATCH [%s] sample %d/%d: ring=%.3f shadow=%.3f\n",
                       phase, i, chart.count, chart.y[ring], expect);
            fails++;
        }
        // a zero in the ring must correspond to the craft actually grounded
        if (chart.y[ring] <= 0.0f && !shadow_ground[n_shadow - chart.count + i]) {
            if (fails < 10)
                printf("PHANTOM ZERO [%s] sample %d: craft was airborne\n", phase, i);
            fails++;
        }
    }
    if (chart.count > 0) {
        int newest = (chart.head - 1 + CHART_N) % CHART_N;
        if (chart.y[newest] != shadow_y[n_shadow - 1]) {
            printf("NEWEST WRONG [%s]: ring=%.3f shadow=%.3f\n",
                   phase, chart.y[newest], shadow_y[n_shadow - 1]);
            fails++;
        }
    }
}

int main(void)
{
    sim_reset(&sim);

    // phase 1: unattended launch — free fall, crash, wreck sits at 0
    for (int f = 0; f < 600; f++) { frame(1.0f / 60.0f, 0, 0); verify("launch"); }
    printf("phase 1 (launch, 10 s unattended): y=%.2f crashed=%d impact=%.1f m/s\n",
           sim.y, sim.crashed, sim.impact);

    // phase 2: reset + fly with thrust bursts
    sim_reset(&sim);
    for (int f = 0; f < 900; f++) {
        int space = (f / 20) % 2;            // 1/3 s bursts
        frame(1.0f / 60.0f, space, 0);
        verify("fly");
    }
    printf("phase 2 (reset + bursts):          y=%.2f v=%.2f\n", sim.y, sim.v);

    // phase 3: frame stalls (window drags) — 0.25 s clamped bursts
    for (int f = 0; f < 80; f++) { frame(0.3f, 1, 0); verify("stall"); }
    printf("phase 3 (frame stalls):            y=%.2f\n", sim.y);

    // phase 4: pauses interleaved
    for (int f = 0; f < 400; f++) { frame(1.0f / 60.0f, f % 3 == 0, (f / 40) % 2); verify("pause"); }

    // phase 5: long mixed run to force several ring wraparounds
    for (int f = 0; f < 6000; f++) {
        int space = ((f / 35) % 3) != 0;
        frame((f % 97 == 0) ? 0.25f : 1.0f / 60.0f, space, 0);
        verify("wrap");
    }
    printf("phase 5 (ring wraparound x%d):      pushes=%d count=%d head=%d\n",
           n_shadow / CHART_N, n_shadow, chart.count, chart.head);

    if (fails == 0) {
        printf("CHART RING OK: %d pushes verified sample-perfect, "
               "no phantom zeros\n", n_shadow);
        return 0;
    }
    printf("FAILED: %d mismatches\n", fails);
    return 1;
}
