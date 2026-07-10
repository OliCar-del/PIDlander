// main.c — PID LANDER: a 1-D thrust-vector control sandbox.
//
// One rigid body on a vertical axis:  m*y'' = u - m*g - c*y' + w(t)
// Integrated with semi-implicit Euler at a fixed 120 Hz, decoupled from the
// render rate by an accumulator loop.
//
// The controller outputs u_cmd = u_ff + PID(e), where u_ff is gravity
// feedforward using the *estimated* mass (the "mass estimate x" slider lies
// to the controller; the integral term absorbs the model error). The command
// passes through a selectable actuator response — INSTANT, LAG (first-order),
// or SLEW (rate-limited) — before becoming applied thrust. The controller
// measures altitude through optional Gaussian sensor noise. Touching down
// faster than the crash-speed slider destroys the craft (R to reset).
//
// Instruments:
//   - strip chart (bottom left): altitude / setpoint / thrust / wind, 20 s
//   - analysis plot (bottom middle, L to cycle): root locus / Bode with
//     margins and an exact Routh stability verdict / phase portrait /
//     predicted linear step response for the current gains
//   - target sequence editor (bottom right): draggable numbered points make a
//     stepped setpoint profile; PLAY runs it on sim time
//   - tracking scores (left panel): each playback records IAE, ITAE, max
//     error, and control effort (sum |du|) so tunes can be compared
//   - model vs reality (right panel): zeta/wn from the gains, predicted step
//     metrics, and measured overshoot/settle from the last real setpoint step
//   - AUTOTUNE: relay (Astrom-Hagglund) experiment measures Ku and Tu, then
//     rule buttons apply Ziegler-Nichols PID/PI or Tyreus-Luyben gains
//
// Keys:  P PID on/off | W wind | ENTER pause | UP/DOWN or click view: setpoint
//        SPACE manual thrust | L cycle plot | R reset

#include "raylib.h"
#include <complex.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "pid.h"
#include "ui.h"

// ---------- simulation constants ----------
#define SIM_DT      (1.0f / 120.0f)  // fixed physics timestep [s]
#define MASS_DEF    1.0f             // default craft mass [kg]
#define G_DEF       9.81f            // default gravity [m/s^2]
#define DRAG        0.15f            // linear drag coefficient [N*s/m]
#define UMAX_DEF    50.0f            // default max thrust [N]
#define UMAX_TOP    100.0f           // max-thrust slider ceiling [N]
#define WORLD_H     100.0f           // visible altitude range [m]
#define START_Y     50.0f            // spawn altitude [m]
#define VCRASH_DEF  5.0f             // default crash touchdown speed [m/s]

// ---------- actuator response ----------
typedef enum { TP_INSTANT, TP_LAG, TP_SLEW } ThrustProf;
static const char *PROF_LABEL[3] = { "INSTANT", "LAG", "SLEW" };
#define ACT_TAU     0.2f             // LAG: first-order time constant [s]
#define ACT_SLEW    60.0f            // SLEW: max thrust rate [N/s]

// ---------- controller constants ----------
#define KP_DEF      4.0f
#define KI_DEF      0.5f
#define KD_DEF      4.0f
#define KP_TOP      50.0f            // slider ceilings
#define KI_TOP      20.0f
#define KD_TOP      30.0f
#define D_TAU       0.05f            // derivative low-pass time constant [s]
#define SP_START    50.0f            // initial setpoint [m]
#define SP_RATE     15.0f            // setpoint slew from arrow keys [m/s]
#define NOISE_TOP   2.0f             // sensor noise sigma slider ceiling [m]

// ---------- wind gust constants ----------
#define GUST_GAP_MIN  2.0f           // quiet time between gusts [s]
#define GUST_GAP_MAX  6.0f
#define GUST_DUR_MIN  1.0f           // gust duration [s]
#define GUST_DUR_MAX  2.0f
#define WIND_AMP_MAX  50.0f          // slider ceiling for peak gust force [N]

// ---------- layout ----------
#define SCREEN_W    1600
#define SCREEN_H    1000
#define TITLE_H     48

#define LPAN_X      8                // telemetry panel (left)
#define LPAN_Y      56
#define LPAN_W      356
#define LPAN_H      640

#define VIEW_X0     372              // flight view (center)
#define VIEW_X1     1128
#define VIEW_Y0     56
#define VIEW_Y1     696
#define MARGIN_PX   40               // padding above 100 m and below 0 m
#define CRAFT_W_PX  30
#define CRAFT_H_PX  40

#define RPAN_X      1136             // controller panel (right)
#define RPAN_Y      56
#define RPAN_W      456
#define RPAN_H      640

// ---------- strip chart (bottom left) ----------
#define CHART_N     600              // samples in the ring buffer
#define CHART_HZ    30               // sample rate -> 20 s window
#define CHART_X     8
#define CHART_Y     704
#define CHART_W     852
#define CHART_HGT   240

// ---------- analysis plot (bottom middle) ----------
#define PLOT_X      868
#define PLOT_Y      704
#define PLOT_W      356
#define PLOT_HGT    240

// ---------- target sequence editor (bottom right) ----------
#define SEQ_MAX     10
#define SEQ_PX      1232             // panel
#define SEQ_PY      704
#define SEQ_PW      360
#define SEQ_PH      240
#define SEQ_GX      1242             // graph area inside the panel
#define SEQ_GY      770
#define SEQ_GW      340
#define SEQ_GH      164

// ---------- run scores ----------
#define SCORE_KEEP  4                // rows shown in the panel

// ---------- wind streak animation ----------
#define WPART       28

// controller structure: which terms are active
typedef enum { CM_P, CM_PI, CM_PD, CM_PID } CtrlMode;
static const char *MODE_LABEL[4] = { "P", "PI", "PD", "PID" };

typedef enum { PLOT_OFF, PLOT_LOCUS, PLOT_BODE, PLOT_PHASE, PLOT_STEP } PlotMode;
static const char *PLOT_LABEL[5] = { "OFF", "LOCUS", "BODE", "PHASE", "STEP" };

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

typedef struct Chart {
    float y[CHART_N];
    float sp[CHART_N];
    float u[CHART_N];
    float w[CHART_N];
    float v[CHART_N];
    int head;       // next write slot
    int count;      // valid samples so far (saturates at CHART_N)
    int div;        // physics-step divider for the sample rate
} Chart;

// Stepped setpoint sequence: numbered points (time, height), sorted in time;
// the setpoint holds `base` until point 1, then holds each point's height
// until the next point's time.
typedef struct Seq {
    int   n;              // active points
    float T;              // timeframe [s]
    float t[SEQ_MAX];     // point times [s], kept ordered
    float y[SEQ_MAX];     // point heights [m]
    bool  playing;
    float t_play;         // playback clock [s], advances with sim time
    float base;           // setpoint captured when PLAY was pressed
    int   drag;           // point index being dragged, -1 = none
} Seq;

// One scored playback of the target sequence.
typedef struct Score {
    int   id;
    float kp, ki, kd;
    int   mode;
    float iae;      // integral of |error| dt          [m*s]
    float itae;     // integral of t*|error| dt        [m*s^2]
    float emax;     // max |error|                     [m]
    float effort;   // sum of |thrust change|          [N]
} Score;

// Relay (Astrom-Hagglund) autotune experiment state.
typedef struct Autotune {
    bool  running;
    float d;            // relay amplitude [N]
    float h;            // switching hysteresis [m]
    int   sign;         // current relay output sign
    int   ups;          // up-switch count (first two = transient, skipped)
    float last_up_t;    // sim time of the previous counted up-switch
    float periods[4];
    int   np;
    float ymin, ymax;   // oscillation envelope during the measuring window
    float t_start;
    float ku, tu;       // measured ultimate gain/period (0 until measured)
    char  result[96];
    float result_t;     // sim time the result banner was set
} Autotune;

// Predicted linear closed-loop unit-step response for the current gains.
#define STEP_N 240
#define STEP_T 10.0f
typedef struct StepPred {
    float curve[STEP_N];
    float ov_pct;       // peak overshoot [%]
    float settle;       // 2% settling time [s], -1 if none inside STEP_T
    bool  diverged;
} StepPred;

// Measurement of the last real setpoint step flown by the craft.
typedef struct StepMeas {
    bool  seen;         // any step measured yet
    bool  active;       // still tracking the current step
    float to, mag, dir; // target, |step|, sign
    float t0;
    float ov;           // overshoot fraction of |step|
    float settle;       // 2% settling time [s], -1 while outside the band
} StepMeas;

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float frand(float lo, float hi)
{
    return lo + (hi - lo) * (float)GetRandomValue(0, 10000) / 10000.0f;
}

// Standard normal via Box-Muller.
static float gauss(void)
{
    float u1 = frand(1e-6f, 1.0f), u2 = frand(0.0f, 1.0f);
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * PI * u2);
}

static float gust_force(Gust *g, float t, float amp_scale)
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

static void sim_reset(SimState *s)
{
    s->y = START_Y;
    s->v = 0.0f;
    s->u = 0.0f;
    s->wind = 0.0f;
    s->landed = false;
    s->crashed = false;
    s->impact = 0.0f;
}

// One fixed physics step: semi-implicit Euler (velocity first, then position).
static void sim_step(SimState *s, float m, float g, float vcrash, float dt)
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

// Actuator response: applied thrust chases the command per the profile.
static float actuate(float u_act, float u_cmd, ThrustProf prof, float dt)
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

// World-to-screen: altitude in meters (up) -> pixel row (down), and back.
static int world_to_px(float y)
{
    float t = y / WORLD_H;           // 0 at ground, 1 at 100 m
    return VIEW_Y1 - MARGIN_PX - (int)(t * (VIEW_Y1 - VIEW_Y0 - 2 * MARGIN_PX));
}

static float px_to_world(float py)
{
    float t = ((float)(VIEW_Y1 - MARGIN_PX) - py)
            / (float)(VIEW_Y1 - VIEW_Y0 - 2 * MARGIN_PX);
    return clampf(t * WORLD_H, 0.0f, WORLD_H);
}

// ---------- strip chart ----------

static void chart_push(Chart *c, float y, float sp, float u, float w, float v)
{
    c->y[c->head] = y;
    c->sp[c->head] = sp;
    c->u[c->head] = u;
    c->w[c->head] = w;
    c->v[c->head] = v;
    c->head = (c->head + 1) % CHART_N;
    if (c->count < CHART_N) c->count++;
}

// Altitude in meters -> pixel row inside the chart area.
static float chart_py(float val)
{
    val = clampf(val, 0.0f, WORLD_H);
    return (float)(CHART_Y + CHART_HGT) - val / WORLD_H * (float)CHART_HGT;
}

static void chart_draw(const Chart *c, float u_max)
{
    DrawRectangle(CHART_X, CHART_Y, CHART_W, CHART_HGT, (Color){ 16, 21, 36, 255 });
    DrawRectangleLines(CHART_X, CHART_Y, CHART_W, CHART_HGT, DARKGRAY);
    for (int m = 0; m <= 100; m += 25) {
        int py = (int)chart_py((float)m);
        DrawLine(CHART_X, py, CHART_X + CHART_W, py, Fade(DARKGRAY, 0.5f));
        DrawText(TextFormat("%d", m), CHART_X + 6, py - 12, 12, DARKGRAY);
    }
    DrawText("HISTORY - last 20 s", CHART_X + CHART_W - 140, CHART_Y + 6, 14, GRAY);

    float dx = (float)CHART_W / (float)(CHART_N - 1);
    for (int i = 1; i < c->count; i++) {
        int i0 = (c->head - c->count + i - 1 + CHART_N) % CHART_N;
        int i1 = (i0 + 1) % CHART_N;
        float x0 = (float)CHART_X + (float)(i - 1) * dx;
        float x1 = (float)CHART_X + (float)i * dx;
        // thrust rescaled to the live [0,u_max], wind centered on the 50 m line
        DrawLineV((Vector2){ x0, chart_py(c->u[i0] / u_max * WORLD_H) },
                  (Vector2){ x1, chart_py(c->u[i1] / u_max * WORLD_H) },
                  Fade(ORANGE, 0.35f));
        DrawLineV((Vector2){ x0, chart_py(50.0f + c->w[i0] * (50.0f / WIND_AMP_MAX)) },
                  (Vector2){ x1, chart_py(50.0f + c->w[i1] * (50.0f / WIND_AMP_MAX)) },
                  Fade(PURPLE, 0.5f));
        DrawLineV((Vector2){ x0, chart_py(c->sp[i0]) },
                  (Vector2){ x1, chart_py(c->sp[i1]) }, SKYBLUE);
        DrawLineV((Vector2){ x0, chart_py(c->y[i0]) },
                  (Vector2){ x1, chart_py(c->y[i1]) }, RAYWHITE);
    }
    int ly = CHART_Y + CHART_HGT - 20;
    DrawText("altitude", CHART_X + 10,  ly, 14, RAYWHITE);
    DrawText("setpoint", CHART_X + 80,  ly, 14, SKYBLUE);
    DrawText(TextFormat("thrust (0-%.0f N)", u_max), CHART_X + 152, ly, 14,
             Fade(ORANGE, 0.8f));
    DrawText(TextFormat("wind (+-%.0f N)", WIND_AMP_MAX), CHART_X + 290, ly, 14,
             Fade(PURPLE, 0.9f));
}

// ---------- target sequence ----------

// Keep points inside the timeframe/world and ordered in time.
static void seq_normalize(Seq *q)
{
    for (int i = 0; i < q->n; i++) {
        if (q->t[i] < 0.05f)  q->t[i] = 0.05f;
        if (i > 0 && q->t[i] < q->t[i - 1] + 0.1f) q->t[i] = q->t[i - 1] + 0.1f;
        if (q->t[i] > q->T)   q->t[i] = q->T;
        q->y[i] = clampf(q->y[i], 0.0f, WORLD_H);
    }
}

// Stepped profile: hold base until point 1, then each height until the next.
static float seq_eval(const Seq *q, float t)
{
    float sp = q->base;
    for (int i = 0; i < q->n; i++) {
        if (t >= q->t[i]) sp = q->y[i];
        else break;
    }
    return sp;
}

static float seq_sx(const Seq *q, float t)
{
    return (float)SEQ_GX + t / q->T * (float)SEQ_GW;
}

static float seq_sy(float y)
{
    return (float)(SEQ_GY + SEQ_GH) - y / WORLD_H * (float)SEQ_GH;
}

// Draw the editor panel and handle its widgets and point dragging.
// live_setpoint is the current target (baseline of the profile when idle).
static void seq_panel(Seq *q, float live_setpoint)
{
    DrawRectangleLines(SEQ_PX, SEQ_PY, SEQ_PW, SEQ_PH, DARKGRAY);
    DrawText("TARGET SEQUENCE", SEQ_PX + 12, SEQ_PY + 8, 16, GRAY);

    if (ui_button((Rectangle){ SEQ_PX + SEQ_PW - 88, SEQ_PY + 5, 80, 24 },
                  q->playing ? "STOP" : "PLAY", q->playing)) {
        q->playing = !q->playing;
        if (q->playing) {
            q->t_play = 0.0f;
            q->base = live_setpoint;
        }
    }

    // point count and timeframe; times rescale proportionally with T
    float nf = ui_slider(11, SEQ_PX + 12, SEQ_PY + 30, 150, "points",
                         (float)q->n, 2.0f, (float)SEQ_MAX, "%.0f");
    q->n = (int)(nf + 0.5f);
    float newT = ui_slider(12, SEQ_PX + 186, SEQ_PY + 30, 160, "seconds",
                           q->T, 5.0f, 60.0f, "%.0f");
    if (fabsf(newT - q->T) > 1e-6f) {
        float scale = newT / q->T;
        for (int i = 0; i < SEQ_MAX; i++) q->t[i] *= scale;
        q->T = newT;
    }

    // graph frame
    DrawRectangle(SEQ_GX, SEQ_GY, SEQ_GW, SEQ_GH, (Color){ 16, 21, 36, 255 });
    DrawRectangleLines(SEQ_GX, SEQ_GY, SEQ_GW, SEQ_GH, DARKGRAY);
    int mid = (int)seq_sy(50.0f);
    DrawLine(SEQ_GX, mid, SEQ_GX + SEQ_GW, mid, Fade(DARKGRAY, 0.5f));
    DrawText("0", SEQ_GX + 4, SEQ_GY + SEQ_GH - 14, 12, DARKGRAY);
    DrawText(TextFormat("%.0fs", q->T), SEQ_GX + SEQ_GW - 28, SEQ_GY + SEQ_GH - 14, 12, DARKGRAY);
    DrawText("50m", SEQ_GX + 4, mid - 14, 12, DARKGRAY);

    // point dragging (allowed even while playing — the profile is live)
    Vector2 mp = GetMousePosition();
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !ui_dragging()) {
        for (int i = 0; i < q->n; i++) {
            float dxp = mp.x - seq_sx(q, q->t[i]);
            float dyp = mp.y - seq_sy(q->y[i]);
            if (dxp * dxp + dyp * dyp < 121.0f) { q->drag = i; break; }
        }
    }
    if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) q->drag = -1;
    if (q->drag >= 0) {
        q->t[q->drag] = (mp.x - (float)SEQ_GX) / (float)SEQ_GW * q->T;
        q->y[q->drag] = ((float)(SEQ_GY + SEQ_GH) - mp.y) / (float)SEQ_GH * WORLD_H;
    }
    seq_normalize(q);

    // stepped profile: horizontal holds with faint vertical connectors
    float base = q->playing ? q->base : live_setpoint;
    float prev_y = base;
    float prev_x = (float)SEQ_GX;
    for (int i = 0; i <= q->n; i++) {
        float seg_end = (i < q->n) ? seq_sx(q, q->t[i]) : (float)(SEQ_GX + SEQ_GW);
        DrawLineV((Vector2){ prev_x, seq_sy(prev_y) },
                  (Vector2){ seg_end, seq_sy(prev_y) }, GOLD);
        if (i < q->n) {
            DrawLineV((Vector2){ seg_end, seq_sy(prev_y) },
                      (Vector2){ seg_end, seq_sy(q->y[i]) }, Fade(GOLD, 0.35f));
            prev_y = q->y[i];
            prev_x = seg_end;
        }
    }

    // numbered points
    for (int i = 0; i < q->n; i++) {
        float px = seq_sx(q, q->t[i]), py = seq_sy(q->y[i]);
        DrawCircleV((Vector2){ px, py }, 6.0f, (q->drag == i) ? RAYWHITE : SKYBLUE);
        DrawText(TextFormat("%d", i + 1), (int)px + 7, (int)py - 16, 12, LIGHTGRAY);
    }

    // playback cursor
    if (q->playing) {
        float cx = seq_sx(q, q->t_play);
        DrawLineV((Vector2){ cx, (float)SEQ_GY }, (Vector2){ cx, (float)(SEQ_GY + SEQ_GH) },
                  Fade(RED, 0.8f));
        DrawText(TextFormat("t = %.1f s", q->t_play), SEQ_GX + SEQ_GW - 80, SEQ_GY + 6, 12, RED);
    }
}

// ---------- root locus ----------

// a*s^2 + b*s + c = 0
static void solve_quadratic(double a, double b, double c, double re[2], double im[2])
{
    double disc = b * b - 4.0 * a * c;
    if (disc >= 0.0) {
        double r = sqrt(disc);
        re[0] = (-b + r) / (2.0 * a); im[0] = 0.0;
        re[1] = (-b - r) / (2.0 * a); im[1] = 0.0;
    } else {
        double r = sqrt(-disc);
        re[0] = re[1] = -b / (2.0 * a);
        im[0] = r / (2.0 * a);
        im[1] = -im[0];
    }
}

// a*s^3 + b*s^2 + c*s + d = 0 (a != 0), via depressed cubic + Cardano/trig.
static void solve_cubic(double a, double b, double c, double d,
                        double re[3], double im[3])
{
    double p = b / a, q = c / a, r = d / a;
    double P = q - p * p / 3.0;
    double Q = 2.0 * p * p * p / 27.0 - p * q / 3.0 + r;
    double shift = -p / 3.0;
    double D = Q * Q / 4.0 + P * P * P / 27.0;

    if (D > 1e-12) {                 // one real root + complex pair
        double sq = sqrt(D);
        double u = cbrt(-Q / 2.0 + sq), v = cbrt(-Q / 2.0 - sq);
        re[0] = u + v + shift;            im[0] = 0.0;
        re[1] = -(u + v) / 2.0 + shift;   im[1] = (u - v) * sqrt(3.0) / 2.0;
        re[2] = re[1];                    im[2] = -im[1];
    } else if (D < -1e-12) {         // three distinct real roots
        double rho = sqrt(-P * P * P / 27.0);
        double arg = -Q / (2.0 * rho);
        if (arg > 1.0) arg = 1.0;
        if (arg < -1.0) arg = -1.0;
        double theta = acos(arg);
        double mag = 2.0 * sqrt(-P / 3.0);
        for (int k = 0; k < 3; k++) {
            re[k] = mag * cos((theta + 2.0 * PI * (double)k) / 3.0) + shift;
            im[k] = 0.0;
        }
    } else {                         // repeated roots
        double u = cbrt(-Q / 2.0);
        re[0] = 2.0 * u + shift;
        re[1] = re[2] = -u + shift;
        im[0] = im[1] = im[2] = 0.0;
    }
}

// Closed-loop poles for m*s^3 + (c+Kd)*s^2 + Kp*s + Ki = 0 (2nd order if Ki=0).
static int poles(double m, double drag, double kp, double ki, double kd,
                 double re[3], double im[3])
{
    if (ki > 1e-6) {
        solve_cubic(m, drag + kd, kp, ki, re, im);
        return 3;
    }
    solve_quadratic(m, drag + kd, kp, re, im);
    return 2;
}

static void locus_draw(int lx, int ly, int lw, int lh,
                       float m, float kp, float ki, float kd)
{
    const double RE_MIN = -9.0, RE_MAX = 3.0, IM_MAX = 6.0;

    DrawRectangle(lx, ly, lw, lh, (Color){ 16, 21, 36, 255 });
    DrawRectangleLines(lx, ly, lw, lh, DARKGRAY);

    // unstable half-plane shading + axes
    int x_zero = lx + (int)((0.0 - RE_MIN) / (RE_MAX - RE_MIN) * (double)lw);
    int y_zero = ly + lh / 2;
    DrawRectangle(x_zero, ly, lx + lw - x_zero, lh, Fade(RED, 0.10f));
    DrawLine(x_zero, ly, x_zero, ly + lh, Fade(RED, 0.6f));
    DrawLine(lx, y_zero, lx + lw, y_zero, Fade(DARKGRAY, 0.8f));

    // branches: sweep Kp with Ki, Kd held at current values
    double re[3], im[3];
    for (int i = 1; i <= 300; i++) {
        double k = (double)KP_TOP * (double)i / 300.0;
        int n = poles(m, DRAG, k, ki, kd, re, im);
        for (int j = 0; j < n; j++) {
            if (re[j] < RE_MIN || re[j] > RE_MAX || fabs(im[j]) > IM_MAX) continue;
            int px = lx + (int)((re[j] - RE_MIN) / (RE_MAX - RE_MIN) * (double)lw);
            int py = ly + (int)((IM_MAX - im[j]) / (2.0 * IM_MAX) * (double)lh);
            DrawCircle(px, py, 1.2f, Fade(LIME, 0.45f));
        }
    }

    // current poles as X marks; red once in the right half-plane
    int n = poles(m, DRAG, kp, ki, kd, re, im);
    for (int j = 0; j < n; j++) {
        if (re[j] < RE_MIN || re[j] > RE_MAX || fabs(im[j]) > IM_MAX) continue;
        int px = lx + (int)((re[j] - RE_MIN) / (RE_MAX - RE_MIN) * (double)lw);
        int py = ly + (int)((IM_MAX - im[j]) / (2.0 * IM_MAX) * (double)lh);
        Color col = (re[j] > 1e-6) ? RED : YELLOW;
        DrawLine(px - 5, py - 5, px + 5, py + 5, col);
        DrawLine(px - 5, py + 5, px + 5, py - 5, col);
    }

    DrawText(TextFormat("ROOT LOCUS: poles vs Kp 0..%.0f (Ki, Kd held)", KP_TOP),
             lx + 8, ly + 6, 13, LIGHTGRAY);
    DrawText("s-plane; filter/actuator/sampling ignored", lx + 8, ly + 22, 12, DARKGRAY);
    DrawText("Re=0", x_zero + 4, ly + lh - 16, 12, Fade(RED, 0.8f));
}

// Generic Routh-Hurwitz test: coefficients in descending powers, degree deg.
// Returns true iff all roots are strictly in the left half-plane.
static bool routh_stable(const double *coef, int deg)
{
    double a[8] = { 0 }, b[8] = { 0 }, c[8] = { 0 };
    int w = deg / 2 + 1;
    for (int i = 0; 2 * i <= deg; i++)     a[i] = coef[2 * i];
    for (int i = 0; 2 * i + 1 <= deg; i++) b[i] = coef[2 * i + 1];
    for (int i = 0; i <= deg; i++)
        if (coef[i] <= 0.0) return false;  // necessary: all same sign
    for (int row = 0; row < deg - 1; row++) {
        if (b[0] <= 0.0) return false;
        for (int j = 0; j < w - 1; j++)
            c[j] = (b[0] * a[j + 1] - a[0] * b[j + 1]) / b[0];
        c[w - 1] = 0.0;
        for (int j = 0; j < w; j++) { a[j] = b[j]; b[j] = c[j]; }
    }
    return b[0] > 0.0;
}

// Exact closed-loop stability for the linear model INCLUDING the derivative
// filter and (for LAG) the actuator: characteristic polynomial
//   s(1+tf s)(1+ta s)(m s^2 + c s) + Kp s(1+tf s) + Ki(1+tf s) + Kd s^2 = 0
static bool loop_stable(float m, float kp, float ki, float kd, ThrustProf prof)
{
    double tf = D_TAU, ta = (prof == TP_LAG) ? ACT_TAU : 0.0;
    double a[6] = {
        ta * tf * (double)m,
        tf * (double)m + ta * (double)m + ta * tf * (double)DRAG,
        (double)m + tf * (double)DRAG + ta * (double)DRAG,
        (double)DRAG + (double)kd + (double)kp * tf,
        (double)kp + (double)ki * tf,
        (double)ki,
    };
    const double *p = a;
    int deg = 5;
    while (deg > 0 && fabs(p[0]) < 1e-12) { p++; deg--; }   // ta or tf = 0
    if (deg > 0 && fabs(p[deg]) < 1e-9) deg--;  // Ki=0: spurious factor of s
    if (deg <= 0 || fabs(p[deg]) < 1e-9) return false;      // pole at origin
    return routh_stable(p, deg);
}

// ---------- Bode plot ----------

// Open-loop L(jw) = C(jw)*A(jw)*G(jw) with the derivative filter and (for
// LAG) the actuator included, so the margins are honest about both. SLEW is
// nonlinear and has no transfer function; it is treated as instant here.
// PM/GM are advisory (this loop can be conditionally stable, where single
// crossings mislead); the STABLE/UNSTABLE verdict is exact via Routh.
static void bode_draw(int lx, int ly, int lw, int lh, float m,
                      float kp, float ki, float kd, ThrustProf prof)
{
    enum { BN = 160 };
    float mag[BN], ph[BN], lg[BN];
    double prev = 0.0;

    for (int i = 0; i < BN; i++) {
        double lgw = -1.5 + 3.5 * (double)i / (double)(BN - 1);  // 0.03..~100 rad/s
        double w = pow(10.0, lgw);
        double complex jw = I * w;
        double complex G = 1.0 / ((double)m * jw * jw + (double)DRAG * jw);
        double complex C = (double)kp + (double)kd * jw / (1.0 + jw * (double)D_TAU);
        if (ki > 1e-6f) C += (double)ki / jw;
        double complex A = (prof == TP_LAG) ? 1.0 / (1.0 + jw * (double)ACT_TAU) : 1.0;
        double complex L = C * A * G;

        mag[i] = (float)(20.0 * log10(cabs(L) + 1e-12));
        double p = carg(L) * 180.0 / PI;
        if (i > 0) {                                   // unwrap
            while (p - prev > 180.0)  p -= 360.0;
            while (p - prev < -180.0) p += 360.0;
        } else {
            // this plant lags at least -90 deg at low frequency; carg() can
            // report ~+180 for a true -180, which would shift the whole
            // unwrapped curve up a turn — pin the first sample negative
            while (p > 0.0) p -= 360.0;
        }
        prev = p;
        ph[i] = (float)p;
        lg[i] = (float)lgw;
    }

    // margins: PM at the last 0 dB crossing, GM at the worst -180 crossing
    float pm = NAN, w_pm = 0.0f, gm = NAN, w_gm = 0.0f;
    for (int i = 1; i < BN; i++) {
        if (mag[i - 1] >= 0.0f && mag[i] < 0.0f) {
            float t = mag[i - 1] / (mag[i - 1] - mag[i]);
            pm = 180.0f + ph[i - 1] + t * (ph[i] - ph[i - 1]);
            w_pm = powf(10.0f, lg[i - 1] + t * (lg[i] - lg[i - 1]));
        }
        if ((ph[i - 1] > -180.0f) != (ph[i] > -180.0f)) {
            float t = (ph[i - 1] + 180.0f) / (ph[i - 1] - ph[i]);
            float g = -(mag[i - 1] + t * (mag[i] - mag[i - 1]));
            if (isnan(gm) || g < gm) {
                gm = g;
                w_gm = powf(10.0f, lg[i - 1] + t * (lg[i] - lg[i - 1]));
            }
        }
    }

    // frame + two subplots (magnitude over phase)
    DrawRectangle(lx, ly, lw, lh, (Color){ 16, 21, 36, 255 });
    DrawRectangleLines(lx, ly, lw, lh, DARKGRAY);
    int my0 = ly + 38, mh = (lh - 60) / 2;
    int py0 = my0 + mh + 8, phh = mh;

    bool stable = loop_stable(m, kp, ki, kd, prof);
    DrawText(stable ? "BODE - closed loop STABLE (incl. filter & lag)"
                    : "BODE - closed loop UNSTABLE (incl. filter & lag)",
             lx + 8, ly + 6, 13, stable ? GREEN : RED);
    const char *pm_s = isnan(pm) ? "--" : TextFormat("%.0f deg", pm);
    DrawText(TextFormat("phase margin %s @ %.1f   gain margin %s @ %.1f rad/s",
             pm_s, w_pm, isnan(gm) ? "--" : TextFormat("%.1f dB", gm), w_gm),
             lx + 8, ly + 22, 12, LIGHTGRAY);

    // decade gridlines + reference lines
    for (int d = -1; d <= 2; d++) {
        int px = lx + (int)(((float)d + 1.5f) / 3.5f * (float)lw);
        DrawLine(px, my0, px, my0 + mh, Fade(DARKGRAY, 0.5f));
        DrawLine(px, py0, px, py0 + phh, Fade(DARKGRAY, 0.5f));
        DrawText(TextFormat("%g", pow(10.0, d)), px + 3, py0 + phh - 12, 12, DARKGRAY);
    }
    int zero_db = my0 + (int)(60.0f / 120.0f * (float)mh);
    DrawLine(lx, zero_db, lx + lw, zero_db, Fade(SKYBLUE, 0.4f));
    DrawText("0 dB", lx + 4, zero_db - 12, 12, Fade(SKYBLUE, 0.7f));
    int m180 = py0 + (int)(180.0f / 360.0f * (float)phh);
    DrawLine(lx, m180, lx + lw, m180, Fade(GOLD, 0.4f));
    DrawText("-180", lx + 4, m180 - 12, 12, Fade(GOLD, 0.7f));

    // traces
    for (int i = 1; i < BN; i++) {
        float x0 = (float)lx + (lg[i - 1] + 1.5f) / 3.5f * (float)lw;
        float x1 = (float)lx + (lg[i] + 1.5f) / 3.5f * (float)lw;
        float m0 = clampf(mag[i - 1], -60.0f, 60.0f);
        float m1 = clampf(mag[i],     -60.0f, 60.0f);
        DrawLineV((Vector2){ x0, (float)my0 + (60.0f - m0) / 120.0f * (float)mh },
                  (Vector2){ x1, (float)my0 + (60.0f - m1) / 120.0f * (float)mh }, SKYBLUE);
        float p0 = clampf(ph[i - 1], -360.0f, 0.0f);
        float p1 = clampf(ph[i],     -360.0f, 0.0f);
        DrawLineV((Vector2){ x0, (float)py0 + (0.0f - p0) / 360.0f * (float)phh },
                  (Vector2){ x1, (float)py0 + (0.0f - p1) / 360.0f * (float)phh }, GOLD);
    }
    if (prof == TP_SLEW)
        DrawText("slew is nonlinear: treated as instant", lx + lw - 230, ly + lh - 14, 12, ORANGE);
}

// ---------- phase portrait ----------

static void phase_draw(const Chart *c, int lx, int ly, int lw, int lh)
{
    const float E_MAX = 60.0f, V_MAX = 40.0f;

    DrawRectangle(lx, ly, lw, lh, (Color){ 16, 21, 36, 255 });
    DrawRectangleLines(lx, ly, lw, lh, DARKGRAY);
    int cx = lx + lw / 2, cy = ly + lh / 2;
    DrawLine(lx, cy, lx + lw, cy, Fade(DARKGRAY, 0.6f));
    DrawLine(cx, ly, cx, ly + lh, Fade(DARKGRAY, 0.6f));
    DrawLine(cx - 5, cy, cx + 5, cy, GOLD);            // target equilibrium
    DrawLine(cx, cy - 5, cx, cy + 5, GOLD);

    for (int i = 1; i < c->count; i++) {
        int i0 = (c->head - c->count + i - 1 + CHART_N) % CHART_N;
        int i1 = (i0 + 1) % CHART_N;
        float e0 = clampf(c->sp[i0] - c->y[i0], -E_MAX, E_MAX);
        float e1 = clampf(c->sp[i1] - c->y[i1], -E_MAX, E_MAX);
        float v0 = clampf(c->v[i0], -V_MAX, V_MAX);
        float v1 = clampf(c->v[i1], -V_MAX, V_MAX);
        float alpha = 0.10f + 0.85f * (float)i / (float)c->count;
        DrawLineV((Vector2){ (float)lx + (e0 + E_MAX) / (2 * E_MAX) * (float)lw,
                             (float)ly + (V_MAX - v0) / (2 * V_MAX) * (float)lh },
                  (Vector2){ (float)lx + (e1 + E_MAX) / (2 * E_MAX) * (float)lw,
                             (float)ly + (V_MAX - v1) / (2 * V_MAX) * (float)lh },
                  Fade(SKYBLUE, alpha));
    }
    if (c->count > 0) {
        int il = (c->head - 1 + CHART_N) % CHART_N;
        float e = clampf(c->sp[il] - c->y[il], -E_MAX, E_MAX);
        float v = clampf(c->v[il], -V_MAX, V_MAX);
        DrawCircleV((Vector2){ (float)lx + (e + E_MAX) / (2 * E_MAX) * (float)lw,
                               (float)ly + (V_MAX - v) / (2 * V_MAX) * (float)lh },
                    3.5f, RAYWHITE);
    }
    DrawText("PHASE PORTRAIT: error (x) vs velocity (y), last 20 s",
             lx + 8, ly + 6, 13, LIGHTGRAY);
    DrawText("spiral into the crosshair = damped convergence",
             lx + 8, ly + 22, 12, DARKGRAY);
}

// ---------- predicted step response ----------

// Simulate the LINEAR closed loop (deviation coordinates: feedforward cancels
// gravity, no saturation or ground) for a unit setpoint step with the current
// gains, filter, and (for LAG) actuator. This is the response the sliders
// "promise"; the flight shows what saturation and disturbances do to it.
static void step_predict(StepPred *sp, float m, float kp, float ki, float kd,
                         ThrustProf prof)
{
    const float dt = 1.0f / 120.0f;
    const int n = (int)(STEP_T / dt);
    const int sub = n / STEP_N;
    float y = 0.0f, v = 0.0f, integ = 0.0f, dfilt = 0.0f, prevy = 0.0f, uact = 0.0f;

    sp->ov_pct = 0.0f;
    sp->settle = -1.0f;
    sp->diverged = false;
    for (int i = 0; i < n; i++) {
        float e = 1.0f - y;
        integ += ki * e * dt;
        float draw = -(y - prevy) / dt;
        prevy = y;
        dfilt += (draw - dfilt) * dt / (D_TAU + dt);
        float ucmd = kp * e + integ + kd * dfilt;
        uact = actuate(uact, ucmd, (prof == TP_LAG) ? TP_LAG : TP_INSTANT, dt);
        v += (uact - DRAG * v) / m * dt;
        y += v * dt;

        if (i % sub == 0) sp->curve[i / sub] = y;
        if (y - 1.0f > sp->ov_pct) sp->ov_pct = y - 1.0f;
        if (fabsf(1.0f - y) > 0.02f) sp->settle = -1.0f;
        else if (sp->settle < 0.0f) sp->settle = (float)i * dt;
        if (fabsf(y) > 100.0f) {
            sp->diverged = true;
            for (int k = (i / sub) + 1; k < STEP_N; k++) sp->curve[k] = y;
            break;
        }
    }
    sp->ov_pct *= 100.0f;
}

static void step_draw(const StepPred *sp, int lx, int ly, int lw, int lh)
{
    const float Y_TOP = 1.8f;                      // plotted range 0..1.8

    DrawRectangle(lx, ly, lw, lh, (Color){ 16, 21, 36, 255 });
    DrawRectangleLines(lx, ly, lw, lh, DARKGRAY);
    DrawText("PREDICTED STEP RESPONSE (linear, no saturation)",
             lx + 8, ly + 6, 13, LIGHTGRAY);
    if (sp->diverged)
        DrawText("UNSTABLE: response diverges", lx + 8, ly + 22, 12, RED);
    else
        DrawText(TextFormat("overshoot %.1f%%   2%%-settle %s", sp->ov_pct,
                 sp->settle < 0.0f ? "> 10 s" : TextFormat("%.1f s", sp->settle)),
                 lx + 8, ly + 22, 12, SKYBLUE);

    int gy = ly + 40, gh = lh - 58;
    int target = gy + (int)((Y_TOP - 1.0f) / Y_TOP * (float)gh);
    DrawLine(lx, target, lx + lw, target, Fade(GOLD, 0.6f));
    DrawText("target", lx + 4, target - 14, 12, Fade(GOLD, 0.8f));
    int band = (int)(0.02f / Y_TOP * (float)gh);
    DrawLine(lx, target - band, lx + lw, target - band, Fade(GOLD, 0.2f));
    DrawLine(lx, target + band, lx + lw, target + band, Fade(GOLD, 0.2f));
    for (int s = 0; s <= (int)STEP_T; s += 2) {
        int px = lx + (int)((float)s / STEP_T * (float)lw);
        DrawLine(px, gy, px, gy + gh, Fade(DARKGRAY, 0.4f));
        DrawText(TextFormat("%ds", s), px + 3, gy + gh + 2, 12, DARKGRAY);
    }

    for (int i = 1; i < STEP_N; i++) {
        float y0 = clampf(sp->curve[i - 1], -0.05f, Y_TOP);
        float y1 = clampf(sp->curve[i],     -0.05f, Y_TOP);
        DrawLineV((Vector2){ (float)lx + (float)(i - 1) / (STEP_N - 1) * (float)lw,
                             (float)gy + (Y_TOP - y0) / Y_TOP * (float)gh },
                  (Vector2){ (float)lx + (float)i / (STEP_N - 1) * (float)lw,
                             (float)gy + (Y_TOP - y1) / Y_TOP * (float)gh },
                  SKYBLUE);
    }
}

// Signed horizontal bar for one controller term, centered at cx.
static void term_bar(const char *label, float value, int cx, int y, Color col)
{
    const float px_per_n = 4.5f;
    DrawText(label, cx - 150, y, 22, col);
    DrawLine(cx, y - 2, cx, y + 20, GRAY);
    int w = (int)(value * px_per_n);
    if (w > 140) w = 140;
    if (w < -140) w = -140;
    if (w >= 0) DrawRectangle(cx, y + 2, w, 16, col);
    else        DrawRectangle(cx + w, y + 2, -w, 16, col);
    DrawText(TextFormat("%+7.2f N", value), cx + 48, y, 18, col);
}

int main(void)
{
    InitWindow(SCREEN_W, SCREEN_H, "PID Lander");
    SetTargetFPS(60);

    SimState sim;
    sim_reset(&sim);

    // Live-tunable plant and controller parameters (sliders).
    float mass = MASS_DEF;
    float gravity = G_DEF;
    float kp = KP_DEF, ki = KI_DEF, kd = KD_DEF;
    float wind_amp = 5.0f;
    float mass_est_ratio = 1.0f;     // controller's mass model / true mass
    float u_max = UMAX_DEF;
    float noise_sigma = 0.0f;        // sensor noise std dev [m]
    float v_crash = VCRASH_DEF;      // crash touchdown speed [m/s]
    CtrlMode mode = CM_PID;
    ThrustProf prof = TP_INSTANT;
    PlotMode plot = PLOT_OFF;

    // PID handles deviations only; feedforward carries (estimated) gravity.
    // Its limits are the actuator range left after feedforward, so the
    // anti-windup logic still sees true saturation: u_ff + out ∈ [0, u_max].
    PID ctrl;
    pid_init(&ctrl, kp, ki, kd, D_TAU, 0.0f, u_max);  // limits set each frame
    bool pid_on = false;
    float setpoint = SP_START;

    Gust gust = { 0 };
    bool wind_on = false;
    bool paused = false;

    Chart chart = { 0 };

    Seq seq = {
        .n = 4, .T = 20.0f,
        .t = { 2, 4, 6, 8, 10, 12, 14, 16, 18, 20 },
        .y = { 30, 60, 20, 70, 40, 80, 25, 55, 65, 35 },
        .playing = false, .t_play = 0.0f, .base = SP_START, .drag = -1,
    };

    // playback scoring
    Score scores[SCORE_KEEP] = { 0 };
    int n_scores = 0, next_run_id = 1;
    bool run_active = false;
    Score run = { 0 };
    float run_uprev = 0.0f;

    Autotune at = { 0 };
    StepPred pred = { 0 };
    StepMeas sm = { 0 };
    float sp_prev_step = SP_START;   // setpoint at the previous physics step

    // wind streak particles (screen space, wrap horizontally)
    float wpx[WPART], wpy[WPART];
    for (int i = 0; i < WPART; i++) {
        wpx[i] = frand((float)VIEW_X0 + 10, (float)VIEW_X1 - 10);
        wpy[i] = frand((float)VIEW_Y0 + 30, (float)VIEW_Y1 - 30);
    }

    float crash_t = -100.0f;         // sim time of the crash, for the animation
    bool prev_crashed = false;

    float acc = 0.0f;                // physics time accumulator [s]
    float t_sim = 0.0f;              // simulated time [s], drives the gusts

    while (!WindowShouldClose()) {
        // ---------- input ----------
        float frame = GetFrameTime();
        if (frame > 0.25f) frame = 0.25f;   // clamp: no spiral of death

        if (IsKeyPressed(KEY_R)) {
            sim_reset(&sim);
            pid_reset(&ctrl);
            seq.playing = false;
            at.running = false;
        }
        if (IsKeyPressed(KEY_P)) {
            pid_on = !pid_on;
            if (pid_on) pid_reset(&ctrl);   // engage with clean state
        }
        if (IsKeyPressed(KEY_W)) {
            wind_on = !wind_on;
            gust.active = false;
            gust.t_next = t_sim + frand(0.5f, 2.0f);  // first gust comes soon
        }
        if (IsKeyPressed(KEY_L))     plot = (PlotMode)((plot + 1) % 5);
        if (IsKeyPressed(KEY_ENTER)) paused = !paused;
        if (IsKeyDown(KEY_UP))   setpoint += SP_RATE * frame;
        if (IsKeyDown(KEY_DOWN)) setpoint -= SP_RATE * frame;
        setpoint = clampf(setpoint, 0.0f, WORLD_H);
        bool thrusting = IsKeyDown(KEY_SPACE);

        // click/drag inside the flight view moves the setpoint
        Vector2 mp = GetMousePosition();
        if (!ui_dragging() && seq.drag < 0 && IsMouseButtonDown(MOUSE_BUTTON_LEFT) &&
            mp.x > VIEW_X0 && mp.x < VIEW_X1 && mp.y > VIEW_Y0 && mp.y < VIEW_Y1)
            setpoint = px_to_world(mp.y);

        // apply controller structure: masked terms contribute nothing
        bool use_i = (mode == CM_PI || mode == CM_PID);
        bool use_d = (mode == CM_PD || mode == CM_PID);
        ctrl.kp = kp;
        ctrl.ki = use_i ? ki : 0.0f;
        ctrl.kd = use_d ? kd : 0.0f;
        if (!use_i) ctrl.i_term = 0.0f;

        // feedforward uses the (possibly miscalculated) mass estimate
        float u_ff = mass_est_ratio * mass * gravity;
        if (u_ff > u_max) u_ff = u_max;
        ctrl.out_min = -u_ff;
        ctrl.out_max = u_max - u_ff;

        // model-vs-reality numbers for the current effective gains
        step_predict(&pred, mass, ctrl.kp, ctrl.ki, ctrl.kd, prof);
        float wn = (ctrl.kp > 0.001f) ? sqrtf(ctrl.kp / mass) : 0.0f;
        float zeta = (ctrl.kp > 0.001f)
                   ? (ctrl.kd + DRAG) / (2.0f * sqrtf(ctrl.kp * mass)) : 0.0f;

        // ---------- fixed-timestep physics ----------
        if (!paused) {
            acc += frame;
            while (acc >= SIM_DT) {
                if (seq.playing) {
                    setpoint = seq_eval(&seq, seq.t_play);
                    seq.t_play += SIM_DT;
                    if (seq.t_play >= seq.T) seq.playing = false;
                }
                sim.wind = wind_on ? gust_force(&gust, t_sim, wind_amp) : 0.0f;

                // real-step measurement: arm on any setpoint jump >= 2 m
                if (fabsf(setpoint - sp_prev_step) >= 2.0f) {
                    sm.seen = true;
                    sm.active = true;
                    sm.to = setpoint;
                    sm.mag = fabsf(setpoint - sp_prev_step);
                    sm.dir = (setpoint > sp_prev_step) ? 1.0f : -1.0f;
                    sm.t0 = t_sim;
                    sm.ov = 0.0f;
                    sm.settle = -1.0f;
                }
                sp_prev_step = setpoint;

                // controller sees the sensor, not the truth
                float meas = sim.y;
                if (noise_sigma > 0.0f) meas += noise_sigma * gauss();

                float u_cmd;
                if (sim.crashed) {
                    u_cmd = 0.0f;                       // wreckage doesn't thrust
                    if (at.running) {
                        at.running = false;
                        snprintf(at.result, sizeof at.result, "AUTOTUNE aborted: crashed");
                        at.result_t = t_sim;
                    }
                } else if (at.running) {
                    // relay experiment: bang-bang thrust around the setpoint
                    float e = setpoint - meas;
                    if (at.sign > 0 && e < -at.h) {
                        at.sign = -1;
                    } else if (at.sign < 0 && e > at.h) {
                        at.sign = +1;
                        at.ups++;
                        if (at.ups == 3) {              // transient over: start measuring
                            at.ymin = at.ymax = sim.y;
                            at.last_up_t = t_sim;
                        } else if (at.ups > 3) {
                            at.periods[at.np++] = t_sim - at.last_up_t;
                            at.last_up_t = t_sim;
                        }
                    }
                    if (at.ups >= 3) {
                        if (sim.y < at.ymin) at.ymin = sim.y;
                        if (sim.y > at.ymax) at.ymax = sim.y;
                    }
                    u_cmd = u_ff + (float)at.sign * at.d;

                    if (at.np >= 3) {                   // enough cycles: compute Ku/Tu
                        float tu = (at.periods[0] + at.periods[1] + at.periods[2]) / 3.0f;
                        float a = (at.ymax - at.ymin) / 2.0f;
                        if (a > 0.01f && tu > 0.05f) {
                            at.ku = 4.0f * at.d / (PI * a);
                            at.tu = tu;
                            kp = clampf(0.6f * at.ku, 0.0f, KP_TOP);   // classic Z-N PID
                            ki = clampf(1.2f * at.ku / tu, 0.0f, KI_TOP);
                            kd = clampf(0.075f * at.ku * tu, 0.0f, KD_TOP);
                            snprintf(at.result, sizeof at.result,
                                     "AUTOTUNE: Ku=%.1f Tu=%.2fs -> Ziegler-Nichols applied "
                                     "(more rules unlocked)", at.ku, at.tu);
                            mode = CM_PID;
                            pid_reset(&ctrl);
                            pid_on = true;
                        } else {
                            snprintf(at.result, sizeof at.result,
                                     "AUTOTUNE failed: oscillation too small");
                        }
                        at.running = false;
                        at.result_t = t_sim;
                    } else if (t_sim - at.t_start > 30.0f) {
                        snprintf(at.result, sizeof at.result,
                                 "AUTOTUNE failed: no steady oscillation in 30 s");
                        at.running = false;
                        at.result_t = t_sim;
                    }
                } else if (pid_on) {
                    u_cmd = u_ff + pid_update(&ctrl, setpoint, meas, SIM_DT);
                } else {
                    u_cmd = thrusting ? u_max : 0.0f;
                }

                sim.u = actuate(sim.u, u_cmd, prof, SIM_DT);
                sim_step(&sim, mass, gravity, v_crash, SIM_DT);
                if (sim.crashed && !prev_crashed) crash_t = t_sim;
                prev_crashed = sim.crashed;

                // update the real-step measurement against true altitude
                if (sm.active) {
                    float over = (sim.y - sm.to) * sm.dir / sm.mag;
                    if (over > sm.ov) sm.ov = over;
                    if (fabsf(sim.y - sm.to) > 0.02f * sm.mag) sm.settle = -1.0f;
                    else if (sm.settle < 0.0f) sm.settle = t_sim - sm.t0;
                    if (t_sim - sm.t0 > 20.0f) sm.active = false;   // freeze result
                }

                // score the playback against the true altitude
                if (run_active && seq.playing) {
                    float ae = fabsf(setpoint - sim.y);
                    run.iae  += ae * SIM_DT;
                    run.itae += seq.t_play * ae * SIM_DT;
                    if (ae > run.emax) run.emax = ae;
                    run.effort += fabsf(sim.u - run_uprev);
                    run_uprev = sim.u;
                }

                t_sim += SIM_DT;
                acc -= SIM_DT;
                if (++chart.div >= 120 / CHART_HZ) {
                    chart.div = 0;
                    chart_push(&chart, sim.y, setpoint, sim.u, sim.wind, sim.v);
                }
            }
        }

        // ---------- render ----------
        BeginDrawing();
        ClearBackground((Color){ 10, 14, 26, 255 });

        // ===== title block =====
        DrawRectangle(0, 0, SCREEN_W, TITLE_H, (Color){ 15, 21, 40, 255 });
        DrawLine(0, TITLE_H, SCREEN_W, TITLE_H, DARKGRAY);
        DrawText("PID LANDER", 16, 9, 30, GOLD);
        DrawText("tune the controller to hold target altitude against gravity, wind,"
                 " noise and actuator limits - then fly the target sequence and beat"
                 " your tracking scores", 230, 16, 15, LIGHTGRAY);
        DrawFPS(SCREEN_W - 95, 14);

        // ===== flight view (center) =====
        int groundPx = world_to_px(0.0f);
        DrawLine(VIEW_X0, groundPx, VIEW_X1, groundPx, GRAY);

        int spPx = world_to_px(setpoint);
        for (int x = VIEW_X0; x < VIEW_X1; x += 20)
            DrawLine(x, spPx, x + 10, spPx, pid_on ? SKYBLUE : DARKBLUE);
        DrawText(TextFormat("%.0f m", setpoint), VIEW_X1 - 60, spPx - 18, 14,
                 pid_on ? SKYBLUE : DARKBLUE);

        for (int m = 20; m <= (int)WORLD_H; m += 20) {
            int py = world_to_px((float)m);
            DrawLine(VIEW_X0, py, VIEW_X0 + 12, py, DARKGRAY);
            DrawText(TextFormat("%d m", m), VIEW_X0 + 16, py - 7, 12, DARKGRAY);
        }

        // wind streaks
        if (wind_on && fabsf(sim.wind) > 0.3f) {
            float alpha = clampf(0.15f + fabsf(sim.wind) / 15.0f, 0.15f, 0.8f);
            float len = clampf(8.0f + fabsf(sim.wind) * 1.2f, 8.0f, 70.0f);
            float dir = (sim.wind > 0.0f) ? 1.0f : -1.0f;
            for (int i = 0; i < WPART; i++) {
                wpy[i] += sim.wind * 6.0f * frame;
                if (wpy[i] > (float)VIEW_Y1) wpy[i] -= (float)(VIEW_Y1 - VIEW_Y0);
                if (wpy[i] < (float)VIEW_Y0) wpy[i] += (float)(VIEW_Y1 - VIEW_Y0);
                DrawLineV((Vector2){ wpx[i], wpy[i] },
                          (Vector2){ wpx[i], wpy[i] - dir * len },
                          Fade(PURPLE, alpha));
            }
        }

        int craftBottom = world_to_px(sim.y);
        int craftCX = (VIEW_X0 + VIEW_X1) / 2;
        int craftX = craftCX - CRAFT_W_PX / 2;
        if (sim.crashed) {
            float ct = t_sim - crash_t;
            // wreckage
            DrawRectangle(craftCX - 22, groundPx - 10, 44, 10, (Color){ 90, 90, 95, 255 });
            DrawRectanglePro((Rectangle){ (float)craftCX - 26, (float)groundPx - 6, 18, 8 },
                             (Vector2){ 0, 0 }, -24.0f, (Color){ 70, 70, 75, 255 });
            DrawRectanglePro((Rectangle){ (float)craftCX + 12, (float)groundPx - 4, 16, 7 },
                             (Vector2){ 0, 0 }, 18.0f, (Color){ 70, 70, 75, 255 });
            // explosion: flash + ballistic debris for ~1.2 s
            if (ct < 0.5f)
                DrawCircleLines(craftCX, groundPx - 6, 12.0f + 220.0f * ct,
                                Fade(ORANGE, 1.0f - 2.0f * ct));
            if (ct < 1.2f) {
                for (int k = 0; k < 10; k++) {
                    float ang = PI * (0.12f + 0.76f * (float)k / 9.0f);
                    float spd = 130.0f + (float)((k * 47) % 90);
                    float px = (float)craftCX + cosf(ang) * spd * ct;
                    float py = (float)groundPx - (sinf(ang) * spd * ct - 200.0f * ct * ct);
                    if (py < (float)groundPx)
                        DrawRectangle((int)px - 3, (int)py - 3, 6, 6,
                                      Fade((k % 2) ? ORANGE : RED, 1.0f - ct / 1.2f));
                }
            }
        } else {
            DrawRectangle(craftX, craftBottom - CRAFT_H_PX, CRAFT_W_PX, CRAFT_H_PX, RAYWHITE);
            if (sim.u > 0.0f) {
                float flame = 10.0f + 14.0f * sim.u / u_max;
                DrawTriangle((Vector2){ (float)craftX + 5,              (float)craftBottom },
                             (Vector2){ (float)craftX + CRAFT_W_PX - 5, (float)craftBottom },
                             (Vector2){ (float)craftCX, (float)craftBottom + flame },
                             ORANGE);
            }
        }
        if (paused)
            DrawText("PAUSED", craftCX - 70, (VIEW_Y0 + VIEW_Y1) / 2 - 40, 34, YELLOW);
        if (at.running)
            DrawText(TextFormat("AUTOTUNING: relay cycle %d, measuring %d/3 periods",
                     at.ups, at.np), VIEW_X0 + 40, VIEW_Y0 + 14, 17, YELLOW);
        else if (at.result[0] && t_sim - at.result_t < 8.0f)
            DrawText(at.result, VIEW_X0 + 40, VIEW_Y0 + 14, 15, GREEN);
        if (sim.crashed)
            DrawText(TextFormat("CRASHED at %.1f m/s - press R to reset", sim.impact),
                     VIEW_X0 + 40, VIEW_Y0 + 44, 20, RED);

        // ===== telemetry panel (left) =====
        DrawRectangleLines(LPAN_X, LPAN_Y, LPAN_W, LPAN_H, DARKGRAY);
        DrawText("TELEMETRY", 24, 66, 16, GRAY);

        const char *status = sim.crashed       ? "CRASHED"
                           : sim.landed        ? "LANDED"
                           : at.running        ? "AUTOTUNING"
                           : (sim.v >  0.5f)   ? "CLIMBING"
                           : (sim.v < -0.5f)   ? "DESCENDING"
                           : (fabsf(setpoint - sim.y) < 0.5f && pid_on) ? "HOLDING"
                                               : "DRIFTING";
        DrawText(status, 24, 90, 32,
                 sim.crashed ? RED : sim.landed ? YELLOW : GREEN);

        float weight = mass * gravity;
        DrawText(TextFormat("altitude  %8.2f m",   sim.y),            24, 136, 22, GREEN);
        DrawText(TextFormat("velocity  %+8.2f m/s", sim.v),           24, 166, 22, GREEN);
        DrawText(TextFormat("thrust    %8.2f N",   sim.u),            24, 196, 22, GREEN);
        DrawText(TextFormat("wind      %+8.2f N",  sim.wind),         24, 226, 22,
                 wind_on ? PURPLE : DARKGRAY);
        DrawText(TextFormat("error     %+8.2f m",  setpoint - sim.y), 24, 256, 22,
                 pid_on ? SKYBLUE : DARKGRAY);
        DrawText(TextFormat("thrust-to-weight  %5.2f",
                 (weight > 0.01f) ? u_max / weight : 99.99f),         24, 290, 18,
                 (weight > u_max) ? RED : GREEN);
        if (weight > u_max)
            DrawText("weight exceeds max thrust!", 24, 312, 14, RED);

        DrawText("CONTROL EFFORT", 24, 336, 16, GRAY);
        if (pid_on) {
            term_bar("P", ctrl.p_term, 200, 360, RED);
            term_bar("I", ctrl.i_term, 200, 388, YELLOW);
            term_bar("D", ctrl.d_term, 200, 416, SKYBLUE);
            DrawText(TextFormat("feedforward %+7.2f N", u_ff), 30, 448, 16, DARKGRAY);
            if (ctrl.sat)
                DrawText("[SATURATED]", 250, 448, 16, ORANGE);
        } else {
            DrawText("engage PID to see terms", 30, 364, 16, DARKGRAY);
        }

        DrawText("TRACKING SCORES (per playback)", 24, 478, 16, GRAY);
        if (n_scores == 0)
            DrawText("play the target sequence to record a score", 24, 500, 14, DARKGRAY);
        for (int i = 0; i < n_scores; i++) {
            int yy = 500 + i * 36;
            DrawText(TextFormat("#%d %s  Kp %.1f  Ki %.2f  Kd %.1f",
                     scores[i].id, MODE_LABEL[scores[i].mode],
                     scores[i].kp, scores[i].ki, scores[i].kd), 24, yy, 14, LIGHTGRAY);
            DrawText(TextFormat("IAE %.1f  ITAE %.0f  max %.1fm  effort %.0fN",
                     scores[i].iae, scores[i].itae, scores[i].emax, scores[i].effort),
                     36, yy + 16, 14, SKYBLUE);
        }

        if (ui_button((Rectangle){ 24, 648, 155, 34 },
                      pid_on ? "PID: ON" : "PID: OFF", pid_on)) {
            pid_on = !pid_on;
            if (pid_on) pid_reset(&ctrl);
        }
        if (ui_button((Rectangle){ 192, 648, 155, 34 },
                      wind_on ? "WIND: ON" : "WIND: OFF", wind_on)) {
            wind_on = !wind_on;
            gust.active = false;
            gust.t_next = t_sim + frand(0.5f, 2.0f);
        }

        // ===== controller panel (right) =====
        DrawRectangleLines(RPAN_X, RPAN_Y, RPAN_W, RPAN_H, DARKGRAY);
        DrawText("CONTROLLER", 1148, 66, 16, GRAY);

        for (int i = 0; i < 4; i++) {
            if (ui_button((Rectangle){ 1148.0f + (float)i * 109.0f, 88, 104, 26 },
                          MODE_LABEL[i], mode == (CtrlMode)i)) {
                mode = (CtrlMode)i;
                pid_reset(&ctrl);
            }
        }
        if (ui_button((Rectangle){ 1148, 120, 140, 26 }, "UNDER", false)) {
            kp = 8.0f; ki = 0.5f; kd = 0.8f;
        }
        if (ui_button((Rectangle){ 1294, 120, 140, 26 }, "TUNED", false)) {
            kp = KP_DEF; ki = KI_DEF; kd = KD_DEF;
        }
        if (ui_button((Rectangle){ 1440, 120, 140, 26 }, "OVER", false)) {
            kp = 1.5f; ki = 0.2f; kd = 6.0f;
        }

        // tuning-rule presets from the measured Ku/Tu (unlocked by AUTOTUNE)
        if (at.ku > 0.0f) {
            if (ui_button((Rectangle){ 1148, 152, 140, 24 }, "Z-N PID", false)) {
                kp = clampf(0.6f * at.ku, 0.0f, KP_TOP);
                ki = clampf(1.2f * at.ku / at.tu, 0.0f, KI_TOP);
                kd = clampf(0.075f * at.ku * at.tu, 0.0f, KD_TOP);
                mode = CM_PID;
            }
            if (ui_button((Rectangle){ 1294, 152, 140, 24 }, "Z-N PI", false)) {
                kp = clampf(0.45f * at.ku, 0.0f, KP_TOP);
                ki = clampf(0.54f * at.ku / at.tu, 0.0f, KI_TOP);
                kd = 0.0f;
                mode = CM_PI;
            }
            if (ui_button((Rectangle){ 1440, 152, 140, 24 }, "NO OVERSHOOT", false)) {
                kp = clampf(at.ku / 2.2f, 0.0f, KP_TOP);   // Tyreus-Luyben
                ki = clampf(at.ku / (2.2f * 2.2f * at.tu), 0.0f, KI_TOP);
                kd = clampf((at.ku / 2.2f) * at.tu / 6.3f, 0.0f, KD_TOP);
                mode = CM_PID;
            }
        } else {
            DrawText("run AUTOTUNE to unlock tuning-rule presets (Z-N, no-overshoot)",
                     1148, 156, 13, DARKGRAY);
        }

        DrawText("thrust:", 1148, 186, 15, LIGHTGRAY);
        for (int i = 0; i < 3; i++) {
            if (ui_button((Rectangle){ 1218.0f + (float)i * 122.0f, 182, 116, 24 },
                          PROF_LABEL[i], prof == (ThrustProf)i))
                prof = (ThrustProf)i;
        }

        kp       = ui_slider(1, 1148, 214, 424, "Kp",              kp,       0.0f, KP_TOP, "%.2f");
        ki       = ui_slider(2, 1148, 252, 424, "Ki",              ki,       0.0f, KI_TOP, "%.2f");
        kd       = ui_slider(3, 1148, 290, 424, "Kd",              kd,       0.0f, KD_TOP, "%.2f");
        wind_amp = ui_slider(4, 1148, 328, 424, "wind peak N",     wind_amp, 0.0f, WIND_AMP_MAX, "%.1f");
        mass     = ui_slider(5, 1148, 366, 424, "mass kg",         mass,     0.2f, 3.0f,   "%.2f");
        gravity  = ui_slider(6, 1148, 404, 424, "gravity m/s2",    gravity,  0.0f, 25.0f,  "%.2f");
        mass_est_ratio = ui_slider(7, 1148, 442, 424, "mass estimate x", mass_est_ratio,
                                   0.5f, 1.5f, "%.2f");
        u_max    = ui_slider(8, 1148, 480, 424, "max thrust N",    u_max,    5.0f, UMAX_TOP, "%.0f");
        noise_sigma = ui_slider(9, 1148, 518, 424, "sensor noise m", noise_sigma,
                                0.0f, NOISE_TOP, "%.2f");
        v_crash  = ui_slider(10, 1148, 556, 424, "crash speed m/s", v_crash, 1.0f, 20.0f, "%.1f");

        if (ui_button((Rectangle){ 1148, 598, 200, 28 },
                      TextFormat("PLOT: %s", PLOT_LABEL[plot]), plot != PLOT_OFF))
            plot = (PlotMode)((plot + 1) % 5);
        if (ui_button((Rectangle){ 1360, 598, 200, 28 }, "AUTOTUNE", at.running)) {
            if (at.running) {
                at.running = false;             // cancel
                snprintf(at.result, sizeof at.result, "AUTOTUNE cancelled");
                at.result_t = t_sim;
            } else {
                float d = fminf(u_ff, u_max - u_ff) * 0.8f;
                if (d < 0.5f) {
                    snprintf(at.result, sizeof at.result,
                             "AUTOTUNE needs thrust authority both ways (check max thrust)");
                    at.result_t = t_sim;
                } else {
                    float ku_keep = at.ku, tu_keep = at.tu;
                    at = (Autotune){ 0 };
                    at.ku = ku_keep;            // keep old rule presets until remeasured
                    at.tu = tu_keep;
                    at.running = true;
                    at.d = d;
                    at.h = 0.5f + 2.0f * noise_sigma;
                    at.sign = +1;
                    at.t_start = t_sim;
                    seq.playing = false;
                    pid_on = false;             // relay takes over
                }
            }
        }

        // model vs reality
        DrawText(TextFormat("model: zeta %.2f   wn %.2f rad/s   (PD approximation)",
                 zeta, wn), 1148, 636, 16, LIGHTGRAY);
        DrawText(pred.diverged
                 ? "predicted step: UNSTABLE"
                 : TextFormat("predicted step: overshoot %.1f%%  settle %s",
                              pred.ov_pct,
                              pred.settle < 0.0f ? "> 10 s"
                                                 : TextFormat("%.1f s", pred.settle)),
                 1148, 656, 16, pred.diverged ? RED : SKYBLUE);
        DrawText(!sm.seen
                 ? "last real step: none yet (click a new target or PLAY)"
                 : TextFormat("last real step: overshoot %.1f%%  settle %s%s",
                              sm.ov * 100.0f,
                              sm.settle < 0.0f ? "--" : TextFormat("%.1f s", sm.settle),
                              sm.active ? "  (live)" : ""),
                 1148, 676, 16, GOLD);

        // ===== analysis plot (bottom middle) =====
        switch (plot) {
        case PLOT_LOCUS:
            locus_draw(PLOT_X, PLOT_Y, PLOT_W, PLOT_HGT, mass, ctrl.kp, ctrl.ki, ctrl.kd);
            break;
        case PLOT_BODE:
            bode_draw(PLOT_X, PLOT_Y, PLOT_W, PLOT_HGT, mass, ctrl.kp, ctrl.ki, ctrl.kd, prof);
            break;
        case PLOT_PHASE:
            phase_draw(&chart, PLOT_X, PLOT_Y, PLOT_W, PLOT_HGT);
            break;
        case PLOT_STEP:
            step_draw(&pred, PLOT_X, PLOT_Y, PLOT_W, PLOT_HGT);
            break;
        default:
            DrawRectangleLines(PLOT_X, PLOT_Y, PLOT_W, PLOT_HGT, DARKGRAY);
            DrawText("ANALYSIS PLOT (press L or the PLOT button)", PLOT_X + 12, PLOT_Y + 10, 14, GRAY);
            DrawText("LOCUS - closed-loop poles vs Kp",       PLOT_X + 12, PLOT_Y + 44, 14, DARKGRAY);
            DrawText("BODE  - margins + stability verdict",   PLOT_X + 12, PLOT_Y + 66, 14, DARKGRAY);
            DrawText("PHASE - error/velocity trajectory",     PLOT_X + 12, PLOT_Y + 88, 14, DARKGRAY);
            DrawText("STEP  - predicted response for gains",  PLOT_X + 12, PLOT_Y + 110, 14, DARKGRAY);
            DrawText("moon: g=1.62   mars: g=3.71",           PLOT_X + 12, PLOT_Y + 144, 14, DARKGRAY);
            DrawText("stability edge: raise Ki, lower Kd,",   PLOT_X + 12, PLOT_Y + 176, 14, DARKGRAY);
            DrawText("or add LAG thrust + noise",             PLOT_X + 12, PLOT_Y + 194, 14, DARKGRAY);
            break;
        }

        chart_draw(&chart, u_max);
        seq_panel(&seq, setpoint);

        // playback scoring lifecycle: arm on PLAY, record on finish/stop
        if (seq.playing && !run_active) {
            run_active = true;
            run = (Score){ 0 };
            run.id = next_run_id;
            run.kp = kp; run.ki = ki; run.kd = kd;
            run.mode = (int)mode;
            run_uprev = sim.u;
        } else if (!seq.playing && run_active) {
            run_active = false;
            if (seq.t_play > 1.0f) {            // ignore aborted blips
                for (int i = SCORE_KEEP - 1; i > 0; i--) scores[i] = scores[i - 1];
                scores[0] = run;
                if (n_scores < SCORE_KEEP) n_scores++;
                next_run_id++;
            }
        }

        DrawText("P: PID   W: wind   L: plots   ENTER: pause   UP/DOWN or click view: setpoint   "
                 "SPACE: manual thrust   R: reset",
                 CHART_X + 2, SCREEN_H - 30, 17, LIGHTGRAY);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
