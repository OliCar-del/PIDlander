// main.c — Phase 5: full flight console for the 1D PID lander.
//
// One rigid body on a vertical axis:  m*y'' = u - m*g - c*y' + w(t)
// Integrated with semi-implicit Euler at a fixed 120 Hz, decoupled from the
// render rate by an accumulator loop.
//
// The controller outputs u = u_ff + PID(e), where u_ff = 0.9*m*g is gravity
// feedforward with a deliberate 10% mass-model error — the integral term
// absorbs the difference. Mass and gravity are live sliders: the plant can
// change under the controller in real time.
//
// Layout: flight view center, telemetry panel left, controller panel right
// (sliders, P/PI/PD/PID structure, presets, optional root locus), 20 s strip
// chart bottom. The root locus sweeps Kp with Ki/Kd held, from the
// continuous-time characteristic equation m*s^3 + (c+Kd)*s^2 + Kp*s + Ki = 0
// (derivative filter and 120 Hz sampling ignored).
//
// Keys:  P PID on/off | W wind | ENTER pause | UP/DOWN or click view: setpoint
//        SPACE manual thrust | L root locus | R reset

#include "raylib.h"
#include <math.h>
#include <stdbool.h>

#include "pid.h"
#include "ui.h"

// ---------- simulation constants ----------
#define SIM_DT      (1.0f / 120.0f)  // fixed physics timestep [s]
#define MASS_DEF    1.0f             // default craft mass [kg]
#define G_DEF       9.81f            // default gravity [m/s^2]
#define DRAG        0.15f            // linear drag coefficient [N*s/m]
#define U_MAX       25.0f            // max thrust [N]
#define WORLD_H     100.0f           // visible altitude range [m]
#define START_Y     50.0f            // spawn altitude [m]

// ---------- controller constants ----------
#define KP_DEF      4.0f
#define KI_DEF      0.5f
#define KD_DEF      4.0f
#define D_TAU       0.05f            // derivative low-pass time constant [s]
#define MASS_EST_RATIO 0.9f          // controller's mass model = 90% of truth
#define SP_START    50.0f            // initial setpoint [m]
#define SP_RATE     15.0f            // setpoint slew from arrow keys [m/s]

// ---------- wind gust constants ----------
#define GUST_GAP_MIN  2.0f           // quiet time between gusts [s]
#define GUST_GAP_MAX  6.0f
#define GUST_DUR_MIN  1.0f           // gust duration [s]
#define GUST_DUR_MAX  2.0f
#define WIND_AMP_MAX  15.0f          // slider ceiling for peak gust force [N]

// ---------- layout ----------
#define SCREEN_W    1280
#define SCREEN_H    840
#define VIEW_X0     280              // flight view spans VIEW_X0..VIEW_X1
#define VIEW_X1     920
#define VIEW_H      600              // main area height; chart lives below
#define MARGIN_PX   40               // padding above 100 m and below 0 m
#define CRAFT_W_PX  30
#define CRAFT_H_PX  40

// ---------- strip chart ----------
#define CHART_N     600              // samples in the ring buffer
#define CHART_HZ    30               // sample rate -> 20 s window
#define CHART_X     10
#define CHART_Y     615
#define CHART_W     1260
#define CHART_HGT   185

// controller structure: which terms are active
typedef enum { CM_P, CM_PI, CM_PD, CM_PID } CtrlMode;
static const char *MODE_LABEL[4] = { "P", "PI", "PD", "PID" };

typedef struct SimState {
    float y;        // altitude [m], up is positive, 0 = ground
    float v;        // vertical velocity [m/s]
    float u;        // current thrust [N], set by input before each step
    float wind;     // current gust force [N], set by the gust model
    bool  landed;   // resting on the ground
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
    int head;       // next write slot
    int count;      // valid samples so far (saturates at CHART_N)
    int div;        // physics-step divider for the sample rate
} Chart;

static float frand(float lo, float hi)
{
    return lo + (hi - lo) * (float)GetRandomValue(0, 10000) / 10000.0f;
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
}

// One fixed physics step: semi-implicit Euler (velocity first, then position).
static void sim_step(SimState *s, float m, float g, float dt)
{
    float a = (s->u - m * g - DRAG * s->v + s->wind) / m;
    s->v += a * dt;
    s->y += s->v * dt;

    if (s->y <= 0.0f) {              // ground contact: clamp, kill velocity
        s->y = 0.0f;
        if (s->v < 0.0f) s->v = 0.0f;
        s->landed = (s->u <= m * g);
    } else {
        s->landed = false;
    }
}

// World-to-screen: altitude in meters (up) -> pixel row (down), and back.
static int world_to_px(float y)
{
    float t = y / WORLD_H;           // 0 at ground, 1 at 100 m
    return VIEW_H - MARGIN_PX - (int)(t * (VIEW_H - 2 * MARGIN_PX));
}

static float px_to_world(float py)
{
    float t = ((float)(VIEW_H - MARGIN_PX) - py) / (float)(VIEW_H - 2 * MARGIN_PX);
    float y = t * WORLD_H;
    if (y < 0.0f) y = 0.0f;
    if (y > WORLD_H) y = WORLD_H;
    return y;
}

// ---------- strip chart ----------

static void chart_push(Chart *c, float y, float sp, float u, float w)
{
    c->y[c->head] = y;
    c->sp[c->head] = sp;
    c->u[c->head] = u;
    c->w[c->head] = w;
    c->head = (c->head + 1) % CHART_N;
    if (c->count < CHART_N) c->count++;
}

// Altitude in meters -> pixel row inside the chart area.
static float chart_py(float val)
{
    if (val < 0.0f) val = 0.0f;
    if (val > WORLD_H) val = WORLD_H;
    return (float)(CHART_Y + CHART_HGT) - val / WORLD_H * (float)CHART_HGT;
}

static void chart_draw(const Chart *c)
{
    DrawRectangle(CHART_X, CHART_Y, CHART_W, CHART_HGT, (Color){ 16, 21, 36, 255 });
    DrawRectangleLines(CHART_X, CHART_Y, CHART_W, CHART_HGT, DARKGRAY);
    for (int m = 0; m <= 100; m += 50) {
        int py = (int)chart_py((float)m);
        DrawLine(CHART_X, py, CHART_X + CHART_W, py, Fade(DARKGRAY, 0.5f));
        DrawText(TextFormat("%d", m), CHART_X + 4, py - 10, 10, DARKGRAY);
    }
    DrawText("last 20 s", CHART_X + CHART_W - 60, CHART_Y + 4, 10, DARKGRAY);

    float dx = (float)CHART_W / (float)(CHART_N - 1);
    for (int i = 1; i < c->count; i++) {
        int i0 = (c->head - c->count + i - 1 + CHART_N) % CHART_N;
        int i1 = (i0 + 1) % CHART_N;
        float x0 = (float)CHART_X + (float)(i - 1) * dx;
        float x1 = (float)CHART_X + (float)i * dx;
        // thrust rescaled from [0,U_MAX], wind centered on the 50 m line
        DrawLineV((Vector2){ x0, chart_py(c->u[i0] / U_MAX * WORLD_H) },
                  (Vector2){ x1, chart_py(c->u[i1] / U_MAX * WORLD_H) },
                  Fade(ORANGE, 0.35f));
        DrawLineV((Vector2){ x0, chart_py(50.0f + c->w[i0] * (50.0f / WIND_AMP_MAX)) },
                  (Vector2){ x1, chart_py(50.0f + c->w[i1] * (50.0f / WIND_AMP_MAX)) },
                  Fade(PURPLE, 0.5f));
        DrawLineV((Vector2){ x0, chart_py(c->sp[i0]) },
                  (Vector2){ x1, chart_py(c->sp[i1]) }, SKYBLUE);
        DrawLineV((Vector2){ x0, chart_py(c->y[i0]) },
                  (Vector2){ x1, chart_py(c->y[i1]) }, RAYWHITE);
    }
    DrawText("altitude", CHART_X + 8,   CHART_Y + CHART_HGT - 16, 10, RAYWHITE);
    DrawText("setpoint", CHART_X + 60,  CHART_Y + CHART_HGT - 16, 10, SKYBLUE);
    DrawText("thrust",   CHART_X + 112, CHART_Y + CHART_HGT - 16, 10, Fade(ORANGE, 0.7f));
    DrawText("wind",     CHART_X + 152, CHART_Y + CHART_HGT - 16, 10, Fade(PURPLE, 0.8f));
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
    for (int i = 1; i <= 240; i++) {
        double k = 20.0 * (double)i / 240.0;
        int n = poles(m, DRAG, k, ki, kd, re, im);
        for (int j = 0; j < n; j++) {
            if (re[j] < RE_MIN || re[j] > RE_MAX || fabs(im[j]) > IM_MAX) continue;
            int px = lx + (int)((re[j] - RE_MIN) / (RE_MAX - RE_MIN) * (double)lw);
            int py = ly + (int)((IM_MAX - im[j]) / (2.0 * IM_MAX) * (double)lh);
            DrawCircle(px, py, 1.0f, Fade(LIME, 0.45f));
        }
    }

    // current poles as X marks; red once in the right half-plane
    int n = poles(m, DRAG, kp, ki, kd, re, im);
    for (int j = 0; j < n; j++) {
        if (re[j] < RE_MIN || re[j] > RE_MAX || fabs(im[j]) > IM_MAX) continue;
        int px = lx + (int)((re[j] - RE_MIN) / (RE_MAX - RE_MIN) * (double)lw);
        int py = ly + (int)((IM_MAX - im[j]) / (2.0 * IM_MAX) * (double)lh);
        Color col = (re[j] > 1e-6) ? RED : YELLOW;
        DrawLine(px - 4, py - 4, px + 4, py + 4, col);
        DrawLine(px - 4, py + 4, px + 4, py - 4, col);
    }

    DrawText("root locus: poles vs Kp 0..20 (Ki, Kd held)", lx + 6, ly + 4, 10, LIGHTGRAY);
    DrawText("s-plane, filter/sampling ignored", lx + 6, ly + 16, 10, DARKGRAY);
    DrawText("Re=0", x_zero + 3, ly + lh - 14, 10, Fade(RED, 0.8f));
}

// Signed horizontal bar for one controller term, centered at cx.
static void term_bar(const char *label, float value, int cx, int y, Color col)
{
    const float px_per_n = 4.0f;
    DrawText(label, cx - 115, y, 20, col);
    DrawLine(cx, y - 2, cx, y + 18, GRAY);
    int w = (int)(value * px_per_n);
    if (w > 110) w = 110;
    if (w < -110) w = -110;
    if (w >= 0) DrawRectangle(cx, y, w, 14, col);
    else        DrawRectangle(cx + w, y, -w, 14, col);
    DrawText(TextFormat("%+6.2f N", value), cx + 32, y, 16, col);
}

int main(void)
{
    InitWindow(SCREEN_W, SCREEN_H, "PID Lander — Phase 5: flight console");
    SetTargetFPS(60);

    SimState sim;
    sim_reset(&sim);

    // Live-tunable plant and controller parameters (sliders).
    float mass = MASS_DEF;
    float gravity = G_DEF;
    float kp = KP_DEF, ki = KI_DEF, kd = KD_DEF;
    float wind_amp = 5.0f;
    CtrlMode mode = CM_PID;

    // PID handles deviations only; feedforward carries (estimated) gravity.
    // Its limits are the actuator range left after feedforward, so the
    // anti-windup logic still sees true saturation: u_ff + out ∈ [0, U_MAX].
    PID ctrl;
    pid_init(&ctrl, kp, ki, kd, D_TAU, 0.0f, U_MAX);  // limits set each frame
    bool pid_on = false;
    float setpoint = SP_START;

    Gust gust = { 0 };
    bool wind_on = false;
    bool show_locus = false;
    bool paused = false;

    Chart chart = { 0 };

    float acc = 0.0f;                // physics time accumulator [s]
    float t_sim = 0.0f;              // simulated time [s], drives the gusts

    while (!WindowShouldClose()) {
        // ---------- input ----------
        float frame = GetFrameTime();
        if (frame > 0.25f) frame = 0.25f;   // clamp: no spiral of death

        if (IsKeyPressed(KEY_R)) { sim_reset(&sim); pid_reset(&ctrl); }
        if (IsKeyPressed(KEY_P)) {
            pid_on = !pid_on;
            if (pid_on) pid_reset(&ctrl);   // engage with clean state
        }
        if (IsKeyPressed(KEY_W)) {
            wind_on = !wind_on;
            gust.active = false;
            gust.t_next = t_sim + frand(0.5f, 2.0f);  // first gust comes soon
        }
        if (IsKeyPressed(KEY_L))     show_locus = !show_locus;
        if (IsKeyPressed(KEY_ENTER)) paused = !paused;
        if (IsKeyDown(KEY_UP))   setpoint += SP_RATE * frame;
        if (IsKeyDown(KEY_DOWN)) setpoint -= SP_RATE * frame;
        if (setpoint > WORLD_H) setpoint = WORLD_H;
        if (setpoint < 0.0f)    setpoint = 0.0f;
        bool thrusting = IsKeyDown(KEY_SPACE);

        // click/drag inside the flight view moves the setpoint
        Vector2 mp = GetMousePosition();
        if (!ui_dragging() && IsMouseButtonDown(MOUSE_BUTTON_LEFT) &&
            mp.x > VIEW_X0 && mp.x < VIEW_X1 && mp.y < VIEW_H)
            setpoint = px_to_world(mp.y);

        // apply controller structure: masked terms contribute nothing
        bool use_i = (mode == CM_PI || mode == CM_PID);
        bool use_d = (mode == CM_PD || mode == CM_PID);
        ctrl.kp = kp;
        ctrl.ki = use_i ? ki : 0.0f;
        ctrl.kd = use_d ? kd : 0.0f;
        if (!use_i) ctrl.i_term = 0.0f;

        // feedforward tracks the live plant (with its built-in 10% mass error)
        float u_ff = MASS_EST_RATIO * mass * gravity;
        if (u_ff > U_MAX) u_ff = U_MAX;
        ctrl.out_min = -u_ff;
        ctrl.out_max = U_MAX - u_ff;

        // ---------- fixed-timestep physics ----------
        if (!paused) {
            acc += frame;
            while (acc >= SIM_DT) {
                sim.wind = wind_on ? gust_force(&gust, t_sim, wind_amp) : 0.0f;
                sim.u = pid_on ? u_ff + pid_update(&ctrl, setpoint, sim.y, SIM_DT)
                               : (thrusting ? U_MAX : 0.0f);
                sim_step(&sim, mass, gravity, SIM_DT);
                t_sim += SIM_DT;
                acc -= SIM_DT;
                if (++chart.div >= 120 / CHART_HZ) {
                    chart.div = 0;
                    chart_push(&chart, sim.y, setpoint, sim.u, sim.wind);
                }
            }
        }

        // ---------- render ----------
        BeginDrawing();
        ClearBackground((Color){ 10, 14, 26, 255 });

        // ===== flight view (center) =====
        int groundPx = world_to_px(0.0f);
        DrawLine(VIEW_X0, groundPx, VIEW_X1, groundPx, GRAY);

        int spPx = world_to_px(setpoint);
        for (int x = VIEW_X0; x < VIEW_X1; x += 20)
            DrawLine(x, spPx, x + 10, spPx, pid_on ? SKYBLUE : DARKBLUE);
        DrawText(TextFormat("%.0f m", setpoint), VIEW_X1 - 55, spPx - 16, 12,
                 pid_on ? SKYBLUE : DARKBLUE);

        for (int m = 20; m <= (int)WORLD_H; m += 20) {
            int py = world_to_px((float)m);
            DrawLine(VIEW_X0, py, VIEW_X0 + 12, py, DARKGRAY);
            DrawText(TextFormat("%d m", m), VIEW_X0 + 16, py - 6, 10, DARKGRAY);
        }

        int craftBottom = world_to_px(sim.y);
        int craftX = (VIEW_X0 + VIEW_X1) / 2 - CRAFT_W_PX / 2;
        DrawRectangle(craftX, craftBottom - CRAFT_H_PX, CRAFT_W_PX, CRAFT_H_PX, RAYWHITE);
        if (sim.u > 0.0f) {
            float flame = 10.0f + 12.0f * sim.u / U_MAX;
            DrawTriangle((Vector2){ (float)craftX + 5,              (float)craftBottom },
                         (Vector2){ (float)craftX + CRAFT_W_PX - 5, (float)craftBottom },
                         (Vector2){ (float)(craftX + CRAFT_W_PX / 2), (float)craftBottom + flame },
                         ORANGE);
        }
        if (paused)
            DrawText("PAUSED", (VIEW_X0 + VIEW_X1) / 2 - 60, VIEW_H / 2 - 40, 30, YELLOW);

        // ===== telemetry panel (left) =====
        DrawRectangleLines(8, 8, 264, 584, DARKGRAY);
        DrawText("TELEMETRY", 20, 16, 16, GRAY);

        const char *status = sim.landed        ? "LANDED"
                           : (sim.v >  0.5f)   ? "CLIMBING"
                           : (sim.v < -0.5f)   ? "DESCENDING"
                           : (fabsf(setpoint - sim.y) < 0.5f && pid_on) ? "HOLDING"
                                               : "DRIFTING";
        DrawText(status, 20, 40, 30, sim.landed ? YELLOW : GREEN);

        float weight = mass * gravity;
        DrawText(TextFormat("alt    %7.2f m",   sim.y),            20,  84, 20, GREEN);
        DrawText(TextFormat("vel    %+7.2f m/s", sim.v),           20, 108, 20, GREEN);
        DrawText(TextFormat("thrust %7.2f N",   sim.u),            20, 132, 20, GREEN);
        DrawText(TextFormat("wind   %+7.2f N",  sim.wind),         20, 156, 20,
                 wind_on ? PURPLE : DARKGRAY);
        DrawText(TextFormat("err    %+7.2f m",  setpoint - sim.y), 20, 180, 20,
                 pid_on ? SKYBLUE : DARKGRAY);
        DrawText(TextFormat("TWR    %7.2f",
                 (weight > 0.01f) ? U_MAX / weight : 99.99f),      20, 204, 20,
                 (weight > U_MAX) ? RED : GREEN);
        if (weight > U_MAX)
            DrawText("weight exceeds max thrust!", 20, 228, 12, RED);

        DrawText("CONTROL EFFORT", 20, 256, 16, GRAY);
        if (pid_on) {
            term_bar("P", ctrl.p_term, 140, 280, RED);
            term_bar("I", ctrl.i_term, 140, 304, YELLOW);
            term_bar("D", ctrl.d_term, 140, 328, SKYBLUE);
            DrawText(TextFormat("FF %+6.2f N%s", u_ff,
                     ctrl.sat ? "  [SATURATED]" : ""), 25, 354, 16,
                     ctrl.sat ? ORANGE : DARKGRAY);
        } else {
            DrawText("engage PID to see terms", 25, 284, 14, DARKGRAY);
        }

        if (ui_button((Rectangle){ 16, 548, 116, 30 },
                      pid_on ? "PID: ON" : "PID: OFF", pid_on)) {
            pid_on = !pid_on;
            if (pid_on) pid_reset(&ctrl);
        }
        if (ui_button((Rectangle){ 140, 548, 116, 30 },
                      wind_on ? "WIND: ON" : "WIND: OFF", wind_on)) {
            wind_on = !wind_on;
            gust.active = false;
            gust.t_next = t_sim + frand(0.5f, 2.0f);
        }
        DrawText("click flight view to set target", 20, 522, 13, DARKGRAY);

        // ===== controller panel (right) =====
        DrawRectangleLines(928, 8, 344, 584, DARKGRAY);
        DrawText("CONTROLLER", 940, 16, 16, GRAY);

        for (int i = 0; i < 4; i++) {
            if (ui_button((Rectangle){ 940.0f + (float)i * 78.0f, 40, 74, 26 },
                          MODE_LABEL[i], mode == (CtrlMode)i)) {
                mode = (CtrlMode)i;
                pid_reset(&ctrl);
            }
        }
        if (ui_button((Rectangle){ 940, 74, 100, 26 }, "UNDER", false)) {
            kp = 8.0f; ki = 0.5f; kd = 0.8f;
        }
        if (ui_button((Rectangle){ 1046, 74, 100, 26 }, "TUNED", false)) {
            kp = KP_DEF; ki = KI_DEF; kd = KD_DEF;
        }
        if (ui_button((Rectangle){ 1152, 74, 100, 26 }, "OVER", false)) {
            kp = 1.5f; ki = 0.2f; kd = 6.0f;
        }

        kp       = ui_slider(1, 940, 112, 312, "Kp",           kp,       0.0f, 20.0f, "%.2f");
        ki       = ui_slider(2, 940, 156, 312, "Ki",           ki,       0.0f,  5.0f, "%.2f");
        kd       = ui_slider(3, 940, 200, 312, "Kd",           kd,       0.0f, 10.0f, "%.2f");
        wind_amp = ui_slider(4, 940, 244, 312, "wind peak N",  wind_amp, 0.0f, WIND_AMP_MAX, "%.1f");
        mass     = ui_slider(5, 940, 288, 312, "mass kg",      mass,     0.2f,  3.0f, "%.2f");
        gravity  = ui_slider(6, 940, 332, 312, "gravity m/s2", gravity,  0.0f, 25.0f, "%.2f");

        if (ui_button((Rectangle){ 940, 376, 160, 26 },
                      show_locus ? "ROOT LOCUS: ON" : "ROOT LOCUS: OFF", show_locus))
            show_locus = !show_locus;

        if (show_locus) {
            locus_draw(938, 412, 326, 172, mass, ctrl.kp, ctrl.ki, ctrl.kd);
        } else {
            DrawText("moon: g=1.62   mars: g=3.71", 940, 420, 14, DARKGRAY);
            DrawText("stability edge: raise Ki and/or Kp,", 940, 444, 14, DARKGRAY);
            DrawText("lower Kd, and watch the chart ring", 940, 462, 14, DARKGRAY);
        }

        chart_draw(&chart);

        DrawText("P: PID   W: wind   L: locus   ENTER: pause   UP/DOWN or click: setpoint   "
                 "SPACE: manual   R: reset",
                 CHART_X, SCREEN_H - 26, 16, LIGHTGRAY);
        DrawFPS(SCREEN_W - 90, 16);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
