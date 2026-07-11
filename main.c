// main.c — PID LANDER: a 1-D thrust-vector control sandbox.
//
// PHYSICS MODEL
//   One rigid body on a vertical axis:
//       m*y'' = u - m*g - c*y' + w(t)
//   where u is thrust (one-sided: u >= 0), c is linear drag, and w(t) is the
//   wind gust force. Integrated with semi-implicit Euler at a fixed 120 Hz,
//   decoupled from the render rate by an accumulator loop.
//
// CONTROL STRUCTURE
//   The controller commands u_cmd = u_ff + PID(e), where u_ff is gravity
//   feedforward computed from the *estimated* mass (the "mass estimate x"
//   slider deliberately lets the model be wrong; the integral term absorbs
//   the error). The command then passes through a selectable actuator
//   response (INSTANT / LAG / SLEW) before becoming applied thrust, and the
//   controller measures altitude through optional Gaussian sensor noise.
//   Touching down faster than the crash-speed slider destroys the craft.
//
// MODULES
//   config.h   every constant and layout coordinate
//   util       clampf / frand / gauss helpers
//   sim        plant dynamics, actuator response, wind gusts
//   pid        the controller (anti-windup, derivative-on-measurement,
//              filtered derivative, bumpless live tuning)
//   chart      20 s history strips (altitude/setpoint over thrust/wind)
//   seq        draggable stepped target-sequence editor with playback
//   plots      root locus / Bode + Routh verdict / phase portrait /
//              predicted step response
//   autotune   relay experiment measuring Ku and Tu for tuning rules
//   ui         immediate-mode sliders and buttons
//
// THIS FILE
//   Owns the game loop, in this order every frame:
//     1. input handling (keys, click-to-set-target)
//     2. controller configuration (mode masking, feedforward, limits)
//     3. fixed-timestep physics (setpoint playback, control law, actuator,
//        plant step, measurements, playback scoring, history sampling)
//     4. rendering (title bar, flight view, both side panels, bottom row)
//
// KEYS
//   P PID on/off | W wind | ENTER pause | UP/DOWN or click view: setpoint
//   SPACE manual thrust | L cycle analysis plot | R reset
//
// SELF-TEST
//   `lander --selftest` flies a sinusoidal setpoint autonomously for 75
//   simulated seconds while cross-checking the entire history ring against
//   a shadow copy of every recorded sample (regression guard for the buffer
//   overrun once caused by the step-prediction curve).

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "raylib.h"

#include "autotune.h"
#include "chart.h"
#include "config.h"
#include "pid.h"
#include "plots.h"
#include "seq.h"
#include "sim.h"
#include "ui.h"
#include "util.h"

// ===================================================================
// types owned by main
// ===================================================================

// Controller structure: which PID terms are active. Masked terms contribute
// nothing (their gain is forced to zero and, for I, the accumulator is held
// clear) so "P" really is proportional-only, and so on.
typedef enum { CM_P, CM_PI, CM_PD, CM_PID } CtrlMode;
static const char *MODE_LABEL[4] = { "P", "PI", "PD", "PID" };

// Which analysis plot occupies the bottom-middle slot.
typedef enum { PLOT_OFF, PLOT_LOCUS, PLOT_BODE, PLOT_PHASE, PLOT_STEP } PlotMode;
static const char *PLOT_LABEL[5] = { "OFF", "LOCUS", "BODE", "PHASE", "STEP" };

// One scored playback of the target sequence, kept for comparison.
typedef struct Score {
    int   id;        // running run number
    float kp, ki, kd;
    int   mode;      // CtrlMode in effect when the run started
    float iae;       // integral of |error| dt            [m*s]
    float itae;      // integral of t*|error| dt          [m*s^2]
    float emax;      // largest |error| seen              [m]
    float effort;    // total actuator movement, sum |du| [N]
} Score;

// Measurement of the last real setpoint step the craft flew, for comparing
// against the linear prediction. Armed automatically by any setpoint jump
// of 2 m or more (mouse click or sequence playback).
typedef struct StepMeas {
    bool  seen;      // at least one step has been measured
    bool  active;    // still tracking the current step
    float to;        // target altitude of the step [m]
    float mag;       // |step size| [m]
    float dir;       // +1 up, -1 down
    float t0;        // sim time the step began [s]
    float ov;        // peak overshoot as a fraction of the step size
    float settle;    // 2% settling time [s], -1 while outside the band
} StepMeas;

// ===================================================================
// panel content layout
// (panel FRAME positions live in config.h; the row positions of the text
//  and widgets INSIDE each panel are main's concern and live here)
// ===================================================================

// -- left (telemetry) panel --
#define LP_TX          (LPAN_X + 16)   // text left margin
#define LP_ROW_TITLE   (LPAN_Y + 10)   // "TELEMETRY"
#define LP_ROW_STATUS  (LPAN_Y + 34)   // big status word
#define LP_ROW_TELEM   (LPAN_Y + 80)   // first telemetry line
#define LP_TELEM_PITCH 30              // spacing between telemetry lines
#define LP_ROW_TTW     (LPAN_Y + 234)  // thrust-to-weight line
#define LP_ROW_ALERT   (LPAN_Y + 256)  // "weight exceeds..." warning
#define LP_ROW_EFFORT  (LPAN_Y + 280)  // "CONTROL EFFORT" header
#define LP_ROW_BARS    (LPAN_Y + 304)  // first P/I/D term bar
#define LP_BAR_PITCH   28
#define LP_ROW_FF      (LPAN_Y + 392)  // feedforward readout
#define LP_ROW_SCORES  (LPAN_Y + 422)  // "TRACKING SCORES" header
#define LP_ROW_SCORE0  (LPAN_Y + 444)  // first score row
#define LP_SCORE_PITCH 36              // two text lines per score row
#define LP_ROW_BUTTONS (LPAN_Y + 592)  // PID / WIND toggle buttons
#define LP_BAR_CENTER  200             // x of the term bars' zero line

// -- right (controller) panel --
#define RP_TX          (RPAN_X + 12)   // content left margin
#define RP_TW          424             // content width (sliders span this)
#define RP_ROW_TITLE   (RPAN_Y + 10)   // "CONTROLLER"
#define RP_ROW_MODE    (RPAN_Y + 32)   // P / PI / PD / PID buttons
#define RP_ROW_PRESET  (RPAN_Y + 64)   // UNDER / TUNED / OVER buttons
#define RP_ROW_RULES   (RPAN_Y + 96)   // tuning-rule buttons (post-autotune)
#define RP_ROW_THRUST  (RPAN_Y + 126)  // actuator profile buttons
#define RP_ROW_SLIDER0 (RPAN_Y + 158)  // first slider
#define RP_SLIDER_PITCH 38             // spacing between sliders
#define RP_ROW_ACTIONS (RPAN_Y + 542)  // PLOT / AUTOTUNE buttons
#define RP_ROW_MODEL   (RPAN_Y + 580)  // model-vs-reality text block
#define RP_MODEL_PITCH 20

// -- flight view decorations --
#define STREAK_MIN_N   0.3f            // hide wind streaks below this force
#define STREAK_PX_PER_N 6.0f           // streak drift speed [px/s per N]
#define BOOM_FLASH_T   0.5f            // crash flash ring duration [s]
#define BOOM_DEBRIS_T  1.2f            // crash debris flight time [s]
#define BOOM_PIECES    10              // debris chunks thrown by a crash
#define BOOM_GRAV_PX   200.0f          // debris gravity [px/s^2]

// -- self-test --
#define ST_DURATION    75.0f           // simulated seconds to fly (~3.7 wraps)
#define ST_SP_MID      50.0f           // sinusoidal setpoint: center [m]
#define ST_SP_AMP      25.0f           // amplitude [m] (never nears ground)
#define ST_SP_RATE     0.15f           // angular rate [rad/s]
#define SH_MAX         20000           // shadow history capacity (pushes)

// ===================================================================
// helpers
// ===================================================================

// World-to-screen: altitude in meters (up) -> flight-view pixel row (down).
static int world_to_px(float y)
{
    float t = y / WORLD_H;             // 0 at ground, 1 at the 100 m ceiling
    return VIEW_Y1 - MARGIN_PX - (int)(t * (VIEW_Y1 - VIEW_Y0 - 2 * MARGIN_PX));
}

// Inverse of world_to_px, for click-to-set-target. Clamped to the world.
static float px_to_world(float py)
{
    float t = ((float)(VIEW_Y1 - MARGIN_PX) - py)
            / (float)(VIEW_Y1 - VIEW_Y0 - 2 * MARGIN_PX);
    return clampf(t * WORLD_H, 0.0f, WORLD_H);
}

// Draw one controller term as a signed horizontal bar centered on a zero
// line at x = cx: positive values extend right, negative left, with the
// numeric value printed beside it.
static void term_bar(const char *label, float value, int cx, int y, Color col)
{
    const float px_per_newton = 4.5f;  // bar scale
    const int   bar_max_px    = 140;   // clamp so huge terms stay in-panel
    const int   bar_h         = 16;

    DrawText(label, cx - 150, y, FS_LARGE, col);
    DrawLine(cx, y - 2, cx, y + 20, GRAY);             // the zero line
    int w = (int)(value * px_per_newton);
    if (w >  bar_max_px) w =  bar_max_px;
    if (w < -bar_max_px) w = -bar_max_px;
    if (w >= 0) DrawRectangle(cx, y + 2, w, bar_h, col);
    else        DrawRectangle(cx + w, y + 2, -w, bar_h, col);
    DrawText(TextFormat("%+7.2f N", value), cx + 48, y, FS_MED, col);
}

// --selftest support: compare the whole history ring against the shadow
// copy of every push. Any divergence, or any zero recorded while the craft
// was airborne, is counted and logged. Returns the mismatch count found.
static int chart_check(const Chart *c, const float *shadow,
                       const unsigned char *shadow_grounded, int shadow_n,
                       FILE *log, const char *tag, float tsim, int *phantom)
{
    int bad = 0;
    for (int i = 0; i < c->count; i++) {
        // walk oldest-to-newest with the same index arithmetic chart_draw uses
        int ring = (c->head - c->count + i + CHART_N) % CHART_N;
        float expect = shadow[shadow_n - c->count + i];
        if (c->y[ring] != expect) {
            bad++;
            if (log && bad <= 5)
                fprintf(log, "[%s t=%.2f] MISMATCH i=%d ring=%d got=%.3f want=%.3f "
                        "head=%d count=%d\n", tag, tsim, i, ring,
                        (double)c->y[ring], (double)expect, c->head, c->count);
        }
        if (c->y[ring] <= 0.05f && !shadow_grounded[shadow_n - c->count + i]) {
            (*phantom)++;
            if (log && *phantom <= 5)
                fprintf(log, "[%s t=%.2f] PHANTOM ZERO i=%d ring=%d head=%d\n",
                        tag, tsim, i, ring, c->head);
        }
    }
    return bad;
}

// Shadow history for --selftest (static: too large for the stack).
static float         sh_y[SH_MAX];   // every pushed altitude, in order
static unsigned char sh_g[SH_MAX];   // 1 if the craft was grounded then

// ===================================================================
// entry point
// ===================================================================

int main(int argc, char **argv)
{
    bool selftest = (argc > 1) && (strcmp(argv[1], "--selftest") == 0);

    InitWindow(SCREEN_W, SCREEN_H, "PID Lander");
    SetTargetFPS(selftest ? 0 : 60);   // selftest: run sim time flat out

    // ---- plant state ----
    SimState sim;
    sim_reset(&sim);

    // ---- live-tunable parameters (each is bound to a slider below) ----
    float mass           = MASS_DEF;   // true craft mass [kg]
    float gravity        = G_DEF;      // gravity [m/s^2]
    float kp = KP_DEF, ki = KI_DEF, kd = KD_DEF;
    float wind_amp       = 5.0f;       // peak gust force [N]
    float mass_est_ratio = 1.0f;       // controller's mass model / true mass
    float u_max          = UMAX_DEF;   // actuator ceiling [N]
    float noise_sigma    = 0.0f;       // sensor noise std dev [m]
    float v_crash        = VCRASH_DEF; // crash touchdown speed [m/s]
    CtrlMode   mode = CM_PID;
    ThrustProf prof = TP_INSTANT;
    PlotMode   plot = PLOT_OFF;

    // ---- controller ----
    // The PID handles deviations only; feedforward carries (estimated)
    // gravity. Its output limits are set every frame to the actuator range
    // REMAINING after feedforward, so anti-windup still sees the true
    // saturation boundary: u_ff + pid_output always lands in [0, u_max].
    PID ctrl;
    pid_init(&ctrl, kp, ki, kd, D_TAU, 0.0f, u_max);
    bool  pid_on   = false;
    float setpoint = SP_START;

    // ---- environment & session state ----
    Gust gust      = { 0 };
    bool wind_on   = false;
    bool paused    = true;             // hold physics until the pilot is ready
    bool launched  = false;            // first unpause dismisses launch banner
    unsigned char push_flags = 0;      // event flags carried to the next sample

    Chart chart = { 0 };

    // Default profile: four points spread over 20 s at varied heights.
    Seq seq = {
        .n = 4, .T = 20.0f,
        .t = { 2, 4, 6, 8, 10, 12, 14, 16, 18, 20 },
        .y = { 30, 60, 20, 70, 40, 80, 25, 55, 65, 35 },
        .playing = false, .t_play = 0.0f, .base = SP_START, .drag = -1,
    };

    // ---- playback scoring ----
    Score scores[SCORE_KEEP] = { 0 };  // newest first
    int   n_scores = 0, next_run_id = 1;
    bool  run_active = false;          // a scored playback is in progress
    Score run = { 0 };                 // accumulators for the current run
    float run_uprev = 0.0f;            // previous thrust, for effort = sum|du|

    Autotune at   = { 0 };
    StepPred pred = { 0 };             // linear prediction for current gains
    StepMeas sm   = { 0 };             // measurement of the last real step
    float sp_prev_step = SP_START;     // setpoint at the previous physics step

    // ---- wind streak particles (screen space; drift with the gust) ----
    float wpx[WPART], wpy[WPART];
    for (int i = 0; i < WPART; i++) {
        wpx[i] = frand((float)VIEW_X0 + 10, (float)VIEW_X1 - 10);
        wpy[i] = frand((float)VIEW_Y0 + 30, (float)VIEW_Y1 - 30);
    }

    float crash_t = -100.0f;           // sim time of the crash (animation clock)
    bool  prev_crashed = false;

    float acc   = 0.0f;                // physics time accumulator [s]
    float t_sim = 0.0f;                // total simulated time [s]

    // ---- selftest bookkeeping ----
    int   sh_n = 0, st_mismatch = 0, st_phantom = 0;
    FILE *stlog = NULL;
    if (selftest) {
        stlog = fopen("selftest.log", "w");
        paused = false;
        launched = true;
        pid_on = true;                 // fly the sinusoidal setpoint autonomously
    }

    while (!WindowShouldClose()) {

        // ===========================================================
        // 1. input
        // ===========================================================
        float frame = selftest ? (1.0f / 60.0f) : GetFrameTime();
        if (frame > FRAME_CLAMP) frame = FRAME_CLAMP;  // no spiral of death
        if (selftest)
            setpoint = ST_SP_MID + ST_SP_AMP * sinf(ST_SP_RATE * t_sim);

        if (IsKeyPressed(KEY_R)) {                 // full flight reset
            sim_reset(&sim);
            pid_reset(&ctrl);
            seq.playing = false;
            at.running = false;
            push_flags |= CF_RESET;                // mark the teleport in history
        }
        if (IsKeyPressed(KEY_P)) {                 // PID engage / disengage
            pid_on = !pid_on;
            if (pid_on) pid_reset(&ctrl);          // engage with clean state
        }
        if (IsKeyPressed(KEY_W)) {                 // wind on / off
            wind_on = !wind_on;
            gust.active = false;
            gust.t_next = t_sim + frand(0.5f, 2.0f);   // first gust comes soon
        }
        if (IsKeyPressed(KEY_L))     plot = (PlotMode)((plot + 1) % 5);
        if (IsKeyPressed(KEY_ENTER)) paused = !paused;
        // before the first launch, any flight input releases the hold too
        if (paused && !launched &&
            (IsKeyPressed(KEY_SPACE) || IsKeyPressed(KEY_P)))
            paused = false;
        if (!paused) launched = true;

        if (IsKeyDown(KEY_UP))   setpoint += SP_RATE * frame;
        if (IsKeyDown(KEY_DOWN)) setpoint -= SP_RATE * frame;
        setpoint = clampf(setpoint, 0.0f, WORLD_H);
        bool thrusting = IsKeyDown(KEY_SPACE);

        // click or drag inside the flight view moves the setpoint there
        // (yields to slider drags and sequence-point drags)
        Vector2 mouse = GetMousePosition();
        if (!ui_dragging() && seq.drag < 0 && IsMouseButtonDown(MOUSE_BUTTON_LEFT) &&
            mouse.x > VIEW_X0 && mouse.x < VIEW_X1 &&
            mouse.y > VIEW_Y0 && mouse.y < VIEW_Y1)
            setpoint = px_to_world(mouse.y);

        // ===========================================================
        // 2. controller configuration for this frame
        // ===========================================================

        // controller structure: masked terms contribute nothing
        bool use_i = (mode == CM_PI || mode == CM_PID);
        bool use_d = (mode == CM_PD || mode == CM_PID);
        ctrl.kp = kp;
        ctrl.ki = use_i ? ki : 0.0f;
        ctrl.kd = use_d ? kd : 0.0f;
        if (!use_i) ctrl.i_term = 0.0f;            // a disabled I holds nothing

        // gravity feedforward from the (possibly miscalculated) mass estimate
        float u_ff = mass_est_ratio * mass * gravity;
        if (u_ff > u_max) u_ff = u_max;
        ctrl.out_min = -u_ff;                      // PID range = what's left of
        ctrl.out_max = u_max - u_ff;               // the actuator after u_ff

        // model-vs-reality numbers for the current effective gains:
        // simulated linear step prediction + second-order PD approximation
        step_predict(&pred, mass, ctrl.kp, ctrl.ki, ctrl.kd, prof);
        float wn = (ctrl.kp > 0.001f) ? sqrtf(ctrl.kp / mass) : 0.0f;
        float zeta = (ctrl.kp > 0.001f)
                   ? (ctrl.kd + DRAG) / (2.0f * sqrtf(ctrl.kp * mass)) : 0.0f;

        // ===========================================================
        // 3. fixed-timestep physics
        // ===========================================================
        if (!paused) {
            acc += frame;
            while (acc >= SIM_DT) {

                // sequence playback drives the setpoint on sim time
                if (seq.playing) {
                    setpoint = seq_eval(&seq, seq.t_play);
                    seq.t_play += SIM_DT;
                    if (seq.t_play >= seq.T) seq.playing = false;
                }
                sim.wind = wind_on ? gust_force(&gust, t_sim, wind_amp) : 0.0f;

                // real-step measurement: arm on any setpoint jump >= 2 m
                if (fabsf(setpoint - sp_prev_step) >= 2.0f) {
                    sm.seen   = true;
                    sm.active = true;
                    sm.to     = setpoint;
                    sm.mag    = fabsf(setpoint - sp_prev_step);
                    sm.dir    = (setpoint > sp_prev_step) ? 1.0f : -1.0f;
                    sm.t0     = t_sim;
                    sm.ov     = 0.0f;
                    sm.settle = -1.0f;
                }
                sp_prev_step = setpoint;

                // the controller sees the sensor, not the truth
                float meas = sim.y;
                if (noise_sigma > 0.0f) meas += noise_sigma * gauss();

                // choose the thrust command for this step
                float u_cmd;
                if (sim.crashed) {
                    u_cmd = 0.0f;                  // wreckage doesn't thrust
                    if (at.running)
                        autotune_cancel(&at, "AUTOTUNE aborted: crashed", t_sim);
                } else if (at.running) {
                    // relay experiment owns the actuator while it runs
                    float u_rel;
                    AtStatus st = autotune_step(&at, setpoint - meas, sim.y,
                                                t_sim, &u_rel);
                    u_cmd = u_ff + u_rel;
                    if (st == AT_DONE) {           // apply classic Ziegler-Nichols
                        kp = clampf(0.6f * at.ku, 0.0f, KP_TOP);
                        ki = clampf(1.2f * at.ku / at.tu, 0.0f, KI_TOP);
                        kd = clampf(0.075f * at.ku * at.tu, 0.0f, KD_TOP);
                        mode = CM_PID;
                        pid_reset(&ctrl);
                        pid_on = true;
                    }
                } else if (pid_on) {
                    u_cmd = u_ff + pid_update(&ctrl, setpoint, meas, SIM_DT);
                } else {
                    u_cmd = thrusting ? u_max : 0.0f;   // manual: all or nothing
                }

                // actuator response, then one plant step
                sim.u = actuate(sim.u, u_cmd, prof, SIM_DT);
                sim_step(&sim, mass, gravity, v_crash, SIM_DT);
                if (sim.crashed && !prev_crashed) crash_t = t_sim;
                prev_crashed = sim.crashed;

                // update the real-step measurement against TRUE altitude
                if (sm.active) {
                    float over = (sim.y - sm.to) * sm.dir / sm.mag;
                    if (over > sm.ov) sm.ov = over;
                    if (fabsf(sim.y - sm.to) > 0.02f * sm.mag) sm.settle = -1.0f;
                    else if (sm.settle < 0.0f) sm.settle = t_sim - sm.t0;
                    if (t_sim - sm.t0 > 20.0f) sm.active = false;  // freeze result
                }

                // score the playback against TRUE altitude
                if (run_active && seq.playing) {
                    float ae = fabsf(setpoint - sim.y);
                    run.iae  += ae * SIM_DT;
                    run.itae += seq.t_play * ae * SIM_DT;
                    if (ae > run.emax) run.emax = ae;
                    run.effort += fabsf(sim.u - run_uprev);
                    run_uprev = sim.u;
                }

                t_sim += SIM_DT;
                acc   -= SIM_DT;

                // sample the history ring at CHART_HZ
                if (++chart.div >= PHYS_HZ / CHART_HZ) {
                    chart.div = 0;
                    chart_push(&chart, sim.y, setpoint, sim.u, sim.wind, sim.v,
                               push_flags);
                    push_flags = 0;
                    if (selftest && sh_n < SH_MAX) {
                        sh_y[sh_n] = sim.y;
                        sh_g[sh_n] = (sim.y <= 0.05f);
                        sh_n++;
                    }
                }
            }
        }

        // selftest checkpoint A: ring must match the shadow after physics
        if (selftest && sh_n > 0)
            st_mismatch += chart_check(&chart, sh_y, sh_g, sh_n, stlog,
                                       "post-physics", t_sim, &st_phantom);

        // ===========================================================
        // 4. render
        // ===========================================================
        BeginDrawing();
        ClearBackground(COL_WINDOW_BG);

        // ---- title bar ----
        DrawRectangle(0, 0, SCREEN_W, TITLE_H, COL_TITLE_BG);
        DrawLine(0, TITLE_H, SCREEN_W, TITLE_H, DARKGRAY);
        DrawText("PID LANDER", 16, 9, FS_TITLE, GOLD);
        DrawText("tune the controller to hold target altitude against gravity, wind,"
                 " noise and actuator limits - then fly the target sequence and beat"
                 " your tracking scores", 230, 16, 15, LIGHTGRAY);
        DrawFPS(SCREEN_W - 95, 14);

        // ---- flight view: ground, setpoint line, altitude ruler ----
        int ground_px = world_to_px(0.0f);
        DrawLine(VIEW_X0, ground_px, VIEW_X1, ground_px, GRAY);

        int sp_px = world_to_px(setpoint);
        for (int x = VIEW_X0; x < VIEW_X1; x += 20)          // dashed target
            DrawLine(x, sp_px, x + 10, sp_px, pid_on ? SKYBLUE : DARKBLUE);
        DrawText(TextFormat("%.0f m", setpoint), VIEW_X1 - 60, sp_px - 18,
                 FS_SMALL, pid_on ? SKYBLUE : DARKBLUE);

        for (int m = 20; m <= (int)WORLD_H; m += 20) {       // ruler ticks
            int py = world_to_px((float)m);
            DrawLine(VIEW_X0, py, VIEW_X0 + 12, py, DARKGRAY);
            DrawText(TextFormat("%d m", m), VIEW_X0 + 16, py - 7, FS_TINY, DARKGRAY);
        }

        // ---- wind streaks (vertical: the gust acts along the flight axis;
        //      speed, length and opacity all scale with the gust force) ----
        if (wind_on && fabsf(sim.wind) > STREAK_MIN_N) {
            float alpha = clampf(0.15f + fabsf(sim.wind) / 15.0f, 0.15f, 0.8f); // opacity
            float len   = clampf(8.0f + fabsf(sim.wind) * 1.2f, 8.0f, 70.0f); // streak length
            float dir   = (sim.wind > 0.0f) ? 1.0f : -1.0f; // streak direction
            for (int i = 0; i < WPART; i++) {
                wpy[i] -= sim.wind * STREAK_PX_PER_N * frame;
                if (wpy[i] > (float)VIEW_Y1) wpy[i] -= (float)(VIEW_Y1 - VIEW_Y0);
                if (wpy[i] < (float)VIEW_Y0) wpy[i] += (float)(VIEW_Y1 - VIEW_Y0);
                DrawLineV((Vector2){ wpx[i], wpy[i] },
                          (Vector2){ wpx[i], wpy[i] + dir * len },
                          Fade(PURPLE, alpha));
            }
        }

        // ---- the craft (or its wreck and explosion) ----
        int craft_bottom = world_to_px(sim.y);
        int craft_cx     = (VIEW_X0 + VIEW_X1) / 2;
        int craft_x      = craft_cx - CRAFT_W_PX / 2;
        if (sim.crashed) {
            float ct = t_sim - crash_t;                      // time since impact
            // flattened hull plus two scattered tilted fragments
            DrawRectangle(craft_cx - 22, ground_px - 10, 44, 10, COL_WRECK);
            DrawRectanglePro((Rectangle){ (float)craft_cx - 26, (float)ground_px - 6, 18, 8 },
                             (Vector2){ 0, 0 }, -24.0f, COL_WRECK_DARK);
            DrawRectanglePro((Rectangle){ (float)craft_cx + 12, (float)ground_px - 4, 16, 7 },
                             (Vector2){ 0, 0 }, 18.0f, COL_WRECK_DARK);
            // expanding flash ring, then ballistic debris chunks
            if (ct < BOOM_FLASH_T)
                DrawCircleLines(craft_cx, ground_px - 6, 12.0f + 220.0f * ct,
                                Fade(ORANGE, 1.0f - ct / BOOM_FLASH_T));
            if (ct < BOOM_DEBRIS_T) {
                for (int k = 0; k < BOOM_PIECES; k++) {
                    // launch angles fan across the upper half-plane; speeds
                    // vary per piece via a small deterministic hash
                    float ang = PI * (0.12f + 0.76f * (float)k / (BOOM_PIECES - 1));
                    float spd = 130.0f + (float)((k * 47) % 90);
                    float px  = (float)craft_cx + cosf(ang) * spd * ct;
                    float py  = (float)ground_px
                              - (sinf(ang) * spd * ct - BOOM_GRAV_PX * ct * ct);
                    if (py < (float)ground_px)
                        DrawRectangle((int)px - 3, (int)py - 3, 6, 6,
                                      Fade((k % 2) ? ORANGE : RED,
                                           1.0f - ct / BOOM_DEBRIS_T));
                }
            }
        } else {
            DrawRectangle(craft_x, craft_bottom - CRAFT_H_PX,
                          CRAFT_W_PX, CRAFT_H_PX, RAYWHITE);
            if (sim.u > 0.0f) {                              // flame ∝ thrust
                float flame = 10.0f + 14.0f * sim.u / u_max;
                DrawTriangle((Vector2){ (float)craft_x + 5, (float)craft_bottom },
                             (Vector2){ (float)craft_x + CRAFT_W_PX - 5, (float)craft_bottom },
                             (Vector2){ (float)craft_cx, (float)craft_bottom + flame },
                             ORANGE);
            }
        }

        // ---- touchdown report: how soft was the landing, at the site ----
        if (sim.grounded && sim.impact >= 0.0f) {
            const char *lt = sim.crashed
                ? TextFormat("CRASHED - %.1f m/s impact", sim.impact)
                : TextFormat("LANDED - %.1f m/s impact", sim.impact);
            Color lc = sim.crashed                    ? RED      // destroyed
                     : (sim.impact <= 0.5f * v_crash) ? GREEN    // soft
                                                      : ORANGE;  // firm
            DrawText(lt, craft_cx - MeasureText(lt, FS_MED) / 2,
                     ground_px + 10, FS_MED, lc);
        }

        // ---- overlays: launch gate / pause / autotune progress / crash ----
        if (paused && !launched) {
            DrawText("PRESS ENTER TO LAUNCH", craft_cx - 220,
                     (VIEW_Y0 + VIEW_Y1) / 2 - 60, FS_HUGE, GOLD);
            DrawText("the craft free-falls until you thrust (SPACE) or engage the PID (P)",
                     craft_cx - 240, (VIEW_Y0 + VIEW_Y1) / 2 - 16, 15, LIGHTGRAY);
        } else if (paused) {
            DrawText("PAUSED", craft_cx - 70, (VIEW_Y0 + VIEW_Y1) / 2 - 40,
                     FS_HUGE, YELLOW);
        }
        if (at.running)
            DrawText(TextFormat("AUTOTUNING: relay cycle %d, measuring %d/3 periods",
                     at.ups, at.np), VIEW_X0 + 40, VIEW_Y0 + 14, 17, YELLOW);
        else if (at.result[0] && t_sim - at.result_t < 8.0f)
            DrawText(at.result, VIEW_X0 + 40, VIEW_Y0 + 14, 15, GREEN);
        if (sim.crashed)
            DrawText(TextFormat("CRASHED at %.1f m/s - press R to reset", sim.impact),
                     VIEW_X0 + 40, VIEW_Y0 + 44, 20, RED);

        // ---- left panel: telemetry ----
        DrawRectangleLines(LPAN_X, LPAN_Y, LPAN_W, LPAN_H, DARKGRAY);
        DrawText("TELEMETRY", LP_TX, LP_ROW_TITLE, FS_BODY, GRAY);

        const char *status = sim.crashed       ? "CRASHED"
                           : sim.landed        ? "LANDED"
                           : at.running        ? "AUTOTUNING"
                           : (sim.v >  0.5f)   ? "CLIMBING"
                           : (sim.v < -0.5f)   ? "DESCENDING"
                           : (fabsf(setpoint - sim.y) < 0.5f && pid_on) ? "HOLDING"
                                               : "DRIFTING";
        DrawText(status, LP_TX, LP_ROW_STATUS, FS_HUGE,
                 sim.crashed ? RED : sim.landed ? YELLOW : GREEN);

        float weight = mass * gravity;                       // craft weight [N]
        int row = LP_ROW_TELEM;
        DrawText(TextFormat("altitude  %8.2f m",   sim.y), LP_TX, row, FS_LARGE, GREEN);
        row += LP_TELEM_PITCH;
        DrawText(TextFormat("velocity  %+8.2f m/s", sim.v), LP_TX, row, FS_LARGE, GREEN);
        row += LP_TELEM_PITCH;
        DrawText(TextFormat("thrust    %8.2f N",   sim.u), LP_TX, row, FS_LARGE, GREEN);
        row += LP_TELEM_PITCH;
        DrawText(TextFormat("wind      %+8.2f N",  sim.wind), LP_TX, row, FS_LARGE,
                 wind_on ? PURPLE : DARKGRAY);
        row += LP_TELEM_PITCH;
        DrawText(TextFormat("error     %+8.2f m",  setpoint - sim.y), LP_TX, row,
                 FS_LARGE, pid_on ? SKYBLUE : DARKGRAY);
        DrawText(TextFormat("thrust-to-weight  %5.2f",
                 (weight > 0.01f) ? u_max / weight : 99.99f), LP_TX, LP_ROW_TTW,
                 FS_MED, (weight > u_max) ? RED : GREEN);
        if (weight > u_max)                                  // hover impossible
            DrawText("weight exceeds max thrust!", LP_TX, LP_ROW_ALERT, FS_SMALL, RED);

        // live P/I/D contributions plus the constant feedforward
        DrawText("CONTROL EFFORT", LP_TX, LP_ROW_EFFORT, FS_BODY, GRAY);
        if (pid_on) {
            row = LP_ROW_BARS;
            term_bar("P", ctrl.p_term, LP_BAR_CENTER, row, RED);
            row += LP_BAR_PITCH;
            term_bar("I", ctrl.i_term, LP_BAR_CENTER, row, YELLOW);
            row += LP_BAR_PITCH;
            term_bar("D", ctrl.d_term, LP_BAR_CENTER, row, SKYBLUE);
            DrawText(TextFormat("feedforward %+7.2f N", u_ff), LP_TX + 6,
                     LP_ROW_FF, FS_BODY, DARKGRAY);
            if (ctrl.sat)                                    // actuator pinned
                DrawText("[SATURATED]", LP_TX + 226, LP_ROW_FF, FS_BODY, ORANGE);
        } else {
            DrawText("engage PID to see terms", LP_TX + 6, LP_ROW_BARS + 4,
                     FS_BODY, DARKGRAY);
        }

        // per-playback tracking scores, newest first
        DrawText("TRACKING SCORES (per playback)", LP_TX, LP_ROW_SCORES, FS_BODY, GRAY);
        if (n_scores == 0)
            DrawText("play the target sequence to record a score",
                     LP_TX, LP_ROW_SCORE0, FS_SMALL, DARKGRAY);
        for (int i = 0; i < n_scores; i++) {
            int yy = LP_ROW_SCORE0 + i * LP_SCORE_PITCH;
            DrawText(TextFormat("#%d %s  Kp %.1f  Ki %.2f  Kd %.1f",
                     scores[i].id, MODE_LABEL[scores[i].mode],
                     scores[i].kp, scores[i].ki, scores[i].kd),
                     LP_TX, yy, FS_SMALL, LIGHTGRAY);
            DrawText(TextFormat("IAE %.1f  ITAE %.0f  max %.1fm  effort %.0fN",
                     scores[i].iae, scores[i].itae, scores[i].emax, scores[i].effort),
                     LP_TX + 12, yy + 16, FS_SMALL, SKYBLUE);
        }

        if (ui_button((Rectangle){ LP_TX, LP_ROW_BUTTONS, 155, 34 },
                      pid_on ? "PID: ON" : "PID: OFF", pid_on)) {
            pid_on = !pid_on;
            if (pid_on) pid_reset(&ctrl);
        }
        if (ui_button((Rectangle){ LP_TX + 168, LP_ROW_BUTTONS, 155, 34 },
                      wind_on ? "WIND: ON" : "WIND: OFF", wind_on)) {
            wind_on = !wind_on;
            gust.active = false;
            gust.t_next = t_sim + frand(0.5f, 2.0f);
        }

        // ---- right panel: controller ----
        DrawRectangleLines(RPAN_X, RPAN_Y, RPAN_W, RPAN_H, DARKGRAY);
        DrawText("CONTROLLER", RP_TX, RP_ROW_TITLE, FS_BODY, GRAY);

        // controller structure buttons (P / PI / PD / PID)
        for (int i = 0; i < 4; i++) {
            if (ui_button((Rectangle){ (float)RP_TX + (float)i * 109.0f,
                                       RP_ROW_MODE, 104, 26 },
                          MODE_LABEL[i], mode == (CtrlMode)i)) {
                mode = (CtrlMode)i;
                pid_reset(&ctrl);
            }
        }
        // damping presets
        if (ui_button((Rectangle){ (float)RP_TX, RP_ROW_PRESET, 140, 26 },
                      "UNDER", false)) {
            kp = 8.0f; ki = 0.5f; kd = 0.8f;       // zeta ~ 0.17: rings hard
        }
        if (ui_button((Rectangle){ (float)RP_TX + 146, RP_ROW_PRESET, 140, 26 },
                      "TUNED", false)) {
            kp = KP_DEF; ki = KI_DEF; kd = KD_DEF; // zeta ~ 1: the baseline
        }
        if (ui_button((Rectangle){ (float)RP_TX + 292, RP_ROW_PRESET, 140, 26 },
                      "OVER", false)) {
            kp = 1.5f; ki = 0.2f; kd = 6.0f;       // zeta ~ 2.5: sluggish
        }

        // tuning-rule presets computed from the measured Ku/Tu; these only
        // exist once AUTOTUNE has run a successful relay experiment
        if (at.ku > 0.0f) {
            if (ui_button((Rectangle){ (float)RP_TX, RP_ROW_RULES, 140, 24 },
                          "Z-N PID", false)) {
                kp = clampf(0.6f * at.ku, 0.0f, KP_TOP);
                ki = clampf(1.2f * at.ku / at.tu, 0.0f, KI_TOP);
                kd = clampf(0.075f * at.ku * at.tu, 0.0f, KD_TOP);
                mode = CM_PID;
            }
            if (ui_button((Rectangle){ (float)RP_TX + 146, RP_ROW_RULES, 140, 24 },
                          "Z-N PI", false)) {
                kp = clampf(0.45f * at.ku, 0.0f, KP_TOP);
                ki = clampf(0.54f * at.ku / at.tu, 0.0f, KI_TOP);
                kd = 0.0f;
                mode = CM_PI;
            }
            if (ui_button((Rectangle){ (float)RP_TX + 292, RP_ROW_RULES, 140, 24 },
                          "NO OVERSHOOT", false)) {   // Tyreus-Luyben rule
                kp = clampf(at.ku / 2.2f, 0.0f, KP_TOP);
                ki = clampf(at.ku / (2.2f * 2.2f * at.tu), 0.0f, KI_TOP);
                kd = clampf((at.ku / 2.2f) * at.tu / 6.3f, 0.0f, KD_TOP);
                mode = CM_PID;
            }
        } else {
            DrawText("run AUTOTUNE to unlock tuning-rule presets (Z-N, no-overshoot)",
                     RP_TX, RP_ROW_RULES + 4, 13, DARKGRAY);
        }

        // actuator response selection
        DrawText("thrust:", RP_TX, RP_ROW_THRUST + 4, 15, LIGHTGRAY);
        for (int i = 0; i < 3; i++) {
            if (ui_button((Rectangle){ (float)RP_TX + 70 + (float)i * 122.0f,
                                       RP_ROW_THRUST, 116, 24 },
                          PROF_LABEL[i], prof == (ThrustProf)i))
                prof = (ThrustProf)i;
        }

        // parameter sliders, one per row
        int sy = RP_ROW_SLIDER0;
        kp       = ui_slider(1, RP_TX, sy, RP_TW, "Kp",           kp,       0.0f, KP_TOP, "%.2f");
        sy += RP_SLIDER_PITCH;
        ki       = ui_slider(2, RP_TX, sy, RP_TW, "Ki",           ki,       0.0f, KI_TOP, "%.2f");
        sy += RP_SLIDER_PITCH;
        kd       = ui_slider(3, RP_TX, sy, RP_TW, "Kd",           kd,       0.0f, KD_TOP, "%.2f");
        sy += RP_SLIDER_PITCH;
        wind_amp = ui_slider(4, RP_TX, sy, RP_TW, "wind peak N",  wind_amp, 0.0f, WIND_AMP_MAX, "%.1f");
        sy += RP_SLIDER_PITCH;
        mass     = ui_slider(5, RP_TX, sy, RP_TW, "mass kg",      mass,     0.2f, 3.0f, "%.2f");
        sy += RP_SLIDER_PITCH;
        gravity  = ui_slider(6, RP_TX, sy, RP_TW, "gravity m/s2", gravity,  0.0f, 25.0f, "%.2f");
        sy += RP_SLIDER_PITCH;
        mass_est_ratio = ui_slider(7, RP_TX, sy, RP_TW, "mass estimate x",
                                   mass_est_ratio, 0.5f, 1.5f, "%.2f");
        sy += RP_SLIDER_PITCH;
        u_max    = ui_slider(8, RP_TX, sy, RP_TW, "max thrust N", u_max,    5.0f, UMAX_TOP, "%.0f");
        sy += RP_SLIDER_PITCH;
        noise_sigma = ui_slider(9, RP_TX, sy, RP_TW, "sensor noise m",
                                noise_sigma, 0.0f, NOISE_TOP, "%.2f");
        sy += RP_SLIDER_PITCH;
        v_crash  = ui_slider(10, RP_TX, sy, RP_TW, "crash speed m/s",
                             v_crash, 1.0f, 20.0f, "%.1f");

        // plot cycler and the autotune trigger
        if (ui_button((Rectangle){ (float)RP_TX, RP_ROW_ACTIONS, 200, 28 },
                      TextFormat("PLOT: %s", PLOT_LABEL[plot]), plot != PLOT_OFF))
            plot = (PlotMode)((plot + 1) % 5);
        if (ui_button((Rectangle){ (float)RP_TX + 212, RP_ROW_ACTIONS, 200, 28 },
                      "AUTOTUNE", at.running)) {
            if (at.running) {
                autotune_cancel(&at, "AUTOTUNE cancelled", t_sim);
            } else {
                // relay amplitude: 80% of whichever direction has less
                // authority, so the experiment can push both ways
                float d = fminf(u_ff, u_max - u_ff) * 0.8f;
                if (d < 0.5f) {
                    autotune_cancel(&at,
                        "AUTOTUNE needs thrust authority both ways (check max thrust)",
                        t_sim);
                } else {
                    // hysteresis widens with sensor noise to avoid chatter
                    autotune_begin(&at, d, 0.5f + 2.0f * noise_sigma, t_sim);
                    seq.playing = false;
                    pid_on = false;                // relay takes over
                }
            }
        }

        // model vs reality: what the theory promises vs what the craft did
        row = RP_ROW_MODEL;
        DrawText(TextFormat("model: zeta %.2f   wn %.2f rad/s   (PD approximation)",
                 zeta, wn), RP_TX, row, FS_BODY, LIGHTGRAY);
        row += RP_MODEL_PITCH;
        DrawText(pred.diverged
                 ? "predicted step: UNSTABLE"
                 : TextFormat("predicted step: overshoot %.1f%%  settle %s",
                              pred.ov_pct,
                              pred.settle < 0.0f ? "> 10 s"
                                                 : TextFormat("%.1f s", pred.settle)),
                 RP_TX, row, FS_BODY, pred.diverged ? RED : SKYBLUE);
        row += RP_MODEL_PITCH;
        DrawText(!sm.seen
                 ? "last real step: none yet (click a new target or PLAY)"
                 : TextFormat("last real step: overshoot %.1f%%  settle %s%s",
                              sm.ov * 100.0f,
                              sm.settle < 0.0f ? "--" : TextFormat("%.1f s", sm.settle),
                              sm.active ? "  (live)" : ""),
                 RP_TX, row, FS_BODY, GOLD);

        // ---- bottom row: analysis plot, history chart, sequence editor ----
        switch (plot) {
        case PLOT_LOCUS:
            locus_draw(PLOT_X, PLOT_Y, PLOT_W, PLOT_HGT,
                       mass, ctrl.kp, ctrl.ki, ctrl.kd);
            break;
        case PLOT_BODE:
            bode_draw(PLOT_X, PLOT_Y, PLOT_W, PLOT_HGT,
                      mass, ctrl.kp, ctrl.ki, ctrl.kd, prof);
            break;
        case PLOT_PHASE:
            phase_draw(&chart, PLOT_X, PLOT_Y, PLOT_W, PLOT_HGT);
            break;
        case PLOT_STEP:
            step_draw(&pred, PLOT_X, PLOT_Y, PLOT_W, PLOT_HGT);
            break;
        default:                                   // PLOT_OFF: explain the modes
            DrawRectangleLines(PLOT_X, PLOT_Y, PLOT_W, PLOT_HGT, DARKGRAY);
            DrawText("ANALYSIS PLOT (press L or the PLOT button)", PLOT_X + 12, PLOT_Y + 10, FS_SMALL, GRAY);
            DrawText("LOCUS - closed-loop poles vs Kp",      PLOT_X + 12, PLOT_Y + 44, FS_SMALL, DARKGRAY);
            DrawText("BODE  - margins + stability verdict",  PLOT_X + 12, PLOT_Y + 66, FS_SMALL, DARKGRAY);
            DrawText("PHASE - error/velocity trajectory",    PLOT_X + 12, PLOT_Y + 88, FS_SMALL, DARKGRAY);
            DrawText("STEP  - predicted response for gains", PLOT_X + 12, PLOT_Y + 110, FS_SMALL, DARKGRAY);
            DrawText("moon: g=1.62   mars: g=3.71",          PLOT_X + 12, PLOT_Y + 144, FS_SMALL, DARKGRAY);
            DrawText("stability edge: raise Ki, lower Kd,",  PLOT_X + 12, PLOT_Y + 176, FS_SMALL, DARKGRAY);
            DrawText("or add LAG thrust + noise",            PLOT_X + 12, PLOT_Y + 194, FS_SMALL, DARKGRAY);
            break;
        }

        chart_draw(&chart, u_max);
        seq_panel(&seq, setpoint);

        // playback scoring lifecycle: arm when PLAY starts, record when the
        // run finishes or is stopped (very short runs are treated as slips)
        if (seq.playing && !run_active) {
            run_active = true;
            run = (Score){ 0 };
            run.id = next_run_id;
            run.kp = kp; run.ki = ki; run.kd = kd;
            run.mode = (int)mode;
            run_uprev = sim.u;
        } else if (!seq.playing && run_active) {
            run_active = false;
            if (seq.t_play > 1.0f) {
                for (int i = SCORE_KEEP - 1; i > 0; i--) scores[i] = scores[i - 1];
                scores[0] = run;
                if (n_scores < SCORE_KEEP) n_scores++;
                next_run_id++;
            }
        }

        DrawText("P: PID   W: wind   L: plots   ENTER: pause   UP/DOWN or click view: setpoint   "
                 "SPACE: manual thrust   R: reset",
                 CHART_X + 2, SCREEN_H - 30, 17, LIGHTGRAY);

        // selftest checkpoint B: ring must still match after all rendering
        if (selftest && sh_n > 0)
            st_mismatch += chart_check(&chart, sh_y, sh_g, sh_n, stlog,
                                       "post-render", t_sim, &st_phantom);

        EndDrawing();

        if (selftest && t_sim > ST_DURATION) break;
    }

    if (selftest) {
        printf("SELFTEST: pushes=%d mismatches=%d phantom_zeros=%d\n",
               sh_n, st_mismatch, st_phantom);
        if (stlog) {
            fprintf(stlog, "SELFTEST: pushes=%d mismatches=%d phantom_zeros=%d\n",
                    sh_n, st_mismatch, st_phantom);
            fclose(stlog);
        }
    }

    CloseWindow();
    return 0;
}
