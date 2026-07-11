// main.c — PID LANDER: a 1-D thrust-vector control sandbox.
//
// One rigid body on a vertical axis:  m*y'' = u - m*g - c*y' + w(t)
// Integrated with semi-implicit Euler at a fixed 120 Hz, decoupled from the
// render rate by an accumulator loop.
//
// Modules:
//   config.h   — every constant and layout coordinate
//   util       — clampf / frand / gauss
//   sim        — plant dynamics, actuator response, wind gusts
//   pid        — the controller (anti-windup, derivative-on-measurement,
//                filtered derivative, bumpless live tuning)
//   chart      — 20 s history strips (altitude/setpoint over thrust/wind)
//   seq        — draggable stepped target-sequence editor with playback
//   plots      — root locus / Bode + Routh verdict / phase portrait /
//                predicted step response
//   autotune   — relay experiment measuring Ku/Tu for tuning rules
//   ui         — immediate-mode sliders and buttons
//
// main.c owns the game loop: input, the fixed-timestep physics loop, the
// controller wiring (feedforward + PID through the actuator), playback
// scoring, and panel layout.
//
// Keys:  P PID on/off | W wind | ENTER pause | UP/DOWN or click view: setpoint
//        SPACE manual thrust | L cycle plot | R reset

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

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

// controller structure: which terms are active
typedef enum { CM_P, CM_PI, CM_PD, CM_PID } CtrlMode;
static const char *MODE_LABEL[4] = { "P", "PI", "PD", "PID" };

typedef enum { PLOT_OFF, PLOT_LOCUS, PLOT_BODE, PLOT_PHASE, PLOT_STEP } PlotMode;
static const char *PLOT_LABEL[5] = { "OFF", "LOCUS", "BODE", "PHASE", "STEP" };

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

// Measurement of the last real setpoint step flown by the craft.
typedef struct StepMeas {
    bool  seen;         // any step measured yet
    bool  active;       // still tracking the current step
    float to, mag, dir; // target, |step|, sign
    float t0;
    float ov;           // overshoot fraction of |step|
    float settle;       // 2% settling time [s], -1 while outside the band
} StepMeas;

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
    bool paused = true;              // hold physics until the pilot is ready
    bool launched = false;           // first unpause shows a launch banner
    unsigned char push_flags = 0;    // event flags carried into the next sample

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

    // wind streak particles (screen space, wrap vertically with the gust)
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
            push_flags |= CF_RESET;         // mark the teleport in the history
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
        // before first launch, any flight input releases the hold too
        if (paused && !launched &&
            (IsKeyPressed(KEY_SPACE) || IsKeyPressed(KEY_P)))
            paused = false;
        if (!paused) launched = true;
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
                    if (at.running)
                        autotune_cancel(&at, "AUTOTUNE aborted: crashed", t_sim);
                } else if (at.running) {
                    float u_rel;
                    AtStatus st = autotune_step(&at, setpoint - meas, sim.y,
                                                t_sim, &u_rel);
                    u_cmd = u_ff + u_rel;
                    if (st == AT_DONE) {                // apply classic Z-N PID
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
                    chart_push(&chart, sim.y, setpoint, sim.u, sim.wind, sim.v,
                               push_flags);
                    push_flags = 0;
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

        // wind streaks (vertical: the gust force acts along the flight axis)
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
        if (paused && !launched) {
            DrawText("PRESS ENTER TO LAUNCH", craftCX - 220, (VIEW_Y0 + VIEW_Y1) / 2 - 60, 34, GOLD);
            DrawText("the craft free-falls until you thrust (SPACE) or engage the PID (P)",
                     craftCX - 240, (VIEW_Y0 + VIEW_Y1) / 2 - 16, 15, LIGHTGRAY);
        } else if (paused) {
            DrawText("PAUSED", craftCX - 70, (VIEW_Y0 + VIEW_Y1) / 2 - 40, 34, YELLOW);
        }
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
                autotune_cancel(&at, "AUTOTUNE cancelled", t_sim);
            } else {
                float d = fminf(u_ff, u_max - u_ff) * 0.8f;
                if (d < 0.5f) {
                    autotune_cancel(&at,
                        "AUTOTUNE needs thrust authority both ways (check max thrust)",
                        t_sim);
                } else {
                    autotune_begin(&at, d, 0.5f + 2.0f * noise_sigma, t_sim);
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
