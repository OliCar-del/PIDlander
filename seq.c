#include "seq.h"

#include <math.h>

#include "raylib.h"

#include "ui.h"
#include "util.h"

#define SEQ_HIT_R2 121.0f   // squared grab radius for a point (11 px circle)

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

float seq_eval(const Seq *q, float t)
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

void seq_panel(Seq *q, float live_setpoint)
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
    DrawRectangle(SEQ_GX, SEQ_GY, SEQ_GW, SEQ_GH, COL_PLOT_BG);
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
            if (dxp * dxp + dyp * dyp < SEQ_HIT_R2) { q->drag = i; break; }
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
