#include "chart.h"

#include "raylib.h"

#include "util.h"

void chart_push(Chart *c, float y, float sp, float u, float w, float v,
                unsigned char flags)
{
    if (y <= 0.05f) flags |= CF_GROUND;
    c->y[c->head] = y;
    c->sp[c->head] = sp;
    c->u[c->head] = u;
    c->w[c->head] = w;
    c->v[c->head] = v;
    c->flag[c->head] = flags;
    c->head = (c->head + 1) % CHART_N;
    if (c->count < CHART_N) c->count++;
}

// strip geometry inside the chart panel
#define ALT_Y   (CHART_Y + 24)       // altitude strip
#define ALT_H   126
#define FRC_Y   (ALT_Y + ALT_H + 10) // force strip
#define FRC_H   52

static float alt_py(float val)       // meters -> altitude-strip row
{
    val = clampf(val, 0.0f, WORLD_H);
    return (float)(ALT_Y + ALT_H) - val / WORLD_H * (float)ALT_H;
}

static float thr_py(float u, float u_max)   // thrust N -> force-strip row
{
    return (float)(FRC_Y + FRC_H) - clampf(u / u_max, 0.0f, 1.0f) * (float)FRC_H;
}

static float wnd_py(float w)         // wind N -> force-strip row (0 centered)
{
    return (float)(FRC_Y + FRC_H / 2)
         - clampf(w / WIND_AMP_MAX, -1.0f, 1.0f) * (float)(FRC_H / 2);
}

void chart_draw(const Chart *c, float u_max)
{
    DrawRectangle(CHART_X, CHART_Y, CHART_W, CHART_HGT, COL_PLOT_BG);
    DrawRectangleLines(CHART_X, CHART_Y, CHART_W, CHART_HGT, DARKGRAY);
    DrawText("HISTORY - last 20 s, newest at right", CHART_X + 20, CHART_Y + 5, 14, GRAY);

    // altitude strip: 0..100 m
    DrawRectangleLines(CHART_X, ALT_Y, CHART_W, ALT_H, Fade(DARKGRAY, 0.9f));
    for (int m = 0; m <= 100; m += 50) {
        int py = (int)alt_py((float)m);
        DrawLine(CHART_X, py, CHART_X + CHART_W, py, Fade(DARKGRAY, 0.5f));
        DrawText(TextFormat("%dm", m), CHART_X + 5, py - 13, 12, DARKGRAY);
    }

    // force strip: thrust 0..u_max (bottom-up), wind centered at zero
    DrawRectangleLines(CHART_X, FRC_Y, CHART_W, FRC_H, Fade(DARKGRAY, 0.9f));
    int wz = FRC_Y + FRC_H / 2;
    DrawLine(CHART_X, wz, CHART_X + CHART_W, wz, Fade(DARKGRAY, 0.5f));
    DrawText("forces", CHART_X + 5, FRC_Y + 2, 12, DARKGRAY);

    float dx = (float)CHART_W / (float)(CHART_N - 1);
    float xr = (float)(CHART_X + CHART_W);      // newest sample lives here
    for (int i = 1; i < c->count; i++) {
        int i0 = (c->head - c->count + i - 1 + 2 * CHART_N) % CHART_N;
        int i1 = (i0 + 1) % CHART_N;
        float x0 = xr - (float)(c->count - i) * dx;
        float x1 = xr - (float)(c->count - 1 - i) * dx;

        DrawLineV((Vector2){ x0, thr_py(c->u[i0], u_max) },
                  (Vector2){ x1, thr_py(c->u[i1], u_max) }, Fade(ORANGE, 0.8f));
        DrawLineV((Vector2){ x0, wnd_py(c->w[i0]) },
                  (Vector2){ x1, wnd_py(c->w[i1]) }, Fade(PURPLE, 0.8f));
        DrawLineV((Vector2){ x0, alt_py(c->sp[i0]) },
                  (Vector2){ x1, alt_py(c->sp[i1]) }, SKYBLUE);
        // altitude: red while on the ground so ground time is unmistakable
        bool grounded = (c->flag[i0] & CF_GROUND) && (c->flag[i1] & CF_GROUND);
        DrawLineV((Vector2){ x0, alt_py(c->y[i0]) },
                  (Vector2){ x1, alt_py(c->y[i1]) }, grounded ? RED : RAYWHITE);
        // reset marker: the craft teleported back to the spawn altitude here
        if (c->flag[i1] & CF_RESET) {
            DrawLineV((Vector2){ x1, (float)ALT_Y },
                      (Vector2){ x1, (float)(ALT_Y + ALT_H) }, Fade(YELLOW, 0.6f));
            DrawText("R", (int)x1 - 3, ALT_Y + 2, 12, Fade(YELLOW, 0.8f));
        }
    }

    int ly = CHART_Y + CHART_HGT - 20;
    DrawText("altitude", CHART_X + 10,  ly, 14, RAYWHITE);
    DrawText("(red = on ground)", CHART_X + 400, ly, 14, Fade(RED, 0.8f));
    DrawText("setpoint", CHART_X + 80,  ly, 14, SKYBLUE);
    DrawText(TextFormat("thrust (0-%.0f N)", u_max), CHART_X + 152, ly, 14,
             Fade(ORANGE, 0.9f));
    DrawText(TextFormat("wind (+-%.0f N)", WIND_AMP_MAX), CHART_X + 290, ly, 14,
             Fade(PURPLE, 0.9f));
}
