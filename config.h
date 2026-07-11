// config.h — every tunable constant and layout coordinate in one place.

#ifndef CONFIG_H
#define CONFIG_H

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// ---------- simulation ----------
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
#define ACT_TAU     0.2f             // LAG: first-order time constant [s]
#define ACT_SLEW    60.0f            // SLEW: max thrust rate [N/s]

// ---------- controller ----------
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

// ---------- wind gusts ----------
#define GUST_GAP_MIN  2.0f           // quiet time between gusts [s]
#define GUST_GAP_MAX  6.0f
#define GUST_DUR_MIN  1.0f           // gust duration [s]
#define GUST_DUR_MAX  2.0f
#define WIND_AMP_MAX  50.0f          // slider ceiling for peak gust force [N]

// ---------- window layout ----------
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

// ---------- misc ----------
#define SCORE_KEEP  4                // score rows shown in the panel
#define WPART       28               // wind streak particles
#define STEP_N      240              // samples in the predicted step curve
#define STEP_T      10.0f            // predicted step horizon [s]

#endif // CONFIG_H
