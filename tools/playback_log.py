#!/usr/bin/env python3
"""Animate the ego and lead car from out/log.csv produced by adas_sim.

usage: python3 tools/playback_log.py [log.csv] [--speed X] [--save out.mp4|out.gif]
Requires matplotlib (pip install matplotlib); --save needs ffmpeg (mp4) or
pillow (gif). Without --save an interactive window is opened.

The log holds no absolute positions, so the scene is reconstructed:
  ego  : arc length s integrated from v_ego and the lane errors, lateral
         offset e_y_true and heading e_psi_true relative to the road
  lead : s = s_ego + gap_true (the lead's rear bumper, as in the simulator)
The road geometry (straight, then a left curve) mirrors src/sim/scenario.c;
adjust the constants below if the scenario changes.

Layout (all map views are true scale):
  top          chase camera that follows the ego around the road
  bottom-left  overview of the whole road, chase-camera window outlined
  bottom-right speed, bumper gap and acceleration over time, ACC mode shaded
"""

import argparse
import csv
import math
from typing import Any

import matplotlib
import numpy as np
import numpy.typing as npt

FloatArray = npt.NDArray[np.floating[Any]]

MODE_NAMES = {0: "OFF", 1: "CRUISE", 2: "FOLLOW", 3: "EBRAKE"}
MODE_COLORS = {0: "gray", 1: "tab:green", 2: "tab:blue", 3: "tab:red"}
LEAD_COLOR = "tab:orange"

# Road (scenario.c) and vehicle (config/params.h) geometry.
STRAIGHT_LEN_M = 200.0
RADIUS_M = 400.0
LANE_WIDTH_M = 3.5
VEH_FRONT_M = 3.8  # rear axle (ego reference point) -> front bumper
VEH_REAR_M = 0.9  # rear axle -> rear bumper
VEH_WIDTH_M = 1.9

# Chase camera: centered on the road this far ahead of the ego, showing
# +-HALF_LEN along the road and [Y_MIN, Y_MAX] across it.
CHASE_AHEAD_M = 35.0
CHASE_HALF_LEN_M = 55.0
CHASE_Y_MIN_M = -6.0
CHASE_Y_MAX_M = 9.0
POST_SPACING_M = 10.0


def box(x0: float, x1: float, y0: float, y1: float) -> FloatArray:
    return np.array([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])


# Outlines in each car's own frame: ego around its rear axle, lead from its
# rear bumper.
EGO_SHAPE = box(-VEH_REAR_M, VEH_FRONT_M, -VEH_WIDTH_M / 2, VEH_WIDTH_M / 2)
LEAD_SHAPE = box(0.0, VEH_FRONT_M + VEH_REAR_M, -VEH_WIDTH_M / 2, VEH_WIDTH_M / 2)
CHASE_WINDOW = box(-CHASE_HALF_LEN_M, CHASE_HALF_LEN_M, CHASE_Y_MIN_M, CHASE_Y_MAX_M)


def load(path: str) -> dict[str, FloatArray]:
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def road_pose(s: FloatArray, offset: FloatArray | float = 0.0) -> FloatArray:
    """World (x, y, heading) rows of the road points at arc length s, offset left.

    One formula covers both segments: on the straight theta is 0, so x = s
    and y = offset.
    """
    theta = np.maximum(s - STRAIGHT_LEN_M, 0.0) / RADIUS_M
    r = RADIUS_M - offset
    x = np.minimum(s, STRAIGHT_LEN_M) + r * np.sin(theta)
    return np.stack([x, RADIUS_M - r * np.cos(theta), theta], axis=-1)


def place(shape: FloatArray, pose: FloatArray) -> FloatArray:
    """Rotate shape points by pose heading, then move them to pose x, y."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    out: FloatArray = shape @ np.array([[c, s], [-s, c]]) + pose[:2]
    return out


def reconstruct(d: dict[str, FloatArray]) -> tuple[FloatArray, FloatArray]:
    """Return ego s and lead s per log row."""
    t, v, e_y, e_psi = d["t_s"], d["v_ego_mps"], d["e_y_true_m"], d["e_psi_true_rad"]
    s_ego = np.zeros_like(t)
    for i in range(1, len(t)):
        k = 1.0 / RADIUS_M if s_ego[i - 1] > STRAIGHT_LEN_M else 0.0
        s_dot = v[i - 1] * math.cos(e_psi[i - 1]) / (1.0 - k * e_y[i - 1])
        s_ego[i] = s_ego[i - 1] + s_dot * (t[i] - t[i - 1])
    return s_ego, s_ego + d["gap_true_m"]


def main() -> None:
    ap = argparse.ArgumentParser(description=(__doc__ or "").split("\n")[0])
    ap.add_argument("log", nargs="?", default="out/log.csv")
    ap.add_argument(
        "--speed", type=float, default=1.0, help="playback speed (default 1)"
    )
    ap.add_argument("--save", metavar="FILE", help="write .mp4/.gif instead of showing")
    ap.add_argument("--stride", type=int, default=1, help="use every Nth log row")
    args = ap.parse_args()

    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.artist import Artist
    from matplotlib.patches import FancyArrowPatch, Patch, Polygon
    from matplotlib.transforms import Affine2D

    d = load(args.log)
    t, mode = d["t_s"], d["mode"].astype(int)
    s_ego, s_lead = reconstruct(d)
    stride = max(1, args.stride)
    dt_ms = (t[1] - t[0]) * 1000.0 * stride
    bumper_gap = d["gap_true_m"] - VEH_FRONT_M

    ego_pose = road_pose(s_ego, d["e_y_true_m"])  # rear axle
    ego_pose[:, 2] += d["e_psi_true_rad"]
    lead_pose = road_pose(s_lead)  # rear bumper
    chase_pose = road_pose(s_ego + CHASE_AHEAD_M)

    fig = plt.figure(figsize=(13, 8.5))
    gs = fig.add_gridspec(
        4,
        2,
        height_ratios=(1.5, 1, 1, 1),
        width_ratios=(1, 1.15),
        hspace=0.45,
        wspace=0.15,
        left=0.05,
        right=0.98,
        top=0.96,
        bottom=0.06,
    )
    chase = fig.add_subplot(gs[0, :])
    hud_ax = fig.add_subplot(gs[1, 0])
    hud_ax.axis("off")
    overview = fig.add_subplot(gs[2:, 0])
    sp, gp, ac = (fig.add_subplot(gs[r, 1]) for r in (1, 2, 3))

    # --- road geometry in world coordinates ---
    ss = np.arange(0.0, s_lead.max() + 2 * CHASE_HALF_LEN_M, 1.0)
    half, shoulder = LANE_WIDTH_M / 2, LANE_WIDTH_M / 2 + 1.0
    surface = np.vstack([road_pose(ss, shoulder), road_pose(ss[::-1], -shoulder)])
    s_posts = np.arange(0.0, ss[-1], POST_SPACING_M)

    # --- chase camera: world artists drawn through a per-frame view transform ---
    view = Affine2D()
    to_view = view + chase.transData
    chase.set_facecolor("#c9dcb5")
    chase.add_patch(Polygon(surface[:, :2], fc="#5f6368", transform=to_view))
    for off in (half, -half):
        e = road_pose(ss, off)
        chase.plot(e[:, 0], e[:, 1], color="white", lw=1.6, transform=to_view)
    c = road_pose(ss)
    chase.plot(
        c[:, 0],
        c[:, 1],
        color="white",
        lw=0.6,
        alpha=0.35,
        ls=(0, (6, 6)),
        transform=to_view,
    )
    for off in (shoulder + 0.5, -shoulder - 0.5):
        p = road_pose(s_posts, off)
        chase.plot(p[:, 0], p[:, 1], "s", color="#3b3b3b", ms=3, transform=to_view)
    ego_car = Polygon(EGO_SHAPE, ec="black", transform=to_view)
    lead_car = Polygon(LEAD_SHAPE, fc=LEAD_COLOR, ec="black", transform=to_view)
    gap_arrow = FancyArrowPatch(
        (0, 0),
        (1, 0),
        arrowstyle="<->",
        mutation_scale=10,
        color="white",
        transform=to_view,
    )
    for patch in (ego_car, lead_car, gap_arrow):
        chase.add_patch(patch)
    gap_label = chase.text(
        0,
        0,
        "",
        transform=to_view,
        ha="center",
        va="center",
        color="white",
        fontsize=9,
        fontweight="bold",
        bbox={"fc": "#3c4043", "ec": "none"},
    )
    chase.set_xlim(-CHASE_HALF_LEN_M, CHASE_HALF_LEN_M)
    chase.set_ylim(CHASE_Y_MIN_M, CHASE_Y_MAX_M)
    chase.set_aspect("equal")
    chase.set_title("chase camera (true scale, road-aligned)", fontsize=10, loc="left")
    chase.set_xlabel("[m]")
    chase.tick_params(labelsize=8)
    chase.legend(
        handles=[
            *(
                Patch(fc=MODE_COLORS[m], ec="black", label=f"ego ({MODE_NAMES[m]})")
                for m in (1, 2, 3)
            ),
            Patch(fc=LEAD_COLOR, ec="black", label="lead"),
        ],
        loc="upper right",
        fontsize=8,
        ncols=4,
        framealpha=0.85,
    )

    # --- readout ---
    badge = hud_ax.text(
        0,
        0.95,
        "",
        va="top",
        color="white",
        fontsize=13,
        fontweight="bold",
        bbox={"fc": "gray", "ec": "none", "pad": 4},
    )
    hud = hud_ax.text(0, 0.55, "", va="top", family="monospace", fontsize=11)

    # --- overview of the whole road ---
    overview.add_patch(Polygon(surface[:, :2], fc="#9aa0a6"))
    overview.plot(ego_pose[:, 0], ego_pose[:, 1], color="0.75", lw=0.8)
    (trail,) = overview.plot([], [], color="tab:blue", lw=1.5, label="ego path")
    ov_ego = Polygon(EGO_SHAPE, color="tab:blue", lw=1.5)
    ov_lead = Polygon(LEAD_SHAPE, color=LEAD_COLOR, lw=1.5)
    window = Polygon(CHASE_WINDOW, fill=False, ls="--")
    for patch in (ov_ego, ov_lead, window):
        overview.add_patch(patch)
    overview.set_aspect("equal")
    overview.margins(0.04)
    overview.set_title("overview (true scale)", fontsize=10, loc="left")
    overview.set_xlabel("x [m]")
    overview.set_ylabel("y [m]")
    overview.tick_params(labelsize=8)
    overview.grid(alpha=0.3)
    overview.legend(
        handles=[trail, Patch(fill=False, ls="--", label="chase-camera window")],
        loc="upper left",
        fontsize=8,
    )

    # --- time-series strips with a moving cursor ---
    sp.plot(t, d["v_ego_mps"], color="tab:blue", label="ego")
    sp.plot(t, d["v_lead_mps"], color=LEAD_COLOR, ls="--", label="lead")
    sp.set_ylabel("speed [m/s]", fontsize=9)
    sp.set_title("ACC mode shaded in the background", fontsize=10, loc="left")
    gp.plot(t, bumper_gap, color="black", label="bumper gap")
    gp.plot(t, d["d_des_m"] - VEH_FRONT_M, color="0.4", ls=":", label="desired")
    gp.axhline(0, color="tab:red", lw=0.8)
    gp.set_ylabel("gap [m]", fontsize=9)
    ac.plot(t, d["a_des_mps2"], color="0.4", ls=":", label="commanded")
    ac.plot(t, d["a_ego_mps2"], color="tab:blue", label="ego")
    ac.set_ylabel("accel [m/s²]", fontsize=9)
    ac.set_xlabel("time [s]")
    cursors = []
    for a in (sp, gp, ac):
        for m, color in MODE_COLORS.items():
            a.fill_between(
                t,
                0,
                1,
                where=mode == m,
                step="post",
                color=color,
                alpha=0.12,
                lw=0,
                transform=a.get_xaxis_transform(),
            )
        a.set_xlim(t[0], t[-1])
        a.grid(alpha=0.3)
        a.tick_params(labelsize=8, labelbottom=a is ac)
        a.legend(loc="upper right", fontsize=7, ncols=2)
        cursors.append(a.axvline(t[0], color="black", lw=1))

    def update(frame: int) -> tuple[Artist, ...]:
        i = frame * stride
        ego, lead, cam = ego_pose[i], lead_pose[i], chase_pose[i]
        m = int(mode[i])

        # Chase camera: cam's road point at the origin, its heading along +x.
        view.clear().translate(-cam[0], -cam[1]).rotate(-cam[2])
        for car, shape, pose in (
            (ego_car, EGO_SHAPE, ego),
            (lead_car, LEAD_SHAPE, lead),
            (ov_ego, EGO_SHAPE, ego),
            (ov_lead, LEAD_SHAPE, lead),
        ):
            car.set_xy(place(shape, pose))
        ego_car.set_facecolor(MODE_COLORS.get(m, "gray"))
        window.set_xy(place(CHASE_WINDOW, cam))
        trail.set_data(ego_pose[: i + 1, 0], ego_pose[: i + 1, 1])

        front = place(np.array([[VEH_FRONT_M, 0.0]]), ego)[0]
        gap_arrow.set_positions(tuple(front), tuple(lead[:2]))
        gap_label.set_position(tuple((front + lead[:2]) / 2))
        gap_label.set_text(f"{bumper_gap[i]:.1f} m")

        badge.set_text(f" t = {t[i]:5.2f} s   ACC {MODE_NAMES.get(m, m)} ")
        badge.set_backgroundcolor(MODE_COLORS.get(m, "gray"))
        hud.set_text(
            f"ego   {d['v_ego_mps'][i]:5.1f} m/s   {d['a_ego_mps2'][i]:+5.2f} m/s²\n"
            f"lead  {d['v_lead_mps'][i]:5.1f} m/s\n"
            f"gap   {bumper_gap[i]:5.1f} m (bumper to bumper)\n"
            f"e_y   {d['e_y_true_m'][i]:+5.2f} m (lane-center offset)"
        )
        for cur in cursors:
            cur.set_xdata([t[i], t[i]])
        return ()  # blit is off, so nothing needs to be returned

    anim = FuncAnimation(
        fig, update, frames=len(range(0, len(t), stride)), interval=dt_ms / args.speed
    )

    if args.save:
        anim.save(args.save, fps=1000.0 / dt_ms * args.speed, dpi=100)
        print(f"saved {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
