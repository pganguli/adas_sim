#!/usr/bin/env python3
"""Plot out/log.csv produced by adas_sim.

usage: python3 tools/plot_log.py [log.csv] [out.pdf]
Requires matplotlib (pip install matplotlib).
"""

import csv
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

MODE_NAMES = {0: "OFF", 1: "CRUISE", 2: "FOLLOW", 3: "EBRAKE"}


def load(path: str) -> dict[str, list[float]]:
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return {k: [float(r[k]) for r in rows] for k in rows[0]}


def main() -> None:
    log_path = sys.argv[1] if len(sys.argv) > 1 else "out/log.csv"
    pdf_path = sys.argv[2] if len(sys.argv) > 2 else "out/plot.pdf"
    d = load(log_path)
    t = d["t_s"]

    fig, ax = plt.subplots(6, 1, figsize=(10, 14), sharex=True)

    ax[0].plot(t, d["v_ego_mps"], label="ego v")
    ax[0].plot(t, d["v_lead_mps"], "--", label="lead v (truth)")
    ax[0].set_ylabel("speed [m/s]")

    gap_est = [
        g if v > 0 else float("nan") for g, v in zip(d["gap_est_m"], d["lead_valid"])
    ]
    ax[1].plot(t, d["gap_true_m"], label="gap truth")
    ax[1].plot(t, gap_est, ".", ms=2, label="gap fused estimate")
    ax[1].plot(t, d["d_des_m"], ":", label="desired gap D0 + T*v")
    ax[1].set_ylabel("gap [m]")
    ax[1].set_ylim(0, 90)

    ax[2].plot(t, d["a_des_mps2"], label="a_des (planner)")
    ax[2].plot(t, d["a_ego_mps2"], label="a (plant)")
    ax[2].set_ylabel("accel [m/s^2]")

    ax[3].plot(t, d["e_y_true_m"], label="e_y truth")
    ax[3].plot(t, d["e_y_est_m"], ".", ms=2, label="e_y estimate")
    ax[3].set_ylabel("lateral offset [m]")

    ax[4].plot(t, d["kappa_true_1pm"], label="kappa truth")
    ax[4].plot(t, d["kappa_est_1pm"], ".", ms=2, label="kappa estimate")
    ax[4].plot(t, d["steer_act_rad"], label="steer angle [rad]")
    ax[4].set_ylabel("curvature / steer")

    ax[5].step(t, d["mode"], where="post")
    ax[5].set_yticks(list(MODE_NAMES))
    ax[5].set_yticklabels(list(MODE_NAMES.values()))
    ax[5].set_ylabel("ACC mode")
    ax[5].set_xlabel("time [s]")

    for a in ax:
        a.grid(alpha=0.3)
        if a.get_legend_handles_labels()[0]:
            a.legend(loc="upper right", fontsize=8)

    fig.tight_layout()
    fig.savefig(pdf_path)
    print(f"saved {pdf_path}")


if __name__ == "__main__":
    main()
