"""오프라인 확인용: gates.yaml 로 min-snap 을 풀어 matplotlib 로 그린다 (ROS 불필요).

    python3 -m adr_planning.plot_trajectory --gates ../adr_bringup/config/gates.yaml --laps 2
"""
import argparse

import numpy as np

from adr_planning.course import course_waypoints, load_course
from adr_planning.min_snap import plan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gates', required=True)
    ap.add_argument('--laps', type=int, default=1)
    ap.add_argument('--approach', type=float, default=0.8)
    ap.add_argument('--v-avg', type=float, default=3.0)
    ap.add_argument('--v-max', type=float, default=6.0)
    ap.add_argument('--a-max', type=float, default=8.0)
    ap.add_argument('--save', default=None, help='png 저장 경로 (없으면 화면 표시)')
    args = ap.parse_args()

    course = load_course(args.gates)
    wp, yaw = course_waypoints(course, args.laps, args.approach)
    tr = plan(wp, yaw, v_avg=args.v_avg, v_max=args.v_max, a_max=args.a_max)
    s = tr.sample_all(0.02)
    v_pk, a_pk = tr.peak()
    print(f'T={tr.duration:.2f}s  v_peak={v_pk:.2f}  a_peak={a_pk:.2f}')

    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(12, 5))
    ax[0].plot(s[:, 1], s[:, 2], label='min-snap')
    ax[0].plot(wp[:, 0], wp[:, 1], 'k.', label='waypoints')
    half = course.inner_size / 2
    for g in course.gates:
        t = np.array([-np.sin(g.yaw), np.cos(g.yaw)]) * half
        ax[0].plot([g.x - t[0], g.x + t[0]], [g.y - t[1], g.y + t[1]], color='orange', lw=4)
        ax[0].annotate(str(g.id), (g.x, g.y))
    ax[0].set_aspect('equal'); ax[0].set_xlabel('x [m]'); ax[0].set_ylabel('y [m]'); ax[0].legend()
    ax[1].plot(s[:, 0], np.linalg.norm(s[:, 4:7], axis=1), label='|v|')
    ax[1].plot(s[:, 0], np.linalg.norm(s[:, 7:10], axis=1), label='|a|')
    ax[1].plot(s[:, 0], s[:, 3], label='z')
    ax[1].set_xlabel('t [s]'); ax[1].legend(); ax[1].grid()
    fig.tight_layout()
    if args.save:
        fig.savefig(args.save, dpi=120)
    else:
        plt.show()


if __name__ == '__main__':
    main()
