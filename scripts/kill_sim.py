#!/usr/bin/env python3
"""Stop this user's ADR simulation stack (including orphan Gazebo processes).

Usage: python3 scripts/kill_sim.py [--dry-run] [--include-viewers]
Gazebo instances belonging to this user are all included, even other worlds.
Optional viewers includes all of this user's RViz/rqt instances.
Does not require ROS setup or sudo. Does not delete files or shared memory.
"""
import argparse
import os
from pathlib import Path
import signal
import time


def read_process(pid):
    try:
        root = Path('/proc') / str(pid)
        if root.stat().st_uid != os.getuid():
            return None
        stat = (root / 'stat').read_text().rsplit(')', 1)[1].split()
        args = (root / 'cmdline').read_bytes().decode(errors='replace').strip('\0').split('\0')
        return {'pid': pid, 'ppid': int(stat[1]), 'start': stat[19],
                'state': stat[0], 'args': args}
    except (OSError, ValueError, IndexError):
        return None


def command(args):
    """Inspect executable/script slots only, never arbitrary shell command text."""
    if not args:
        return []
    name = Path(args[0]).name
    if name.startswith(('python', 'ruby')) and len(args) > 1:
        return args[1:] if not args[1].startswith('-') else []
    return args


def matches(args, viewers=False):
    args = command(args)
    if not args:
        return False
    name = Path(args[0]).name
    if name == 'ros2':
        return len(args) > 3 and args[1] == 'launch' and args[2] in {'adr_sim', 'adr_bringup', 'adr_vio'}
    if name in {'gz', 'ign'}:
        return len(args) > 1 and args[1] in {'sim', 'gazebo'}
    if args[0].startswith(('gz sim ', 'ign gazebo ')) or name in {'gzserver', 'gzclient'}:
        return True
    if name == 'px4':
        return 'px4_sitl' in args[0] or (args[0] == './bin/px4' and '-d' in args)
    if name in {'MicroXRCEAgent', 'micro-xrce-dds-agent'}:
        return any(args[i:i+2] == ['-p', '8888'] for i in range(len(args)-1))
    if '/lib/adr_' in args[0] or '/lib/ov_msckf/' in args[0]:
        return True
    if name == 'parameter_bridge':
        return any(a in {'__node:=gz_bridge', '__node:=imu_bridge'} for a in args)
    if name == 'rviz2' and any(Path(a).name == 'adr.rviz' for a in args[1:]):
        return True
    return viewers and name in {'rviz2', 'rqt', 'rqt_image_view'}


def alive(process):
    current = read_process(process['pid'])
    return current is not None and current['start'] == process['start'] and current['state'] != 'Z'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true', help='list targets without sending signals')
    parser.add_argument('--include-viewers', action='store_true', help='also close all rqt/RViz instances owned by this user')
    options = parser.parse_args()
    processes = {int(p.name): info for p in Path('/proc').iterdir()
                 if p.name.isdigit() and (info := read_process(int(p.name))) is not None}
    protected = {os.getpid()}
    pid = os.getppid()
    while pid in processes and pid not in protected:
        protected.add(pid)
        pid = processes[pid]['ppid']
    selected = {pid for pid, p in processes.items()
                if pid not in protected and p['state'] != 'Z' and matches(p['args'], options.include_viewers)}
    # Capture descendants before parents can exit and reparent their children.
    while True:
        children = {pid for pid, p in processes.items()
                    if p['ppid'] in selected and pid not in protected and p['state'] != 'Z'}
        if children <= selected:
            break
        selected |= children
    targets = [processes[pid] for pid in sorted(selected)]
    for p in targets:
        print(f"{p['pid']:>8}  {' '.join(p['args'])}", flush=True)
    if not targets:
        print('No matching simulation processes.')
    if options.dry_run or not targets:
        return 0
    print('Stopping listed processes: SIGINT → SIGTERM → SIGKILL', flush=True)
    # Signal only the captured identities: do not kill newly started simulations.
    for sig, timeout in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 1)):
        for p in targets:
            if alive(p):
                try:
                    os.kill(p['pid'], sig)
                except ProcessLookupError:
                    pass
                except PermissionError as exc:
                    print(f"Cannot signal PID {p['pid']}: {exc}")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and any(alive(p) for p in targets):
            time.sleep(0.1)
        if not any(alive(p) for p in targets):
            print('All listed processes stopped.')
            return 0
    print('Still running:', [p['pid'] for p in targets if alive(p)])
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
