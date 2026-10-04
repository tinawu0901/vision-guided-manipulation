#!/usr/bin/env python3
"""Start or stop only the processes owned by this demo session."""

import argparse
from pathlib import Path
import subprocess
from datetime import datetime

import psutil


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = Path.home() / 'vision_guided_ws/work/gazebo_results'


def stop(name):
    path = OUTPUT / f'{name}.pid'
    if not path.exists():
        return
    try:
        process = psutil.Process(int(path.read_text()))
        command = ' '.join(process.cmdline())
        if 'vision_guided_simulation' not in command and 'run_camera_vision' not in command and 'camera_vision' not in command:
            raise RuntimeError(f'PID file no longer belongs to demo: {command}')
        owned = process.children(recursive=True) + [process]
        for item in owned:
            try:
                item.terminate()
            except psutil.NoSuchProcess:
                pass
        _, remaining = psutil.wait_procs(owned, timeout=5)
        for item in remaining:
            item.kill()
    except psutil.NoSuchProcess:
        pass
    path.unlink(missing_ok=True)


def start(name, command):
    stop(name)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    with (OUTPUT / f'{name}.log').open('w') as log:
        process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
    (OUTPUT / f'{name}.pid').write_text(str(process.pid))
    print(f'{name}: PID {process.pid}, log {OUTPUT / (name + ".log")}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['start', 'stop', 'vision'])
    parser.add_argument('--gui', default='true', choices=['true', 'false'])
    parser.add_argument('--scene')
    args = parser.parse_args()
    if args.action == 'stop':
        stop('vision')
        stop('launch')
    elif args.action == 'start':
        stop('vision')
        stop('launch')
        previous = [path for path in OUTPUT.glob('*')
                    if path.name != 'runs' and path.suffix != '.pid']
        if previous:
            archive = OUTPUT / 'runs' / datetime.now().strftime('%Y%m%d-%H%M%S')
            archive.mkdir(parents=True, exist_ok=True)
            for path in previous:
                path.replace(archive / path.name)
        command = ['bash', str(ROOT / 'scripts/run_gazebo.sh'), f'gui:={args.gui}']
        if args.scene:
            command.append(f'scene:={args.scene}')
        start('launch', command)
    else:
        start('vision', ['bash', str(ROOT / 'scripts/run_camera_vision.sh')])
