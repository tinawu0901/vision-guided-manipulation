#!/usr/bin/env python3
"""Compose saved Gazebo frames and detection evidence into a compact GIF."""

import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def export(folder, destination):
    records = json.loads((folder / 'frames/frames.json').read_text())
    records = [record for record in records if record['state'] != 'waiting']
    detection = Image.open(folder / 'camera_detected.jpg').convert('RGB')
    verification = json.loads((folder / 'verification.json').read_text())
    if not verification['passed']:
        raise ValueError('Verification failed; retain diagnostics before exporting a successful demo.')
    font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    font = ImageFont.truetype(font_path, 18)
    small = ImageFont.truetype(font_path, 15)
    frames = []
    labels = {'waiting': 'Starting scene', 'complete': 'Sequence complete', 'failed': 'Motion failed'}
    steps = ['Approach', 'Reach', 'Lift']
    for record in records:
        overview = Image.open(folder / 'frames' / record['file']).convert('RGB')
        canvas = Image.new('RGB', (1280, 596), '#18202b')
        canvas.paste(detection.resize((640, 480)), (0, 62))
        canvas.paste(overview.resize((640, 480)), (640, 62))
        draw = ImageDraw.Draw(canvas)
        draw.text((18, 10), 'YOLO + RGB-D localization', font=font, fill='white')
        draw.text((658, 10), 'Gazebo Panda: approach / reach / lift', font=font, fill='white')
        state = record['state']
        label = steps[record['step']] if state == 'executing' else labels.get(state, state)
        draw.text((658, 36), f'{label} | simulated time {record["simulation_time_s"]:.1f} s',
                  font=small, fill='#7be1b0')
        draw.text((18, 36), 'Detection snapshot from the same scene', font=small, fill='#b5c5d8')
        line = (f'XY localization: {verification["localization_xy_error_m"]*1000:.1f} mm   '
                f'Endpoint: {verification["endpoint_error_m"]*1000:.1f} mm   '
                f'Orientation: {verification["orientation_error_rad"]*180/3.141592653589793:.1f} deg')
        draw.text((18, 549), line, font=small, fill='white')
        draw.text((18, 574), 'Static bottle; reach demonstration. Playback accelerated; no physical grasp.',
                  font=small, fill='#b5c5d8')
        frames.append(canvas.resize((960, 447)))
    if len(frames) < 2:
        raise ValueError('Insufficient recorded overview frames.')
    durations = [250]*len(frames)
    durations[0], durations[-1] = 1000, 2000
    destination.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(destination, save_all=True, append_images=frames[1:],
                   duration=durations, loop=0, optimize=True)
    frames[-1].save(destination.with_suffix('.jpg'), quality=95)
    print(f'Saved {destination} ({len(frames)} frames)')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--folder', type=Path,
                        default=Path.home() / 'vision_guided_ws/work/gazebo_results')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    export(args.folder, args.output or args.folder / 'gazebo-demo.gif')
