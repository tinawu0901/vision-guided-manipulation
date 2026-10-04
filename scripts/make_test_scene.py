#!/usr/bin/env python3
"""Create an independent Gazebo scene fixture with a displaced bottle."""

import argparse
from pathlib import Path
import yaml

parser = argparse.ArgumentParser()
parser.add_argument('--y', type=float, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
if abs(args.y) > .15:
    parser.error('Test displacement must stay within +/- 0.15 m of the table centre.')
root = Path(__file__).resolve().parents[1]
scene = yaml.safe_load((root / 'vision_guided_simulation/config/scene.yaml').read_text())
scene['bottle']['y'] = args.y
args.output.write_text(yaml.safe_dump(scene, sort_keys=False))
print(args.output)
