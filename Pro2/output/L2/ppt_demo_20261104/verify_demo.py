"""Verify supplementary L2 recordings without rewriting formal experiment analyses."""
from __future__ import annotations

import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess


ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
FFMPEG = Path('/home/xqw/miniconda3/envs/rllab/lib/python3.11/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2')


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def relative(path: Path) -> str:
    return str(path.relative_to(ROOT))


def normalized_config(config: dict) -> dict:
    normalized = json.loads(json.dumps(config))
    normalized.pop('output', None)
    normalized.pop('config', None)
    normalized['state_weights'][9] = '<controlled variable>'
    return normalized


def main() -> None:
    results = []
    sources = []
    run_configs = []
    for condition in ('low', 'baseline', 'high'):
        run_dir = OUT / condition
        manifest_path = run_dir / 'run.json'
        manifest = json.loads(manifest_path.read_text())
        video = run_dir / 'demo.mp4'
        source_path = ROOT / f'Pro2/experiments/L2/mpc_vx_weight/{condition}.json'
        source_config = json.loads(source_path.read_text())
        sources.append(normalized_config(source_config))
        run_configs.append(normalized_config(manifest['config']))

        with (run_dir / 'states.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        finite = all(math.isfinite(float(value)) for row in rows for value in row.values())
        statuses = {}
        for key in ('controller:mpc_qp_status', 'controller:wbc_qp_status'):
            counts = {}
            for row in rows:
                value = str(float(row[key]))
                counts[value] = counts.get(value, 0) + 1
            statuses[key] = {
                'counts': counts,
                'nonzero_samples': sum(float(row[key]) != 0 for row in rows),
            }

        probe = subprocess.run(
            [str(FFMPEG), '-hide_banner', '-nostats', '-i', str(video), '-map', '0:v:0',
             '-an', '-f', 'null', '-', '-progress', 'pipe:1'],
            text=True, capture_output=True, check=False,
        )
        duration_match = re.search(r'Duration: (\d+):(\d+):(\d+\.\d+)', probe.stderr)
        assert duration_match, probe.stderr
        duration = int(duration_match[1]) * 3600 + int(duration_match[2]) * 60 + float(duration_match[3])
        frames = [int(v) for v in re.findall(r'^frame=(\d+)$', probe.stdout, re.M)]
        video_info = next(line.strip() for line in probe.stderr.splitlines() if 'Video:' in line)
        decode_log = run_dir / 'video_decode_verification.txt'
        decode_log.write_text(probe.stderr + '\n' + probe.stdout)

        checks = {
            'run_completed': manifest['status'] == 'completed',
            '12000_steps': manifest['steps'] == 12000,
            '1200_samples': len(rows) == manifest['samples'] == 1200,
            'finite_csv_values': finite,
            'mpc_status_zero': statuses['controller:mpc_qp_status']['nonzero_samples'] == 0,
            'wbc_status_zero': statuses['controller:wbc_qp_status']['nonzero_samples'] == 0,
            'video_saved': manifest['video_saved'],
            'full_decode_succeeded': probe.returncode == 0,
            'decoded_360_frames': bool(frames) and frames[-1] == 360,
            'video_duration_12_seconds': abs(duration - 12) < .001,
            'video_30fps': '30 fps' in video_info,
            'video_960x720': '960x720' in video_info,
        }
        results.append({
            'condition': condition,
            'source_config': relative(source_path),
            'state_weights_9': source_config['state_weights'][9],
            'run_directory': relative(run_dir),
            'video': relative(video),
            'checks': checks,
            'pass': all(checks.values()),
            'video_info': video_info,
            'video_duration_s': duration,
            'decoded_video_frames': frames[-1] if frames else 0,
            'video_fps': manifest['video_fps'],
            'simulated_duration_s': manifest['simulation_duration_s'],
            'recorded_samples': len(rows),
            'sample_time_bounds_s': [float(rows[0]['time_s']), float(rows[-1]['time_s'])],
            'status_checks': statuses,
            'controller_library_sha256': manifest['controller_library_sha256'],
            'source_config_sha256': digest(source_path),
            'run_manifest_sha256': digest(manifest_path),
            'states_sha256': digest(run_dir / 'states.csv'),
            'video_sha256': digest(video),
            'decode_log': relative(decode_log),
        })

    cross_checks = {
        'source_configs_only_state_weights_9_differs': all(config == sources[0] for config in sources),
        'effective_configs_only_state_weights_9_and_output_metadata_differ': all(config == run_configs[0] for config in run_configs),
        'controller_library_identical': len({result['controller_library_sha256'] for result in results}) == 1,
        'condition_values_0p2_1_5': [result['state_weights_9'] for result in results] == [.2, 1, 5],
    }
    payload = {
        'verified_at': datetime.now().astimezone().isoformat(),
        'purpose': 'PPT supplementary demonstration recordings; not replacements for existing formal L2 data or analyses',
        'verification_command': 'python3 Pro2/output/L2/ppt_demo_20261104/verify_demo.py',
        'ffmpeg_binary': str(FFMPEG),
        'cross_condition_checks': cross_checks,
        'recordings': results,
        'pass': all(cross_checks.values()) and all(result['pass'] for result in results),
        'sampling_boundary': 'MPC/WBC status checks use 100 Hz states.csv samples; video and run completion are verified independently.',
    }
    (OUT / 'verification.json').write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n')
    assert payload['pass'], 'Supplementary recording verification failed; inspect verification.json'
    print(json.dumps({'pass': payload['pass'], 'videos': [r['video'] for r in results]}, ensure_ascii=False))


if __name__ == '__main__':
    main()
