"""Record the 0/50/100 N, 0.5 s lateral-push comparison in separate run folders."""
from pathlib import Path
import argparse
import json
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT = 'Pro2/output/L2/lateral_push/long_push_20261105'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root', default=DEFAULT_OUTPUT)
    parser.add_argument('--isaac-root', default='/home/xqw/isaacsim6.0.1')
    args = parser.parse_args()
    destination = (ROOT / args.output_root).resolve()
    destination.relative_to((ROOT / 'Pro2/output/L2/lateral_push').resolve())
    destination.mkdir(parents=True, exist_ok=True)
    results = []
    for name in ['no_push', 'push_50N', 'push_100N']:
        output = destination / name
        command = [sys.executable, 'Pro2/run_isaac.py', '--isaac-root', args.isaac_root,
                   '--no-build', '--headless', '--level', 'L2',
                   '--config', f'Pro2/experiments/L2/lateral_push/{name}.json',
                   '--record-video', '--video-fps', '30', '--output', str(output)]
        if (output / 'run.json').exists():
            raise RuntimeError(f'Run already exists; choose a fresh --output-root: {output}')
        output.mkdir(parents=True, exist_ok=True)
        (output / 'recording_command.json').write_text(json.dumps(command, ensure_ascii=False, indent=2)+'\n')
        print('RECORD', name, flush=True)
        with (output / 'console.log').open('w') as stream:
            process = subprocess.run(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
        run_path = output / 'run.json'
        data = json.loads(run_path.read_text()) if run_path.exists() else {}
        result = dict(condition=name, returncode=process.returncode,
                      run_status=data.get('status'), actual_steps=data.get('steps'),
                      planned_duration_s=data.get('config',{}).get('duration'),
                      video_saved=data.get('video_saved',False),
                      video_duration_s=data.get('video_duration_s'), error=data.get('error'))
        results.append(result)
        (destination/'recording_results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')
        expected_stop = data.get('status') == 'failed' and '机器人失稳' in str(data.get('error',''))
        if not data.get('video_saved') or not (data.get('status') == 'completed' or expected_stop):
            raise RuntimeError(f'Recording did not produce valid evidence: {output}')
        print(json.dumps(result,ensure_ascii=False), flush=True)
    print('Recordings saved; simulation completion and instability are reported separately.', flush=True)


if __name__ == '__main__':
    main()
