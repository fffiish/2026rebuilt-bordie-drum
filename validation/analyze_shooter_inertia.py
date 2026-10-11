#!/usr/bin/env python3
"""Summarize the production-controller inertia characterization; no hardware access."""
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
RAW = ROOT / 'logs/generated/shooter-inertia'
OUT = ROOT / 'validation'
rows = list(csv.DictReader((RAW / 'summary.csv').open()))
checks = ('startup_pass', 'steady_pass', 'recovery_pass', 'stop_pass')
result = {
    'revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
    'candidate': 'working tree with corrected simulation inertia; production controller unchanged',
    'inertia_lb_in2': 6.5,
    'inertia_kg_m2': 6.5 * 0.45359237 * 0.0254 ** 2,
    'scenario_count': len(rows),
    'filters': {},
    'sources': {
        'motor': 'https://docs.revrobotics.com/brushless/neo/vortex',
        'pinned_encoder_api': 'https://maven.revrobotics.com/com/revrobotics/frc/REVLib-java/2026.0.5/REVLib-java-2026.0.5-sources.jar',
    },
    'source_sha256': {},
    'raw_summary_sha256': hashlib.sha256((RAW / 'summary.csv').read_bytes()).hexdigest(),
}
for name in sorted({r['filter'] for r in rows}):
    cases = [r for r in rows if r['filter'] == name]
    result['filters'][name] = {
        'cases': len(cases),
        'passes': {k: sum(r[k] == 'true' for r in cases) for k in checks},
        'minimum_speed_ratio': min(float(r['min_rpm']) / float(r['target_rpm']) for r in cases),
        'maximum_speed_ratio': max(float(r['max_rpm']) / float(r['target_rpm']) for r in cases),
        'nominal_cases': [r for r in cases if r['can_phase_ms'] == '0' and r['command_delay_ms'] == '0' and r['drag_multiplier'] == '1.0'],
    }
for name in (
    'src/main/java/frc/robot/subsystems/shooter/Shooter.java',
    'src/main/java/frc/robot/constants/shooter/ShooterConstants.java',
    'src/test/java/frc/robot/subsystems/shooter/ShooterInertiaSimulationTest.java',
):
    result['source_sha256'][name] = hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
reports = [ET.parse(p).getroot() for p in (ROOT / 'build/test-results/test').glob('TEST-*.xml')]
result['junit_totals'] = {k: sum(int(r.get(k, 0)) for r in reports) for k in ('tests', 'failures', 'errors', 'skipped')}
(OUT / 'shooter-inertia.results.json').write_text(json.dumps(result, indent=2) + '\n')
shutil.copyfile(RAW / 'summary.csv', OUT / 'shooter-inertia-summary.csv')
print(json.dumps({k: v['passes'] for k, v in result['filters'].items()}, indent=2))

# Static scientific plot; raw and visual artifacts remain ignored under logs/generated.
os.environ.setdefault('MPLCONFIGDIR', str(Path(tempfile.gettempdir()) / 'frc-inertia-matplotlib'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig, axes = plt.subplots(2, 2, figsize=(12, 7), constrained_layout=True)
for row, state in enumerate(('kShootingNear', 'kShootingSoft')):
    target = next(float(r['target_rpm']) for r in rows if r['state'] == state)
    for name, label, color in (
        ('pinned-default-assumption', '100 ms / depth 64 assumption', '#bb392d'),
        ('explicit-fast-assumption', '10 ms / depth 2 assumption', '#2674b6'),
    ):
        data = list(csv.DictReader((RAW / f'{name}-{state}-12V.csv').open()))
        times = [float(r['seconds']) for r in data]
        speed = [float(r['true_rpm']) for r in data]
        for ax in axes[row]:
            ax.plot(times, speed, label=label, color=color, lw=1.5)
    for col, (low, high, title) in enumerate(((0, 1.2, 'Startup'), (2, 3.5, 'Steady operation'))):
        ax = axes[row, col]
        ax.axhspan(target * .9, target * 1.1, color='#87b685', alpha=.18, label='Acceptance band ±10%')
        ax.axhline(target, color='#222222', ls='--', lw=1, label='Target')
        ax.set_xlim(low, high)
        ax.set_title(f'{"Normal" if row == 0 else "Soft"} — {title}')
        ax.set_xlabel('Time since spin-up request (s)')
        ax.set_ylabel('True drum speed (RPM)')
        ax.grid(alpha=.2)
        if col == 1: ax.set_ylim(target * .75, target * 1.35)
axes[0, 0].legend(fontsize=8)
fig.suptitle('6.5 lb·in² inertia: current 12 V bang-bang controller\nFour Vortex motors, ideal 37 A/motor limiting, nominal drag, 12 V bus, zero added command delay', fontsize=12)
fig.savefig(RAW / 'speed-comparison.png', dpi=170)
plt.close(fig)
