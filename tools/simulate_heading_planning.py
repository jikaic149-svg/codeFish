"""Offline A/B of heading references at the same geometry, surge and yaw model."""

import argparse
import csv
import math
from pathlib import Path

import numpy as np

from simulate_path_planning import CAMERA_DELAY, DT, make_guide
from path_guidance import wrap_to_pi


TRACE_DIR = Path(__file__).resolve().parents[1] / 'build/checks/host-vision/motion-performance-report'
VARIANTS = ('current', 'no_curve_angle', 'point_14', 'point_24', 'point_35', 'by_geometry')
ROUTES = {
    'straight_offset': (((.35, .8), (2.55, .8)), (.35, 1.0), 0., .30),
    'straight_yaw60': (((.35, .8), (2.55, .8)), (.35, .8), 60., .30),
    'left': (((.35, 1.35), (1.55, 1.35), (1.55, .23)), (.35, 1.35), 0., .30),
    'right': (((.35, .23), (1.55, .23), (1.55, 1.35)), (.35, .23), 0., .30),
    's_bend': (((.35, .30), (1.20, .30), (1.20, 1.23), (2.63, 1.23)), (.35, .30), 0., .30),
    'u_rounded': (((.35, .30), (1.85, .30), (1.85, 1.26), (.35, 1.26)), (.35, .30), 0., .30),
    'u_unrounded': (((.35, .30), (2.00, .30), (2.00, 1.25), (.35, 1.25)), (.35, .30), 0., .30),
    'right_sharp': (((.35, .23), (1.55, .23), (1.55, 1.35)), (.35, .23), 0., .60),
}


def read_run(file):
    """Use fish uptime and the actual received heading target, not wall-clock pairings."""
    with file.open(encoding='utf-8-sig', newline='') as source:
        rows = [r for r in csv.DictReader(source)
                if (r['imu_valid'], r['heading_active']) == ('True', 'True')
                and r['allocation_profile'] == 'hybrid_gait_tail_bias']
    if not rows:
        return None
    t, yaw, target = np.array([[float(r[k]) for k in ('uptime_ms', 'yaw_left_deg', 'target_left_deg')]
                              for r in rows]).T
    dt = np.diff(t) / 1000.
    valid = (dt >= .035) & (dt <= .45)
    # Average rates on both sides of a sample; exclude holes instead of smoothing through them.
    rate = (np.diff(yaw) + 180.) % 360. - 180.
    rate = rate / np.where(valid, dt, 1.)
    error = (target - yaw + 180.) % 360. - 180.
    mask = valid[:-1] & valid[1:]
    return np.array([v[mask] for v in (dt[1:], rate[:-1], error[1:-1], rate[1:])]).T


def estimate(data):
    """Estimate a bounded yaw response; use a robust one-step error for sparse telemetry."""
    dt, old, err, observed = data.T
    best = (math.inf, None)
    for gain in (.35, .55, .8, 1.15, 1.65, 2.3):
        for tau in (.20, .35, .55, .85, 1.25, 1.8, 2.7):
            for cap in (6., 9., 12., 16., 22., 30., 40.):
                prediction = old + (-np.expm1(-dt / tau)) * (np.clip(gain * err, -cap, cap) - old)
                loss = np.mean(np.minimum(abs(prediction - observed), 45.))
                if loss < best[0]:
                    best = (loss, (gain, tau, cap))
    return best[1]


def validation(data, model):
    dt, old, err, observed = data.T
    gain, tau, cap = model
    predicted = old + (-np.expm1(-dt / tau)) * (np.clip(gain * err, -cap, cap) - old)
    return float(np.median(abs(predicted - observed))), float(np.median(abs(old - observed)))


def target_angle(variant, result, guide, observed, last, elapsed):
    if variant == 'current':
        return result['psi_ref_rad']
    if variant == 'no_curve_angle':
        raw = result['psi_path_rad'] + result['cross_track_feedback']
    else:
        # Longer preview on fallback folds is a diagnostic comparison, not a launch permission.
        preview = (.35 if guide.path_geometry['status'] == 'unachievable' else .24
                   ) if variant == 'by_geometry' else int(variant.split('_')[-1]) / 100.
        dx, dy = guide._point_at(result['along_m'] + preview) - observed
        raw = math.atan2(-dy, dx) if math.hypot(dx, dy) > 1e-6 else last
    # Apply the same circular slew limit as PathGuidance; do not give a variant free bandwidth.
    maximum = guide.heading_reference_rate_rps * elapsed
    return wrap_to_pi(last + min(max(wrap_to_pi(raw - last), -maximum), maximum))


def run(route, variant, model, speed, delay):
    points, start, initial_deg, rounding = route
    guide = make_guide()
    guide.desired_turn_radius_m = rounding
    pos = np.asarray(start)
    yaw = last = math.radians(initial_deg)
    yaw_rate = 0.
    unit = lambda angle: np.array((math.cos(angle), -math.sin(angle)))
    guide.start(points, pos, 0., unit(yaw))
    segments = np.diff(guide.path, axis=0)
    length2 = np.sum(segments**2, axis=1)
    final_direction = segments[-1] / math.sqrt(length2[-1])
    threshold_end = max(0., guide.cumulative[-1] - .12)
    history, trace = [], []
    gain, tau, cap = model
    # ponytail: speed is held by design; rate cap scales with speed at constant effective
    # radius. No sideslip identification or extrapolation to the current steering recipe.
    cap *= speed / .08
    rate_limit = math.radians(cap)
    steps = round(delay / DT)
    captured = False
    for i in range(math.ceil((guide.cumulative[-1] / speed + 24.) / DT)):
        stamp = i * DT
        history.append(pos)
        obs = history[max(0, i - steps)]
        direction = unit(yaw)
        result = guide.update(obs, stamp, speed_mps=speed, body_heading=direction, velocity=speed * direction)
        reference = target_angle(variant, result, guide, obs, last, DT if i else 0.)
        change = wrap_to_pi(reference - last)
        last = reference
        error = wrap_to_pi(reference - yaw)
        requested = gain * error
        delta = pos - guide.path[:-1]
        projected = np.clip(np.sum(delta * segments, axis=1) / length2, 0., 1.)
        distance = float(np.linalg.norm(delta - projected[:, None] * segments, axis=1).min())
        terminal = float(np.linalg.norm(pos - guide.path[-1]))
        trace.append((stamp, *pos, *obs, result['along_m'], distance, terminal, math.degrees(reference),
                      math.degrees(yaw), math.degrees(error), math.degrees(change) / DT,
                      abs(requested) > rate_limit))
        if result['along_m'] >= threshold_end:
            if terminal <= .12:
                captured = True  # Capture, not a stop; surge was held for the whole experiment.
                break
            if (pos - guide.path[-1]) @ final_direction > .12:
                break
        yaw_rate += -math.expm1(-DT / tau) * (min(max(requested, -rate_limit), rate_limit) - yaw_rate)
        yaw = wrap_to_pi(yaw + DT * yaw_rate)
        pos = pos + DT * speed * unit(yaw)
    arr = np.asarray(trace)
    return (captured, float(np.quantile(arr[:, 6], .95)), float(arr[:, 6].max()),
            float(np.quantile(abs(arr[:, 10]), .95)), float(arr[:, 12].mean()),
            guide.path_geometry['status'], arr)


def probe(case='right', x=1.07):
    """Compare references at one on-path pose near the incoming right bend."""
    points, _, _, rounding = ROUTES[case]
    guide = make_guide()
    guide.desired_turn_radius_m = rounding
    p = (x, .23)
    guide.start(points, p, 0., (1., 0.))
    data = guide.update(p, 1., speed_mps=.08, body_heading=(1., 0.))
    return {v: math.degrees(target_angle(v, data, guide, np.asarray(p), 0., 1.)) for v in VARIANTS}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace-dir', type=Path, default=TRACE_DIR)
    parser.add_argument('--csv', type=Path, help='export summary; refuses to overwrite')
    parser.add_argument('--check', action='store_true')
    opts = parser.parse_args()
    runs = {p.name.split('_', 1)[0]: data for p in sorted(opts.trace_dir.glob('*_fish_active_unique.csv'))
            if (data := read_run(p)) is not None and len(data) >= 25}
    if len(runs) < 2:
        parser.error('need at least two HYBRID fish traces to fit and cross-validate')
    models = {name: estimate(np.concatenate([other for key, other in runs.items() if key != name]))
              for name in runs}
    for name, model in models.items():
        mae, persistence = validation(runs[name], model)
        print(f'held out {name}: n={len(runs[name])}, gain/tau/cap={model}, '
              f'one-step yaw-rate median error={mae:.1f} deg/s (persistence {persistence:.1f})')
    print('Fixed-pose angles (CCW+, deg):', {v: round(a, 1) for v, a in probe().items()})
    print('Fixed pose on unrounded corner (CCW+, deg):',
          {v: round(a, 1) for v, a in probe('right_sharp', 1.27).items()})
    summaries = []
    for name, model in models.items():
        for speed, delay in ((.08, CAMERA_DELAY), (.06, CAMERA_DELAY), (.08, 0.)):
            for label, route in ROUTES.items():
                for variant in VARIANTS:
                    capture, p95, peak, yaw, saturated, geometry, arr = run(route, variant, model, speed, delay)
                    summaries.append((name, speed, delay, label, variant, capture, p95, peak, yaw, saturated, geometry))
                    if opts.check:
                        assert np.isfinite(arr).all(), (label, variant)
                        assert np.max(abs(arr[:, 11])) <= 35. + 1e-6, (label, variant)
    if opts.check:
        angles = probe()
        assert angles['current'] < -10. and abs(angles['point_14']) < 1e-6
        assert probe('right_sharp', 1.27)['current'] < -10.
        assert any(r[5] and r[4] == 'current' for r in summaries)
        assert all(r[-1] == 'rounded' for r in summaries if r[3] in ('left', 'right', 's_bend', 'u_rounded'))
        assert all(r[-1] == 'unachievable' for r in summaries if r[3] in ('u_unrounded', 'right_sharp'))
        by_case = {(r[:4], r[4]): r for r in summaries}
        assert all(r[6] < by_case[r[:4], 'current'][6] for r in summaries
                   if r[4] == 'point_24' and r[3] in ('left', 'right', 's_bend'))
    for speed, delay in ((.08, CAMERA_DELAY), (.06, CAMERA_DELAY), (.08, 0.)):
        print(f'\nFixed surge {speed:g} m/s; position delay {delay:g} s. '
              'By route: mean P95 distance in cm across yaw fits | captures.')
        print(f'{"variant":16}', ''.join(f'{name:>18}' for name in ROUTES))
        for variant in VARIANTS:
            cells = []
            for label in ROUTES:
                sample = [r for r in summaries if r[1:5] == (speed, delay, label, variant)]
                cells.append(f'{100 * np.mean([r[6] for r in sample]):.1f} | '
                             f'{sum(r[5] for r in sample)}/{len(sample)}')
            print(f'{variant:16}', ''.join(f'{text:>18}' for text in cells))
    if opts.csv:
        with opts.csv.open('x', encoding='utf-8', newline='') as output:
            writer = csv.writer(output)
            writer.writerow(('held_out_run', 'speed_mps', 'delay_s', 'route', 'variant', 'capture',
                             'p95_path_error_m', 'max_path_error_m', 'p95_heading_error_deg', 'yaw_cap_fraction',
                             'geometry'))
            writer.writerows(summaries)


if __name__ == '__main__':
    main()
