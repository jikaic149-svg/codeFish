"""Compare path planning, steering and position-jump rejection on a synthetic offline plant."""

import argparse
import csv
import json
import math
from pathlib import Path
import sys
from tempfile import TemporaryDirectory

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'vision'))
from control_config import CONTROL
from curve_speed import CurveSpeedPlanner
from path_guidance import PathGuidance, wrap_to_pi


DT = .1
SPEEDS = (.04, .07, .08)
LEFT_RADII = (.22, .27, .34)
RIGHT_RADII = (.25, .31, .39)
COAST_TAU = 2.0
POOL = (3., 1.6)
CAMERA_DELAY = CONTROL.get('camera_latency_s', .3)
SPIKE_M = .35
SPIKE_S = 10.
TERMINAL_SPIKE_S = 15.4
ROUTES = {
    'straight_stop': ((.25, .80), (1.75, .80)),
    'left_angle': ((.25, 1.30), (1.55, 1.30), (1.55, .30)),
    'right_angle': ((.25, .30), (1.55, .30), (1.55, 1.30)),
    'tight_u': ((.25, .40), (1.40, .40), (1.40, .65), (.25, .65)),
}


class RouteRejected(ValueError):
    """The experimental planner cannot produce a drivable route."""


def experimental_planner(folder):
    # Keep the fictitious envelope out of vision/configs and the runtime config.
    profile = dict(schema=1, drive_mode='HYBRID', profile_id='SYNTHETIC_OFFLINE_ONLY',
                   coast_method='zero_forward_keep_steering', source_segments=['synthetic'],
                   samples=[dict(speed_mps=v, coast_distance_m=v * COAST_TAU,
                                 left_radius_m=left, right_radius_m=right)
                            for v, left, right in zip(SPEEDS, LEFT_RADII, RIGHT_RADII)])
    (folder / 'synthetic.json').write_text(json.dumps(profile), encoding='utf-8')
    # The same table defines the physics and the planner only in the nominal case.
    planner = CurveSpeedPlanner(dict(CONTROL['curve_speed'], enabled=True, profile_file='synthetic.json'), folder)
    if planner.profile is None:
        raise RuntimeError(f'Synthetic profile failed to load: {planner.status}: {planner.error}')
    return planner


def make_guide(planner=None):
    settings = dict(CONTROL['guidance'])
    return PathGuidance(k_f=settings.pop('cross_track_gain'),
                        min_speed_target_mps=settings.pop('min_speed_mps'),
                        arrival_radius_m=.015 if planner else .20,
                        curve_speed_planner=planner, **settings)


def rollout(points, guide, radius_scale=1., coast_scale=1., delay_s=0., spike_at_s=None,
            short_lookahead=False, guard=False):
    """Vary position feedback only; heading and speed feedback are still ideal."""
    position = np.array(points[0])
    heading = speed = yaw_rate = 0.
    try:
        start = guide.start(points, position, 0., (1., 0.))
    except ValueError as error:
        if guide.curve_speed_planner is None:
            raise
        raise RouteRejected(str(error)) from error
    if guide.curve_speed_planner is not None and guide.curve_speed_planner.budget is None:
        raise RouteRejected('cruise speed outside the synthetic planning envelope')

    cutoff = None
    trace = []
    history = []
    delay_steps = round(delay_s / DT)
    spike_step = None if spike_at_s is None else round(spike_at_s / DT)
    accepted, accepted_step = position, 0
    for step in range(1001):
        time = step * DT
        history.append(position)
        raw = history[max(0, step - delay_steps)]
        if step == spike_step:
            raw = raw + (SPIKE_M, 0.)
        # Prototype only: a bounded single-frame position gate, not a dropout handler.
        rejected = guard and np.linalg.norm(raw - accepted) > .02 + .15 * DT * (step - accepted_step)
        if not rejected:
            accepted, accepted_step = raw, step
        direction = np.array((math.cos(heading), -math.sin(heading)))
        result = guide.update(accepted, time, speed_mps=speed,
                              velocity=speed * direction, body_heading=direction)
        if cutoff is None and result['arrived']:
            cutoff = time
        target = result['speed_target_mps'] if cutoff is None else 0.
        trace.append((time, *position, math.degrees(heading), speed, target, result['along_m'],
                      *raw, *accepted, int(rejected)))
        # No reverse braking; thrust can never slow the fish faster than coasting.
        if cutoff is not None and speed < .003:
            break
        speed = max(speed * math.exp(-DT / (COAST_TAU * coast_scale)),
                    target + (speed - target) * math.exp(-DT / .8))
        # ponytail: aggregate the yaw PID/actuator response; no tail-cycle forces or sideways drift.
        reference_heading = result['psi_ref_rad']
        if cutoff is None and short_lookahead:
            aim = [np.interp(result['along_m'] + .14, guide.cumulative, axis) for axis in guide.path.T]
            dx, dy = aim - accepted
            reference_heading = math.atan2(-dy, dx)
        error = wrap_to_pi(reference_heading - heading) if cutoff is None else 0.
        radii = LEFT_RADII if error >= 0. else RIGHT_RADII
        radius = float(np.interp(speed, SPEEDS, radii)) * radius_scale
        requested_yaw_rate = float(np.clip(2. * error, -speed / radius, speed / radius))
        yaw_rate += -math.expm1(-DT / .65) * (requested_yaw_rate - yaw_rate)
        heading = wrap_to_pi(heading + DT * yaw_rate)
        position = position + DT * speed * np.array((math.cos(heading), -math.sin(heading)))

    trace = np.asarray(trace)
    xy = trace[:, 1:3]
    reference = np.asarray(points)
    segments = np.diff(reference, axis=0)
    delta = xy[:, None, :] - reference[:-1]
    fraction = np.clip(np.sum(delta * segments, axis=-1) / np.sum(segments**2, axis=-1), 0., 1.)
    distance = np.linalg.norm(delta - fraction[:, :, None] * segments, axis=-1).min(axis=1)
    last_direction = segments[-1] / np.linalg.norm(segments[-1])
    endpoint_error = float(np.linalg.norm(xy[-1] - reference[-1]))
    overshoot = max(0., float(np.max((xy - reference[-1]) @ last_direction)))
    outside = bool(np.any((xy < 0.) | (xy > POOL)))
    stopped = cutoff is not None and trace[-1, 4] < .003
    progress = trace[:, 6] if cutoff is None else trace[trace[:, 0] <= cutoff, 6]
    return dict(
        geometry=('rounded' if guide.curve_speed_planner and guide.curve_speed_planner.rounded
                  else start['path_geometry'].get('status', 'unchanged')),
        stopped=stopped, max_error=float(distance.max()), endpoint_error=endpoint_error,
        overshoot=overshoot, outside=outside, trace=trace, cutoff=cutoff,
        max_progress_step=float(np.diff(progress).max(initial=0.)),
        rejected_frames=int(trace[:, -1].sum()),
    )


def compare(folder):
    scenarios = (
        ('nominal', 1., 1., 0., None, ROUTES),
        ('weaker', 1.25, 1.35, 0., None, ROUTES),
        ('delay_only', 1., 1., CAMERA_DELAY, None, ROUTES),
        ('camera_fault', 1., 1., CAMERA_DELAY, SPIKE_S, ROUTES),
        ('terminal_fault', 1., 1., CAMERA_DELAY, TERMINAL_SPIKE_S, ('straight_stop',)),
    )
    for scenario, radii, coast, delay, spike_at, cases in scenarios:
        variants = ('current', 'plan_only', 'plan_track') + (('plan_guard',) if spike_at is not None else ())
        for case in cases:
            for variant in variants:
                planner = experimental_planner(folder) if variant != 'current' else None
                try:
                    data = rollout(ROUTES[case], make_guide(planner), radii, coast, delay, spike_at,
                                   short_lookahead=variant in ('plan_track', 'plan_guard'),
                                   guard=variant == 'plan_guard')
                except RouteRejected as error:
                    data = {'rejected': str(error)}
                yield (scenario, case, variant), data


def self_check(results):
    assert all(np.isfinite(r['trace']).all() for r in results.values() if 'trace' in r)
    for plant in ('nominal', 'weaker'):
        current = results[plant, 'right_angle', 'current']
        plan = results[plant, 'right_angle', 'plan_only']
        track = results[plant, 'right_angle', 'plan_track']
        assert current['geometry'] == 'unachievable' and plan['geometry'] == 'rounded'
        assert not plan['stopped'] and track['stopped'] and track['endpoint_error'] < .05
        assert track['max_error'] < plan['max_error'] and not track['outside']
        assert results[plant, 'tight_u', 'current']['geometry'] == 'unachievable'
        assert 'rejected' in results[plant, 'tight_u', 'plan_track']
        straight = results[plant, 'straight_stop', 'plan_only']
        assert straight['stopped'] and straight['endpoint_error'] < .06
        assert straight['endpoint_error'] < results[plant, 'straight_stop', 'current']['endpoint_error']
    fault = results['camera_fault', 'straight_stop', 'plan_track']
    guarded = results['camera_fault', 'straight_stop', 'plan_guard']
    assert fault['max_progress_step'] > .25 and guarded['max_progress_step'] < .06
    assert guarded['rejected_frames'] == 1 and guarded['stopped']
    terminal = results['terminal_fault', 'straight_stop', 'plan_track']
    guarded = results['terminal_fault', 'straight_stop', 'plan_guard']
    delayed = results['delay_only', 'straight_stop', 'plan_track']
    assert math.isclose(terminal['cutoff'], TERMINAL_SPIKE_S) and terminal['endpoint_error'] > .15
    assert guarded['rejected_frames'] == 1 and guarded['stopped']
    assert abs(guarded['endpoint_error'] - delayed['endpoint_error']) < .01


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv', type=Path, help='export trajectory samples; refuses to overwrite')
    parser.add_argument('--check', action='store_true', help='check baseline, stopping and jump rejection')
    args = parser.parse_args()
    with TemporaryDirectory(prefix='robotfish-planner-') as folder:
        results = dict(compare(Path(folder)))
    if args.check:
        self_check(results)
    print('SYNTHETIC OFFLINE ONLY | Weaker plant: turn radius +25%, coast time +35%.')
    print('Current: live guidance. Plan only: rounded path, coast envelope, 1.5 cm arrival gate.')
    print('Plan + track: same planning with heading aimed at a point 14 cm ahead.')
    print(f'Camera: {CAMERA_DELAY:g} s position delay; fault adds one +{100 * SPIKE_M:g} cm frame'
          f' at {SPIKE_S:g} s, or at {TERMINAL_SPIKE_S:g} s for terminal_fault.')
    print('Plan + guard rejects implausible motion.')
    print(f'{"scenario":14} {"route":15} {"variant":10} {"result":7} {"max err":>8} '
          f'{"end err":>8} {"overshoot":>10} {"max ds":>8} {"skipped":>7} {"outside":>8} '
          f'{"cutoff s":>8} (distance: cm)')
    for (scenario, case, variant), data in results.items():
        label = f'{scenario:14} {case:15} {variant:10}'
        if 'rejected' in data:
            print(f'{label} rejected: {data["rejected"]}')
            continue
        state = 'stopped' if data['stopped'] else 'timeout'
        cutoff = '-' if data['cutoff'] is None else f'{data["cutoff"]:.1f}'
        print(f'{label} {state:7} {100 * data["max_error"]:8.1f} '
              f'{100 * data["endpoint_error"]:8.1f} {100 * data["overshoot"]:10.1f} '
              f'{100 * data["max_progress_step"]:8.1f} {data["rejected_frames"]:7} '
              f'{str(data["outside"]):>8} {cutoff:>8} [{data["geometry"]}]')
    if args.csv:
        with args.csv.open('x', encoding='utf-8', newline='') as output:
            writer = csv.writer(output)
            writer.writerow(('scenario', 'route', 'variant', 't_s', 'x_m', 'y_m', 'heading_deg',
                             'speed_mps', 'target_speed_mps', 'progress_m', 'raw_x_m', 'raw_y_m',
                             'accepted_x_m', 'accepted_y_m', 'frame_rejected'))
            for key, data in results.items():
                writer.writerows((*key, *row) for row in data.get('trace', ()))
        print(f'Trace: {args.csv}')


if __name__ == '__main__':
    main()
