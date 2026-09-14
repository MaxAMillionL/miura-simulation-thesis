"""Interactive viewer: drive and fold the robot with the keyboard."""
import argparse
import csv
import json
import math
import queue
import time
from pathlib import Path
import mujoco.viewer
from .control import Simulation
from .motion import RobotMotion
from .build import ROOT

CONTROLS = (
    'Arrows: drive fwd/back, turn left/right\n'
    'C: fold   O: unfold\n'
    'Space: brake and hold fold\n'
    'P: pause / resume\n'
    'R: reset folded   F: reset flat'
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=3600, help='How long to keep the viewer open')
    parser.add_argument('--paused', action='store_true', help='Start with physics paused')
    parser.add_argument('--log', type=Path, default=ROOT / 'output/telemetry.csv')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be positive and finite')

    sim = Simulation()
    motion = RobotMotion(sim)
    paused = args.paused
    keys = queue.SimpleQueue()

    viewer = mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=keys.put)
    with viewer.lock():
        viewer.cam.lookat[:] = [0, 0, .06]
        viewer.cam.distance = .9
        viewer.cam.azimuth = 130
        viewer.cam.elevation = -35
        viewer.opt.geomgroup[3] = False
    print(f'Assumed simulation motor torque limit: {sim.config["motor_torque_limit_nm"]:g} N m; '
          'real motor specs remain unknown.', flush=True)
    print(CONTROLS.replace('\n', ' | '), flush=True)

    linear = sim.config['drive_linear_speed_m_s']
    yaw = sim.config['drive_yaw_speed_rad_s']
    args.log.parent.mkdir(parents=True, exist_ok=True)
    max_error = 0.
    elapsed = 0.
    try:
        with args.log.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['time', 'motor', 'target_rad_s', 'speed_rad_s', 'wheel_speed_rad_s',
                              'commanded_torque_nm', 'closure_error_m', 'fold_1_rad', 'body_x_m', 'body_y_m'])
            while elapsed < args.seconds - 1e-9 and viewer.is_running():
                start = time.monotonic()
                while not keys.empty():
                    code = keys.get()
                    if code in (262, 263, 264, 265):
                        motion.drive({265: linear, 264: -linear}.get(code, 0.),
                                     {263: yaw, 262: -yaw}.get(code, 0.))
                    elif code in (ord('C'), ord('O')):
                        motion.forward_m_s = motion.yaw_rad_s = 0.
                        motion.set_fold(1.0 if code == ord('C') else 0.)
                    elif code == 32:  # Space
                        motion.stop()
                    elif code == ord('P'):
                        paused = not paused
                    elif code in (ord('R'), ord('F')):
                        sim.reset(flat=code == ord('F'))
                        motion = RobotMotion(sim)
                        paused = True
                    print(f'fold target {motion.fold_target_rad:.2f} rad, paused={paused}', flush=True)
                commands = motion.targets(dt=0. if paused else .01)
                targets = {n: c.value for n, c in commands.items()}
                sim.stop()
                if not paused:
                    sim.command(commands)
                state = sim.read() if paused else sim.step(.01)
                max_error = max(max_error, state['closure_error_m'])
                if not paused:
                    elapsed += .01
                    body = sim.data.body('panel_0').xpos
                    for n, values in state['motors'].items():
                        writer.writerow([state['time'], n, targets.get(n, 0.), values['velocity_rad_s'],
                                          values['wheel_velocity_rad_s'], values['commanded_torque_nm'],
                                          state['closure_error_m'], state['fold_rad'][0], body[0], body[1]])
                viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                    f'{"PAUSED" if paused else "RUNNING"}\n{CONTROLS}',
                    f'\nFold target / actual (rad): {motion.fold_target_rad:.2f} / {state["fold_rad"][0]:.2f}'))
                viewer.sync()
                time.sleep(max(0, .01 - (time.monotonic() - start)))
    finally:
        sim.stop()
        viewer.close()
    summary = {'simulation_seconds': elapsed, 'motors': len(sim.names), 'max_closure_error_m': max_error,
               'final_fold_rad': sim.read()['fold_rad'], 'warnings': sim.data.warning.number.tolist(),
               'log': str(args.log)}
    args.log.with_suffix('.summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
