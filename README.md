# Reachy Mini local daemon layers

This repository owns the catalog and source of independently controlled daemon
layers. It is not a Reachy application and does not run as another service.
The generic transaction, verification and rollback machinery lives in the
sibling `reachy_mini_layer_manager` repository.

## The independently controlled layers

| Name | Role | Current code |
|---|---|---|
| `face-frame-sync` | permanent correction | accepted overlay `08d9e40ca`, proposed upstream as [PR #1396](https://github.com/pollen-robotics/reachy_mini/pull/1396) |
| `body-yaw-calibration` | permanent correction | corrected bumpless `2026-09-12.3` active on robot |
| `face-loss-return` | permanent safety correction | loss-only 40°/s world-neutral return cap; deployed and hardware-accepted |
| `body-motor-igain` | permanent correction | body-motor `position_i_gain = 25` plus a ±120° body-yaw clamp; `2026-09-14.4` active on robot |
| `movement-diagnostics` | temporary, read-only instrumentation | target chain plus controller-bound, health-verified body-motor telemetry |

The manager is not another layer. It owns one final systemd drop-in and one
bootstrap hook so independently developed Python layers cannot accidentally
overwrite each other's `PYTHONPATH` or compete for Python's single
`sitecustomize` import.

The existing face and diagnostic behavior is not reimplemented here. The face
overlay remains a normal package overlay. The bootstrap calls each enabled
hook's `apply()` function. The body-yaw hook applies one stateless quadratic to
joint 0 on the real robot backend after kinematics; logical daemon targets,
measured angles, other joints, and simulation backends remain unchanged.
`enable_motors()` has an explicit scoped bypass so its measured-position pin is
never calibrated, including when it is called while torque is already enabled.
For daemon gotos, the layer preserves the raw motor target already in force at
trajectory entry. It holds that target through any tracking IK tick before the
goto emits its first real waypoint, then captures that waypoint as the exact
logical interpolation origin and blends monotonically to the calibrated raw
endpoint. Every exit, including normal completion and failure, reconciles the
daemon's logical slot from authoritative joint 0 before the trajectory context
is removed, so the next IK tick cannot introduce an exit step. Context setup,
trajectory mapping and exit reconciliation are serialized by one per-backend
lock, which is never held across the awaited goto. A small extreme-
angle move whose calibrated endpoint would reverse the requested direction
uses an uncalibrated same-direction endpoint for that move. Direct `set_target`
commands retain the ordinary stateless mapping. Motor enable also reconciles
its raw measured-position pin so the next tracking IK tick reproduces it.

The movement diagnostic contains no calibration formula of its own. Run its
ordinary `--calibration-profile` with both layers enabled to record the logical
body target, corrected joint-0 motor target, and measured endpoint.

The body-motor I-gain hook addresses a different failure from the body-yaw
calibration. The body motor ships with `position_i_gain = 0`, so it is under
proportional-only control: it stops where `Kp * error` balances Coulomb
friction and a cable-spring restoring torque, arriving 2-8° short of target and
further short at larger angles. The calibration layer compensates the *steady
mapping*; it cannot make the servo close a residual error. This hook writes
`position_i_gain = 25` to address 82 of motor `body_rotation` (ID 10) on every
backend construction, because that register is volatile and returns to 0 on
every power cycle. No other register is touched. The gain was bracketed on
hardware: I=12 still fell 0.97° short, I=25 gives 0.39/0.53/0.70° endpoint
error at ±20/45/90° with a single crossing, I=50 oscillates and I=200 hunts at
roughly 2 Hz and jammed the robot. A D gain was tested and rejected: it acts on
noisy stick-slip position and made overshoot worse (1.92° -> 2.72°).

The same layer owns the ±120° body-yaw clamp, because the clamp exists only
*because* of the gain. With a non-zero integrator the servo never gives up: sent
a target the mechanism cannot reach, it winds up and pushes until something
gives. That jammed this robot twice — at ±160° (I=200, 567 PWM, motor overload)
and at -150° (I=25, stalled at -143.6° holding 324 PWM). The reachable envelope
is asymmetric, roughly +150° / -143°. The clamp is applied *before* inverse
kinematics, so the six Stewart joints are solved from the bounded angle; a
post-IK clamp would bound joint 0 while leaving the head solving the unclamped
angle. It is also applied again immediately before publishing, which closes the
window between an IK update and the publish. `set_head_operation_mode` is
deliberately **not** clamped: it pins the target to the *measured* position, and
clamping a measured angle already beyond the bound would command exactly the
jump that pin exists to prevent.

The verifier for this layer fails closed. It refuses to report success unless
the gain is confirmed in force — from live motor telemetry when
`movement-diagnostics` is enabled, otherwise from the status record the hook
writes at apply time, validated by the writing process still being alive rather
than by wall-clock age. An earlier version returned success when telemetry was
unavailable, which let the manager print "enabled and verified" while the
register write had failed.

Because the hook writes at backend construction, `deploy` alone does not put a
new version in force: the running daemon keeps the version it imported at
start. The verifier will correctly refuse a version mismatch. `disable` then
`enable` (each of which restarts the daemon) loads the new hook.

The face-loss-return hook leaves the daemon's two-second hold and fixed
world-neutral destination unchanged. Once loss is sustained, it caps only the
orientation change of the tracking aim at 40°/s. Ordinary detected-face
tracking and reacquisition still use the stock daemon response. Elapsed time
used for one setpoint is capped at 0.1 s, so a delayed control-loop tick cannot
produce a catch-up step larger than 4°. Translation retains the stock daemon
interpolation. This is the accepted permanent mitigation in the deployed
baseline, while remaining narrow enough to be deliberately superseded by a
future complete face-loss policy. It does not make the body-following app a
face-loss owner. The 40°/s value bounds commanded tracking-aim orientation; measured
actuator speed and the body controller's response remain hardware acceptance
questions. Slowing the head does not change the world-neutral endpoint or
guarantee that the body will not follow that return. In the current body app,
loss-driven relative yaw can reach Zone A's fixed 1.5 s goto or Zone B's
immediate absolute-target path; neither body path inherits this 40°/s cap.
Hardware acceptance must therefore test and measure the head alone before
enabling body following.

Regression coverage includes both initial torque states, failure during
`enable_motors()`, repeated IK updates, caller-array preservation, and
physical-backend-only patching, normal/failing/interrupted trajectory cleanup,
post-exit IK continuity, inward/outward monotonicity, motor-enable follow-up IK,
tracking IK before the first real waypoint in both directions, the actual
+130° to +123° direction-preservation fallback through normal completion, and
the exact Run-10 inward-handoff regression. Loss-return coverage adds timeout
boundaries, no first-tick jump, 3D rate limiting, delayed-loop protection,
stock translation, reacquisition, timeout-crossing, physical-backend scope,
both owned and production-style inherited revert paths, and manager
independence. I-gain coverage adds the exact gain value, register isolation (no
write outside address 82), refusal to guess the motor ID, the clamp on both the
pre-IK and the publish path, consistency of the returned IK solution with the
clamped angle, the absence of any unbounded publish, status-record liveness, and
manifest hashes. The current layer suite contains 73 tests.

## Local commands

Run the sibling manager from the development computer. It reads this catalog by
default; set `REACHY_LAYER_CATALOG_DIR` when the repositories are not siblings.

```bash
../reachy_mini_layer_manager/manage.sh deploy
../reachy_mini_layer_manager/manage.sh list
../reachy_mini_layer_manager/manage.sh verify
../reachy_mini_layer_manager/manage.sh enable movement-diagnostics
../reachy_mini_layer_manager/manage.sh disable movement-diagnostics
../reachy_mini_layer_manager/manage.sh enable face-frame-sync
../reachy_mini_layer_manager/manage.sh disable face-frame-sync
../reachy_mini_layer_manager/manage.sh enable face-loss-return
../reachy_mini_layer_manager/manage.sh disable face-loss-return
../reachy_mini_layer_manager/manage.sh enable body-motor-igain
../reachy_mini_layer_manager/manage.sh disable body-motor-igain
```

`deploy` copies the manager, bootstrap and existing diagnostic files to the
robot. It does **not** change the active daemon configuration and does not
restart the daemon.

The first `enable` or `disable` adopts the effective configuration it finds:
known old paths become the corresponding named layers and unknown paths are
preserved as the base `PYTHONPATH`. It then writes the single authoritative
`zzzz-reachy-local-layers.conf`. Older drop-ins remain recoverable on disk, but
the manager's later-sorting file owns the effective value from then on. Do not
use the older per-layer scripts for activation after adoption.

## Safety and restart behavior

`list` and `verify` are read-only. `enable` and `disable`:

1. require the daemon to report exactly one `motor_control_mode`, equal to
   `disabled`;
2. save the previous manager state and drop-in;
3. restart the daemon (not the robot);
4. wait for its API and verify the exact effective layer order and every active
   layer;
5. restore the previous files and restart again if anything fails.

This transaction detects a hook that fails during an intentional change. On a
later unattended daemon restart, however, the shared `sitecustomize` bootstrap
currently logs a hook exception and lets the daemon continue. An enabled
safety-motivated layer can therefore fall back to stock behavior without a
fail-closed daemon state. That limitation is unresolved and must not be
confused with the manager's transactional enable-time verification.

There is no physical power cycle. Python patches cannot be safely hot-unloaded
from the running process, so one daemon restart while the motors are disabled
is intentional. Enable diagnostics once for a batch of tests, then disable them
once afterward.

## Daemon updates

`list` reports the underlying installed `reachy-mini` package version, and each
layer declares the daemon versions against which it has been checked. A daemon
update does not silently make an old overlay compatible: `enable` and `verify`
will refuse an unsupported pairing.

The maintenance sequence is therefore: inspect the inventory, disable affected
layers while motors are disabled, update the daemon, rebuild/review the layer
against that version, enable it, and let the manager restart and verify it.

## On-robot files

- `/usr/local/bin/reachy-layers` — command entry point
- `/etc/reachy-mini-layer-manager/layers.json` — layer definitions
- `/etc/reachy-mini-layer-manager/state.json` — adopted enabled/base state
- `/opt/reachy-mini-layers/bootstrap/` — the single hook loader
- `/opt/reachy-mini-layers/movement-diagnostics/` — diagnostic code, installed
  but inert when disabled
- `/opt/reachy-mini-layers/body-yaw-calibration/` — body-yaw correction,
  installed but inert when disabled
- `/opt/reachy-mini-layers/face-loss-return/` — permanent loss-return limiter,
  installed but inert when disabled
- `/opt/reachy-mini-layers/body-motor-igain/` — body-motor I-gain write and
  body-yaw clamp, installed but inert when disabled
- `/tmp/reachy_body_motor_igain.json` — the status record the hook writes at
  apply time, read by the verifier when live telemetry is unavailable
- `.../zzzz-reachy-local-layers.conf` — the only manager-owned daemon drop-in

Deploying manager files does not enable the body-yaw correction. Activation is
a separate `enable body-yaw-calibration` operation while motors are disabled.
