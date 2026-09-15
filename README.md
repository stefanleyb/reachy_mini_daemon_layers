# Reachy Mini Daemon Layers

Small, reversible fixes for your Reachy Mini's own software — each one packaged
so you can switch it on, check it worked, and take it back off.

This is the collection. The machinery that installs and verifies them lives in
the companion repository,
[reachy_mini_layer_manager](https://github.com/stefanleyb/reachy_mini_layer_manager).
You need both.

## What's in here

| Layer | What it fixes, in plain terms |
|---|---|
| **body-motor-igain** | The body motor stops short of where it was told to go. This makes it finish the move. On the robot it was built for, it cut the error from 3.5–5.5° to about 1°. |
| **body-yaw-calibration** | The body's turns are slightly off, and moves start with a visible jolt. This corrects the angle and makes moves start and end smoothly. |
| **face-loss-return** | When the robot loses sight of a face, the head can snap back fast enough to be startling. This caps how quickly it returns. |
| **movement-diagnostics** | Temporary, read-only. Exposes what the robot is *actually* being told to do internally, so you can measure problems instead of guessing. Not a correction — turn it off when you're done. |
| **face-frame-sync** | The face tracker uses each camera frame with the head position from the *wrong* moment, causing overshoot. Proposed upstream as [PR #1396](https://github.com/pollen-robotics/reachy_mini/pull/1396) — if that merges, this layer becomes unnecessary. |

⚠️ **These were tuned on one specific robot.** Motor behaviour varies between
units. `body-motor-igain` in particular sets a value that suited *that* robot —
yours may need a different one, or none at all. Read what a layer does before
enabling it, and treat the numbers as a starting point rather than a
prescription.

## Getting started

**1.** Clone both repositories side by side:

```
some-folder/
├── reachy_mini_layer_manager/     ← the tooling
└── reachy_mini_daemon_layers/     ← this repository
```

**2.** From the *manager* repository, install onto the robot. Nothing is switched
on yet:

```bash
cd ../reachy_mini_layer_manager
./manage.sh deploy
```

**3.** Disable the robot's motors, then enable the layer you want:

```bash
./manage.sh enable face-loss-return
```

**4.** Check it took effect, and switch it off again whenever you like:

```bash
./manage.sh verify
./manage.sh disable face-loss-return
```

If enabling fails its check, the previous state is restored for you.

## How a layer is put together

Each folder under `layers/` holds the layer's code plus a `verify.py` that
proves it is genuinely in force — not merely installed. `layers.json` is the
catalog: for every layer it records the version, install path, ordering,
which daemon versions it suits, and a SHA-256 for each file, so a modified or
partially copied file is detected rather than silently used.

Most layers hook the daemon through a single `apply()` entry point called at
start-up. `face-frame-sync` is the exception: it is a package overlay of patched
daemon source, which is why its folder holds documentation rather than code.

## Compatibility

Built and tested against **daemon/SDK 1.10.0** on a Reachy Mini Wireless. Each
layer names the daemon versions it suits, and the verifier refuses to report
success on a mismatch. After a daemon update, re-verify before trusting a layer.

## Running the tests

No robot required:

```bash
python tests/test_catalog.py          # the catalog itself: hashes, ordering, verifiers
python tests/test_body_motor_igain.py
python tests/test_body_yaw_calibration.py
python tests/test_face_loss_return.py
```

## Status and licence

Working tooling from a personal project, not an officially supported product.
Not affiliated with Pollen Robotics.

---

# Technical reference

The material below assumes familiarity with the daemon internals.

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
