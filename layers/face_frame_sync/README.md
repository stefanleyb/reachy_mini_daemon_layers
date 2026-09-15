# Face Frame-Synchronization Overlay

The catalog registers the accepted `08d9e40ca` package overlay installed on the
robot. Its source history remains in the upstream Reachy Mini checkout rather
than being copied into this catalog.

The overlay is proposed upstream as
[pollen-robotics/reachy_mini#1396 — *fix(tracking): prevent overshoot from stale
camera frames*](https://github.com/pollen-robotics/reachy_mini/pull/1396),
on branch `fix/face-tracking-frame-pose-sync`. Commit
`08d9e40ca` on that branch is exactly the version this catalog records, so if
the pull request is merged upstream this layer becomes unnecessary: the fix
would ship in the daemon itself.

The files and hashes recorded in `../../layers.json` define what the manager
verifies at the installed overlay path.
