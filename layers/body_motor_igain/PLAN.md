# Body Motor I-Gain Layer Plan

- Catalog version: `2026-09-14.4`.
- Preserve the verified I-gain write, pre-IK/publish clamp, runtime motor-ID
  derivation and fail-closed verifier.
- Revalidate compatibility before enabling it with any daemon version other
  than those declared in `layers.json`.

Catalog state does not by itself establish whether the layer is currently
enabled on the robot.
