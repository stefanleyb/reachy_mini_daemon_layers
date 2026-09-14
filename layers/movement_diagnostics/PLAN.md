# Movement Diagnostics Layer Plan

- Catalog version: `2026-09-13.1`.
- Remain temporary, read-only and independently enabled.
- Own the target-chain and motor-telemetry instrumentation source here; the
  diagnostic client consumes its output contract without owning daemon hooks.
- Do not infer robot activation from catalog presence.
