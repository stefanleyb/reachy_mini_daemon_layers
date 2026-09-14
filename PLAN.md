# Daemon Layers Plan

## Current responsibility

Own the source, manifest metadata, verification code and compatibility boundary
of each independently enabled daemon layer. The manager consumes this catalog;
it does not own these implementations.

## Current work

- Keep manifest versions, compatibility declarations, required files and hashes
  synchronized with every layer release.
- Preserve cross-layer ordering and integration tests.
- Replace workspace-relative source borrowing with files owned by this
  repository.

Each layer has a short local plan. Robot enabled state is established only by a
fresh manager status check; catalog presence does not establish activation.
