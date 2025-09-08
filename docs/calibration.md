# Calibration & Time Sync

- All sensors synced via PTP.
- Mid-360: per-point UNIX ns timestamps (`timestamp`).
- Ouster: per-point offset `t` in ns, plus frame-level stamp.
- Avia: frame-level stamps only.
