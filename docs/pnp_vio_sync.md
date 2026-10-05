# PnP–VIO synchronization

The drift corrector retains 10 seconds of VIO poses (`sync_history_s`), rather
than 300 samples whose duration shrinks with the input frequency. PnP waits up
to 5 wall-clock seconds (`sync_wait_s`) for exact-stamp or bracketing VIO samples.
Bracketing poses must be no more than `max_dt` (0.15 s) apart. Position is linearly
interpolated; the synchronizer never substitutes a newer pose or extrapolates.
The pending queue is bounded at 500 PnP messages, and VIO history at 10000 samples.

The KF advances in measurement timestamp order. Output uses a temporary copy
predicted to the VIO output timestamp, stopping drift extrapolation after
`hold_timeout`. Publishing output does not advance the measurement-time state.
Duplicate measurements and measurements older than the last applied update are
rejected explicitly; queued measurements are sorted. Restart the estimator when
resetting simulation time so measurements from different clock epochs cannot mix.

`PnP sync` logs report received, matched, duplicate_or_late, wait_timeout,
vio_gap, history_expired, overflow, pending, distance rejects and invalid/old
VIO inputs. These counts diagnose gaps instead of silently discarding PnP.
The full corrected Path is sampled and published at `path_rate_hz` (5 Hz),
while corrected odometry and drift continue at the VIO callback rate.

Validation: delayed and reordered samples, duplicate suppression, interpolation,
missing/expired history, large pose gaps and bounded queues are unit tested.
A recorded cross flight (20261005T155706Z_cross_6f80a254) replayed in recorder
arrival order matched 307 of 308 PnP messages; one timed out without a bracket.
The old matcher matched 308 offline, despite only 139 online KF updates in the
original process log. Recorder arrival order is not the corrector callback order;
this replay does not reproduce or prove the original real-time scheduling issue.
A fresh flight is required to verify sustained update counts and trajectory error.
