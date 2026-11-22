# Mimosa Validation Log

This is a living document tracking end-to-end validation work on `mimosa`
(crash fixes, accuracy evaluation against public datasets, config review,
visualization tooling). It is meant to be read and updated across sessions —
by different agents and by team members reviewing the work. When you pick
this up:

- Update the **Current Status** table in place.
- Append a new dated entry under **Session Log** rather than editing old
  entries (old entries are a historical record).
- Move an item from **Open Items** to **Session Log** when it's resolved,
  and update **Known Issues** accordingly.

## Current Status (as of 2026-11-22)

| Sequence (ENWIDE, profile `enwide`) | Runs without crashing | ATE RMSE | RPE RMSE @10m | coin-lio baseline (ATE / RTE) |
|---|---|---|---|---|
| field_s | yes (after fix below) | 0.276 m | 0.059 m (0.6%) | 0.743 m / 1.60% |
| field_d | yes (after fix below) | 0.217 m | 0.079 m (0.8%) | 0.581 m / 1.83% |
| tunnel_s/d, intersection_s/d, runway_s/d, katzensee_s/d | **not yet tested** | — | — | — |

mimosa currently beats the coin-lio baseline on both tested sequences. The
untested sequences include the dataset's actual degenerate-geometry cases
(tunnel, intersection) — see Open Items, this matters.

## Known Issues

1. **(FIXED on branch `doron_changes`)** `graph::Manager::getStateUptoNoLock`
   (`src/mimosa/src/graph/manager.cpp`) threw an uncaught `std::runtime_error`
   and aborted the whole node when a LiDAR scan's raw header timestamp was
   older than the graph's earliest state by more than 4ms — which happens
   reliably right at initialization when LiDAR filtering latency (~100ms)
   is comparable to the scan period and `pose_init_wait_secs` is short (the
   `enwide` profile uses 0.2s). 100% reproducible on both `field_s` and
   `field_d` before the fix. Fixed by extending the existing "within 4ms,
   use the first state" leniency to the general case instead of throwing;
   `declareNoLock` already handles this situation gracefully for the actual
   graph update, so this only affects a prior-seeding lookup, not graph
   correctness.
2. **(Open)** `enwide/params.yaml` has `project_on_degneneracy: false` and
   `degen_thresh_rot: 0` (degeneracy-aware registration effectively
   disabled), despite ENWIDE's entire premise being degenerate-geometry
   robustness (tunnel/corridor/runway sequences). Worth confirming whether
   this is deliberate before trusting results on those sequences — a poor
   result there could mean "safeguard was off," not "mimosa can't handle
   degeneracy."
3. **(Open)** `enwide/params.yaml` sets `integration_sigma: 0` with the
   config's own comment "Unclear what this is. Set to 0 for now." The
   `m113` profile sets it to `1.0e-4` with an explanation (numerical
   regularization for GTSAM's integration step). Inconsistent across
   profiles; worth resolving even if `0` turns out to be harmless.
4. **(Cosmetic)** Inline comments on `degen_thresh_trans`/`degen_thresh_rot`
   reference meters/degrees (`# 0.01 # m`, `# 0.03 # 0.05 # deg`), but the
   runtime field type is actually `num_virtual_features` (unitless) — stale
   documentation from a prior formulation of degeneracy detection.
5. **(Low confidence, worth checking)** The IMU noise comment
   `# Datasheet ICM-20948` (copy-pasted identically into `newer_college`'s
   config) may not match the Ouster OS0-128 Rev D's actual internal IMU —
   one external source (the `evalio` project's dataset docs) names it as a
   TDK IAM-20680HT. Doesn't affect the tuned noise values, just their
   documented provenance.

## Evaluation Methodology Notes

- **ENWIDE ground truth has no real orientation** — every pose's quaternion
  is the identity placeholder `(0,0,0,1)`. This makes `evo_rpe -r trans_part`
  invalid: it composes relative translation through each pose's rotation,
  and comparing a "rotated-by-real-estimate-orientation" relative vector
  against a "rotated-by-fake-identity" one produces huge, meaningless
  numbers (we saw ~10m RMSE on a trajectory with 0.2m ATE before catching
  this). Use `-r point_distance` instead (plain Euclidean distance between
  positions, no rotation involved) — this is what the repo's own
  `scripts/dataset_evaluation.py` already does; `scripts/evaluate.sh` uses
  `trans_part` but was written for a different dataset with real orientation
  ground truth, so don't copy its flags for ENWIDE.
- `graph_manager_odometry.tum` (the TUM trajectory log under
  `logs_directory`) interleaves IMU-rate forward predictions with the
  lower-rate graph-optimized corrections in one file (both go through
  `Manager::publishStateEstimate`), so ~7% of consecutive lines go backward
  in time by a few ms. Didn't end up mattering for ATE/RPE here, but if a
  future metric depends on strict path monotonicity (e.g. arc-length-based
  pairing), filter to strictly increasing timestamps first.
- `scripts/dataset_evaluation.py` and `scripts/evaluate.sh` are both
  hardcoded to the original authors' personal machine paths
  (`/home/cyanide/...`, `/Downloads/enwide`) and `dataset_evaluation.py`
  references the stale `lidar_manager_odometry.log` path rather than the
  current `graph_manager_odometry.tum`. Neither runs as-is; both would need
  updating before relying on them again.

## How to Reproduce This Evaluation

1. Download ENWIDE sequences from
   `http://robotics.ethz.ch/~asl-datasets/2024_ICRA_ENWIDE/<seq>/` (bag +
   `gt-<seq>.csv`) — this mirror was unreachable from inside the sandboxed
   agent environment (connection refused/timeout on both plain curl and the
   WebFetch tool), so this currently has to be done from a regular browser.
   Official dataset page: `https://projects.asl.ethz.ch/datasets/enwide`.
2. The dataset's bags are ROS1 (`.bag`). Convert with the `rosbags` Python
   package (installed in `mimosa_ws/.venv`):
   `rosbags-convert --src <seq>.bag --dst ros2_bag --src-typestore ros1_noetic --dst-typestore ros2_jazzy`
3. Run: `ros2 launch mimosa mimosa.launch.py profile:=enwide viz:=false use_sim_time:=true`,
   then in another terminal: `ros2 bag play <ros2_bag_dir> --clock --disable-keyboard-controls`.
4. Copy `/tmp/mimosa/graph_manager_odometry.tum` out before the next run
   overwrites it.
5. Evaluate (venv has `evo` installed):
   `evo_ape tum gt-<seq>.csv est.tum -a -r trans_part`
   `evo_rpe tum gt-<seq>.csv est.tum -r point_distance --all_pairs -a -d 10 -u m`

## Visualization

RViz (`profile:=enwide viz:=true`) now has a third "Ground Truth" path
display (green, `src/mimosa/mimosa/rviz/mimosa.rviz`) alongside the existing
live/optimized path displays, subscribed to `/gt/path`. It expects a
`nav_msgs/Path` in the `mimosa_map` frame. Since ENWIDE's ground truth is in
its own (Leica total-station) frame, it needs the inverse of the per-sequence
Umeyama alignment (from the `evo_ape -v` output) applied before publishing,
mapping GT points into mimosa's native, unaligned frame:
`p_mimosa = R^T @ (p_gt - t)` where `R, t` are evo's printed EST→REF alignment.
A throwaway publisher script doing this lived only in the agent session's
scratchpad, not in the repo — it republishes the full GT path once per
second (not synced to playback time, so it appears all at once rather than
growing with the bag). If this is useful going forward, it should be
promoted into `scripts/` with the alignment computed automatically rather
than pasted in by hand per sequence.

## Open Items

- [ ] Run the remaining 6 ENWIDE sequences, especially the degenerate ones
      (tunnel, intersection) — the real point of this dataset.
- [ ] Resolve Known Issue #2 (degeneracy protection off) before trusting
      results on degenerate sequences.
- [ ] Resolve Known Issue #3 (`integration_sigma: 0`).
- [ ] Decide whether the GT-overlay RViz tooling is worth productizing into
      `scripts/`.
- [ ] `scripts/dataset_evaluation.py` / `evaluate.sh` need rewriting (paths,
      stale log reference, RPE pose_relation) if they're meant to be used
      again rather than superseded by ad hoc runs like this one.

## Session Log

### 2026-11-22 — Initial ENWIDE validation, crash found and fixed
- Set up `enwide` evaluation from scratch: no dataset present locally, no
  `evo`/`rosbags` installed. Installed both into `mimosa_ws/.venv`.
- Downloaded `field_s`/`field_d` manually (sandbox couldn't reach the
  dataset mirror), converted ROS1→ROS2.
- First runs crashed 100% of the time (both sequences) — root-caused to
  `graph::Manager::getStateUptoNoLock`, see Known Issue #1.
- Branched to `doron_changes`, fixed it, rebuilt, re-ran both sequences
  clean to completion.
- First RPE numbers were nonsense (~10m RMSE despite ~0.2m ATE) — traced to
  wrong `evo_rpe` pose_relation for position-only ground truth, see
  Evaluation Methodology Notes.
- Got clean, trustworthy ATE/RPE for both sequences (table above); both beat
  the coin-lio baseline.
- Reviewed the rest of `enwide/params.yaml` by hand; filed Known Issues
  #2–#5 from that review.
- Added RViz ground-truth path overlay (see Visualization).
