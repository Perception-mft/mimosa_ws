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

| Sequence (ENWIDE, profile `enwide`) | Outcome | ATE RMSE | RPE RMSE @10m | coin-lio baseline (ATE / RTE) |
|---|---|---|---|---|
| field_s | clean run (after fix below) | 0.276 m | 0.059 m (0.6%) | 0.743 m / 1.60% |
| field_d | clean run (after fix below) | 0.217 m | 0.079 m (0.8%) | 0.581 m / 1.83% |
| tunnel_s | ran, but graph didn't initialize until 66.3s into the 238s recording (see Known Issue #6) | 0.781 m | 0.149 m (1.49%) | 0.743 m / 1.60% |
| tunnel_d | **never initialized at all** — no crash, just an empty output trajectory (see Known Issue #6) | — | — | 0.581 m / 1.83% |
| intersection_s/d, runway_s/d, katzensee_s/d | not yet tested | — | — | — |

mimosa beats the coin-lio baseline on both field sequences. On tunnel_s it's
roughly on par (slightly worse ATE, slightly better RPE) but that comparison
isn't apples-to-apples — mimosa's trajectory is missing the first 66s (very
plausibly the actual tunnel entrance, the hardest/most degenerate part).
tunnel_d produced nothing at all. See Known Issue #6 before concluding
anything about mimosa's degeneracy handling from these two results.

| Sequence (RELLIS-3D, profile `rellis`) | Outcome | Notes |
|---|---|---|
| `example_merged` (60s "Merged" example) | clean run, no crash | net displacement 54.5m vs wheel-odom's 51.8m (~5% off); but mimosa's total path length (83.7m) is ~50% longer than wheel odom's (56.2m) — see RELLIS-3D section below for interpretation. |
| `00002` full sequence (~415s, `20200215_hill_1.bag`) | **clean on every run since the Known Issue #10 fix** (2/2 full-length completions, flat ≤27ms per-scan cost); all pre-fix crashes now suspected to be a symptom of #10, not an independent bug | screen recording of a pre-fix crashing run is at `results/rellis_00002_run_20261125_085311.mp4`; post-fix full clean run at `results/rellis_00002_run_20261125_105711.mp4`. GPS ground-truth path (no curated GT exists) extracted, plotted, and overlaid in RViz — see Visualization section. Also surfaced and fixed Known Issue #10 (180° heading bug); post-fix heading matches GPS within 3-7°, distance within ~2-5% (`results/rellis_00002_run4_estimate.tum`). |

RELLIS-3D has no ground-truth trajectory in this clip, so this is a sanity
check (does it track roughly the right shape/distance), not an ATE/RPE
evaluation.

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
6. **(Open, limitation not a bug)** mimosa bootstraps by waiting for a
   stationary IMU window (`imu::Manager::estimateAttitude`,
   `src/imu/manager.cpp:225`) gated on `pose_init_max_acc_std` (default
   0.2 m/s²), `pose_init_max_gyro_std` (0.02 rad/s), `pose_init_max_gyro_mean`
   (0.05 rad/s), `pose_init_max_gravity_error` (1.0 m/s²) — `enwide` doesn't
   override any of these. On `tunnel_d` (a "dynamic"/running ENWIDE
   sequence), real debug-level numbers pulled from a live run showed
   `gyro_std`/`gyro_mean` sitting at 0.03–0.7 rad/s — 2 to 35x over
   threshold — for the entire 119s recording, so the graph never once
   initializes: no crash, just a silently empty output trajectory. On
   `tunnel_s` (the "smooth"/walking variant) it eventually finds one
   qualifying moment, but not until 66.3s in. Any sequence with no initial
   stationary pause will hit this. Worth deciding whether to (a) loosen the
   thresholds via config, (b) add a non-stationary initialization path, or
   (c) accept this as a documented precondition for using mimosa.
7. **(Config-authoring gotcha, not a code bug)** `lidar.photometric`'s
   constructor needs its *entire* field set populated (FIR filter arrays,
   `edgelet_patch_offsets`, etc.) even when `enabled: false` — it builds
   fixed-size lookup tables unconditionally. A new profile (`rellis`) that
   initially specified only `log_level`+`enabled` under `photometric`
   segfaulted (SIGSEGV) during `mimosa_node` startup. Fix: copy the full
   `photometric:` block from an existing `enabled: false` profile (`euroc`
   or `m113`) verbatim when photometric isn't being used yet. Worth fixing
   properly upstream (give the struct real defaults so a minimal disabled
   block is actually safe) rather than relying on every new profile author
   knowing to copy-paste this.
8. **(Open, possible pre-existing typo)** While deriving `rellis`'s
   `T_B_S` from RELLIS-3D's bag-embedded tf tree, the arithmetic
   (lidar-to-sensor z offset minus imu-to-sensor z offset, both read from
   the Ouster metadata: 36.17mm - 7.645mm = 28.525mm) landed almost exactly
   on `enwide/params.yaml`'s own committed `T_B_S` z component — except
   `enwide`'s is `0.0028525` (one decimal digit short of `0.028525`, i.e.
   off by 10x). `enwide` is already validated with low ATE, so this either
   doesn't matter much in practice (z-translation error partially absorbed
   by the graph) or there's a compensating error elsewhere — not changed
   here since it's out of scope for the `rellis` work, but worth a
   deliberate look.
9. **(Likely a symptom of Known Issue #10, not an independent bug —
   unreproduced since that fix)** On the full `00002` RELLIS-3D sequence
   (~415s, vs the 60s demo clip Known Issue #7 was tested against),
   `mimosa_node` crashed (SIGABRT) with an uncaught `std::runtime_error`
   from `imu::Manager::getInterpolatedMeasurements`
   (`src/imu/manager.cpp:412`, "IMU lookup requiring first message of the
   buffer ... This should never happen"). Proximate mechanism: per-scan
   LiDAR processing time climbed steadily over the run (from ~10-20ms early
   on to 100-330ms later), eventually exceeding the 10Hz scan arrival rate
   and the `rellis` profile's `max_buffer_duration: 1.1s` IMU rolling-buffer
   window — a real-time backlog, not a one-off timestamp bug. Confirmed
   across three separate runs with the crash landing at a different point
   each time (~207s, then ~344s and ~365s in two runs done while also
   screen-recording the RViz window, which adds CPU contention) —
   consistent with a resource-accumulation crash rather than a fixed repro
   timestamp. Originally suspected driver: the local scan-to-map voxel
   structure growing unbounded as the vehicle covers real outdoor terrain
   over several minutes (the 60s clip never ran long enough for this to
   matter).
   **Update — root cause revised**: all runs that exhibited the climbing
   processing-time trend and/or crashed were run with the *pre-fix* broken
   `T_B_S` rotation (Known Issue #10, identity instead of 180°-about-Z).
   Checked every full-length attempt by comparing each config file's mtime
   against each run's launch timestamp: `run2` (09:47:58) and `run3`
   (10:07:25) both predate the #10 fix (applied 10:27:37/10:27:56) and
   neither reached the end of the bag; `run4` (10:28:27) and a later run
   (10:57:11) both postdate the fix and both completed the *entire* 415s
   sequence cleanly, reaching the identical final bag timestamp
   (`1581797565.14`) with "process has finished cleanly." Per-scan
   processing time in both post-fix full runs is flat and bounded for the
   whole run — `run4`: 3464 scans, mean 8.67ms, max 24.85ms, 0 scans over
   100ms; the later run: 3135 scans, mean 9.26ms, max 27.09ms, 0 scans over
   100ms (`results/rellis_00002_run4_launch.log`,
   `results/rellis_00002_run_launch.log`) — no trace of the previous
   climbing-backlog pattern at all. Likely mechanism: with the point cloud
   180° misrotated relative to the growing map, scan-to-map ICP had to work
   harder (worse convergence) as the map grew, which produces exactly this
   kind of slow upward per-scan cost over a multi-minute run; once the
   rotation was fixed, ICP converges cheaply throughout and the backlog that
   fed Known Issue #9 never accumulates.
   **Status**: the crash code path itself
   (`imu::Manager::getInterpolatedMeasurements`'s uncaught throw) is still
   present and unguarded in `src/imu/manager.cpp` (unchanged since
   `9d2a76b`, 2026-11-18) — this was never patched, only worked around by
   fixing #10 — so it should stay on the radar as a latent robustness gap.
   But with 2/2 full-length clean runs post-fix and zero full-length clean
   runs pre-fix, there's no current evidence it's reachable under normal
   conditions anymore. Treating the three previously-proposed fix options
   (graceful degradation / bound map-growth cost / document-and-move-on) as
   deprioritized rather than needing an immediate decision; revisit if a
   future long run on the corrected config ever reproduces it.
10. **(FIXED on branch `doron_changes`)** mimosa's `lidar::Manager` applied
    zero rotation when consuming raw Ouster LiDAR points (`T_B_S` rotation
    was identity in both `enwide` and `rellis`), silently assuming the
    points already shared the body/IMU frame's axes. They don't: the
    Ouster's `lidar_to_sensor_transform` metadata (present in both
    `enwide/os_enwide.json` and the generic `os_dummy.json` `rellis` uses —
    a fixed Ouster firmware constant, not per-sensor) encodes a 180°-about-Z
    rotation between the `os_lidar` frame points are published in and the
    `os_sensor`/`os_imu` frame mimosa treats as the body frame
    (`src/lidar/manager.cpp:18` only ever extracted the z-translation
    component of this matrix, discarding the rotation). Independently
    confirmed via the RELLIS bag's own `/tf_static`
    (`os1_sensor -> os1_lidar` = 180° about Z) and the point cloud's own
    `header.frame_id` (`ouster1/os1_lidar`, not `os1_sensor`). Effect: a
    constant 180° heading bias on the entire estimated trajectory/map —
    self-consistent internally (ICP/odometry still converge fine within the
    rotated frame), so it was invisible to ENWIDE's `evo_ape -a` evaluation
    (Umeyama alignment silently absorbs a constant rotation offset before
    computing ATE) but directly visible once the unaligned RELLIS GPS
    overlay was compared against a live run — this is how it was found.
    **Fix**: changed `T_B_S`'s rotation quaternion from identity to
    `(0,0,1,0)` (180° about Z) in both `rellis/params.yaml` and
    `enwide/params.yaml`; the existing translation component needed no
    change (verified by composing the tf chain explicitly — the
    `imu↔sensor` leg has identity rotation, so translations combine by
    simple addition independent of this rotation). Config-only fix, no
    rebuild needed (`--symlink-install`). **Verified**: comparing mimosa's
    post-fix estimated trajectory (`results/rellis_00002_run4_estimate.tum`)
    against the GPS ground truth, heading now matches within 3-7° (was
    ~180° before) and distances match within ~2-5%, at multiple intervals
    (first 20s, first 60s, full run) — see Visualization section.
    `T_B_OdometryLoggerFrame` (a separate, purpose-built output-logging
    transform, unrelated to point consumption) was deliberately left
    untouched. Not checked: whether `hornbill`/`magpie`/`parrot`/`lapwing`
    (other Ouster-based profiles, per their own metadata) have the same
    bug — out of scope this session.

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

**RELLIS-3D reuses the same "Ground Truth" display** — no `mimosa.rviz`
changes needed, confirmed by publishing to `/gt/path` and screenshotting the
result. Source: `/vectornav/GPS` (`sensor_msgs/NavSatFix`), extracted from
the original ROS1 bag (`rosbags-convert --include-topic /vectornav/GPS
--include-topic /vectornav/Odom`, since the main `rellis` conversion
deliberately dropped these — see RELLIS-3D Profile section) into
`datasets/rellis/00002_hill_1_gps_ros2`. Lat/lon converted to a local ENU
tangent plane anchored at the first fix (flat-earth approximation, fine at
this ~100m scale). A static 3D plot of this same GPS trajectory (colored by
elapsed time) is at `results/rellis_00002_gps_trajectory_3d.png`, with the
raw data in `results/rellis_00002_gps_trajectory.csv`.

**Sensor-layout extrinsic between the Vectornav and the Ouster IMU IS
applied** (this was initially missed — the first pass anchored the raw GPS
antenna position directly, with no lever-arm correction). `/vectornav/GPS`
and `/vectornav/Odom` are both stamped in frame `imu_link`, which this
bag's own `/tf_static` places relative to the Ouster via
`imu_link -> os1_frame -> ouster1/os1_sensor -> ouster1/os1_imu` (same tree
Known Issue #8's `T_B_S` derivation used). Composing that chain gives a
~0.46m lever arm `[0.456253, 0.011775, -0.077645]` (Vectornav body axes)
from the Vectornav to the Ouster's internal IMU — mimosa's body origin for
this profile. Each GPS fix is corrected by this lever arm rotated by the
nearest-in-time `/vectornav/Odom` orientation (GPS itself has no
orientation), producing `results/rellis_00002_gps_realtime.csv` (columns:
`t_abs,x,y,z`, absolute epoch matching LiDAR/IMU header stamps).

**Update: the large heading mismatch this caveat originally warned about
was real, but it was mimosa's bug, not a missing alignment step.** Comparing
this unaligned overlay against a live run is exactly what surfaced Known
Issue #10 (the LiDAR 180°-about-Z rotation bug) — the GPS path appeared
rotated ~180° from mimosa's estimate. After that fix, heading now matches
within 3-7° with no alignment applied at all (see Known Issue #10's
verification numbers). A residual few-degrees offset and the lever-arm
approximation above are the only remaining caveats; a real Umeyama-style
alignment (via `evo`, same as ENWIDE) would still be the right way to get
an actual ATE/RPE number rather than eyeballing/diffing headings, but it's
no longer hiding a gross error the way it did before Known Issue #10 was
found.

**Realtime (sim-time-synced) playback**: `tum_path_publisher.py` dumps the
whole static path at a fixed wall-clock rate, which doesn't grow alongside
a bag replay. Added `scripts/sim_time_path_publisher.py` — a generic node
(not RELLIS/GPS-specific) that takes any `t_abs,x,y,z` CSV and only
publishes poses whose timestamp has already been reached in ROS sim time,
so the `/gt/path` line grows in step with `ros2 bag play --clock`. Needs
`use_sim_time:=true` and a live `/clock` source. Verified against a 10x-
speed bag replay + RViz-only (no `mimosa_node` needed, since `/gt/path`'s
frame_id matches the Fixed Frame directly): screenshots at two points in
the replay showed a visibly longer partial path, confirming growth, not a
static dump.
  `python3 src/mimosa/mimosa/scripts/sim_time_path_publisher.py results/rellis_00002_gps_realtime.csv --frame_id mimosa_map --topic_name /gt/path --hz 5.0`

## RELLIS-3D Profile

Added a `rellis` profile (`src/mimosa/mimosa/config/rellis/params.yaml`,
wired into `PROFILES` in `launch/mimosa.launch.py`) for the
[RELLIS-3D dataset](https://github.com/unmannedlab/RELLIS-3D) — real
off-road data from a Warthog UGV (Texas A&M / CCDC ARL), chosen because the
other configured-but-unused profiles (`hornbill`/`lapwing`/`magpie`/`parrot`)
are the authors' own private platforms with no public data, and
`newer_college` turned out to be gated behind a request form, not a direct
mirror (no automated access possible; see that dataset's `/download/` page).

What's wired: only `/os1_cloud_node/points` (Ouster OS1 LiDAR) +
`/os1_cloud_node/imu` (its own embedded IMU) — the same "LiDAR with its own
internal IMU" pattern as `enwide`, deliberately skipping the dataset's
separate Vectornav VN-300 IMU/GPS and wheel odometry for a first pass.
`lidar.photometric.enabled: false` (see Known Issue #7 for why its block
still needs to be fully populated).

**First real-data run** (the "Merged" 60s example, converted ROS1→ROS2 with
only `/os1_cloud_node/{points,imu}` + `/tf`/`/tf_static` kept via
`rosbags-convert --include-topic`): ran clean end-to-end, no crash, one
benign "older than first state" warning right at init (same Known Issue #6
edge case, already handled without crashing). The dataset has no
Leica/GPS-style ground truth in this clip, so there's no ATE/RPE number —
instead cross-checked against the bag's own
`/warthog_velocity_controller/odom` wheel odometry (not fed into mimosa at
all, purely an external sanity check):
- Net displacement: mimosa 54.5m vs wheel-odom 51.8m (~5% apart) — the
  right ballpark, rules out a grossly wrong extrinsic/scale.
- Total path length: mimosa 83.7m vs wheel-odom 56.2m — mimosa's path is
  ~50% longer, i.e. visibly more wobble/noise than the wheel odometry's
  near-straight-line path. Plausibly from the still-unverified IMU noise
  values (borrowed from `enwide`) and/or the `T_B_S` extrinsic below (now
  tf-derived but still not independently verified) rather than a sign of a
  fundamental problem, but this needs more than one clip to actually
  characterize.

`lidar.T_B_S` is no longer an identity placeholder: it's now derived from
this bag's own `/tf_static` (`ouster1/os1_sensor` → `ouster1/os1_imu`,
`ouster1/os1_sensor` → `ouster1/os1_lidar`), composed with **identity
rotation** — matching `enwide`'s existing convention, since mimosa's
`lidar::Manager` only ever consumes a per-point z-offset from the Ouster
metadata JSON (`src/lidar/manager.cpp:18`,
`z_offset_ = -lidar_to_sensor_transform[11] / 1000`); it never applies an
extrinsic *rotation* from that metadata, so the 180°-about-Z rotation this
bag's `/tf_static` reports between `os1_sensor` and `os1_lidar` should not
be baked into `T_B_S` either. Translation: `[-0.006253, 0.011775, 0.028535]`
(sensor→imu offset negated, stacked with the lidar→sensor z offset). Still
flagged as best-effort, not independently calibrated — see Known Issue #8
for an arithmetic discrepancy this turned up in `enwide`'s own committed
value.

**Still unverified, before trusting results from this profile:**
- IMU noise densities are copied verbatim from `enwide`'s (itself borrowed/
  inflated from an "ICM-20948" datasheet assumption) — unverified for
  RELLIS-3D's actual OS1 embedded IMU.
- `pose_init_wait_secs: 1.0` (vs `enwide`'s 0.2) is a guess that a ground
  vehicle sits still briefly at recording start — unverified; see Known
  Issue #6 for what happens when that assumption is wrong. (It worked fine
  on this clip.)
- The `T_B_S` rotation-identity assumption above, and the z-translation
  arithmetic generally (Known Issue #8).

**Download note:** the dataset's smallest example ("Synced 60-second
example", 2GB) does **not** include the IMU topic per its own README table —
only `/os1_cloud_node/points` + camera topics; confirmed by inspecting its
connections directly (only `/pylon_camera_node/*` + `/os1_cloud_node/points`,
no IMU). Use the **"Merged" 60-second example (4.2GB)** instead — it has the
IMU and the tf-embedded extrinsic calibration used above. All downloads are
Google Drive links (no direct mirror), listed in RELLIS-3D's GitHub README.

## Open Items

- [ ] Investigate `rellis`'s excess path length vs wheel odometry (83.7m vs
      56.2m, see RELLIS-3D section) — try with verified-not-borrowed IMU
      noise values, or a longer/more varied RELLIS-3D clip, to tell apart
      "real estimation noise" from "this one clip is just noisy."
- [ ] Resolve Known Issue #8 (`enwide`'s `T_B_S` z component looks off by
      10x vs the metadata-JSON arithmetic) — confirm whether it's a real
      bug or intentional/compensated.
- [ ] Run the remaining 4 untested ENWIDE sequences (intersection, runway,
      katzensee).
- [ ] Decide how to handle Known Issue #6 (stationary-init requirement) —
      affects both `tunnel_d`'s total failure and `tunnel_s`'s 66s-late,
      partial-coverage result.
- [ ] Resolve Known Issue #2 (degeneracy protection off) before trusting
      results on degenerate sequences.
- [ ] Resolve Known Issue #3 (`integration_sigma: 0`).
- [ ] Known Issue #9 is deprioritized (now believed to be a downstream
      symptom of #10, unreproduced in 2/2 post-fix full runs) but its crash
      code path (`imu::Manager::getInterpolatedMeasurements`'s uncaught
      throw) is still unguarded — revisit if it ever reproduces again on
      the corrected config.
- [ ] Compute a real alignment (evo-style, like ENWIDE) between the RELLIS
      GPS ground-truth path and mimosa's estimated trajectory
      (`results/rellis_00002_run4_estimate.tum` exists now) for a real
      ATE/RPE number — the gross heading mismatch that made this seem more
      urgent turned out to be Known Issue #10 (now fixed); what's left is
      upgrading from a 3-7°-heading / ~2-5%-distance eyeball check to a
      proper aligned metric.
- [ ] Check whether `hornbill`/`magpie`/`parrot`/`lapwing` (other
      Ouster-based profiles) have the same Known Issue #10 rotation bug —
      not checked this session, out of scope.
- [x] ~~Decide whether the GT-overlay RViz tooling is worth productizing
      into `scripts/`.~~ Reused as-is for RELLIS-3D (second dataset) via
      the existing `tum_path_publisher.py` + a one-off TUM export — no new
      tooling needed, `mimosa.rviz`'s "Ground Truth" display is already
      dataset-agnostic.
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
- Downloaded and ran `tunnel_s`/`tunnel_d`. No crash on either (the fix
  held), but `tunnel_d` never initialized at all and `tunnel_s` didn't
  initialize until 66.3s in — root-caused with real debug-level IMU stats,
  see Known Issue #6.
- Wired RViz's `Intensity Image`/`New Feature Image`/`Tracked Keyframe
  Features` displays to mimosa's actual photometric image topics (they
  existed in `mimosa.rviz` already but were disabled with empty topic
  strings). Note `New Feature Image` will stay blank regardless — its
  drawing call is commented out in `src/lidar/photometric.cpp:781`
  (pre-existing, unrelated to this session).
- Looked for a second real-world dataset to validate against. Newer
  College turned out to be request-gated (no direct mirror). Settled on
  RELLIS-3D (real off-road UGV data, matches the user's own off-road
  perception work) and scaffolded a new `rellis` profile — see the
  RELLIS-3D section above. Hit and fixed a segfault while doing so (Known
  Issue #7). Not yet run against real data; blocked on the user downloading
  the (corrected) "Merged" 60s example from Google Drive.

### 2026-11-22 — RELLIS-3D first real-data run
- User first downloaded the "Synced" example by mistake; confirmed by
  inspecting its connections directly (no IMU topic at all, as the download
  note predicted) before deleting it.
- Downloaded and converted the "Merged" 60s example instead (ROS1→ROS2,
  filtered to `/os1_cloud_node/{points,imu}` + `/tf`/`/tf_static` via
  `rosbags-convert --include-topic` to avoid converting the full 8.9GB bag).
- Derived `T_B_S` from the bag's own `/tf_static` (see RELLIS-3D section)
  instead of the identity placeholder, after checking that mimosa's
  `lidar::Manager` doesn't independently apply any extrinsic rotation from
  Ouster metadata — turned up Known Issue #8 (possible `enwide` typo) along
  the way.
- Ran the full 60s clip with `viz:=true`: clean end-to-end, no crash, graph
  reached key 563. Cross-checked net displacement and path length against
  the bag's own (unused-by-mimosa) wheel odometry — see RELLIS-3D section
  for the numbers and what they do/don't tell us.

### 2026-11-25 — Full RELLIS-3D `00002` sequence: recording, GPS extraction, GT overlay
- Downloaded and converted the full `00002` sequence ("Merged", ~415s,
  `20200215_hill_1.bag`) earlier in the week; this session re-ran it with
  `viz:=true` and screen-recorded the RViz window to
  `results/rellis_00002_run_20261125_085311.mp4` (~7.4min, 2574x1420@20fps)
  for a visual record of mimosa running on real off-road data. Installed
  `ffmpeg` and `vlc` (not previously on this machine) to do the capture and
  playback; set `vlc` as the default `video/mp4` handler via `xdg-mime`.
- The run crashed partway through on Known Issue #9 (newly documented this
  session, previously only noted in agent memory) — confirmed reproducible
  across three independent runs, each crashing at a different point,
  consistent with the resource-accumulation root cause already suspected.
- Researched the RELLIS-3D dataset's public materials (GitHub README):
  found per-sequence YouTube videos (00002's is
  https://www.youtube.com/watch?v=aXaOmzjHmNE) and confirmed there is no
  curated ground-truth trajectory for any sequence — only raw Vectornav
  VN-300 GPS/INS and wheel odometry, same conclusion as the 60s-clip run.
- Extracted `/vectornav/GPS` + `/vectornav/Odom` from the original ROS1 bag
  (previously excluded from the main `rellis` conversion) into
  `datasets/rellis/00002_hill_1_gps_ros2`. Built a 3D trajectory plot from
  the GPS fixes (`results/rellis_00002_gps_trajectory_3d.png` +
  underlying CSV) — shows a ~27m x 117m out-and-back loop with ~12m of
  elevation change, matching the "hill_1" name.
- Wired this GPS path into mimosa's existing RViz "Ground Truth" overlay
  (reusing `scripts/tum_path_publisher.py` unmodified, no `mimosa.rviz`
  changes) and confirmed visually (screenshot) that it renders correctly in
  the `mimosa_map` frame. See Visualization section for the no-alignment
  caveat — this is a shape/scale check, not an aligned ATE-style overlay.
- Ran a live `viz:=true` replay of the full `00002` sequence a second time
  (no recording) with this GPS overlay running alongside — stopped
  manually by the user partway through (~282s in, no crash this time,
  further than the three prior runs that did crash on Known Issue #9;
  consistent with it being a load-dependent backlog, not a fixed-point
  bug).
- User asked whether the dataset's own sensor layout (tf tree) was used for
  the Vectornav-to-Ouster extrinsic. It hadn't been for the GPS overlay —
  the first pass anchored the raw GPS antenna position with no lever-arm
  correction. Fixed: composed the bag's own `/tf_static` chain
  (`imu_link -> os1_frame -> ouster1/os1_sensor -> ouster1/os1_imu`) to get
  the ~0.46m Vectornav-to-Ouster-IMU lever arm, applied it to each GPS fix
  rotated by the nearest `/vectornav/Odom` orientation sample (confirmed
  first via header timestamps that GPS/LiDAR/IMU all share one clock
  domain, start times within 0.32s of each other over the 415s run). See
  Visualization section for the full derivation and the remaining
  (larger) no-heading-alignment caveat this doesn't fix.
- Built `scripts/sim_time_path_publisher.py` for a true sim-time-synced
  "growing" GT path (the existing `tum_path_publisher.py` only does a
  static full-path dump), since the user wanted the GPS path published in
  realtime rather than all at once. Verified it grows correctly against a
  10x-speed bag replay before handing back.
- Committed the above (`VALIDATION_LOG.md` + `sim_time_path_publisher.py`
  only — not `datasets/`, `results/`, or unrelated pre-existing untracked
  IDE files) on explicit request.
- User watched a live `viz:=true` run with the GPS overlay and reported the
  GPS ground-truth path looked ~180° off in heading from mimosa's own
  estimate. Investigated rather than assuming it was the GPS pipeline's own
  bug: confirmed via three independent sources (the bag's `/tf_static`, the
  point cloud's own `header.frame_id = ouster1/os1_lidar`, and the Ouster
  `lidar_to_sensor_transform` matrix in both `os_dummy.json` and
  `os_enwide.json`) that this is Known Issue #10 — see Known Issues. Fixed
  by correcting `T_B_S`'s rotation (identity → 180° about Z) in both
  `rellis/params.yaml` and `enwide/params.yaml`; no code change, no rebuild
  needed (symlinked configs).
- Verification run hit a real RViz hang (confirmed via the window manager's
  own "not responding" dialog, not just a quiet-looking view) — killed the
  stuck `rviz2` process and relaunched a fresh instance against the same
  live topics without disturbing `mimosa_node`/bag playback underneath.
  Screenshots alone proved unreliable for judging heading (default camera
  zoomed too close to the point cloud to see the overall path shape, and
  one capture caught the user's unrelated browser window covering RViz).
  Switched to a numeric check instead: this run completed the full 415s
  sequence cleanly (no Known Issue #9 crash this time — see that issue's
  update), so compared mimosa's persisted TUM log
  (`results/rellis_00002_run4_estimate.tum`) against the GPS ground truth
  directly — heading within 3-7°, distance within ~2-5%, confirming the fix
  (see Known Issue #10 and the Visualization section update).

### 2026-11-25 (continued) — Known Issue #9 reclassified as a symptom of #10
- Session was interrupted by an ungraceful machine reboot; resumed by
  reading this log, `git status`/`git log`, and the `results/` directory
  rather than re-asking the user what had happened. Found three ROS nodes
  (a one-off `gated_pointcloud_relay.py` script, RViz, and
  `sim_time_path_publisher.py`) that had all died simultaneously with
  `ExternalShutdownException`/context-invalid errors at the same wall-clock
  moment — consistent with the reboot killing the ROS graph mid-session,
  not a code bug. That in-progress recording attempt produced no output
  file and its script lived in a since-deleted scratchpad; not resumed.
- User raised the hypothesis that Known Issue #9's crash may have been
  *caused* by Known Issue #10's bad rotation rather than being an
  independent bug, and asked to re-verify #9 without rolling back the #10
  fix. Checked config file mtimes against each run's launch-log start
  timestamp to confirm which runs actually used the corrected rotation
  (see Known Issue #9's "Update — root cause revised"): two full-length
  post-fix runs existed already (`run4` and a later run, both never rolled
  back), both completed the entire sequence cleanly with flat per-scan
  processing time, vs. zero full-length clean completions on the old,
  broken rotation. Updated Known Issue #9, the Current Status table, and
  Open Items accordingly — reclassified from "open bug needing a fix
  decision" to "likely a downstream symptom of #10, unreproduced since the
  fix, crash code path still unguarded but deprioritized."
