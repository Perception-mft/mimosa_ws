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
| `00002` full sequence (~415s, `20200215_hill_1.bag`) | **crashes partway through** on Known Issue #9 | reproducible across 3 runs, crash point varies (~207-365s) — real-time backlog, not a fixed bug location; screen recording of a run up to the crash is at `results/rellis_00002_run_20261125_085311.mp4`. GPS ground-truth path (no curated GT exists) extracted and plotted — see Visualization section. |

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
9. **(Open)** On the full `00002` RELLIS-3D sequence (~415s, vs the 60s
   demo clip Known Issue #7 was tested against), `mimosa_node` crashes
   (SIGABRT) with an uncaught `std::runtime_error` from
   `imu::Manager::getInterpolatedMeasurements` (`src/imu/manager.cpp:412`,
   "IMU lookup requiring first message of the buffer ... This should never
   happen"). Root cause: per-scan LiDAR processing time climbs steadily
   over the run (from ~10-20ms early on to 100-330ms later), eventually
   exceeding the 10Hz scan arrival rate and the `rellis` profile's
   `max_buffer_duration: 1.1s` IMU rolling-buffer window — a real-time
   backlog, not a one-off timestamp bug. Confirmed across three separate
   runs with the crash landing at a different point each time (~207s, then
   ~344s and ~365s in two runs done while also screen-recording the RViz
   window, which adds CPU contention and plausibly shifts when the backlog
   crosses the threshold) — consistent with a resource-accumulation crash
   rather than a fixed repro timestamp. Most likely driver: the local
   scan-to-map voxel structure growing unbounded as the vehicle covers real
   outdoor terrain over several minutes (the 60s clip never ran long enough
   for this to matter). Three non-exclusive fix options proposed, not yet
   decided: (1) make `getInterpolatedMeasurements` degrade gracefully
   instead of throwing, mirroring Known Issue #1's pattern; (2) tune
   `lidar.geometric`'s map-growth settings (`lru_horizon`, voxel leaf
   sizes, `point_skip_divisor`) to bound per-scan cost on long sequences;
   (3) just document as a known limitation and move on. See Open Items.

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

**No heading/yaw alignment is applied, and this is a bigger caveat than the
lever arm above** — there's no Umeyama transform to invert (unlike ENWIDE)
because there's no mimosa trajectory run yet to align against, and no
independent heading reference was used (the VN-300's own fused heading in
`/vectornav/Odom` could arguably serve as one, but hasn't been). The GPS
path is anchored at the same origin convention as mimosa's own start pose
(both ≈ (0,0,0)) but its X/Y axes are true East/North, while `mimosa_map`'s
are whatever the vehicle was heading at t=0 — so position/shape/scale are
comparable, but the whole GPS path may appear rotated relative to mimosa's
estimated path until a real alignment is computed (e.g. via `evo` once both
trajectories exist in TUM format, same as the ENWIDE workflow).

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
- [ ] Decide on a fix for Known Issue #9 (long-sequence IMU-buffer crash on
      RELLIS-3D `00002`) — band-aid graceful degradation, bound map-growth
      cost, or document-and-move-on.
- [ ] Compute a real alignment (evo-style, like ENWIDE) between the RELLIS
      GPS ground-truth path and mimosa's estimated trajectory once a run
      exists to align against — current overlay has no heading alignment
      (see Visualization section).
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
