# TPL (Terrestrial Panning Lidar) — Handoff / How It Works

Status snapshot as of 2026-08-07. Companion to the original design brief at
`D:\Downloads\lidar_scanner_project_brief.md` (mechanical design, density
settings, multi-station registration plan) — that document is still
accurate for the physical/optical design. This one covers the software
that actually got built, which diverged from the brief's original
electronics plan (see "Architecture pivot" below), plus everything added
this session.

## What this system is

**Project name: TPL — Terrestrial Panning Lidar** (named 2026-08-10; the
`branding/` logo assets predate the name being settled, hence the mismatch
if you're reading old commit history/notes that just say "the LiDAR
scanner"). Both names refer to the same system throughout this doc.

A tripod-mounted Velodyne VLP-16 on a stepper-driven tilt axis. The VLP-16
spins internally in azimuth; the external tilt axis rotates that whole
assembly about a second axis to fill in the gaps between the 16 fixed
laser channels and build up a dense, near-spherical point cloud per
tripod station. See the brief for the mechanical rationale.

**The external axis's orientation changed this session: vertical (z),
not horizontal.** `scanner_description/urdf/scanner.urdf.xacro`'s
`tilt_axis_xyz` was `0 1 0` (horizontal, chosen specifically to be
90&nbsp;deg / roughly-perpendicular to the VLP-16's own already-vertical
spin axis — that perpendicularity is *why* it fills the gaps between
channels at all: rotating about the *same* axis the sensor already spins
about wouldn't sample any new elevation angles). It's now `0 0 1`
(vertical), which is **parallel** to the sensor's own spin axis, not
perpendicular. On its own that would undermine the stated gap-filling
purpose above — for it to still make sense, the sensor's own mount
orientation needs to also be rotated (via `vlp16_config`'s
`mount_roll_deg`/`mount_pitch_deg`, see "VLP-16 configuration" below) so
its native spin plane isn't flat/horizontal any more, restoring a
perpendicular relationship between the two rotations, just with the roles
swapped: external axis now provides azimuth/pan, the tilted sensor's own
spin now provides the elevation sweep. Flagged explicitly rather than
silently changed, since it's a real kinematic consequence, not just a
label swap.

**Done**: `mount_roll_deg` set to 90 live against the real running
`vlp16_config` node (`mount_pitch_deg` left at 0 — roll and pitch are
mathematically equivalent ways to make the sensor's spin axis
horizontal, roll was chosen so the sensor's own local +X, i.e. its
forward-facing edge, stays pointing the same way rather than pivoting
onto its face; revisit if the real mechanical mount actually tips the
other way). Verified against the *actual broadcast* `/tf_static`
transform, not just the parameter value: the sensor's local spin axis
(0,0,1) now resolves to (0,&nbsp;-1,&nbsp;0) in `tilt_link`'s frame —
z-component exactly 0, i.e. perpendicular to the vertical tilt axis. Set
via a one-off direct `rclpy` `SetParameters` service call rather than the
`ros2 param set` CLI, which has been unreliable (hangs indefinitely, no
error) for service calls specifically in this environment all session —
topic pub/sub via the CLI works fine, only service calls are affected;
root cause not investigated. `mount_x`/`mount_y`/`mount_z` untouched
(still 0, unrelated to this).

**Gotcha hit on 2026-08-21**: this had been set once already (see git/
session history), but a stack restart in between sessions came back up
with `mount_roll_deg`/`mount_pitch_deg`/`mount_yaw_deg` all `0.0` again —
i.e. the fix silently didn't stick. `~/.lidar_scanner_settings.json` *is*
supposed to persist this across restarts (`vlp16_config` replays it at
startup via `_publish_mount_transform()` in `__init__`), so either the
settings file itself got reset/overwritten between sessions, or a fresh
one was in play — not root-caused, just re-applied. **Also note**: the
previously-calibrated `mount_yaw_deg` (135, from real-unit calibration)
came back as `0.0` too and was *not* restored by this fix — only
roll/pitch were touched, since yaw doesn't affect the
perpendicularity property (rotation about Z can't change a vector's own
Z-component) and the correct real-unit yaw value wasn't re-provided this
time. If the real unit still needs that 135° yaw calibration, it needs to
be re-applied (from the GUI or another live `SetParameters` call) —
check `~/.lidar_scanner_settings.json`'s current value before assuming
it's still calibrated. Separately: reading back the *just-published*
`/tf_static` transform immediately after a change was observed to
sometimes return a stale/different retained value on the very first
read, then settle to the correct one on a re-read a moment later —
consistent with this session's other unexplained WSL2/DDS networking
flakiness (see the `ros2` CLI service-hang note above); not chased
further, just re-read to confirm before trusting a single check.

**Done**: `tilt_axis_bridge`'s `reverse_direction` set to `true` live
(was `false`) — the physical motor was turning opposite to what ROS's
joint_state/tf reported. This parameter exists precisely for that: it
flips the sign in both `_rad_to_axis` (outgoing move-command targets)
and `_axis_to_rad` (incoming encoder readback) together, so commanded
direction and reported direction stay consistent with each other, not
just individually "fixed." Homing/zero-point itself is unaffected (raw
driver call, no rad<->axis conversion involved). Set via the same
one-off direct `rclpy` `SetParameters` approach as the mount-offset
fixes above (`ros2 param set` CLI unreliable here). **Not yet confirmed
against real hardware motion** — jog the axis from the GUI and confirm
the physical direction now matches the commanded one before relying on
this for a real scan.

**Correction (2026-08-21, same day)**: the `true` value above was wrong
for the real hardware, or the wiring/mount changed in between — confirmed
`reverse_direction=false` is what actually makes commanded and physical
direction agree. **Separately observed**: after a stack restart, the
live parameter can read back as its correct/persisted value (`false`)
while the physical direction is *still* wrong until the *same* value is
explicitly re-applied via `set_parameters` — i.e. loading the persisted
default at node startup and setting the identical value later via
`set_parameters` were not behaviorally equivalent at least once. Not
root-caused (nothing in `_on_set_parameters` for `reverse_direction` does
anything beyond persisting to the settings file — no reconnect, no
driver-side call), and not reproduced again since. If it recurs: re-apply
the value explicitly (even if the readback already looks correct) as the
known workaround, and note whether a stack restart happened in between.

**Found and fixed (2026-08-21): stalls were being silently reported as a
normal stop/settle/home-complete.** Symptom reported: jogging the tilt
axis worked fine moving *toward* home but "hit a software limit and
stopped" moving *away* from home, effectively only allowing negative
motion. Live diagnostics against the real hardware (`read_enable_status`/
`read_stall_status` over the existing `~/driver_command` escape hatch)
showed the real cause: moving away from home was stalling the motor,
which auto-disables it (`read_enable_status` -> `False`,
`read_stall_status` -> `True`). `tilt_axis_bridge`'s state machine only
checked `read_motor_status() in (1,)` ("Stopped") to decide a
move/jog/sweep had arrived, or that homing had finished — Stopped-because-
disabled-and-stalled is indistinguishable from Stopped-because-arrived by
that check alone, so a stall was silently reported as `settled`/`done`,
exactly matching the "hits a limit and stops" symptom. Worse: the same
blind spot in the `STATE_HOMING` branch meant a motor that stalls (or is
already disabled) before it ever reaches the driver's own Homing status
would have `set_zero_point()` called on whatever arbitrary position it's
sitting at — silently miscalibrating home. Reproduced and confirmed live:
a `home` request "completed" (`settled`) with the motor disabled and the
axis sitting ~23° off from any prior zero, never having moved.

Fixed in `tilt_axis_bridge/node.py`'s `_tick_state_machine`: both the
homing-complete check and the moving/sweeping/jogging-arrived check now
also call `read_enable_status()` before accepting the stop as legitimate;
if the motor is disabled, the node instead enters a new `STATE_STALLED`
("stalled" on `~/status`) and logs an error, rather than calling
`set_zero_point()` or transitioning to `SETTLING`/`SETTLED`. A fresh
home/move/jog/sweep command still works normally from `STATE_STALLED`
(those are checked unconditionally at the top of `_tick_state_machine`,
not gated on current state) — the fix only stops a stall from being
*mistaken* for success, it doesn't auto-recover from one. Rebuilt
(`colcon build --packages-select tilt_axis_bridge`) but **not yet
reloaded into the running stack** — needs a restart of at least that node
to take effect.

**Resolved (2026-08-21, follow-up)**: it wasn't actually a stall/torque
issue — it was `home_direction` (raw MKS driver register, `set_home_params`,
GUI: Config → Motor → Homing/limits → Home params → Dir). Homing was
seeking the mechanical limit on the *wrong* physical side, so `0°` landed
on the far end instead of the near end — meaning "jog toward `TILT_MIN`"
was a real, working move (mistaken for the broken direction) and "jog
toward `TILT_MAX`" had nowhere to go (mistaken for the stall). Flipping
`home_direction` (CW -> CCW) and re-homing fixed it — confirmed by the
user directly. The `STATE_STALLED` detection fix above is still real and
worth keeping (a genuine stall/disable blind spot existed and is now
caught), it just wasn't the cause of *this* particular symptom.

**Found and fixed (2026-08-21, same session): a shared, systemic bug
that had already silently corrupted persisted settings across all three
nodes, and outright crashed `scan_aggregator`.** `_default(name,
fallback) = persisted.get(name, fallback)` reads a raw value back from
`~/.lidar_scanner_settings.json` (JSON) and hands it straight to
`declare_parameter(name, that_value)`. JSON doesn't distinguish `0` from
`0.0` — any float-typed setting that ever got persisted as a whole number
round-trips back as a Python **int**, and `declare_parameter()` locks a
parameter's ROS type to whatever Python type its default argument has.
So a float parameter whose last-saved value happened to be whole (e.g.
`mount_roll_deg: 90`) would silently get declared **INTEGER** on the next
startup. Usually harmless-looking, but `scan_aggregator` crashed outright
at startup (`InvalidParameterTypeException`) because its own
`config/params.yaml` launch override (`sweep_min_deg: 0.0`, a real
DOUBLE) collided with the wrong INTEGER type the corrupted settings file
had caused it to declare. This is also almost certainly the root of the
still-unexplained "`reverse_direction` reads back correct but the
physical direction is still wrong until you re-set it anyway" gotcha
noted further up — if a parameter round-trips through a
declared-INTEGER/sent-as-DOUBLE mismatch, the value that actually lands
may not be what was intended even though `set_parameters` reports success.

Fixed at the root: every genuinely-float `declare_parameter(...,
_default(name, fallback))` call across `tilt_axis_bridge/node.py`,
`scan_aggregator/node.py`, and `vlp16_config/node.py` now wraps the
default in `float(...)`, so the declared type is always DOUBLE regardless
of what shape happens to be sitting in the settings file (genuinely-int
fields like `sweep_speed_rpm`/`sweep_accel`/`preview_max_points` were
deliberately left alone). Also did a one-time repair of
`~/.lidar_scanner_settings.json` itself, converting every already-corrupted
bare-int float field (`mount_roll_deg`, `mount_pitch_deg`, `sweep_min_deg`,
`sweep_max_deg`, etc.) to a proper float, so currently-loaded nodes get
clean values immediately rather than waiting for each field to be
individually touched again. Rebuilt all three packages.
`scan_aggregator` specifically had died and stayed dead (no auto-respawn
under plain `ros2 launch`) — restarted it as a detached standalone
process (`setsid ... < /dev/null > log 2>&1 &`, not just `&`+`disown`
inside one `wsl.exe` invocation, which turned out not to survive past
that invocation returning) rather than restarting the whole stack, to
avoid disturbing the tilt-axis homing/calibration work above. `tilt_axis_bridge` and `vlp16_config` were still running fine
(never crashed — nothing currently overrides those exact parameter
names at launch) so weren't restarted; they'll pick up the fix on their
next natural restart.

**Found and fixed (2026-08-21, same session): the GUI never loaded
current values into the Motor/Scan/VLP16 settings form on connect.**
Settings were persisting and loading correctly on the ROS side the whole
time (confirmed directly against the live parameters) — the GUI simply
never asked. `applyTiltAxisBridgeSettingsToForm`/
`applyScanAggregatorSettingsToForm`/`applyVlp16ConfigSettingsToForm`
already existed and worked fine, but were only ever invoked from the
Load-Settings-file flow; the Connect button's handler never called them
with the live nodes' actual parameter values, so the form fields just sat
at their static HTML markup defaults until something else happened to
populate them. Fixed in `web/tilt_axis_gui/index.html`: added
`refreshSettingsFormFromLiveNodes()` (calls `getParametersAsObject`
against all three nodes and feeds the results into the existing
apply*ToForm helpers, each independently try/caught so one node being
down doesn't block the others), called it from the Connect handler
alongside the other post-connect setup. Syntax-checked (`node --check`
equivalent on each inline `<script>` block) but not yet exercised against
a live rosbridge connection — worth confirming in the browser that the
form actually populates correctly on next connect.

**Found and fixed (2026-08-21, later same session): a duplicate
`scan_aggregator` process caused erratic tilt-axis motion during a live
scan.** Reported as "moves left and right and jumps around... always
ends up in the correct place... moving the wrong direction first before
correcting." Root cause: two `scan_aggregator` processes were alive at
once, both subscribed to the same topics, each independently stepping
through its own `_targets_rad` sequence and publishing conflicting
`cmd_position` targets to the same physical `tilt_axis_bridge` — the
orphan was a standalone instance from an earlier manual restart (see the
crash-fix entry above) that never got cleaned up once the user restarted
the full `bringup.launch.py` stack afterward (which brought up its own,
now-non-crashing `scan_aggregator`). Diagnosed by watching
`/scan_aggregator/status` and seeing two interleaved step counters
(e.g. `297/451` and `346/451` alternating); confirmed via `ps` that one
process's parent was the real launch tree and the other was the leftover
`setsid`-detached one. Killed the orphan only, left the real one
(mid-scan) running undisturbed — verified the status/position stream
returned to a single clean, monotonic sequence immediately after.
**Lesson, not yet enforced anywhere in tooling**: any node started
standalone outside `ros2 launch` needs to be explicitly torn down once
the real stack is restarted, or it'll silently keep running and fight
the "real" instance. Worth a full `bringup.launch.py` restart at the
next natural opportunity to consolidate everything back under one
process tree — `tilt_axis_bridge`/`vlp16_config`/`scan_aggregator` are
currently a mix of launch-tree and standalone-`setsid` processes from
this session's various fixes.

**Found and fixed (2026-08-21, later still): scan presets saved stale
values, not what was in the form.** The step-and-stare and sweep-scan
fields on the Scan tab are only ever pushed to `scan_aggregator`'s live
ROS parameters when Start Scan/Start Sweep Scan is clicked (see
`setScanParams`/`setSweepScanParams` in `index.html`) — there's no
separate Apply for them, unlike the VLP-16 capture-window fields just
below (which do have their own Apply button). `assembleScanPreset()`
built the saved preset from a live `get_parameters` call against the
node, though, not from the form — so editing the Start/End/Step/etc.
fields and clicking "Save As New..." without first starting a scan
silently saved whatever was last actually live (stale, possibly from a
much earlier run or the startup default), not the values just typed in.
That's the direct explanation for "preset values aren't recalled" —
the preset never captured the intended values in the first place.
Fixed: `assembleScanPreset()` now reads directly off the form fields
(`scanStartDeg`/`scanEndDeg`/.../`vlpViewWidthDeg`), matching exactly
what Start Scan would push, so what's saved is always what's currently
shown regardless of whether it's ever been made live. `output_dir` was
already included in both the save and load path (`SCAN_AGGREGATOR_PARAM_TYPES`
has it, `applyScanAggregatorSettingsToForm` sets it on load) — just
subject to the same stale-read bug, now fixed the same way.
`assembleConfigPreset()` (the *other* preset type, Config tab) wasn't
touched — every field there already has its own live-apply control
(individual Set buttons, matching that tab's UX model), so it doesn't
have this particular gap.

**Added (2026-08-21, later still): stopping a scan early now still
offers to name/save it.** Previously `~/stop_scan` always called
`_abort()`, which discarded every point captured so far — no `.pcd` was
ever written, so the GUI's rename popup (which only ever fires off a
`"done: ... -> path"` status string) never had anything to trigger on.
Fixed in `scan_aggregator/node.py`'s `_on_stop_scan`: now flushes
whatever's sitting in the current stop's not-yet-transformed capture
buffer (step-and-stare only — sweep mode has no buffer, points are
already accumulated as each cloud arrives) and calls `_finish_run()`,
the exact same path a normal completed run takes. `_finish_run()`
already falls back to `_abort()` on its own if genuinely nothing was
captured yet (e.g. stopped during homing), so that case is still handled
correctly. No GUI change was needed — the rename popup already triggers
generically off the status string, regardless of *how* the scan reached
`STATE_DONE`.

Both scan_aggregator fixes above are built and running (restarted as a
single detached standalone process again, per the lesson two entries up
— not yet folded back into the main launch tree).

**Investigated (2026-08-22): Load Settings failing a whole batch of
persisted mks_driver/vlp16-hardware commands at once** (`set_work_mode`,
`set_microstep`, ... all 12 mks_driver commands, then `set_rpm`,
`set_returns`, `set_fov`, `set_net`, `set_host` for the VLP16, all in one
Load Settings click) with `driver not connected` / socket timeouts /
`No route to host`, despite the user confirming both the motor driver
link and the VLP16 were genuinely fine moments later. The mks_driver
failures were instant, real backend rejections (`tilt_axis_bridge`'s
`_on_driver_command` checks `self._motor_ready` and rejects immediately
if false) and the VLP16 failures were slow real socket timeouts — both
kinds failing in the same ~20s window points to a real, transient
connectivity gap at that exact moment (serial link mid-reconnect and the
VLP16's network route both down together — consistent with something at
the WSL2/host networking layer, not a code bug in either node), not a
persistent problem. Not root-caused further (matches this session's
broader pattern of unexplained transient WSL2 networking blips).

The real, actionable gap: these are one-shot replay commands with no
retry, so a blip at exactly the wrong moment permanently loses that
Load-Settings pass silently — the link being healthy again afterward
doesn't retroactively apply anything. **Immediate fix for the user**: if
this happens, just click Load Settings again once the link is confirmed
up. **Fixed in code** (`web/tilt_axis_gui/index.html`): added
`sendWithRetry()` (3 attempts, 1.5s apart) and wrapped each per-command
call in both `applyMksDriverSettingsLive` and `applyVlp16HwSettingsLive`
with it, so a brief transient blip during Load Settings no longer needs
the user to notice the log and manually retry. Syntax-checked only, not
yet exercised against a real transient-disconnect scenario.

**Found and fixed (2026-08-22): every scan came out upside-down, and no
mount_roll_deg/mount_pitch_deg value could ever fix it.** Worked through
this at length with the user. Confirmed the tilt axis's own vertical
orientation and `mount_roll_deg=90` (spin axis horizontal) are correct;
the physical mount also has a genuine ~45 deg twist around that
now-horizontal spin axis (the puck is bolted on at an angle, confirmed
via photos — the spin axis clearly has a strong horizontal component,
ruling out "still basically vertical" as the problem). All four
90-degree-spaced twist candidates around that axis (`mount_pitch_deg` =
45, 135, 225, 315) were tested on real hardware and **all four failed**.
That exhaustive result matters: `_quaternion_from_euler` only ever
composes a *proper* rotation (determinant +1), and a proper rotation can
never invert exactly one axis while leaving the other two alone — that's
a reflection (determinant -1). So if the data genuinely only has Z wrong
(as it visually appeared, and as confirmed below), no value of
`mount_roll_deg`/`mount_pitch_deg`/`mount_yaw_deg` could *ever* have
fixed it — the four-candidate search was mathematically doomed from the
start, not just unlucky.

Decisive evidence: RViz's Orbit view controller has an `Invert Z Axis`
property (`scanner_bringup/config/scanner.rviz`) — this is a **camera
control only** (changes which way mouse-drag orbiting moves the camera,
like an invert-mouse-Y setting; verified by reading the actual rviz
config, it lives under `Views > Orbit`, nothing to do with the
PointCloud2 display or the data). The user found that enabling it made
an already-saved, unchanged `.pcd` file's scan look completely correct.
A pure camera setting fixing the view of already-saved data is only
possible if the *data itself* is genuinely Z-inverted — confirming the
reflection-not-rotation diagnosis directly, without needing to trust the
photo-derived angle guesses at all.

Also independently confirmed via direct analysis of a saved `.pcd`
(loaded and plotted the real point cloud, not just reasoning about it —
see `scratchpad/analyze_pcd.py`/`plot_pcd.py`/`check_mirror.py` from this
session if useful as a template later): the merged cloud showed no sharp
floor-plane peak and an odd hourglass-shaped Z distribution pinched at
the sensor's own origin, i.e. real distortion beyond a simple flip — this
was set aside once the user's direct RViz test gave a decisive, simpler
answer, but is worth keeping in mind if new distortion shows up after
this fix (the plotting scripts in `scratchpad/` are a reusable way to
actually look at scan data directly instead of guessing from
descriptions).

**Fixed properly in `scan_aggregator/node.py`**, not left as an RViz-only
display trick (which wouldn't affect the saved `.pcd` files themselves,
only how they happen to look in that one viewer/session): added a new
`invert_z_axis` boolean parameter, persisted the same way as everything
else. Applied in `_transform_and_accumulate` — the single point both
step-and-stare and sweep mode funnel every point through — negating
column 2 (Z) of each merged point before it's kept, so both the live
preview and the final `.pcd` get it. Defaults to `False` (preserves prior
behavior for anyone else's setup); set to `True` live for this rig via a
direct `SetParameters` call and confirmed persisted. Also wired into the
GUI (`web/tilt_axis_gui/index.html`): a new "Invert Z axis" checkbox on
the Scan tab, included in `SCAN_AGGREGATOR_PARAM_TYPES`, pushed by both
Start Scan and Start Sweep Scan (`setScanParams`/`setSweepScanParams`),
and included in scan presets (`assembleScanPreset`/
`applyScanAggregatorSettingsToForm`). Rebuilt and restarted
`scan_aggregator` (single detached process, per the standing lesson a
few entries up).

**Not fixed**: *where* the real reflection is actually introduced
upstream (the VLP-16 driver/calibration, or some other convention
mismatch) — `invert_z_axis` cancels the symptom at the output stage
rather than fixing it at its true source, which remains unidentified.
Fine for this project's purposes (a static compensating flip is a
legitimate, permanent fix here), but worth knowing this is a
workaround, not a root-cause fix, if it ever needs revisiting.

**Extended (2026-08-22, later same day): added `invert_x_axis`/
`invert_y_axis` alongside `invert_z_axis`, and moved all three off the
Scan tab onto Config > VLP-16.** After enabling `invert_z_axis`, the user
found the scan was then mirrored in X/Y — expected in hindsight: flipping
only Z is one reflection (determinant -1); if the real underlying issue
needs an odd number of axis flips to fully cancel (which the Z-alone
result suggests), one flip alone was never going to be enough on its
own. Same implementation pattern as `invert_z_axis`: independent
`declare_parameter` bools, applied in `_transform_and_accumulate`
(negate columns 0/1 of the merged x,y,z,intensity array). All three are
independent toggles, not a single "flip everything" option, since which
combination is actually needed is rig-specific.

Moved per explicit request: the checkboxes now live in Config > VLP-16's
"Sensor mount offset" fieldset, next to `mount_roll_deg`/etc — the
existing `applyMountOffsetBtn` ("Apply") now pushes to *two* services in
one click: `vlp16_config/set_parameters` for the mount offset/rotation
(as before) and `scan_aggregator/set_parameters` for the three invert
flags (new) — different nodes, same conceptual "apply my orientation
calibration" action from the user's point of view. Correspondingly
removed from the Scan tab entirely: no longer in `assembleScanPreset()`
(scan presets are range/capture-only again) or pushed by Start Scan/
Start Sweep Scan — `SCAN_AGGREGATOR_PARAM_TYPES` still carries all three
for `refreshSettingsFormFromLiveNodes()`/Load Settings to populate the
(relocated) checkboxes correctly, just via a new
`SCAN_AGGREGATOR_INVERT_PARAM_TYPES` subset for the mount-offset button's
own push. Rebuilt and restarted `scan_aggregator` (single detached
process again); confirmed via `get_parameters` that all three declare
correctly and default `False` on the fresh process. **Not yet re-applied
live** — the previous session's `invert_z_axis=true` didn't survive this
restart (settings file showed no `invert_*` keys at all when checked,
consistent with this project's recurring "a GUI Save/Load Settings round
trip can clobber an out-of-band change" pattern rather than anything new)
-- the user needs to check the appropriate Invert X/Y/Z boxes on Config >
VLP-16 and click Apply again before the next scan.

**Found and fixed (2026-08-22, later same day): `Stop Scan` reported
"service call timed out" even though the stop (and save) actually
succeeded.** Direct regression from the "stop scan still saves/offers to
name it" feature added earlier this session: `_on_stop_scan` calls
`_finish_run()`, which used to concatenate and `write_pcd()` the merged
cloud *synchronously*, inline, on this node's single-threaded executor --
blocking that thread (and therefore the pending service response) for
however long the write took. Timed it directly: a realistic ~5M-point
merge took ~10s to write via `write_pcd`'s `np.savetxt`. That's past
typical client-side service-call timeouts, so `Stop Scan` reported
failure even though the save was, moments later, actually succeeding
server-side -- confirmed by finding the real output files on disk despite
the reported "timeout". (Also found *two* `scan_aggregator` processes
running again when investigating this -- same recurring pattern as
before, see the process-fragmentation entries above; cleaned up the
orphan, left the real launch-tree one running. This keeps happening
because every fix this session that needed a rebuild has restarted
`scan_aggregator` as a standalone `setsid` process rather than through
the real launch tree, and the user has separately restarted the full
stack at least twice in between without me tearing the standalone one
down first. Genuinely worth doing a full, clean `bringup.launch.py`
restart at the next opportunity to stop this class of bug from
recurring.)

Fixed in `scan_aggregator/node.py`: `_finish_run()` now snapshots
`self._merged_points` (a shallow list copy -- shares the same underlying
array references, doesn't duplicate the data) and hands the actual
concatenate+`write_pcd`+rename-relevant state updates off to a daemon
background thread (new `_write_output_in_background`), so the calling
context (a service callback, or the normal end-of-run tick) returns
immediately. Added a new `STATE_SAVING` ("saving") status state so the
GUI/user can see a save is in progress rather than the node just going
quiet for several seconds. Deliberately did *not* clear
`self._merged_points` when snapshotting -- not needed for correctness
(new points can't land in it once state leaves
STATE_CAPTURING/STATE_SWEEP_SCANNING, and a new scan can't start until
STATE_DONE resets it) -- and leaving it populated is what keeps
`_maybe_publish_preview`'s existing "completed cloud stays visible after
a run finishes" behavior working through STATE_SAVING/STATE_DONE, same
as before this change. Rebuilt and restarted; verified end-to-end with a
live test scan: `stop_scan` now responds in ~0.01-0.04s regardless of
how much data had accumulated, and the status stream correctly walks
`capturing -> saving -> done: N stops -> path` with a real file written.

**Worth knowing**: normal (non-stopped) scan completions were *always*
subject to this same ~10s blocking write (just without a client watching
for a service response, so it never surfaced as a visible "error") --
this fix removes that hitch for every run, not just manually stopped
ones.

**Found and fixed (2026-08-28): the save itself (not just its blocking of
the service response, fixed above) was the dominant chunk of total scan
time -- switched `pcd_writer.py` from ASCII to binary PCD.** User-reported
symptom: a 180 deg step-and-stare scan at a 0.65 deg step (~277 stops,
default `revolutions_per_stop=2`/`rotation_rate_hz=10` -> tens of millions
of merged points for a scan this size) took ~10 minutes total, with a
large, visible chunk of that sitting in `STATE_SAVING`. Root cause:
`write_pcd()` wrote `DATA ascii` via `np.savetxt`, which formats every
point through a Python-level per-row string-format call -- fundamentally
CPU-bound and slow at real scan-scale point counts, unlike a single bulk
binary write. Compounded by this rig's currently-configured `output_dir`
(`/mnt/d/Downloads/test scans`, confirmed via the live settings file) --
a Windows-drive path reached through WSL2's DrvFs/9p bridge, the same
path class already flagged elsewhere in this doc as slower/less reliable
than a native WSL path -- and ASCII's own larger footprint (~38
bytes/point vs binary's 16) means more bytes have to cross that slower
path in the first place. Fixed: `write_pcd()` now writes `DATA binary` --
one `ndarray.tofile()` call over the already-float32 merged array (no
precision loss vs. the old `"%.4f"/"%.3f"` ASCII formatting; points reach
this function as float32 already, from `_transform_and_accumulate`'s
`rfn.structured_to_unstructured(..., dtype=np.float32)`). Standard,
widely-supported PCD variant (PCL, CloudCompare, Open3D, MeshLab all read
binary PCD natively) -- not a proprietary format, so no downstream
ICP-tooling compatibility loss. Rebuilt (`colcon build --packages-select
scan_aggregator`) and deployed same day: confirmed no scan was in
progress, cleanly shut down the running stack (`SIGINT` to the
`ros2 launch` root, same mechanism the GUI's Shutdown Everything button
uses -- verified zero orphaned processes afterward, per the standing
lesson elsewhere in this doc about standalone restarts leaving orphans),
relaunched `bringup.launch.py` fresh, and confirmed a clean startup log
(`scan_aggregator ready`, `rosbridge_websocket` on port 9090,
`tilt_axis_bridge` reconnected to the Pico, no errors). **Still not
measured**: an actual before/after wall-clock timing of `STATE_SAVING`
against a real scan this size -- the fix is deployed and reasoned to be
correct, but the real speedup number on this rig is still unconfirmed,
worth a real timed run to close the loop. Switching `output_dir` off the
`/mnt/*` path to a native one (e.g. the default `~/lidar_scans`) is a
separate, independent lever on the same symptom -- worth doing regardless
of this fix, and already the documented
workaround for the unrelated cloud-drop bug under "Open items" that shares
the same `/mnt/*`-path root cause class.

**Found and fixed (2026-08-28): step-and-stare was still dropping a real,
non-trivial fraction of clouds even after the earlier self-defeating-
timeout fix -- a genuine residual data-loss bug, not a false alarm.**
User-reported symptom: `tf lookup failed, dropping one cloud: Lookup
would require extrapolation into the future` warnings scattered
throughout a real scan's log (not the earlier bug's 100%/every-cloud
pattern -- a real 180 deg/0.65 deg-step run still completed, 125 stops,
13.9M points), roughly one or two per stop. Root cause: the earlier fix
(dropping the lookup's own timeout to zero-wait, see the entry above
under "Scan modes") correctly stopped the *compounding* executor-
starvation bug, but left `_transform_and_accumulate` doing a single
zero-wait attempt with **no retry at all** -- so any cloud whose
`header.stamp` was even a few ms newer than the newest `/tf` sample
already cached (a routine, small race: VLP-16 clouds publish at ~10 Hz,
`robot_state_publisher` republishes `/tf` at 60 Hz on its own independent
timer, see the `publish_frequency` fix elsewhere in this doc -- the two
are not phase-locked) was dropped permanently, for good, right there.
This is exactly the scenario that doc's own RViz investigation already
described in the abstract ("a naive one-shot `lookup_transform` test
without retry semantics will look broken even when [a retrying consumer]
would actually render fine") -- except unlike RViz's `MessageFilter`
(which retries and never permanently loses a frame, just delays
rendering), this code had no retry path, so every one of those routine
few-ms races was genuine, permanent lost scan data, not just a log
warning.

Fixed in `scan_aggregator/node.py`, mirroring `tf2_ros::MessageFilter`'s
own real strategy (queue + retry on later spins) rather than blocking
inside one lookup call (still correctly avoided, per the reasoning in the
entry above -- retrying *across* ticks is what lets ordinary spinning
between attempts actually deliver a fresher `/tf` sample, unlike a
blocking timeout on the same starved executor thread). Split
`_transform_and_accumulate` into `_try_transform_and_accumulate` (one
attempt, returns the caught exception on failure instead of logging/
dropping) and a thin `_transform_and_accumulate` wrapper that queues a
failed attempt onto `self._pending_transforms` (a list of
`(cloud_msg, first_attempt_time)`) instead of dropping it immediately.
New `_retry_pending_transforms()`, called once per tick (10 Hz, i.e. the
executor gets several more chances to process fresh `/tf` between
attempts -- `robot_state_publisher`'s 60 Hz means a retry on the very
next tick almost always succeeds, matching the ~5-50ms gaps actually seen
in the reported warnings) retries every pending cloud; a cloud still
unresolved after `TF_RETRY_TIMEOUT_S` (1.0s, new module constant) is a
genuine, real drop and gets the same warning message as before, just
delayed to when it's actually earned rather than logged on the very first
racy attempt. `_pending_transforms` is reset alongside `_merged_points`
in both `_on_start_scan`/`_on_start_sweep_scan` (a fresh run shouldn't
inherit a stale retry queue from whatever ran before it) and in `_abort`
(alongside the existing `_capture_buffer` reset there, same reasoning).
Applies to both modes unchanged -- step-and-stare's `_finish_capture`
loop and sweep's direct per-cloud call in `_on_pointcloud` both still
just call `_transform_and_accumulate`, now with retry-on-drop built in
underneath rather than a code change at either call site.

Rebuilt (`colcon build --packages-select scan_aggregator`, syntax-checked
first). **Not yet deployed or measured against a real scan** -- the stack
was found not running (no launch process alive) when this fix landed, so
there was nothing to restart; will take effect whenever the stack is next
launched. Next real scan should be watched for whether the `tf lookup
failed` warnings drop out almost entirely (a few genuine 1s-timeout drops
under real conditions would still be expected/normal, just far fewer than
before) -- worth a real before/after comparison to close the loop, same
as the still-unmeasured binary-PCD save-time entry just above.

**Found and fixed (2026-08-28): continuous sweep produced duplicate
"ghost" copies of real structures, fanned at different angles, clustered
right at the two ends of the sweep -- a real geometry bug, not a
mount/offset calibration problem.** User-reported symptom, from a real
outdoor sweep scan (5-195 deg, sweep_speed_rpm=2, sweep_accel=2,
sweep_duration_s=60, sweep_edge_margin_deg=5, viewed in CloudCompare):
several distinct duplicate copies of the same real pole-like structure,
each shifted to a different apparent angle, all clustered at the sweep's
start/end travel limits -- initially suspected as a mount/rotation-offset
problem (the kind `mount_yaw_deg`/`invert_*_axis` fix elsewhere in this
doc), but those apply a single *constant* transform to every point, which
can't explain several *distinct* ghost copies appearing only near the
travel limits. Root-caused in `tilt_axis_bridge/node.py`'s `_tick_sweep`:
every other motion path in this file (a single move, a jog, homing) waits
through `STATE_SETTLING` (`settle_time_s`, default 0.3s) after the driver
reports Stopped, specifically so real mechanical ring-down/backlash
finishes before that position is trusted for anything -- but continuous
sweep's own turnaround-reversal logic never got that treatment, and used
to command the reversed move on the *exact same tick* "Stopped" was first
detected, with no settle dwell and no stall check at all. At
`sweep_accel=2` -- confirmed against the actual MKS RS485 manual
(`D:\Downloads\MKS SERVO42&57D_RS485 User Manual V1.0.9.pdf`, part 10.2:
`acc=0` is an immediate stop, and *larger* values ramp *more* gradually,
so a small value like 2 is already close to the instant/abrupt end of
that scale) -- reversing a loaded tilt axis from full speed in one
direction to full speed in the other, instantly, is a genuinely violent
mechanical event for a real geared/lever mechanism: plausible real
backlash/ringing between the motor's own encoder (which
`read_cumulative_encoder()` reports accurately -- confirmed, it's polled
fresh every tick) and the physical VLP-16 puck, which isn't perfectly
rigidly coupled to it. tf2/joint_state faithfully reports what the
*motor* thinks its position is; if the puck is still physically
oscillating through a range of *true* angles while the encoder already
reports "arrived," every point captured in that window gets transformed
using an angle that doesn't match where the sensor actually was -- one
mis-angled burst of points per turnaround reversal, which is exactly
"duplicate copies at the wrong angle, clustered right at each end."
Roughly matches the observed count too: at 2 RPM (12 deg/s) over a 190
deg span, one-way transit takes ~15.8s, so a 60s sweep does about 3
full turnaround reversals -- consistent with the several distinct ghost
copies seen in the screenshot.

Fixed by giving the turnaround reversal the same `settle_time_s` dwell
(and the same Stopped-but-disabled stall check every other motion path
already has -- a full-speed reversal is also plausibly the single
highest-torque moment in the whole sweep, a real stall risk on hardware
with an already-documented stall history) that every other motion type in
this file already gets, via a new `STATE_SETTLING` branch inside
`_tick_sweep` itself (self-consistent with `_on_sweep_enable`'s existing
end-of-sweep STATE_SETTLING handling -- if sweep gets disabled exactly
during one of these new mid-sweep dwells, the general STATE_SETTLING
handler in `_tick_state_machine` picks it up and finishes it the same
way, no special-casing needed). No `scan_aggregator`-side change was
needed: its existing `sweep_edge_margin_deg` (angle-proximity-based, not
time-based) already discards clouds near either boundary, and the axis
now sits genuinely stationary at that boundary angle for the whole new
dwell, so the existing filter naturally covers it too. Adds
`settle_time_s` x (number of turnarounds) to total sweep time --
negligible in practice (~3 x 0.3s = 0.9s out of a 60s sweep).

Rebuilt (`colcon build --packages-select tilt_axis_bridge`,
syntax-checked first). **Not yet deployed or verified against a real
sweep** -- the stack was found not running when this fix landed, so
there was nothing to restart; will take effect on next launch. Next real
outdoor sweep scan should be checked for whether the duplicate-ghost
artifact is gone. If it persists (i.e. backlash/ring-down genuinely
outlasts `settle_time_s`, or `sweep_edge_margin_deg=5` isn't wide enough
to cover whatever ringing amplitude remains), the next lever to try is
raising `sweep_accel` itself (a more gradual ramp reduces the whip-crack
at its source, rather than just waiting it out) -- a real scan-time
tradeoff, unlike the settle-dwell fix above.

**Changed (2026-08-28): `sweep_duration_s` now counts from when real data
starts being gathered, not from when sweep motion starts.** User request,
raised right after the turnaround-ghost investigation above: scan time
shouldn't include homing etc. Homing itself was already excluded --
`scan_aggregator`'s duration deadline was only ever set in `_start_sweep`,
which only runs *after* homing completes -- but the deadline used to be
set right there, meaning the initial transit from home (0 deg) to
`sweep_min_rad` (e.g. 5 deg) was still counted against the requested
duration, even though that whole transit produces no usable data (it's
always within `sweep_edge_margin_deg` of the boundary, so every cloud
during it is already dropped). Fixed in `scan_aggregator/node.py`:
`_start_sweep` no longer computes `_sweep_deadline` at all (now `None`
until real data starts, new `_sweep_data_started` flag); `_on_pointcloud`'s
sweep branch computes it the moment the *first* non-edge-margin cloud is
about to be processed, i.e. the moment real coverage actually begins.
`_tick`'s end-of-sweep check and `_publish_status`'s remaining-time
display both updated to treat `_sweep_deadline is None` as "hasn't
started counting yet" (status text says so explicitly, still showing the
dropped-edge-cloud count) rather than crashing on `None - float` or (the
sharper pre-existing risk this surfaced) ending the sweep on literally the
first tick after `STATE_SWEEP_SCANNING` begins, since the old code's
`_sweep_deadline` would otherwise have stayed at its stale initial value
while duration>0 was configured, and `_now() >= 0.0` is trivially true.
`sweep_duration_s <= 0` (unbounded, run until manually stopped) behavior
is unchanged. Rebuilt (`colcon build --packages-select scan_aggregator`,
syntax-checked first); **not yet deployed or tested against a real
sweep** -- stack wasn't running when this landed, will take effect next
launch.

## Architecture pivot (important context)

The brief originally specified an ESP32-S2 running micro-ROS as the tilt
axis's onboard controller, talking RS485 to the MKS SERVO42D driver
on-board. That firmware was fully built (`firmware/tilt_controller/`, ESP-IDF
+ micro-ROS component, homing/move state machine, USB-CDC transport — see
the now-closed task list) and got to the point of compiling and running.

It was then **abandoned** after an extensive RS485 receive-path fault was
isolated to the ESP32 board itself (confirmed with a pure GPIO loopback
test that bypassed RS485 entirely and still received nothing). Rather than
debug a specific board further, the whole architecture moved the RS485
protocol logic off the microcontroller entirely:

- A **Raspberry Pi Pico** now runs a minimal USB-CDC⟷RS485 **transparent
  byte-pipe bridge** (firmware lives outside this repo, at
  `D:\Downloads\MKS_Servo_Tester\src\main.c` — originally built as a C#
  desktop protocol tester, `D:\Downloads\MKS_Servo_Tester\app\...`, whose
  `MksDriver.cs` was the reference this project's Python driver was ported
  from). The Pico does not run ROS or micro-ROS — it just forwards raw
  bytes between its USB serial port and RS485 at a configurable baud.
- All MKS protocol logic, the homing/move/sweep state machine, and PID
  tuning now live in a plain PC-side ROS2 node
  (`tilt_axis_bridge/tilt_axis_bridge/node.py`) that talks to the Pico over
  a regular serial port (`/dev/ttyACM0` under WSL2, via USB/IP passthrough
  — see "Running it" below).

Net effect: no DDS-agent, no micro-ROS transport debugging, no
microcontroller-side protocol code to maintain. The `firmware/` directory
was dead weight kept around for reference/salvage only — it was never
part of the running system, and was **deleted outright 2026-08-29** (per
explicit request, once the project was under git -- still recoverable
from history if ever needed, `git log -- firmware/`). Also removed a
2.1GB untracked vendored dependency
(`micro_ros_espidf_component`) that was sitting on disk alongside it,
excluded from git from the first commit but never actually deleted until
now.

**Bridge hardware changed again, 2026-08-31**: the Raspberry Pi Pico
itself (running the custom transparent byte-pipe firmware described
above) was replaced with an off-the-shelf USB<->RS485 adapter, confirmed
working by the user. Initially described as CH340-based; checked live
against the actual connected device (`Get-PnpDevice` on the PC side —
the only serial device reporting `Status: OK`, every other VID:PID
including the old Pico's showed up as a stale/disconnected entry) and
it's actually **FTDI FT232-family, VID:PID 0403:6001**, not CH340 --
worth remembering if this needs revisiting, since a wrong VID:PID would
silently break the reconnect-after-replug autodetect (see
`BRIDGE_VID_PID` in `tilt_axis_bridge/node.py`) without an obvious
symptom pointing at *why*. Confirmed both the WSL2 kernel's `ftdi_sio`
module and the earlier `ch341` one are available (`modprobe -n -v` on
each resolved cleanly) before deciding this, in case it's ever swapped
for a genuinely CH340-based module later.

No protocol-level changes needed -- this bridge is still just "a plain
serial port carrying MKS frame bytes across RS485," the exact same
abstraction the code already assumed regardless of what's on the other
end of the USB cable. What actually needed updating, all cosmetic/
config rather than behavioral:
- `BRIDGE_VID_PID` (renamed from `PICO_VID_PID`) now `(0x0403, 0x6001)`,
  was `(0x2E8A, 0x000A)`.
- Default `serial_port` now `/dev/ttyUSB0`, was `/dev/ttyACM0` -- FTDI
  (and CH340) enumerate via the generic USB-serial framework producing
  `ttyUSB*` nodes, unlike the Pico's CDC-ACM-class `ttyACM*`.
- `scripts/start_scanner.ps1`'s `usbipd list` grep, same VID:PID swap.
- Every "Pico"-specific comment/docstring/description across
  `tilt_axis_bridge` (`node.py`, `mks_driver.py`, `mks_protocol.py`,
  `setup.py`, `package.xml`), both `bringup.launch.py` files' default
  port + docstring examples, the GUI's "PC↔Pico" labels/hints, and the
  tracked root `settings.json` snapshot -- updated to generic
  "bridge"/FTDI language rather than assuming Pico specifically.
- `mks_driver.py`'s uplink-header resync loop (which used to be
  explained as working around a Pico-USB-CDC-specific stray leading
  0x00 byte) was deliberately **left unchanged** -- it's a no-op against
  a clean byte stream (matches the header on the first read) and stays
  real protection if any future bridge ever reintroduces framing noise,
  so there was nothing to actually fix there, just the comment
  explaining it.

Rebuilt (`colcon build --packages-select tilt_axis_bridge
scanner_bringup`, syntax-checked first, `package.xml` XML-validated
given the `<->` needed escaping to `&lt;-&gt;`).

**Verified live the same day**, once the user completed the one manual
step this needed (`usbipd bind --busid 1-2`, elevated -- couldn't be
completed non-interactively from here, a UAC dialog can't be clicked
through): woke WSL2, attached the bridge (`usbipd attach`), confirmed
`dmesg` inside WSL2 shows `Detected FT232R` / `now attached to ttyUSB0`
-- matching `BRIDGE_VID_PID`/the new default port exactly, not just
assumed. Did a full clean stack restart (no prior process running, so no
SIGINT/orphan-cleanup needed this time) and confirmed
`tilt_axis_bridge`'s own log: `Connected to MKS driver on /dev/ttyUSB0 @
115200 baud`. This isn't just a port-open message -- traced the exact
code path (`_try_connect_driver` in `node.py`): that line is only
reached after both `set_enable(True)` and `read_config_params()`
succeed, each a real multi-byte RS485 round-trip that raises
`MksCommandError` (logged as "Failed to open MKS driver...") if the link
isn't genuinely working. Neither failed -- real bidirectional RS485
communication through the new bridge to the actual MKS driver confirmed,
including the motor genuinely being enabled and its config genuinely
read back, not just a successful `serial.open()`. Single clean process
confirmed via `ps aux` (no orphan from the restart). Velodyne poll()
timeouts appeared in the same log (sensor not currently connected/
powered) -- unrelated to this change, not chased further here.

**Not done this pass**: an actual physical jog/motion command wasn't
issued -- this confirms the *protocol* link end-to-end (enable + config
readback), not that commanded motion visibly moves the real axis. Worth
a real jog from the GUI next time someone's at the hardware, though
there's no specific reason to expect that layer behaves differently now
that the protocol layer is confirmed working identically to before.

## Software architecture (as built)

```
                    ┌─────────────────────────┐
   USB (WSL2/usbipd)│  Raspberry Pi Pico       │  RS485 (A/B)
   /dev/ttyACM0 ─────┤  transparent byte bridge├──────────► MKS SERVO42D driver ─► stepper
                    └─────────────────────────┘                                    + tilt axis

ROS2 graph (all nodes on the PC / WSL2):

  tilt_axis_bridge (node.py)        scan_aggregator (node.py)
    owns the Pico serial link         drives the scan, has no build-time
    homing/move/sweep state machine   coupling to tilt_axis_bridge --
    ~/cmd_position, ~/status,         talks to it purely over its public
    ~/joint_state, ~/home, ~/stop,    topics/services
    ~/shutdown_stack,
    ~/driver_command/response
              │                                   │
              │ joint_state                       │ /velodyne_points
              ▼                                   ▼
      robot_state_publisher ──tf──►  scan_aggregator's tf2 lookup
      (scanner_description URDF)     (transforms each cloud into base_link)

  velodyne_driver_node → velodyne_transform_node → /velodyne_points
       (scanner_bringup, wraps the stock Velodyne ROS2 driver)

  rosbridge_websocket (ws://localhost:9090) ── serves the browser GUI
```

### ROS2 packages (`ros2_ws/src/`)

- **`tilt_axis_bridge`** — the PC-side driver node described above.
  - `mks_driver.py` — blocking MKS SERVO42/57D RS485 protocol client
    (`MksDriver`), ported from the C# reference tool. One method per
    manual function code (move, home, PID, config read/write, diagnostics).
    Frame handling resyncs on the uplink header byte because the Pico's
    USB-CDC link reliably prepends one stray `0x00` before the real frame.
  - `mks_protocol.py` — raw frame build/parse (not read this session, but
    referenced by `mks_driver.py`).
  - `commands.py` — generic `{"command", "params"} → result` dispatcher so
    the GUI's advanced-options tabs can drive **every** `MksDriver` method
    over one `~/driver_command`/`~/driver_response` topic pair instead of
    ~40 hand-written `.srv` interfaces.
  - `node.py` — the ROS2 node itself: parameters, state machine, service/topic
    wiring. (`pid_autotune.py` and its orchestration in `node.py` existed
    earlier this session and were deleted along with FOC mode — see "PID
    tuning" below.)
- **`scan_aggregator`** — the scan controller. Two independent modes (see
  below), both merge transformed clouds into one `.pcd` file per run.
  Also serves a `~/list_dir_request`/`~/list_dir_response` topic pair
  (same request/response-over-topic pattern as `tilt_axis_bridge`'s
  `driver_command`, to avoid a custom `.srv` package) backing the GUI's
  output-dir folder browser, and a `~/rename_output_request`/
  `~/rename_output_response` pair backing its post-scan naming popup --
  see "Web GUI" below for both.
- **`scanner_bringup`** — VLP-16 driver + `velodyne_transform_node` launch,
  wrapped with an `enable_pointcloud` toggle; full-stack `bringup.launch.py`
  with optional `record_bag`.
- **`scanner_description`** — URDF/xacro + `robot_state_publisher` launch;
  this is where the tilt-axis lever-arm/offset geometry lives.
- **`firmware/`** — **deleted 2026-08-29** (recoverable via `git log --
  firmware/` if ever needed). Was the abandoned ESP32-S2 micro-ROS
  firmware; never built or run by anything above, kept for reference
  only until removed outright.

### Web GUI (`web/tilt_axis_gui/index.html`)

Single self-contained HTML file, hand-rolled rosbridge v2 (JSON-over-WebSocket)
client — no build step, no framework. Connects to `ws://localhost:9090` by
default (the `rosbridge_websocket` node started by `tilt_axis_bridge`'s
launch file). Visual style is the "Hardened Control Room" theme (Bahnschrift
type, squared-off panels, beveled edges, corner rivets, phosphor-green
accent) — see "Decisions made" below. Tabs:

- **Scan** — step-and-stare range/capture settings, continuous-sweep-scan
  settings (including `sweep_edge_margin_deg`), Start/Stop buttons for both,
  live progress readout. Each mode's Output dir field has a **Browse…**
  button next to it that opens an in-page folder-browser modal (navigate
  into subfolders, Up, create-new-folder, Select This Folder) rather than a
  native OS dialog -- the page runs in a Windows browser while `output_dir`
  is a WSL2-side path, a different filesystem the browser has no path-level
  access to, and `showDirectoryPicker()` doesn't expose raw paths anyway.
  The modal instead browses the real filesystem via `scan_aggregator`'s
  `list_dir_request`/`list_dir_response` topic pair. Since that node runs
  WSL2-side, Windows drives are reachable too, via WSL2's own `/mnt/c`,
  `/mnt/d`, etc. mount points -- the path field is editable (type a path +
  Go, or Enter) specifically so reaching those doesn't require clicking
  "Up" past `/` and drilling into `mnt` by hand. Windows profile dirs
  commonly have far more subfolders than anything on the WSL2 side, so the
  list is a tall (340px) always-scrollbar-visible box with an explicit
  "N subfolders (dotfiles hidden)" count above it -- the count exists
  specifically so a long listing scrolled past the fold doesn't read as
  incomplete. One inaccessible entry (permission-denied system folders,
  common under `/mnt/c`) is skipped rather than aborting the whole listing,
  and the sort is case-insensitive.

  When a scan finishes, a "Scan complete" popup appears automatically
  (triggered off `~/status`'s `"done: ... -> path"` text, once per newly
  completed run -- that status line republishes at 10Hz for as long as the
  node sits in `STATE_DONE`, so the GUI tracks the last path it already
  popped a dialog for to avoid re-showing it every tick). The file is
  already on disk under its auto-generated default name
  (`scan_<timestamp>.pcd`) by the time the popup can appear -- entering a
  name and clicking Save (or Enter) renames it in place via
  `rename_output_request`/`response`; leaving the field blank and clicking
  Save (or Enter, or clicking outside the dialog) all just keep the default
  name, with no round trip to the node at all. Deliberately a rename-after-
  write rather than deferring the write until a name is confirmed -- a
  closed tab or dropped rosbridge connection before naming can't lose scan
  data either way, since the default-named file already exists. A collision
  with an existing filename is reported as an error in the dialog rather
  than overwriting.
- **Motor** (top-level tab, contains two sub-tabs — restructured this
  session from the original four, see "Decisions made"):
  - **Jog** — everyday operation: Enable/Disable/EMERGENCY STOP/Release
    Stall, live raw status readout, Homing, **Go to degree** (absolute move,
    degrees — was radians), **Arrow-key jog** (Left/Right = toward 0/260
    deg at a configurable Speed, Shift+Left/Right = a separate
    configurable Shift speed, both capped at 40 via the same `max="40"`
    UI hint used elsewhere), Continuous sweep on/off (the raw
    tilt_axis_bridge feature, independent of scan_aggregator), and
    Step-and-stare sweep (motor-only, no point-cloud capture, for
    verifying motion without running a real scan).

    Arrow-key jog publishes a signed RPM to `tilt_axis_bridge`'s
    `~/jog_speed` (positive/negative pick the direction, 0 = stop -- an
    arbitrary sign convention belonging to this topic alone). Under the
    hood this is an ordinary position-mode-4 move
    (`_apply_jog_speed`/`move_absolute_axis`) targeting whichever end of
    the allowed range the sign points at, reusing `_rad_to_axis`
    completely unchanged -- **not** an open-ended speed-mode `run_speed`
    jog, which is what this actually was through two earlier, buggier
    iterations (see the "took three iterations" gotcha below for the full
    story if touching this again). Scoped tightly on the GUI side too,
    everything fails toward "stop, don't move": ignored unless the Jog
    subtab is the actually-visible tab, ignored while focus is in any
    text/number field (so using arrow keys to move a text cursor can never
    be misread as a jog command), OS key-repeat events are ignored (the
    initial keydown already started the jog), and the motor is
    force-stopped on keyup, on switching tabs mid-jog without releasing
    the key, and on the browser window losing focus (alt-tab etc., since a
    keyup can be missed entirely if focus moves away from the page first)
    -- three independent paths to the same stop call rather than relying
    on keyup alone.
  - **Config** — everything else: raw motion testing (common
    direction/speed/accel, speed-mode jog, position mode 1/2 — direct
    `MksDriver` calls via `~/driver_command`), **Direction** (a
    `reverse_direction` checkbox + Set button — a `tilt_axis_bridge`
    parameter, not a driver register, applied inside `_rad_to_axis`/
    `_axis_to_rad`, the choke point every position-based command and the
    position readback flow through: Go to degree, Continuous sweep,
    Step-and-stare, and Arrow-key jog (jog didn't always route through
    here -- see the gotcha below). Flips the sign on both ends so commands
    and the displayed position always agree. **Persists across restarts**
    (`~/.tilt_axis_bridge_settings.json`, WSL2-side -- see the gotcha
    below) -- a physically-backwards mount doesn't change between
    restarts, so it shouldn't need re-setting after every one. Deliberately
    doesn't touch homing's search direction or the raw Direction (bus)
    dropdown, which stay literal CW/CCW selections), driver sync/readback,
    motor
    configuration (work mode, current, microstep, baud rate,
    **PC↔Pico serial link baud**),
    protection, homing/limit params, bus/multi-motor settings,
    calibration/identity, factory reset, bulk 34-byte config dump. No PID
    or auto-tune panel — see "PID tuning" below, both were removed this
    session along with FOC mode itself.
- Header status bar: connection state, live tilt position **in degrees**
  (converted client-side from the radians the node publishes), tilt_axis_bridge
  status, scan_aggregator status. Global EMERGENCY STOP button, plus
  **Shutdown Everything** (`~/shutdown_stack` service — SIGINTs the
  `ros2 launch scanner_bringup` process, cascading a clean shutdown of the
  whole stack; confirms before firing, since the response often can't make
  it back before rosbridge itself goes down as part of the same cascade).

## Scan modes

Both modes live in `scan_aggregator`, share tf2-based transform/merge logic
(`_transform_and_accumulate`), and write one `.pcd` file per run to
`~/lidar_scans/scan_<timestamp>.pcd` (WSL2-side path).

- **Step-and-stare** (`~/start_scan`) — the brief's original, validated
  approach. Home once, then for each tilt step: move → settle
  (`settle_extra_s` beyond tilt_axis_bridge's own "settled" report) →
  capture a fixed number of VLP-16 revolutions → transform+accumulate →
  advance. Motion-blur-free; each stop's cloud transform uses one tilt
  angle, not an interpolated one.
- **Continuous sweep** (`~/start_sweep_scan`, added this session) — home
  once, then push sweep range/speed/accel into `tilt_axis_bridge` via
  `set_parameters` and enable its existing continuous-sweep feature; clouds
  are transformed and folded in live as they arrive (no capture window to
  buffer against). Faster coverage, but points are captured while the axis
  is moving — tf2 interpolates the tilt angle per cloud-message timestamp,
  not per individual point, so some smear within each cloud is inherent to
  this mode. `sweep_duration_s: 0.0` (default) runs until `~/stop_scan` is
  called; a nonzero value stops automatically after that many seconds. A
  long unbounded sweep keeps accumulating points in RAM — fine for a normal
  session, but prefer rosbag2 raw-packet recording over this for very long
  or memory-constrained runs.
  `sweep_edge_margin_deg` (default 5.0, 0 disables) discards clouds
  captured within that many degrees of either turnaround, where the axis
  is changing direction on top of the smear already inherent to this mode.
  Implemented by subscribing to `tilt_axis_bridge`'s own `~/joint_state`
  (`_on_joint_state`) and checking the latest tilt position against
  `sweep_min_rad`/`sweep_max_rad` (cached at sweep start) before folding a
  cloud in (`_in_sweep_edge_margin`) — coarse (~10 Hz joint_state
  resolution), which is fine for a "within N degrees" cutoff. Dropped-cloud
  count shows live in the sweep status text and in the completion log.

Both modes' parameters live in `scan_aggregator/config/params.yaml`.

**Real-hardware bug found and fixed this session: `lookup_transform`'s
0.2s timeout was self-defeating and dropped every single cloud.**
`_transform_and_accumulate`'s `tf_buffer.lookup_transform(...,
timeout=Duration(seconds=0.2))` runs on the same single-threaded executor
as this node's own `TransformListener` -- the only thing that could ever
deliver the awaited `/tf` message *is* the executor thread currently
blocked waiting for it, so a nonzero timeout here can't actually help, it
can only waste time. Step-and-stare's `_finish_capture` calls this once
per buffered cloud in a tight loop; with real VLP-16 data (never exercised
by any earlier mocked test, which never had a real multi-cloud backlog to
drain against real wall-clock time), each failed lookup burned the full
0.2s with the executor starved the whole time -- so `/tf` couldn't be
delivered to satisfy even the *next* lookup in the same loop either,
compounding stop over stop into a growing (then roughly steady-state
~9s, observed) lag and a 100% cloud-drop rate. Symptom was `[WARN]
tf lookup failed, dropping one cloud: Lookup would require extrapolation
into the future` on every cloud, no point cloud file produced, and no
errors at all from `tilt_axis_bridge` (the motor link was fine the whole
time -- this was pure scan_aggregator-side self-starvation, not a
comms/hardware fault). Diagnosed by first ruling out motor-comms drops (no
errors logged), a stale-timestamp bug (`_publish_joint_state` stamps
correctly with a fresh `get_clock().now()` at actual publish time), WSL2
CPU/RAM starvation (16 cores, RAM free), and rviz2 GPU/render load
(warnings persisted identically with `rviz:=false`) -- what was left was
the growing-then-plateauing *shape* of the gap across successive
warnings, which pointed at a compounding self-inflicted stall rather than
an external cause. Fixed by dropping the timeout entirely (defaults to a
zero-wait lookup): every timestamp reaching this function is already in
the past (the capture window that produced it has already closed), so
there was never anything to usefully wait for -- either tf2's buffer
already has it cached from ordinary spinning between capture windows, or
it never will, and a zero-wait lookup finds out immediately either way
without stalling anything. Applies equally to sweep mode's per-cloud call
into the same function, for the same underlying reason, though it's less
likely to compound there since there's no buffered tight loop. Not yet
re-verified end-to-end against real hardware after this fix (package
rebuilt and syntax/import-checked only) -- next real scan attempt should
confirm clouds stop getting dropped.

**Step-and-stare's GUI fieldset shows a live "Estimated time"** (recalculated
on every keystroke, pure client-side, no backend round trip). Deliberately
a floor, not a real prediction: `stops × (settle_extra_s + revolutions_per_stop
/ rotation_rate_hz)`, where `rotation_rate_hz` is derived from the VLP-16
tab's RPM field (`rpm / 60`) rather than hardcoded — changing RPM there
changes this estimate too (see "VLP-16 configuration" below for how RPM
also propagates to the actual backend params, not just this display). It
excludes homing and every move between stops entirely, since per-stop move
time (accel ramp + actual travel, driven by `tilt_axis_bridge`'s
`move_speed_rpm`/`move_accel`, on a different tab) hasn't been empirically
measured yet — see "Empirically measure per-stop settle time" in Open
Items. Real total will be longer than shown.

## Live point cloud preview

`scan_aggregator` publishes `sensor_msgs/PointCloud2` on `~/preview_points`
(so `/scan_aggregator/preview_points`) — the in-progress merged cloud,
republished on a throttle for the whole run, not just the final `.pcd`.

**rviz2 now auto-launches** as part of `bringup.launch.py` (a `Node` action,
`scanner_bringup/launch/bringup.launch.py`), pre-configured via
`scanner_bringup/config/scanner.rviz` (Grid, TF, and a PointCloud2 display
already subscribed to `/scan_aggregator/preview_points`, fixed frame
`base_link`) — so the normal "Start LiDAR Scanner" path just works, no
manual viewer setup needed. This machine has WSLg, so the window shows up
as a normal window on the Windows desktop with no extra config. Tied into
the launch tree deliberately (not started separately from the PowerShell
script): Shutdown Everything / Ctrl-C on the stack closes it too, instead
of leaving an orphaned window to hunt down. `rviz:=false` launch arg skips
it (e.g. a headless/SSH session with no WSLg).

Foxglove Studio also still works as an alternative if preferred — its
"Rosbridge" connection can use the same `ws://localhost:9090` the GUI
already connects to, no extra install on the ROS2 side. Same topic/frame.

Verification note: the rviz2 config was confirmed to load without any
YAML/schema errors and correctly subscribe to the topic (standalone
`rviz2 -d scanner.rviz` test, checked stdout), and a synthetic 12,000-point
cloud in the exact wire format `_make_preview_cloud` produces was published
and received with no errors. Actually eyeballing rendered points in the
window was not done — a screenshot via computer-use was requested and
declined, so this is verified at the ROS/log level, not visually confirmed
against a running window. Worth a quick look once you're at the machine.

Implementation (`_maybe_publish_preview`/`_make_preview_cloud` in
`scan_aggregator/node.py`): rebuilds the full merged cloud from
`_merged_points` and republishes it, throttled and decimated, rather than
streaming incrementally — simplest correct thing given the merge already
lives entirely in RAM. Knobs, applied in this order:
- `preview_publish_period_s` (default 1.0) — republish throttle.
- `preview_decimation` (default 0.5) — fraction of points to keep,
  applied unconditionally via a fixed stride (`round(1/decimation)`), not
  just once a size cap is hit. A live preview is for watching coverage and
  shape build up, not a faithful render, so it's deliberately coarser than
  the final `.pcd` from the start — lower CPU cost to build/publish each
  tick and lower render load in the viewer. 1.0 disables this stride
  (full density).
- `preview_max_points` (default 500000, 0 disables) — second cap on top of
  the fraction stride, for when even the decimated count is still large on
  a long scan; widens the stride further rather than applying a second
  independent one. The full merge and the final `.pcd` are never touched,
  only the preview copy.

`preview_enabled` (default true) turns the feature off entirely. All four
are persisted like every other `scan_aggregator` parameter (see below).
Step-and-stare only gets a new preview after each stop finishes (points are
buffered per-stop, not per-cloud, in that mode — see `_finish_capture`);
sweep mode updates as fast as the throttle allows, since it folds each
cloud in as it arrives. The completed cloud keeps being republished
(on the same throttle) after a run reaches `done`, so a viewer left open
still shows the final result without needing to resubscribe.

Verified with an isolated-`ROS_DOMAIN_ID` functional test (publish/throttle/
decimation-fraction/decimation-cap/enable-flag, byte-layout round-tripped
back through numpy) — not yet eyeballed against real VLP-16 data in
rviz2/Foxglove, since that needs the physical sensor attached and a live
GUI session.

**Testing gotcha this uncovered**: a `ScanAggregatorNode()` built for a
local test still persists every `set_parameters()` call through the
*module-level* `SETTINGS_PATH` — which defaults to the real, shared
`~/.lidar_scanner_settings.json` regardless of `ROS_DOMAIN_ID` isolation.
The domain-isolation gotcha (see below) only stops a test node from
talking to the live *stack*; it does nothing to stop it from silently
overwriting the live *settings file* if the test calls `set_parameters`.
This actually happened once while building `preview_decimation`: a test run
left `preview_enabled: false` and other test junk sitting in the real file,
caught only because the next run's defaults came out wrong. Fixed by having
that test monkeypatch `scan_aggregator.node.SETTINGS_PATH` to a scratch
path before constructing the node — do the same for any future
`tilt_axis_bridge`/`scan_aggregator` test that calls `set_parameters` or
otherwise triggers `_on_set_parameters`.

## VLP-16 configuration

Two genuinely different things, both called "VLP-16 settings" colloquially
but living in different places and reached differently — worth keeping
straight:

- **Layer 1 (ROS driver parameters)** — how the already-running
  `velodyne_driver`/`velodyne_pointcloud` packages process packets already
  being received. Plain ROS2 parameters (`device_ip`, `model`, `rpm`,
  `min_range`/`max_range`, `view_direction`/`view_width`, `cut_angle`,
  `organize_cloud`, ...) on `velodyne_driver_node`/`velodyne_transform_node`.
- **Layer 2 (sensor hardware)** — actual settings on the VLP-16 unit
  itself (motor RPM, return type, field of view, network config), reached
  over the sensor's own embedded web server's `/cgi/setting*` HTTP API —
  nothing to do with ROS parameters at all. Sourced from the VLP-16 User
  Manual's web-interface/curl sections (ManualsLib-hosted copy, pages
  76-84) — not yet cross-checked against a physical unit's actual
  responses, since it hasn't arrived.

The critical fact connecting them: **the ROS driver's own `rpm` parameter
is documented as descriptive, not prescriptive** — setting it does *not*
command the sensor to spin at that rate, it only tells the driver what
rate to *assume* for its own timing math. The real, physical RPM only
ever changes via the Layer 2 HTTP API. Get these out of sync (e.g. change
the sensor's actual RPM without telling the ROS parameter, or vice versa)
and the driver's internal timing assumptions quietly stop matching reality.

A second, easy-to-miss consequence of the same fact: `scan_aggregator`'s
own `rotation_rate_hz` parameter (not a Layer 1 driver parameter, but
still just an assumption `scan_aggregator` makes about the sensor) drives
the *real* per-stop capture duration during a step-and-stare run
(`_start_capture`: `duration = revolutions_per_stop / rotation_rate_hz`).
Change RPM via the VLP-16 tab without this updating too, and a real scan
silently captures too few or too many revolutions per stop relative to
what `revolutions_per_stop` was meant to represent — not just a cosmetic
mismatch like the driver's `rpm`, an actual data-quantity error.

Split across the GUI's Scan tab and a new VLP-16 tab specifically because
of how often each half actually changes:
- **Scan tab → "VLP-16 capture window"** — Layer 1's `min_range`/
  `max_range`/`view_direction`/`view_width`, the ones plausibly worth
  changing per scan/environment (narrowing range, excluding a known
  obstruction). In meters/degrees in the GUI (converted to/from radians
  for `view_direction`/`view_width` on the wire).
- **New VLP-16 tab** — Layer 2 hardware settings plus the rest of Layer 1
  that's realistically "set once and forget" (`device_ip`, node-name
  wiring). Motor RPM/return type/FOV, network configuration (flagged as
  higher-risk — a bad address/mask can strand the sensor), host ports,
  Save Configuration / Reset Sensor actions, and a live raw-status readout.

### New package: `vlp16_config`

`ros2_ws/src/vlp16_config/` — `sensor_client.py` (the HTTP client,
`urllib` only, no new dependency), `commands.py` (a `dispatch(client,
command, params)` table, identical shape to `tilt_axis_bridge/commands.py`
for the MKS driver), `node.py`. Two responsibilities:

1. **Layer 2 dispatch**: `~/hw_command` (request) / `~/hw_response`
   (result) — same request/response-over-topic shape as
   `tilt_axis_bridge`'s `~/driver_command`. Commands: `set_rpm`,
   `set_returns`, `set_fov`, `set_net`, `set_host`, `save_config`,
   `reset_sensor`, `get_status`. A successful `set_rpm` also pushes that
   value into **two** other nodes' parameters automatically (`set_parameters`
   client calls, both fire-and-forget, `_sync_rpm` in `node.py`): `rpm` on
   `velodyne_driver_node` (the descriptive/prescriptive sync described
   above) and `rotation_rate_hz` (`rpm / 60`) on `scan_aggregator` (the
   real-capture-duration sync described above) — new `scan_aggregator_node_name`
   parameter (default `/scan_aggregator`) names the target, same
   read-once-at-startup handling as the other two node-name params.
2. **Layer 1 ownership on `velodyne_transform_node`'s behalf**: this node
   declares its own `min_range`/`max_range`/`view_direction`/`view_width`
   parameters and pushes them into `velodyne_transform_node` via
   `set_parameters` — both live, whenever they change (`_on_set_parameters`
   → `_push_capture_params_to_transform_node`), and once at its own
   startup (replaying whatever was persisted, since
   `velodyne_transform_node` — a third-party node — has no
   settings-persistence hook of its own to do this itself, unlike every
   node this project owns directly).

`device_ip`/`http_timeout_s`/`status_poll_period_s`/
`velodyne_driver_node_name`/`velodyne_transform_node_name` are this node's
own ROS parameters too, persisted the standard way. Node names are read
once at `__init__` (matches `scan_aggregator`'s `tilt_node_name` handling
— a wiring parameter, not expected to change without a restart).

A periodic timer (`status_poll_period_s`, default 2.0, 0 disables) calls
`GET /cgi/status.json` and republishes the raw response on `~/status` as
`{"reachable": bool, "raw": <parsed JSON or null>, "error": <str or
null>}`. **Deliberately not parsed into named fields** — no confirmed
schema for this endpoint was found while building this (categories are
documented: GPS/PPS, motor state, rotation phase, laser state; exact key
names aren't), so the GUI just pretty-prints the raw JSON rather than risk
silently mis-mapping real hardware once it's attached. Map specific fields
once a real response can be inspected.

Wired into `scanner_bringup/launch/bringup.launch.py` alongside the other
nodes (own `launch/bringup.launch.py`, included the same way
`scan_aggregator`'s is).

### A real bug this caught: stale reads inside the parameter validator

`add_on_set_parameters_callback` handlers run as *validators*, before
rclpy actually commits the new value — `self.get_parameter(x).value`
inside that callback still returns the **old** value. First cut of
`_on_set_parameters` called `self.get_parameter("device_ip").value` and
`_push_capture_params_to_transform_node()` (which itself called
`get_parameter` for all four capture params) directly inside the
callback — meaning a live change to `device_ip`, or to any of
`min_range`/`max_range`/`view_direction`/`view_width`, would always apply
the *previous* value, one change behind, not the one just requested. Every
other node in this project avoids this by only using `get_parameter`
*after* the callback returns (next tick), never inside it — this one broke
that rule and it wasn't obvious until an actual test set `device_ip` to a
mock server's address and the HTTP call still went to the old default and
timed out. Fixed by reading the new value straight off the `params`
argument (`{p.name: p.value for p in params}`) instead of calling
`get_parameter` for anything that might be mid-change; `overrides` on
`_push_capture_params_to_transform_node` carries exactly that.

### tilt_link -> velodyne mount offset (fine-tuning the sensor's physical mount)

Six new ROS parameters on `vlp16_config` -- `mount_x`/`mount_y`/`mount_z`
(metres) and `mount_roll_deg`/`mount_pitch_deg`/`mount_yaw_deg` (degrees)
-- persisted the standard way and exposed as a new "Sensor mount offset"
fieldset on the Config > VLP-16 page. This is the tilt-axis-to-sensor-
origin lever arm from the "Open items" calibration note, now live-tunable
from the GUI instead of a `scanner_description` xacro edit + rebuild.

Moving it out of the URDF meant taking the `velodyne` link and
`tilt_link_to_velodyne` fixed joint out of `scanner.urdf.xacro` entirely
-- `robot_state_publisher` and a second, independent broadcaster both
publishing the same parent/child frame pair would race each other, so
this URDF now stops at `tilt_link` and `vlp16_config` owns the
`tilt_link -> velodyne` edge by itself. `vlp16_config` republishes it on
every mount-param change (`_publish_mount_transform`, same
`overrides`-carries-the-uncommitted-value pattern as the stale-reads bug
above -- values changed *right now* by the `_on_set_parameters` call this
was invoked from aren't committed on `self` yet either).

**A real bug this caught: `tf2_ros.StaticTransformBroadcaster` cannot
actually update a transform once sent.** First cut used
`StaticTransformBroadcaster.sendTransform()`, the "normal"/documented way
to publish a static transform. Its actual implementation (confirmed by
reading the installed source, not guessing) keys accumulated transforms
by `child_frame_id` and **only ever adds one the first time it sees that
child frame** -- every later `sendTransform()` call for the same child
frame (`"velodyne"` here) is silently dropped, and it just keeps
re-publishing whatever was sent *first*. That class is built for
"declare a fixed set of static transforms once at startup," not "this one
needs to change live," which is exactly what this feature needs. Every
mount-param change appeared to succeed (parameter committed, settings
file updated, no error) while the actually-broadcast transform silently
stayed frozen at whatever it was set to first (0,0,0/identity, since that
happens at construction) -- caught by an isolated test that checked the
*subscriber's* received value after a live change, not just that
`set_parameters` returned successfully. Fixed by not using
`StaticTransformBroadcaster` at all: a plain `create_publisher(TFMessage,
"/tf_static", QoSProfile(depth=1, durability=TRANSIENT_LOCAL,
history=KEEP_LAST))` -- the exact same topic/QoS `StaticTransformBroadcaster`
itself uses internally -- publishing a fresh `TFMessage(transforms=[...])`
on every change gets the same "late subscribers still see the latest
value" latching behavior from the QoS/DDS layer directly, with no
one-shot-per-frame restriction in the way.

Quaternion math (`_quaternion_from_euler`, roll-pitch-yaw -> `(x,y,z,w)`,
matching `tf_transformations.quaternion_from_euler`'s default `sxyz`
convention) is hand-written rather than pulling in a dependency for one
formula -- verified against known cases (identity at all-zero angles,
90-degree yaw/roll each giving the expected single-axis quaternion).

Verified in an isolated `rclpy` test (own `ROS_DOMAIN_ID`, isolated
`SETTINGS_PATH`, same pattern as this package's other tests): startup
default is an identity transform; a live `set_parameters` change is
correctly reflected in what a subscriber actually receives on
`/tf_static` (this is the check that caught the broadcaster bug above);
the change persists to the settings file; a fresh node picks up a
persisted (non-identity) mount offset and re-publishes it correctly at
startup. Re-ran the existing full `vlp16_config` test suite too -- no
regressions from the import/dependency changes. **Not yet done**:
anything against a real, physically-measured mount offset -- the six
values still default to 0 (CAD's "in line with the optical centre"
assumption), same as before this feature, only *how* to enter a real
measurement changed.

**Live preview**: a small SVG diagram (`updateMountPreview()`) sits right
in the same fieldset, recomputed client-side on every keystroke across
all six fields (no backend round trip) -- a top-down schematic (shaft
icon fixed at centre, disc = the puck at its current X/Y offset, three
short lines = the puck's own rotated X/Y/Z axes). Z (height) has no
natural position in a top-down view, so it's only a text readout, not
plotted. Its rotation math (roll about X, then pitch about the new Y,
then yaw about the new Z, then drop the Z component for the projection)
is written to exactly mirror `vlp16_config`'s own
`_quaternion_from_euler`, not just approximate it -- verified in-browser
against hand-calculated cases (identity, a translation, a 90-degree yaw)
with exact-match results, and cross-checked that a 90-degree yaw rotating
local +X onto local +Y in this preview agrees with what the Python
quaternion formula produces for the same input. Large offsets are
clamped to a fixed radius so the puck never draws off-diagram.

**Revised for clarity after first-cut feedback** ("the tilt axis should
remain stationary, and the puck should move" -- true of the underlying
code from the start, `updateMountPreview()` only ever touches the
lever-arm line, puck circle, its label, and the three axis-indicator
lines, never the shaft's own fixed geometry, but that wasn't obvious to
look at). At all-zero offset the puck circle sits exactly on top of the
shaft icon with nothing to visually distinguish "the fixed thing" from
"the thing that happens to be at the same spot right now" -- fixed by
adding a small dot at the exact pivot point, drawn last (on top of
everything, including the puck when they overlap) so that one point
reads as unambiguously immovable regardless of what else is drawn over
it; a moving "PUCK" text label that tracks the puck's position; two
static dashed reference rings (50mm/100mm at the current scale) so a
given displacement has something fixed to visually judge it against; and
the scale itself increased (300 -> 800 px/metre) since a realistic few-
centimetre offset was previously only a few pixels of movement -- easy to
mistake for "nothing moved" even when it technically had. Re-verified
in-browser: the shaft's own SVG attributes and the pivot dot's position
are byte-identical before/after changing every field, while the puck/
label/lever-arm/axis-lines all move to the expected recalculated
position.

### Verification

No physical sensor yet, so: a standalone `http.server`-based mock stood in
for the VLP-16's `/cgi/*` endpoints, with fake `velodyne_driver_node`/
`scan_aggregator`/`velodyne_transform_node` (plain `rclpy.node.Node`s with
declared parameters — rclpy auto-provides `get_parameters`/`set_parameters`
services for any node's declared parameters, no hand-rolled service
needed). Isolated `ROS_DOMAIN_ID` **and** isolated `SETTINGS_PATH`
(monkeypatched — see the gotcha on this from `preview_decimation`'s own
testing, same fix applied here from the start). Covered: every hw_command
applies correctly against the mock and its result is readable back;
`set_rpm` syncs both `velodyne_driver_node.rpm` and
`scan_aggregator.rotation_rate_hz` (= rpm/60); unknown commands fail cleanly;
periodic status polling publishes; capture-window params push live into
`velodyne_transform_node` and persist; a fresh node picks up persisted
defaults and re-pushes them at startup. Separately, the GUI was verified
against a mocked rosbridge: correct hw_command payloads, correct
deg→rad conversion on the capture-window Apply button, live status
rendering (both reachable and unreachable), the Reset Sensor `confirm()`
gate, and — mirroring the `MKS_DRIVER_SETTING_COMMANDS` incident from the
original settings-persistence work — confirmed `assembleVlp16HwSettings()`
excludes `save_config`/`reset_sensor` (one-shot actions, not persistent
settings) via the same explicit-allowlist pattern
(`VLP16_HW_SETTING_COMMANDS`), not a repeat of that bug.

**Not yet done**: anything against the real sensor. `status.json`'s
schema, the exact RPM/FOV/return-type value ranges the sensor actually
enforces, and whether the CGI endpoints/param names in the manual match
this specific unit's firmware are all unverified until it arrives.

**Known gap, out of scope for this feature**: Scan tab fields (including
the new VLP-16 capture-window ones) are push-only from the form to the
node — `applyFullSettings` (Load Settings) updates live parameter values
but never updates the form fields themselves, so Start Scan right after a
Load can silently overwrite just-loaded values with stale form contents.
Pre-existing across the whole Scan tab, not introduced here; flagged as
its own follow-up rather than fixed inline.

## GUI navigation: Scan / Config

Top-level nav is two buttons: **Scan** and **Config**. Everything that used
to be its own top-level Motor/VLP-16 tab now lives nested one level deeper,
under Config's own subnav (three pages: **Motor**, **VLP-16**, **General**)
— Motor, in turn, keeps its pre-existing Jog/Config sub-subnav unchanged.
So the full path to, say, the raw MKS driver fields is now Config > Motor >
Config (three clicks, two of them labeled "Config" at different levels —
disambiguated in this doc as "Config > Motor > Config sub-page" or just
"the Motor page's Config sub-tab").

**General** is new: it holds what used to be the top of Motor's Config
sub-tab — the whole-page Save Settings As/Load Settings fieldset and the
Config preset fieldset (see "Presets" below) — pulled out because neither
one is actually Motor-specific, and grouping them with raw MKS driver
register fields no longer made sense once VLP-16 got its own page too.
Nothing about *what* those two fieldsets do changed, only where they live.

Tab-switching JS (`nav.tabs` for the top level, `nav.subtabs` for every
level below it) is scoped per-`<nav>` element — each subnav's buttons only
ever toggle its own sibling pages, keyed by `element.id === 'tab-' +
button.dataset.subtab`. This is what lets Config's Motor/VLP-16/General
subnav and Motor's own Jog/Config subnav coexist on the page without one
clobbering the other's active state; before this session there was only
ever one `nav.subtabs`, so a single hardcoded page-id list sufficed. Page
ids were namespaced to avoid collisions once nesting went three deep:
`tab-config` (top-level Config), `tab-config-motor`/`tab-config-vlp16`/
`tab-config-general` (Config's three subpages), `tab-jog`/`tab-motor-config`
(Motor's own two — note `tab-motor-config`, not `tab-config`, precisely
because `tab-config` was already taken by the new top-level tab).

Verified in-browser (static file preview, no rosbridge needed for
navigation itself): all three levels switch independently and correctly
(clicking Config's VLP-16 subtab doesn't disturb Motor's own Jog/Config
state, and vice versa); `isJogTabVisible()` — the arrow-key-jog gate, see
"Arrow-key jog" below — correctly tracks the new three-deep path
(`tab-config` + `tab-config-motor` + `tab-jog` all active) rather than the
old two-deep one it was written against.

## Settings persistence and save/load

Every live-settable value on this page — every node's ROS2 parameters and
the raw mks_driver/VLP-16-hardware config fields their respective
driver-command-style escape hatches drive — lives in one shared JSON file
(`~/.lidar_scanner_settings.json`, WSL2-side), with five top-level
sections: `tilt_axis_bridge`, `scan_aggregator`, `mks_driver`,
`vlp16_config`, `vlp16_hardware`. The latter two were added alongside
`vlp16_config` (see "VLP-16 configuration" below) following exactly this
same pattern — `vlp16_config`'s own ROS parameters via the standard
`get_parameters`/`set_parameters` path, `vlp16_hardware`'s hw_command
settings via the identical `data-vlp16cmd`/allowlist mechanism described
for `mks_driver` just below, not a new mechanism. Built this way
specifically so it can also become the file format presets save/load
against later (a preset is just one of these files saved somewhere other
than the default path) — see "Open items" below.

**Auto-load at session start**: each node reads its own section at
`__init__` and seeds every `declare_parameter`'s default from it (falling
back to the hardcoded literal if nothing's been saved yet — a launch-time
params file, if `scanner_bringup` ever passes one, still takes precedence
over this, since `declare_parameter`'s default is the lowest-precedence
source). `tilt_axis_bridge` additionally replays the `mks_driver` section
against the driver — but only once, at the first successful connect after
the node starts, not on every reconnect. That distinction matters: this
node already has a gotcha on record (below) for `_try_connect_driver`
forcing a hardcoded work mode on every reconnect and silently reverting a
live-but-unsaved change; re-applying a settings file on every reconnect
would be the same anti-pattern with extra steps. A
`self._mks_driver_settings_applied` flag enforces the once-per-session
rule.

**Live persistence on change**: both nodes register an
`add_on_set_parameters_callback` that writes every changed parameter back
into their own section (read-modify-write, so the other node's section —
or another key already in this one — is never clobbered). The
`mks_driver` section is different: it's *not* auto-persisted on every
`driver_command` the way parameters are, since most `driver_command`
traffic is read-only queries (`read_speed`, `read_pulses`, ...) that have
no business being replayed as a setting. It's populated only by the GUI's
explicit Save action.

**GUI: Save Settings As / Load Settings** (Config &gt; General page, top
fieldset — see "GUI navigation: Scan / Config" below for where this and
every other tab moved to when Motor/VLP-16 were nested under a new
top-level Config tab).

- *Save* assembles the full snapshot: `rcl_interfaces/srv/GetParameters`
  against both nodes for their sections (the live, authoritative values —
  not whatever happens to be sitting in a form field, possibly stale or
  never submitted), plus the `mks_driver` section built by calling
  `resolveParams()` — the same function that already resolves a Config
  tab button's current form-field values before actually sending a
  command — against every button's `data-params` template, without
  pressing the button. **Not** every `data-cmd` button, though: only ones
  in an explicit allowlist (`MKS_DRIVER_SETTING_COMMANDS`) matching
  commands.py's own "Motor configuration" / "Protection" / "Homing /
  limits" / "Closed-loop tuning" / "Bus / multi-motor" categories. A
  structural heuristic ("has a non-empty `data-params` template") looked
  right at first but wasn't: `set_enable`'s template is a literal
  `{"enable":true}`, and `run_speed`/`move_*_pulses`/`move_*_axis`/
  `go_home` all take form-field-driven params too without being
  persistent settings at all — capturing `set_enable` as a "setting"
  would have meant Load Settings force-enabling the motor every single
  time regardless of what the user actually wanted. Caught by testing the
  real end-to-end Save flow against a mocked backend, not by testing
  `assembleMksDriverSettings()`/`getDriverSettingCommands()` in isolation
  — worth remembering if this list needs extending later: verify the
  *assembled JSON*, not just that the filter function runs.
- *Load* reads an arbitrary file (via `tilt_axis_bridge`'s
  `~/read_settings_file_request`/`response`), applies the
  `tilt_axis_bridge`/`scan_aggregator` sections via `set_parameters`,
  applies the `mks_driver` section by calling `sendDriverCommand` for each
  entry (also updating the Motor page's Config sub-tab form fields and the
  write-only fields' localStorage, same as if the user had pressed each
  Set button by hand), then calls `refreshConfigFromDriver()` to re-sync
  anything with a real read-back. Finally writes the same loaded content
  to the fixed default path — so it becomes what auto-loads at the *next*
  session start too, satisfying "load a different file mid-session, and
  it becomes the new most-recent" without needing a second, separate
  "most recent" pointer file.
- Both read/write via new topics on `tilt_axis_bridge`
  (`~/read_settings_file_request`/`response`,
  `~/write_settings_file_request`/`response`, same request/response-over-
  topic pattern as `scan_aggregator`'s
  `list_dir_request`/`rename_output_request`), and browse an arbitrary
  path via a dedicated new modal (`settingsFileModal`, distinct from the
  existing output-dir folder browser — built separately rather than
  retrofitted, so nothing about that already-working picker was put at
  risk). The browser is backed by `scan_aggregator`'s `list_dir_request`,
  extended with an opt-in `file_filter` param (e.g. `.json`) that returns
  a new `files` array alongside the existing directory `entries` —
  omitted entirely (not just empty) when `file_filter` isn't given, so the
  output-dir picker's response shape and behavior are unchanged.

Verified end to end (mocked rosbridge, no real hardware): Save assembles
the correct three sections and excludes action commands; Load applies
values live to both nodes and the driver, then re-persists the loaded
content to the default path; both nodes' generic parameter persistence
round-trips correctly (set live → written to disk → a fresh node picks it
up); `_apply_persisted_mks_driver_settings` replays correctly once and is
skipped on a simulated reconnect; `read_settings_file_request`/
`write_settings_file_request` round-trip correctly, including creating a
missing parent directory and reporting a clean error for a missing file.

### Presets: Scan vs Config, independently named/loaded

Two scoped, independently-named preset types, split by GUI tab rather than
by backend section — the opposite of the design floated in the old "Open
items" note (driver/motor config riding along inside a scan preset); the
user explicitly wanted them kept separate. What each preset type
*contains* is unchanged from the first cut of this feature:

- **Scan preset** (Scan tab, top fieldset) — `scan_aggregator`'s whole
  section, plus only the four `vlp16_config` fields the Scan tab itself
  shows (the "VLP-16 capture window" fieldset: `min_range`, `max_range`,
  `view_direction`, `view_width`).
- **Config preset** (Config &gt; General page, second fieldset — moved here
  from the Motor page's Config sub-tab when Motor/VLP-16 got nested under
  the new top-level Config tab, see "GUI navigation: Scan / Config" above)
  — `tilt_axis_bridge`, `mks_driver`, `vlp16_hardware`, and the *rest* of
  `vlp16_config` (`device_ip`, `http_timeout_s`, `status_poll_period_s`,
  the three peer-node-name params) — i.e. everything set once and left
  alone: Direction, the raw driver config fields, and the whole VLP-16
  page.

`vlp16_config` is the only section split field-by-field rather than taken
whole, since its own ROS parameters mix "changes per scan" (capture
window, shown on the Scan tab) and "set once" (wiring/timeouts, shown on
the VLP-16 page) — `VLP16_SCAN_PARAM_NAMES` in the GUI is the single list
both `assembleScanPreset`/`assembleConfigPreset` check against, so the two
preset types can never both claim (or both drop) the same field.

**Storage and UI were reworked from the first cut** (separate
`~/.lidar_scanner_presets/{scan,config}/<name>.json` files, browsed via the
Save Settings As file-picker modal) **to living inside the same shared
settings file everything else does**
(`~/.lidar_scanner_settings.json`), as two new top-level keys —
`scan_presets`/`config_presets`, each a `{name: <preset content>}` dict —
per explicit user request: presets should ride along with the main
save/load, not be their own separate files to lose track of. Concretely:

- `assembleFullSettings()` (backs Save Settings As) now also reads the
  *current* default file's `scan_presets`/`config_presets` and folds them
  into the snapshot it writes — otherwise "Save Settings As" to a fresh
  path would silently drop every saved preset, since presets aren't live
  node state the way every other section is. `applyFullSettings()` needed
  no equivalent change: Load Settings already re-writes the *entire*
  loaded file content back to the default path verbatim, so a loaded
  file's own `scan_presets`/`config_presets` ride along for free.
- Each preset type is now a `<select>` (`scanPresetSelect`/
  `configPresetSelect`) plus **Load**/**Save As New…**/**Delete** buttons,
  replacing the old modal-driven Save/Load-As-file pair. All three read or
  read-modify-write the default settings file directly
  (`readDefaultSettingsFileOrEmpty()`/`requestWriteSettingsFile` against
  `DEFAULT_SETTINGS_PATH`) — there's no separate preset-file browser modal
  anymore, `settingsFileModal` is back to being just Save Settings
  As/Load Settings' own file picker.
  - **Save As New…** (`saveNamedPresetAs`) prompts (native `prompt()`) for
    a name, assembles the current live Scan-tab/Config-pages state via the
    existing `assembleScanPreset`/`assembleConfigPreset`, writes it into
    `existing[presetsKey][name]`, confirming first if that name already
    exists (`confirm()`) since it would silently overwrite otherwise.
  - **Load** (`loadNamedPreset`) reads the selected entry, applies it live
    via the existing `applyScanPreset`/`applyConfigPreset`, then also
    merges its sections into the default file's own top-level sections via
    `mergeSectionsIntoDefaultSettingsFile()` (renamed from the first cut's
    `mergeIntoDefaultSettingsFile`, same field-by-field-per-section
    `Object.assign` merge, same reasoning: `mks_driver`/`vlp16_hardware`
    have no ROS-parameter callback of their own to self-persist, so this
    is what makes a loaded Config preset survive a real restart — the
    ROS-param sections already self-persist on `set_parameters` and this
    is redundant-but-harmless for those).
  - **Delete** (`deleteNamedPreset`) removes the selected entry after a
    `confirm()` gate (destructive, no undo).
  - `refreshPresetSelect()` repopulates a dropdown from the default file's
    current preset dict, called after every Save/Delete, after Load
    Settings finishes (a loaded file may carry a different preset library
    than what was there before), and once on Connect (mirroring
    `refreshConfigFromDriver()`).

Verified in-browser against a mocked `requestReadSettingsFile`/
`requestWriteSettingsFile` backed by an in-memory fake file (no real
rosbridge needed for this part), with `prompt`/`confirm` stubbed: Save As
New writes the assembled content under the given name and the dropdown
auto-selects it; `assembleFullSettings()`'s result carries both preset
dicts through; Load passes the stored content to `applyScanPreset`
unchanged and `mergeSectionsIntoDefaultSettingsFile` correctly lands
`scan_aggregator`'s fields at the file's top level; Delete removes the
entry and the dropdown falls back to the "(no presets saved)" placeholder
option. Not yet exercised against a real rosbridge/live nodes (same
caveat as the rest of this GUI's settings machinery).

## PID tuning

**FOC mode and the auto-tuner were both removed this session** (see
"Decisions made" below for the why). This section is deliberately short now
— most of what used to be written here (a Twiddle-search auto-tuner, a
manual Kp/Ki/Kd/Kv panel, a whole "stay in FOC" recommendation) described
`set_pid_vfoc` (`0x96`)/FOC-only functionality that no longer exists
anywhere in this codebase: no GUI panel, no `~/start_autotune` service, no
`pid_autotune.py` (deleted), no FOC option in the Work mode dropdown.

This system now runs **Close mode** (`BusClose`, `0x04`) exclusively — the
Work mode dropdown no longer offers `PulseVFoc`/`BusVFoc` at all. Close
mode's PID register (`0x97`, `set_pid_close`) is still sitting at its
factory defaults (Kp=0xC8 Ki=0x50 Kd=0xFA Kv=0x12C) — **nothing has tuned
it**. It's reachable only via the raw driver-command escape hatch
(`commands.py`'s dispatch table still has `set_pid_close`, unlike
`set_pid_vfoc` which was removed from it), with no dedicated GUI fieldset.
Same no-read-back caveat as before applies: there's no query command for
either PID register, so the driver only remembers what was last written.

If Close-mode tuning becomes worth doing properly, the FOC-era autotune
code (git history, or ask for it to be rebuilt) followed a decent pattern
worth reusing: gate every gain change on the driver confirming the motor
is genuinely idle (`read_motor_status()`, not internal state), Twiddle
search Kp/Kd only (Ki/Kv held fixed — see the git history for the
reasoning), and start the search at a known-good point with real headroom
in the bounds rather than the factory defaults.

**Note on holding current**: `set_holding_current_percent` (`0x9B`,
exposed in the GUI as "Holding current ratio") only applies in OPEN/CLOSE
work modes per the manual — so on this system's current Close mode, it
actually does something now (this was a no-op back when the system ran
FOC).

## Running it

**Superseded, 2026-09-07: the Windows+WSL2 setup below is retired as a
deployment target, per explicit decision.** The Raspberry Pi 4 (see
"Raspberry Pi 4 deployment" further down) is now the sole way this
project runs -- confirmed as a genuinely self-contained field unit,
surviving a cold reboot with real hardware attached. This section is
kept as-is purely as historical reference for how the Windows/WSL2 dev
environment worked while this project was laptop-tethered; don't spend
effort keeping it current, and don't suggest it as a fallback. The
`enable_pointcloud`/RAM work mentioned below (from the original Pi
field-recording investigation) is still live and directly relevant to
the Pi path, not just leftover from this retired one.

### Raspberry Pi 4 deployment (in progress)

Goal: a self-contained field unit, not dependent on this laptop/WSL2.
Options weighed with the user (2026-08-28/29) before landing here: a Pi 5
(rejected, expensive for what's needed), an x86 mini PC (still arguably
the technically smoothest option -- native architecture, no ARM
recompile, no more WSL2/usbipd-class bugs -- but the user prefers the Pi
form factor/ecosystem), an old Android phone as the compute (rejected --
no real ROS2 Android support, no wired Ethernet, fragile USB-host serial
for the Pico; redirected to phone-as-GUI-client instead, since the
existing web GUI is already a plain rosbridge WebSocket client and
"remote/phone access" was already an open item below), and an Intel
Compute Stick STK1A32SC (2GB RAM, no wired Ethernet, ~2015-era Cherry
Trail Atom -- rejected as too tight for this architecture's
whole-cloud-in-RAM design and the multi-node ROS2 process overhead, on
top of needing a USB-Ethernet dongle workaround for the VLP-16). Landed
on: **Raspberry Pi 4 Model B, 4GB** -- ordered, hardware not yet arrived
as of 2026-08-29. Onboard screen still an open question -- see "Decisions
made"-adjacent chat log if picking this back up: leaning toward a small
HDMI/DSI touchscreen bolted to the Pi directly, or reusing an old Android
phone purely as a browser client for the existing GUI over the Pi's own
WiFi hotspot (not yet built).

**Repo now exists** (there was none before this): a git repo was
initialized in this directory 2026-08-29 and pushed to a **private**
GitHub repo, `https://github.com/LeonSutliffe/TPL_LIDAR` (owner:
LeonSutliffe). `.gitignore` excludes `ros2_ws/{build,install,log}`
(colcon-regenerated), ESP-IDF build artifacts under `firmware/`, and
critically `firmware/tilt_controller/components/micro_ros_espidf_component/`
-- a 2.1GB vendored third-party checkout that was sitting on disk
unnoticed until staging for this commit; excluded outright rather than
submoduled, since `firmware/` is dead-weight/reference-only already (see
"Architecture pivot" above) and nothing rebuilds that component. This
repo is meant to be the sync mechanism between this laptop and the Pi
once it arrives (clone/pull on the Pi, push from wherever changes are
made) rather than manual file copying -- and gives an assistant a normal
way to reach the Pi's copy of the code (SSH in and `git pull`/work
directly, same pattern already used to reach WSL2 from this environment)
once it's on the network, instead of needing a fundamentally different
workflow per machine.

**Hardware arrived and is now SSH-reachable, 2026-09-07** (`tpl@<pi-lan-ip>`).
What actually got flashed turned out to be **Debian GNU/Linux 13
(trixie)**, aarch64, with a desktop session present (`~/.Xauthority`,
populated `~/Desktop`) -- not the plan's Raspberry Pi OS Lite (Bookworm)
image; not corrected retroactively since nothing downstream broke
because of it (see README's step 1 for the honest record). The onboard
GPIO screen (this doc's step 10 / README's) is confirmed working on this
actual image -- user completed it directly over SSH.

**The RoboStack-vs-native-apt open question is now resolved: RoboStack
via micromamba works, confirmed live against the real Pi, not just
checked in the abstract.** Verified in order: (1)
`micromamba search -c robostack-jazzy -c conda-forge --platform
linux-aarch64` for `ros-jazzy-velodyne*` from the dev laptop's own WSL2
micromamba install -- confirmed all five velodyne sub-packages resolve
for `linux-aarch64` without touching the Pi at all; (2) a `--dry-run`
of the full intended package set (898 packages, ~1GB) -- resolved
clean; (3) tried substituting `ros-jazzy-ros-base` for
`ros-jazzy-desktop` (741 packages instead) since the Pi has no use for
`rviz2`/rqt/X11 (see step 10 above) -- dry-run confirmed
`robot_state_publisher`/`tf2_ros`/`tf2_sensor_msgs`/`sensor_msgs_py`/
`xacro` all still resolve without `desktop`; (4) actually installed
that exact set on the Pi over SSH (micromamba bootstrapped for
`linux-aarch64` this time, not `linux-64`) -- succeeded, ~5.9GB
installed; (5) `colcon build` on the cloned repo -- all 5 packages
built clean in ~11s (two emit the same pre-existing, harmless CMake
`cmake_minimum_required` deprecation warning seen on the Windows/WSL2
side, not a new issue); (6) `ros2 pkg list` after sourcing
`install/setup.bash` -- every custom package and every `velodyne_*`
package correctly registered. Exact commands now live in README's step
4/5 rather than duplicated here.

Practical note on how this was driven: SSH'd in from the dev laptop's
WSL2 shell using a small ad hoc Python `pty`-based script (not
`sshpass`/`expect` -- installing `sshpass` needed `sudo`, which needed
an interactive password this session couldn't supply non-interactively)
to answer the password prompt programmatically. Also hit, and worked
around rather than root-caused: the SSH connection running the actual
`micromamba create` appeared to hang on the local end well after the
remote side had actually finished (confirmed via a second, independent
SSH connection: no `micromamba` process running remotely, environment
already fully populated) -- classic "child process/lingering fd keeps
the channel open past the remote command's actual exit" territory, not
investigated further since polling around it via a fresh connection
was sufficient. Worth remembering if driving further long-running Pi
commands this way: launch them detached on the Pi itself
(`nohup ... > log 2>&1 & disown`) and poll the log via short-lived
connections, rather than trying to hold one SSH session open for the
whole duration.

**Hardware wired and the full stack run end-to-end on the Pi, same
session (2026-09-07).** VLP-16 (Ethernet) and the USB<->RS485 bridge
both physically connected. One real gap found that the plan hadn't
anticipated: the VLP-16 doesn't DHCP, so `eth0` came up with link
detected but no IPv4 address at all -- needed a manual static IP on its
subnet. This Pi uses NetworkManager (`nmcli`), with the Ethernet
profile already named `netplan-eth0` (a generic Debian trixie image
detail, not a Raspberry Pi OS one -- see the OS note further up):
```bash
sudo nmcli connection modify netplan-eth0 ipv4.method manual \
    ipv4.addresses 192.168.1.100/24 ipv4.gateway "" ipv4.dns ""
sudo nmcli connection up netplan-eth0
```
(`192.168.1.100` arbitrary, anything but `.201` -- the VLP-16's factory
address per `scanner_bringup/config/vlp16.yaml`; no gateway/DNS, direct
point-to-point link.) `ping 192.168.1.201` went from 100% loss to
sub-millisecond round-trips immediately after. Exact commands now in
README's step 2.

With that fixed, `ros2 launch scanner_bringup bringup.launch.py
rviz:=false` on the Pi produced: `tilt_axis_bridge: Connected to MKS
driver on /dev/ttyUSB0 @ 115200 baud` (same real
`set_enable`/`read_config_params` round-trip verification used on the
Windows/WSL2 side, not just a port-open), and critically **no**
`Velodyne poll() timeout` warnings at all -- confirmed further with
`ros2 topic hz /velodyne_points` showing a real, stable, live ~16Hz
point cloud rate, not just the absence of an error. `ps aux` confirmed
exactly one instance of each node process, no orphans. **This is the
first time the full scanner stack has run end-to-end on the Pi itself,
with real hardware, rather than the laptop.** Updating this note rather
than leaving it stale: every item below has since been confirmed
individually the same day (USB storage with a real scan, systemd via
cold reboot, the onboard screen via kiosk auto-launch + a real
screenshot, and the WiFi hotspot by the user directly joining from a
phone) -- see the field-readiness rollout entry and the onboard-screen
entry further down for each. A physical jog/motion command also wasn't
issued this session (same caveat as the earlier FTDI bridge verification -- protocol
link confirmed, visible motion not re-confirmed on this specific run).

**Unrelated but discovered/handled the same session, worth recording
since it affects the dev laptop itself**: the WSL2 mirrored-networking
bug from 2026-08-28 (see "Known gotchas") recurred, but as a *different*
failure signature this time -- `wsl: An internal error occurred. Error
code: CreateInstance/CreateVm/ConfigureNetworking/0x8007054f`, falling
back to `networkingMode None`, and this time a plain `wsl --shutdown` +
relaunch (the previously-documented fix) did **not** clear it, even
after a longer pause and a second attempt. Since reaching the Pi over
SSH needs real WSL2 networking but *not* mirrored mode specifically (no
`usbipd`/USB passthrough involved in anything Pi-related -- the RS485
bridge is plugged directly into the Pi, not passed through Windows),
worked around it rather than root-causing a Hyper-V-level bug mid-task:
`%USERPROFILE%\.wslconfig` was changed from `networkingMode=mirrored` to
`networkingMode=nat`, `wsl --shutdown` + relaunch confirmed real
networking restored (`hostname -I` returns an address again). **This is
a live, current change to the dev machine, not yet reverted**: the
Windows/WSL2 workflow's own `usbipd`-based USB passthrough (used by
`scripts/start_scanner.ps1` for the FTDI bridge, per the "Setting up
from scratch" section that no longer exists as a standalone README
section but is described in HANDOFF's "Running it") requires mirrored
mode specifically and will not work again until mirrored mode is either
fixed or manually restored in `.wslconfig`. Not investigated further --
**and per the decision below, not worth investigating going forward**:
the laptop path is retired as of this same session, so this bug is now
just an inert fact about the dev machine's state, not a blocker for
anything.

**Decision, later the same session: the Windows/WSL2 laptop path is
retired entirely.** User no longer needs it to work at all -- the Pi is
now the sole deployment target. The "Running it" section above is kept
for historical reference only; don't spend effort fixing it (including
the WSL2 networking bug just described) or suggesting it as a fallback.

**Field-readiness rollout, 2026-09-07: USB storage rule, WiFi hotspot
profile, and systemd auto-start all installed and largely verified on
the real Pi, same session as the hardware-wiring/RoboStack work above.**
Full exact commands in README steps 6/8/9; summary here:

- **USB automount**: udev rule installed and reload-confirmed
  (`udevadm control --reload-rules` succeeded). `uid=1000,gid=1000` in
  the rule checked directly against the real user (`tpl`, uid/gid 1000
  -- not assumed from the `pi`-user default the original plan cited).
  Not exercised with an actual stick plugged in (none available).
- **WiFi hotspot**: `TPL-Hotspot` profile created and configured
  (AP mode, shared IPv4, WPA2-PSK), autoconnect-priority set so home
  WiFi (10) beats the hotspot (0), and both `tpl-wifi-hotspot`/
  `tpl-wifi-home` override scripts created and made executable.
  **Deliberately never activated** (`nmcli connection up TPL-Hotspot`)
  -- doing so switches `wlan0` immediately and would have cut the SSH
  session doing this work (same network). So: configured and plausible,
  not field-proven -- the actual "join `TPL-Scanner` from a phone" flow
  still needs a real test, and the priority-based auto-fallback's
  actual out-of-range trigger was never exercised either (would need
  physically moving the Pi out of home WiFi range).
- **systemd auto-start**: found and fixed a real bug in the plan's own
  `ExecStart` before deploying it -- it only sourced this workspace's
  `install/setup.bash`, never the ROS2 distro env itself
  (`~/micromamba/envs/ros2/setup.bash`); the Windows/WSL2 side got that
  implicitly via `micromamba run -n ros2 ...`, but a bare systemd
  `ExecStart` has no such wrapper, so the original unit would have
  failed outright with `ros2: command not found`. Fixed by sourcing both
  in order. Also swapped `pi`/`/home/pi` for the real `tpl`/`/home/tpl`
  throughout both units. **Verified with an actual `sudo reboot`, real
  hardware attached the whole time**: both services came up `active` on
  their own, `eth0`'s static IP (from the networking fix above)
  persisted, and -- checked the systemd journal, not just `is-active`
  -- `tilt_axis_bridge` re-connected to the real MKS driver and
  `rosbridge_websocket`/`scan_aggregator` started clean, zero manual
  steps. `curl localhost:8080` returned the GUI (`200`, exact byte count
  match). `ps aux` post-reboot confirmed exactly one instance of every
  node -- no duplicates left over from the pre-reboot manual run (which
  was cleanly `SIGINT`'d first, specifically to avoid that). **This is
  the actual "power on, wait, done" goal, genuinely proven on real
  hardware, not just planned.**

**Real motor stall during first USB-storage test scan, root-caused to a
missing settings migration -- then a second, deeper bug found and fixed
underneath it.** With a real USB stick inserted and `output_dir` pointed
at `/mnt/tpl_usb`, a first test scan's homing move stalled the motor
outright (`tilt_axis_bridge: Motor stopped while moving and is now
disabled -- likely stalled`), then the driver stopped responding to
*any* command at all (`MKS command failed: function 0x31: expected 10
bytes, got 0`). User's own diagnosis, confirmed correct: the Pi's
`~/.lidar_scanner_settings.json` was freshly created this session (see
the RoboStack/build entry above) and had never received the real
calibration from the project's actual hardware history --
`invert_x/y/z_axis`, the mount offsets, and critically
`mks_driver.set_home_params.home_direction` (the exact fix for the
previously-documented "home_direction was searching the wrong physical
side" bug) were all still at fresh-install defaults, not the corrected
values. User copied the real settings file (dated 2026-08-28, from the
last real-hardware calibration session) onto the USB stick; installed
on the Pi with two stale fields corrected first (`serial_port`:
`/dev/ttyACM0` -> `/dev/ttyUSB0` for the FTDI bridge migration;
`output_dir` and all three scan-preset copies of it: the old Windows
path -> `/mnt/tpl_usb`). After a full physical power cycle (driver
wasn't responding to comms at all -- a settings fix alone couldn't have
addressed that symptom) and reconnecting cleanly, a retry of the same
test scan completed successfully: 3 stops, 220,095 points, real homing
and motion both working correctly with the corrected `home_direction`.

**Second bug, found investigating why the just-fixed `output_dir` didn't
actually appear in the completed scan's output path**: the scan wrote to
the hardcoded default (`~/lidar_scans`) despite the settings file, live
parameter, *and* a config reload all agreeing on `/mnt/tpl_usb`.
Root cause: `scan_aggregator/launch/aggregator.launch.py` passed a
packaged `config/params.yaml` via `parameters=[params_path]` --  an
explicit launch-time parameter override, which in ROS2 unconditionally
beats whatever a node's own `declare_parameter(name, default)` call
would otherwise resolve to, regardless of what's in
`~/.lidar_scanner_settings.json` or what the GUI's "Save Settings"
claims to persist. Every restart silently discarded whatever had been
saved, reverting to that YAML's hardcoded literals -- for *every*
parameter it listed, not just `output_dir`. **This is almost certainly
the real explanation for the previously-unexplained "mount_yaw_deg
silently lost to a settings-file reset" symptom** noted earlier in this
project's history and never root-caused at the time. Confirmed
`tilt_axis_bridge`/`vlp16_config` don't have this bug -- neither passes
a static params file, both rely purely on their own
`_load_settings_section`-backed defaults, which is the actually-intended
design per `node.py`'s own comments. Fixed by removing the YAML
override entirely (its values were an exact, harmless-until-restart
duplicate of the node's own fallback literals -- migrated its two
genuinely useful comments into `node.py` next to the relevant
`declare_parameter` calls, then deleted the file, its `setup.py`
`data_files` entry, and the now-stale `install/scan_aggregator/share/.../config`
build artifact). Rebuilt and restarted on the Pi: `ros2 param get
/scan_aggregator output_dir` now correctly returns `/mnt/tpl_usb` with
**zero live overrides**, straight from the settings file, exactly as the
persistence design always intended. This bug would have affected the
Windows/WSL2 path identically, for as long as that code existed --
never surfaced there because a restart between a settings change and
the next scan was apparently rare enough in practice.

Originally planned 2026-08-29 (superseded by the above, kept for
context on the reasoning):
automatic USB-stick mounting via a udev rule + `systemd-mount` at a fixed
path (so `output_dir` is set once, regardless of which physical stick is
plugged in); the Pi becoming its own WiFi access point via
NetworkManager's built-in AP mode (`ipv4.method shared`, no
hostapd/dnsmasq needed); and serving `web/tilt_axis_gui/index.html`
itself over a plain HTTP server so a phone joined to that hotspot can
actually load the page. One real code change landed alongside this (not
just planning): `index.html`'s rosbridge address field now defaults to
`ws://` + `location.hostname` + `:9090` instead of always
`ws://localhost:9090`, so a phone loading the GUI from the Pi gets a
working default without typing an IP by hand -- `location.hostname` is
empty for the existing `file://` desktop workflow, so that path is
unaffected (verified: served locally over a plain HTTP server, confirmed
the field auto-fills; the `file://` fallback wasn't independently
verified in-browser due to a tooling limitation navigating `file://`
URLs, but is guaranteed by spec regardless). Also confirmed directly
against the installed `rosbridge_websocket` source (not assumed):
`address` parameter defaults to `""`, which Tornado's `bind_sockets`
binds as all-interfaces -- it was already reachable from other machines
on the network with zero config, before any of this session's changes.

**Easiest path** (current Windows/WSL2 workflow, still how to run it
today while the Pi migration is in progress): double-click the "Start
LiDAR Scanner" desktop shortcut
(`D:\Desktop\Start LiDAR Scanner.lnk`), which runs
`scripts\start_scanner.ps1`. That script: wakes WSL2, finds the Pico by
VID:PID (`2e8a:000a`, not a fixed busid — the busid has changed across
replugs) via `usbipd`, attaches it, launches the full ROS2 stack in its own
console window via `scripts\launch_stack.sh`, waits, then opens the GUI in
the default browser.

**Manual equivalent**, from an already-attached-Pico WSL2 shell:
```bash
source /mnt/d/Downloads/LIDAR/ros2_ws/install/setup.bash
ros2 launch scanner_bringup bringup.launch.py
```
Useful launch args: `serial_port:=/dev/ttyACM1` (if enumeration differs),
`record_bag:=true` (raw `/velodyne_packets` + tf + joint_state/status,
rosbag2, so calibration can be redone as post-processing later),
`enable_pointcloud:=false` (skip the CPU-heavy per-point XYZ conversion —
only meaningful if `scan_aggregator` isn't expected to produce a `.pcd`
this run), `rviz:=false` (skip the auto-launched live-preview rviz2 window
— see "Live point cloud preview" above). Then open
`web/tilt_axis_gui/index.html` directly and connect to `ws://localhost:9090`.

## Branding

`branding/` holds a "TPL" wordmark logo (laser-scan lines feeding into a
blocky, point-cloud-styled "TPL" mark) — `tpl_logo_color.svg`/
`tpl_logo_mono.svg` sources, PNG exports at 128–2000px in `branding/export/`,
and a generated `branding/tpl_logo.ico` (multi-res 16–256px, padded onto a
square transparent canvas since the wordmark itself is wide/short — built
via `branding/make_ico.py`, not hand-drawn). **TPL = Terrestrial Panning
Lidar** — this is now the project's actual name (see "What this system
is" above); the logo assets predate that being settled, but the initials
happened to already match.

Applied so far:
- **GUI favicon**: `<link rel="icon">`/`apple-touch-icon` in
  `web/tilt_axis_gui/index.html`'s `<head>`, referencing
  `../../branding/export/tpl_logo_color_128.png`/`_256.png` by relative
  path (not inlined as a data URI) — fine as long as `branding/` stays two
  levels up from the GUI file, which is the existing repo layout.
- **GUI header logo**: the mono SVG's markup is inlined directly into the
  `<header>` (a `.brand-logo` class, not `<img src="...">`) specifically so
  its `fill="currentColor"`/`stroke="currentColor"` picks up `--accent`
  (the panel UI's single green accent color) via CSS — an `<img>`-referenced
  SVG can't inherit page CSS this way, and the mono source's own hardcoded
  `style="color:#0f172a"` would otherwise render near-invisible against the
  header's near-black background. Replaced the old `header h1::before`
  CSS-content diamond glyph, rather than stacking both.
- **Desktop shortcut icon**: `D:\Desktop\Start LiDAR Scanner.lnk` (not
  tracked in this repo — made by hand, no script created it) had its
  `IconLocation` set to `branding/tpl_logo.ico` via `WScript.Shell` COM
  (previously a generic `shell32.dll,137`). `scripts/set_shortcut_icon.ps1`
  redoes this — re-run it if the shortcut ever gets recreated, since
  nothing currently regenerates the `.lnk` itself.

Not touched: `package.xml` maintainer/description fields (already
package-specific and accurate, not generic placeholder text — no clear
branding gap there), rviz2's own window icon (third-party app, not
meaningfully brandable from a launch-file config).

## Known gotchas / operational notes

- **Found and fixed (2026-08-28): WSL2's network stack can silently wedge
  into `networkingMode 'none'` even though `.wslconfig` specifies
  `mirrored`, breaking both USB passthrough and the GUI's rosbridge
  connection at once.** Reported symptoms: `usbipd: error: Networking mode
  'none' is not supported.` on attach, and the GUI's `ws://localhost:9090`
  connection failing outright. Root-caused live: `%USERPROFILE%\.wslconfig`
  correctly said `networkingMode=mirrored` (`[wsl2]` section), but
  `usbipd attach --wsl --busid 1-2` reported `Detected networking mode
  'none'`, and inside the already-running WSL2 distro `hostname -I` came
  back completely empty with `/etc/resolv.conf` missing entirely --
  confirming the VM's network stack itself was down, not a config file
  problem or a code bug in this project. This explains both symptoms at
  once: `usbipd attach` needs a working WSL network path to reach the VM
  at all (refuses outright under `'none'`), and the GUI's
  `ws://localhost:9090` depends on that exact same network layer for
  WSL2's localhost-forwarding to reach `rosbridge_websocket` inside the
  VM -- the ROS2 stack itself was actually still running fine the whole
  time (confirmed via `ps aux` inside WSL: `bringup.launch.py` and every
  child node, including `rosbridge_websocket`, were alive), just cut off
  from Windows by the broken network layer. Not caused by an unapplied
  `.wslconfig` edit either -- the file's mtime predated this incident by
  over a week, so this was a live degradation of an already-running VM,
  not a stale-config issue. Fixed with a full `wsl --shutdown` (kills
  every WSL process, including the already-broken stack) followed by
  restarting the distro -- confirmed after restart: `hostname -I` returns
  real addresses, `usbipd attach` reports `Detected networking mode
  'mirrored'` and succeeds, and the Pico enumerates at `/dev/ttyACM0`
  again. **Not root-caused further** -- why mirrored mode degrades to
  `'none'` on an already-running VM (vs. failing to start, or falling back
  to NAT) wasn't investigated; if it recurs, `wsl --shutdown` + relaunch
  via the start script is the known, confirmed-working fix. Since the
  shutdown kills the whole stack, the start script (or its manual
  equivalent under "Running it") needs to be re-run afterward -- this
  isn't a live-reconnect situation.
- **WSL2 idles and drops the USB passthrough.** If it's been a while since
  the last run, re-run the start script (or at least the usbipd attach
  step) before assuming the driver should just be there.
- **The bridge's busid is not stable across replugs** — always look it
  up by VID:PID, not a hardcoded busid. Was `2e8a:000a` (Pico) before the
  2026-08-31 hardware swap; now `0403:6001` (FTDI FT232) — see
  "Architecture pivot" above.
- **Live work-mode switches may need a driver reset to actually take.**
  Observed this session switching FOC ↔ Close (before FOC was removed
  entirely — see "Decisions made"); the underlying mechanism is about live
  `0x82 SetWorkMode` calls in general, so it's still worth knowing with
  only Open/Close left. Symptom was the motor becoming noticeably louder/
  rougher after switching live via Config → Motor configuration → Work
  mode → Set Mode — separately from (and possibly compounding) the
  now-fixed forced-FOC-on-reconnect bug elsewhere in this list. Restarting
  the whole stack fixed it; working theory is the driver's internal
  control loop can end up in a half-switched state without a following
  reset. A full stack restart is overkill for this — try Config → Reset →
  **Reset & Restart** (`0x0D`, driver-level, auto-restarts on completion —
  see manual 5.5.2) right after changing Work mode first.
- **Hard project-wide speed limit: 40 RPM.** `MksDriver.MAX_ALLOWED_RPM = 40`
  in `mks_driver.py`, enforced inside `_rpm_to_wire_speed()` — the single
  choke point every speed-taking method already converts real RPM through
  (`run_speed`, `move_relative_pulses`, `move_absolute_pulses`,
  `move_relative_axis`, `move_absolute_axis`, `set_home_params`), so this
  covers the whole program (state machine moves, sweep, homing, and the
  GUI's raw driver-command escape hatch) from one place. Raises
  `ValueError` rather than silently clamping, matching this file's existing
  convention. Not a runtime parameter on purpose — it's a code constant so
  it can't be raised by mistake from the GUI. All RPM defaults that used to
  exceed it (`move_speed_rpm` 300→40, `sweep_speed_rpm` 150→40 in both
  `tilt_axis_bridge` and `scan_aggregator`, plus the GUI's Speed fields)
  were lowered to stay under it, and the GUI's number inputs got a
  `max="40"` hint — but that's just UI guidance; the driver-level raise is
  what actually guarantees the limit.
- **Hard rotation limit: 0-260 deg from home** (`TILT_MAX_DEG`,
  `tilt_axis_bridge/node.py` — 270 initially, tightened to 260 after
  real-hardware testing). Enforced in `tilt_axis_bridge`'s
  `_rad_to_axis()` against the *logical* (pre-`reverse_direction`-flip)
  value — what "0 deg = home, `TILT_MAX_DEG` = far limit" actually means
  to an operator, so this has to live here rather
  than in `MksDriver`: `reverse_direction` flips the sign between this
  logical space and the driver's raw axis counts, and a fixed bound
  expressed in raw-axis space would silently protect the *wrong* physical
  direction depending on that parameter. Covers everything with a
  destination — Go to degree, step-and-stare, both sweep modes, and
  Arrow-key jog (see below) — since `_rad_to_axis` is this node's own
  documented single choke point for all of them. Raises `ValueError`,
  caught by `_tick()`'s `except ValueError` branch (added alongside this
  — `_tick()` used to only catch `MksCommandError`, so a rejected command,
  this or the pre-existing RPM cap, fell into the broad catch-all below it
  and spuriously marked the node disconnected even though the link was
  fine and nothing had been sent to the driver).

  `MksDriver.MAX_ROTATION_DEG = 260.0` (symmetric +/-260, raw axis-count
  space, `mks_driver.py`, kept equal to `TILT_MAX_DEG` so this layer is
  never looser than the authoritative one above it) exists underneath as a
  second, coarser layer — it can't express the real asymmetric range since
  it has no concept of
  `reverse_direction`, but it still catches a wildly out-of-range value
  from the GUI's raw driver-command escape hatch (Motor page's Config
  sub-tab, Position mode 3/4 fields), which bypasses `_rad_to_axis`
  entirely. Same
  `MAX_ALLOWED_RPM`-style code-constant-not-parameter philosophy, same
  "raise rather than clamp" convention. Neither layer touches the
  pulse-count position modes (`move_absolute_pulses`/`move_relative_pulses`)
  or `run_speed`'s continuous jog — pulses are microstep-scaled and
  nothing here tracks the motor's steps/rev, and nothing in this project
  calls `run_speed` anymore (see Arrow-key jog's history below). Both stay
  raw/manual-testing-only paths, same as this codebase's existing
  bulk-config-write escape hatch.
- **Arrow-key jog took three iterations to get right — worth knowing the
  history before touching it again.**
  1. First version drove it via the raw `run_speed`/`stop_speed`
     driver-command escape hatch, with no rotation limit at all — reported
     as the axis moving freely past 0/270 deg.
  2. Second version added the rotation limit above, but as a *symmetric*
     +/-270 deg bound (wrong — home already sits close to the mechanical
     hard stop on the negative side, so negative rotation was never meant
     to be allowed at all, same reasoning as `tilt_start_deg`'s margin and
     "don't sweep into negative degrees" elsewhere in this list), and
     enforced it for jog via a bespoke per-tick watchdog (`_tick_jog`,
     since `run_speed` has no destination for `_rad_to_axis` to check up
     front) that also didn't apply `reverse_direction` — reasoned by wrong
     analogy to the raw Direction (bus) dropdown, which correctly never
     touches it, but jog lives in the Jog tab next to Go to
     degree/sweep/step-and-stare, which all *do* flip with it. Reported as
     two separate bugs in sequence: "direction reverse doesn't work,
     tested with speed mode," then, after fixing that, "it continues to
     move until I stop the jog" past the limit (the watchdog's one-tick
     lag meant it effectively wasn't stopping it at all in practice), "then
     it won't move at all" (the fix for the first bug refused *both*
     directions once past the limit, not just the one making it worse).
  3. **Current version, and the actual fix for all of the above at once:**
     jog no longer uses `run_speed` at all. `_apply_jog_speed` drives it as
     an ordinary position-mode-4 move (`move_absolute_axis`) targeting
     whichever end of the allowed range (`TILT_MIN_DEG`/`TILT_MAX_DEG`)
     the commanded sign points at, reusing `_rad_to_axis` completely
     unchanged — the exact same logical-target,
     `reverse_direction`-aware, bound-checked pipeline Go to degree
     already uses. This can't overshoot past the limit (the target itself
     is capped there, not just "stopped once detected past it" a tick
     later), can't refuse a recovery move (the target is always one of
     the two valid boundaries, so a jog issued from any out-of-range
     position is still a legal move back toward range — nothing to
     special-case), and needs no jog-specific completion logic (reuses
     the same "driver reports Stopped" check `STATE_MOVING`/
     `STATE_SWEEPING`/`STATE_JOGGING` all share). `_tick_jog` and the
     standalone watchdog are gone entirely. Verified with a mocked driver
     on an isolated `ROS_DOMAIN_ID`: both directions target the correct
     boundary, `reverse_direction` flips the resulting raw axis value, a
     jog issued from a simulated 400 deg (way out of range) is still
     accepted and moves back toward 0, and stop / no-op-while-idle both
     behave correctly.

  **Testing note, learned the hard way on this feature:** verifying
  `tilt_axis_bridge` logic by constructing a real `TiltAxisNode()` is
  dangerous while the real stack is already running — it opens a second
  connection to the *same* `/dev/ttyACM0` (pyserial didn't raise, both
  sides just started writing to it) and briefly re-sends
  `set_enable(True)` to real hardware during `__init__`, before a test can
  swap in a fake driver. Hit this firsthand once; no lasting harm (the
  live node self-healed, `ros2 node list` and `~/status` checked clean
  afterward), but avoid it going forward — stub `_try_connect_driver` to a
  no-op before constructing, and isolate with a scratch `ROS_DOMAIN_ID`
  (e.g. `export ROS_DOMAIN_ID=77`), so a test touches no real serial port
  and no real DDS graph at all.
- **A rebuilt package is not a restarted process.** Every fix above was
  `colcon build`-ed into the install tree, but the already-running
  `tilt_axis_bridge`/`scan_aggregator` processes keep executing whatever
  was loaded into memory when they started — rebuilding never hot-reloads
  a live ROS2 node. Confirmed this session: a process that had been up
  since before *any* of the jog work (checked via `ps -o lstart`) was
  still running through two full rounds of "fixed and rebuilt," so every
  test against it was silently exercising stale code — symptoms escalated
  from "no limit enforced" to "direction doesn't work" to "does nothing at
  all" purely because the GUI (a plain file, fresh on every browser
  reload) kept moving on to newer topics/behavior while the backend
  didn't. If a fix doesn't seem to be taking effect, check process start
  time against the last build/edit before assuming the code itself is
  still wrong — Shutdown Everything (or Ctrl-C + relaunch) is the fix, not
  more debugging.
- **`reverse_direction` used to reset to `False` on every restart — fixed.**
  It's a live-settable ROS2 parameter (`set_parameters`, driven by the
  GUI's Direction checkbox + Set button), but ROS2 parameters don't
  persist across a node restart on their own -- only whatever the node was
  launched with (its declared default, or a launch-time params file)
  survives one, and a physically-backwards mount doesn't change between
  restarts. Fixed with a small settings file this node owns entirely
  (`SETTINGS_PATH` = `~/.tilt_axis_bridge_settings.json`, WSL2-side, JSON):
  `_load_persisted_reverse_direction()` reads it (best-effort -- missing
  or corrupt just falls back to `False`) to seed the parameter's default
  *before* it's declared in `__init__`, and `_on_set_parameters` writes
  the new value out (via the generic `_save_persisted_setting`, a
  read-modify-write so unrelated keys in the file aren't clobbered --
  written generically since other settings may want the same treatment
  later) whenever it actually changes. Verified the full round trip with a
  mocked/stubbed driver on an isolated `ROS_DOMAIN_ID`: set True -> file
  written -> a fresh node picks up True -> set back False -> a third node
  picks up False.
- **Reconnect after a Pico disconnect used to need a full restart —
  fixed.** `serial_port` is a fixed launch-time parameter (default
  `/dev/ttyACM0`), and `_try_connect_driver()`'s reconnect loop
  (`_tick`, every `RECONNECT_INTERVAL_S`) kept retrying that exact same
  path forever. A USB-CDC reset or a fresh `usbipd attach` after a
  disconnect doesn't guarantee the Pico re-enumerates at the same
  `/dev/ttyACM<N>` — WSL2 commonly hands it a different number — so the
  node could end up permanently stuck retrying a now-stale path even
  though the Pico was genuinely back and working, and only a full stack
  restart (which re-resolves everything from scratch) recovered. Fixed
  by `_autodetect_pico_port()`: before every (re)connect attempt, scan
  `serial.tools.list_ports.comports()` for `PICO_VID_PID` (`2e8a:000a`,
  the same VID:PID `start_scanner.ps1` already greps `usbipd list` for)
  and prefer that device's actual path over the configured one, falling
  back to the configured `serial_port` unchanged if no matching device is
  currently present. No launch/param changes needed — this runs inside
  the existing reconnect loop, so recovery after a Pico replug is now
  automatic within one `RECONNECT_INTERVAL_S` tick instead of needing a
  restart.
- **RPM was silently wrong after any node restart until manually
  refreshed.** `mks_driver.py`'s RPM->wire-speed conversion
  (`_rpm_to_wire_speed`, see the microstep/RPM scaling entry below) relies
  on the driver's own cached `self._microstep`, which only gets updated by
  an explicit `set_microstep()` call or by `read_config_params()` running
  (previously only triggered by the GUI's manual "Refresh from driver").
  A fresh `MksDriver` object defaults to microstep=16 -- so after any
  `tilt_axis_bridge_node` restart (this session had several, for rebuilds)
  or reconnect, every RPM command was silently off by
  `actual_microstep/16`x (e.g. 4x at microstep=64) until someone happened
  to click Refresh afterward. Fixed by having `_try_connect_driver()` call
  `read_config_params()` right after opening the link, so the cached
  microstep is always correct the moment the node connects, not dependent
  on a separate manual GUI action ever happening.
- **`_try_connect_driver()` used to unconditionally force work_mode back
  to BusVFoc (`0x05`) on every connect/reconnect — fixed.** This silently
  reverted a switch to Close mode (see "Decisions made" -- Close mode is
  what's actually running as of this session) on every restart/reconnect,
  including the routine ones from a USB-CDC reset -- confirmed this
  session as "every restart, the GUI reports the driver has returned to
  FOC mode." The `set_work_mode()` call on connect is now removed
  entirely: the driver already remembers its own work mode across power
  cycles, so this code has no business reasserting one every time it
  reconnects. Whatever mode is set via the GUI's Work mode dropdown (or
  the driver's own onboard menu) now actually persists across restarts.
- **Microstep=256 needed a driver fix.** The MStep wire field is a single
  byte (0-255), so 256 doesn't literally fit — `bytes([256])` raised
  `ValueError` before anything even reached the wire, which is why setting
  it used to fail outright. Fixed in `mks_driver.py`'s `set_microstep()` by
  sending 256 as wire byte 0 (`microstep % 256`), matching the on-screen
  MStep menu which offers 256 as a normal option. `read_config_params()`
  now also maps a read-back byte of 0 back to 256 for the same reason.
- **Homing overrun (hits the limit, then keeps driving further into it)
  on Mode=MechanicalLimit — resolved by raising Origin offset.** This
  project uses stall detection against a physical hard stop (confirmed
  this session), which is MKS's confusingly-named **MechanicalLimit**
  mode (torque/stall-based, no dedicated switch — not the same thing as
  "a mechanical endstop switch", which is what this driver instead calls
  Mode=**Endstop**). Manual 7.5's worked example describes the firmware
  reversing direction after the stall to reach the Origin offset position,
  but that turned out not to be an unconditional "always backs off"
  guarantee -- ruled out several other candidate causes empirically this
  session before landing on the real one:
  - Confirmed via "Refresh from driver" that `home_mode` genuinely read
    back as MechanicalLimit (not accidentally still Endstop).
  - Flipping `home_direction` (CW/CCW) changed which physical direction
    the whole search+overrun happened in, but the overrun itself
    persisted either way -- not a direction-inversion issue.
  - `home_current_ma` (Current (mA), stayed at the default 400mA
    throughout) was **not** the cause -- stall detection itself was
    triggering fine.
  - A small/negative Origin offset (-100) made no visible difference.
  - **Raising Origin offset to a larger value fixed it.** Best working
    theory: `retValue` isn't purely "distance to back off by" -- it likely
    interacts with the axis's accumulated coordinate position at the
    moment of stall (which isn't reset per homing attempt), so a small
    offset can still resolve to "continue in the same direction" while a
    large enough one forces the reverse path. Treat this as empirically
    confirmed behavior on this hardware, not a fully-explained protocol
    detail -- the manual doesn't spell out this interaction.
  Practical upshot: **use a large Origin offset for MechanicalLimit
  homing on this system** (small values like the ~100s range aren't
  reliable) -- current known-working value not yet recorded here, update
  this note with the actual number in use. This is a config value living
  entirely on the driver (no ROS default to rebuild) -- push changes via
  "Home torque / offset" → Set, and re-verify with "Refresh from driver"
  since it can't be inferred from the GUI's own default alone.
- **Don't leave duplicate `tilt_axis_bridge_node` processes running.**
  Two processes fighting over `/dev/ttyACM0` produces exactly the symptom
  reported twice this session — tilt state rapidly flashing between `idle`
  and `disconnected`, with garbled RS485 reads. Diagnose with `ps aux`,
  kill whichever instance is the orphan (keep the one with rosbridge/velodyne/
  scan_aggregator siblings from a full-stack launch).
- **Transient RS485 glitches are normal, not fatal.** `MksCommandError`
  ("expected N bytes, got M", checksum mismatches) happens occasionally
  under real half-duplex timing and is retried/tolerated throughout the
  codebase (`_tick`). A bare (non-`MksCommandError`) exception means the
  link itself is gone (USB-CDC reset, port vanished) — that's when the
  node degrades to `disconnected` and starts its 5s reconnect loop.
- **PID gains only take effect while the motor is idle** (manual 5.3) —
  nothing in this codebase currently enforces this (no automated PID path
  exists post-autotune-removal; the raw `set_pid_close` escape hatch is
  fired manually by whoever's using the GUI). If a Close-mode tuning UI or
  automation gets built later, gate every gain change on the driver
  confirming idle via `read_motor_status()` directly, not tracked state —
  this was load-bearing for the old FOC autotuner, confirmed against real
  hardware.
- **Don't sweep into negative degrees.** Home (0°) sits close to the
  mechanical hard stop — the same reason step-and-stare's safe range
  starts at `tilt_start_deg=5.0`, not 0. A negative `sweep_min_rad` drives
  the arm into that physical limit instead of a controlled PID stop, which
  presents as a much harder/more abrupt halt at that end than at the
  (safe) positive end — initially looked like a gravity/PID asymmetry but
  isn't (the axis rotation isn't gravity-loaded either way). Confirmed
  this session: a -30°..180° sweep stopped hard at -30°; switching to
  0°..180° fixed it. Defaults for `sweep_min_rad`/`sweep_min_deg` were
  changed from -0.5 rad/-30° to 0.0 after this finding (`node.py` in both
  `tilt_axis_bridge` and `scan_aggregator`, plus `scan_aggregator/config/
  params.yaml`) — but this only guards the *defaults*; nothing stops
  someone from typing a negative value into the GUI or a launch override,
  so the same hard-stop can still happen if the far (high) end of travel
  is approached too closely too. Note there are *two* separate Min (deg)
  fields in the GUI backed by this default — Motor tab's "Continuous
  sweep" (`sweepMin`, drives `tilt_axis_bridge` directly) and Scan tab's
  "Continuous sweep scan" (`sweepScanMinDeg`, drives `scan_aggregator`,
  which pushes into `tilt_axis_bridge`). The first fix only updated
  `sweepMin`'s HTML default; `sweepScanMinDeg` was still shipping `-30`
  until the sweep-edge-margin work below caught it — worth double-checking
  both if this class of default ever needs touching again.
- **RViz "could not transform from [velodyne_points]" + a slow-looking
  update rate — root cause was `robot_state_publisher`'s default
  `publish_frequency` (20.0 Hz), not anything in this project's own
  nodes.** First suspected (and ruled out) the tilt USB device not being
  attached in WSL2 (that was a real, separate issue that occurred earlier
  the same session after a reboot — fixed via `usbipd attach --wsl
  --busid <id>`, see the bullet above on WSL2 dropping USB passthrough —
  but fixing it alone did not resolve this symptom). Root-caused by
  writing a one-off `rclpy` script that replicated what RViz's
  `PointCloud2` display actually does — call `tf2_ros.Buffer.
  lookup_transform(fixed_frame, cloud.header.frame_id, cloud.header.stamp)`
  for each incoming `/velodyne_points` message — and it reliably
  reproduced "Lookup would require extrapolation into the future" on
  ~94% of clouds. Velodyne publishes point clouds at 20 Hz;
  `robot_state_publisher` republishes `/tf` on its own internal timer at
  `publish_frequency` (default 20.0 Hz, completely decoupled from how
  often `tilt_axis_bridge` actually publishes `joint_state` — confirmed
  live via `list_parameters`/`get_parameters` against the running
  `robot_state_publisher` node), so the two 20 Hz streams were just
  barely tied, and any phase jitter between them left a cloud's exact
  timestamp newer than the newest available transform. **First fix
  attempt (wrong node): added a second, faster (50 Hz) timer in
  `tilt_axis_bridge/node.py` to decouple `_publish_joint_state()` from
  the slower serial-polling `poll_rate_hz` tick** (`joint_state_rate_hz`
  param, defaults 50.0 — this is a real, independently-good change, kept
  in the code) — confirmed `joint_state` really did jump to ~50 Hz, but
  `/tf` itself stayed at ~16.67 Hz regardless, proving the bottleneck
  was downstream in `robot_state_publisher`, not the input rate. **Actual
  fix: `scanner_description/launch/description.launch.py` now passes
  `publish_frequency: 60.0`** in `robot_state_publisher`'s parameters
  dict (was previously only passing `robot_description`, silently
  inheriting the 20 Hz default). Verified live end-to-end after
  rebuilding both packages and a full clean stack restart: a
  naive immediate-lookup test (no retry) still showed most attempts
  "failing" even at 60 Hz — but that test is stricter than what RViz
  actually does. A second, more faithful simulation that mimics
  `tf2_ros::MessageFilter`'s real behavior (queue a failed lookup and
  retry it on every subsequent `/tf` message, the way RViz's own display
  filter works, instead of failing on the first attempt) showed **198/198
  clouds resolved successfully, 0 timeouts, nothing left stuck in the
  queue** over a 10s live sample — confirming the fix actually works
  end-to-end, not just "the raw numbers went up." If this resurfaces
  after some future change, don't just look at whether `joint_state` or
  `/tf` volume/Hz "looks fine" in isolation — a naive one-shot
  `lookup_transform` test without retry semantics will look broken even
  when RViz would actually render fine; use the retry-simulation
  approach (or just watch RViz directly) to get a true answer.

**Added (2026-09-10): "Shutdown Everything" now powers off the Pi
itself, not just the ROS2 stack.** Direct consequence of the Pi 4 now
being the sole deployment target (see "Raspberry Pi 4 deployment"
above): on a field unit with no monitor/keyboard normally attached, just
SIGINT-ing the launch tree (the button's prior behavior) isn't actually
"safe to walk away from" — `tpl-scanner.service` would simply relaunch
it (`Restart=on-failure`), and even if it didn't, the Pi itself would
stay powered and drawing current in the field with nothing watching it.

Implemented in `tilt_axis_bridge/node.py`'s `_on_shutdown_stack`: after
the existing SIGINT-the-launch-tree step, it now also spawns a detached
background process (`subprocess.Popen(["bash", "-c", "sleep
POWEROFF_DELAY_S && sudo shutdown -h now"], start_new_session=True)`)
that waits `POWEROFF_DELAY_S` (new module constant, `8.0`, an untuned
generous guess -- not measured against how long the real SIGINT cascade
actually takes to finish stopping the motor/closing the serial port)
before cutting power. Deliberately **not** an in-process timer/delay on
this node's own thread -- this node is itself one of the things the
SIGINT cascade tears down, so anything scheduled on its own thread would
never fire once the process exits along with everything else;
`start_new_session=True` is what lets the sleep+shutdown survive that.
GUI changes: `index.html`'s existing confirm-dialog text updated to
actually describe what the button now does (physical poweroff, needs the
Pi physically power-cycled to use again -- was previously worded as if
it only affected the ROS2 stack); `status.html` (the onboard kiosk panel)
gained its own "Shutdown" button wired to the same `~/shutdown_stack`
service, previously only reachable from the full `index.html` GUI --
confirmed `confirm()` still renders as a real, touch-usable native dialog
under Chromium kiosk mode, so it gets the same are-you-sure gate rather
than a bare one-tap poweroff.

**Not yet verified against real hardware, and a real deployment
prerequisite is unconfirmed**: `tilt_axis_bridge` runs as the `tpl` user
under `tpl-scanner.service` (see step 9 in `README.md`) with no
controlling terminal, so `sudo shutdown -h now` will block forever on an
interactive password prompt unless `tpl` has **passwordless sudo**
scoped to the shutdown command specifically. Whether that sudoers rule
already exists on the real Pi was not checked this pass (this session
has no shell access to the deployed device) -- if it's missing, the
button will still SIGINT the stack successfully (that part doesn't need
sudo) but the Pi will just sit there indefinitely, never actually
powering off, with no error surfaced anywhere the GUI can see. Needs a
`visudo` entry (e.g. `tpl ALL=(ALL) NOPASSWD: /sbin/shutdown`) confirmed
on the real unit, then a real end-to-end test (click the button, confirm
the Pi actually loses power ~8s later) before this is trusted for field
use. Added as an open item below.

**Found and fixed (2026-09-11): the poweroff genuinely never worked,
on every single press, and the sudo prerequisite above wasn't the real
reason.** User-reported symptom, from real use, on the actual field
unit: "when pressing shutdown on either the pi or gui, all the ros side
seems to close, but the pi itself doesnt actually turn off." First
checked the suspected prerequisite from the entry above -- `sudo -n -l`
showed `tpl` already has blanket `(ALL) NOPASSWD: ALL` on this rig
(granted some other way, not this project's own doing), so that
wasn't it. Root-caused properly rather than guessed at, with direct
live evidence at each step, not just log-reading:
`systemctl show tpl-scanner.service` confirmed `KillMode=control-group`
(the default, never overridden by this project's own unit file) and
`Restart=on-failure`/`RestartSec=5s`. Sent a raw `SIGINT` straight to
the real `ros2 launch` process (bypassing the ROS service, to isolate
systemd's own reaction from anything in this project's Python) and
watched: the service went `inactive` within 1 second and **stayed**
inactive for the full 8s window -- so `Restart=on-failure` wasn't
actually re-triggering the stack (a real, if secondary, thing this
entry's original reasoning got right for the wrong assumed reason).
The actual mechanism: planted a real marker process directly inside
`tpl-scanner.service`'s own cgroup
(`echo $PID > /sys/fs/cgroup/system.slice/tpl-scanner.service/cgroup.procs`)
and sent the same `SIGINT` to the main launch process -- the marker
process was dead within 1 second, confirmed by direct polling, not
inferred. `KillMode=control-group` means systemd kills *every*
process in a service's cgroup as part of its own normal stop sequence
the moment the tracked main process exits, restart or not --
and `subprocess.Popen(..., start_new_session=True)` (the original
implementation) does a `setsid()`, detaching from the controlling
terminal/process group, but that is **not** the same thing as cgroup
membership: the spawned `sleep 8 && sudo shutdown -h now` process
stayed a member of `tpl-scanner.service`'s cgroup the whole time, and
died within ~1 second of the SIGINT above -- roughly 7 seconds before
it would ever have reached the actual `sudo shutdown` call, on every
single press, unconditionally. This fully explains the reported
symptom without any flakiness or timing luck involved.

Fixed by scheduling the poweroff through systemd itself instead of a
detached child process of this node: `sudo systemd-run --unit=tpl-
poweroff --collect --on-active=<POWEROFF_DELAY_S> /sbin/shutdown -h
now`. A `systemd-run`-created timer/service is registered directly
with the system's own PID 1 in its own transient unit under
`system.slice` -- never a child of `tpl-scanner.service`'s cgroup at
all, so its lifecycle is completely independent of that service's own
stop/restart behavior. `--collect` auto-removes the transient unit's
metadata once it fires, so repeated button presses (e.g. testing)
don't accumulate dead unit entries. Wrapped the `subprocess.run` call
in the same `subprocess.SubprocessError` handling this method already
uses for the `pgrep` call above, plus a non-zero-exit-code check
logged as an error -- best-effort, matching the rest of this method's
philosophy (the SIGINT teardown itself always still happens
regardless of whether the poweroff scheduling succeeds).

**Verified directly, empirically, not just reasoned through**: before
touching the fix, planted a `systemd-run`-scheduled marker-file task
and confirmed it fired correctly on its own (5s delay, file appeared
a couple seconds later than expected due to systemd's own timer
accuracy slop -- harmless for an 8s/real-world delay). Then repeated
the exact cgroup-kill scenario from the root-cause investigation above,
this time scheduling the marker via `systemd-run` instead of a plain
child process, and sent the identical `SIGINT` to the main launch
process: the marker fired correctly regardless, fully surviving the
same kill that reliably destroyed the old approach. Deployed the fix
to the real Pi (scp'd the file -- not committed to git yet, matches
this session's other deployments -- rebuilt with `colcon build
--packages-select tilt_axis_bridge`, restarted via `sudo systemctl
restart tpl-scanner.service`, confirmed a single clean process tree
afterward). **The actual end-to-end poweroff itself (click the real
button, confirm the Pi genuinely loses power ~8s later) was
deliberately not triggered this pass** -- doing so ends the SSH
session mid-investigation and needs the user physically present to
power the unit back on, so it needs a fresh, explicit go-ahead in the
moment rather than reusing an earlier one from before this specific
fix existed. Still the single most important item on the testing
checklist below.

**Found and fixed (2026-09-11): `~/home` could silently "succeed" with
zero real motion, and separately, a stuck driver call could wedge the
whole node.** Surfaced while hardware-verifying the new onboard-screen
tabs (see roadmap entry above): the user pressed the new Control tab's
Home button for real and reported "the motor homed before, but is
doing nothing now." Two distinct real bugs, found in sequence, not one:

1. **The wedge (recovered, root cause not fully isolated).** The very
   first real `~/home` press (from the touchscreen) hung -- confirmed
   on screen via the new error-handling UI actually working correctly
   (`home failed: service call timed out: /tilt_axis_bridge/home`,
   rendered in red exactly as coded). A follow-up `~/stop` also hung,
   and `/tilt_axis_bridge/status` stopped publishing entirely --
   consistent with `tilt_axis_bridge`'s single-threaded executor
   getting stuck inside a blocking call (most likely a synchronous
   serial read into the MKS driver with no/long timeout, given `_tick`
   and every service handler share that one thread), not a bug in the
   new UI code, which dispatched and rendered the failure exactly as
   written. Recovered with `sudo systemctl restart tpl-scanner.service`
   (~30s clean stop, all 7 processes came back up fine, motor status
   read `idle` afterward). The exact trigger was never isolated --
   flagged here rather than left silently unmentioned, in case it
   recurs.

   **It did recur, same day, 2026-09-11, during unrelated sweep-
   deskewing testing** (see the "Per-point sweep deskewing" entry under
   Distant features below) -- same signature exactly: process alive but
   stuck in kernel state D (confirmed via `ps aux`, not just inferred
   from symptoms this time), preceded by a burst of "MKS command
   failed: expected N bytes, got 0" warnings, `tilt_axis_bridge/status`
   going silent. Recovered the same way. Confirmed unrelated to
   whatever was being tested both times (front-panel UI code the first
   time, `scan_aggregator`-only changes the second) -- this is a real,
   recurring reliability issue in `tilt_axis_bridge`'s own serial/MKS
   driver handling, two occurrences in one session now, still not
   root-caused. Worth real investigation (a genuine timeout on the
   serial read, or figuring out what actually triggers it) rather than
   continuing to just restart through it if it keeps happening.

   **A third occurrence, same day, during the homing-race-condition fix
   below.** Same signature again (`tilt_axis_bridge/status` and
   `~/joint_state` both silent over a 15s window while `scan_aggregator`
   kept publishing fine on the same connection -- confirming rosbridge
   itself, not just `tilt_axis_bridge`). This one happened mid-homing
   during a real scan, and -- notably -- the fix below caught it
   correctly instead of masking it: it genuinely waited the full
   `homing_timeout_s` and reported a real abort, rather than the old
   buggy behavior of falsely believing homing had already finished and
   plowing ahead regardless. Recovered the same way; the verification
   scan completed cleanly on retry with no fourth occurrence. Three
   wedges in one session now -- still worth real investigation rather
   than continuing to restart through it indefinitely.

2. **The real bug: a race in `_tick_state_machine`'s `STATE_HOMING`
   handling, found, root-caused, fixed, and verified.** After the
   restart, `~/home` returned "homing started" but status went straight
   to `settled` with the joint_state position unchanged -- exactly the
   reported symptom, and initially indistinguishable from "it just
   homed instantly because it was already at the home position" (which
   it was, at the time -- `_last_position_rad` read ~0.0004 rad).
   Proved it was a real bug, not a coincidence, by moving the axis away
   first (`~/cmd_position` to 0.5 rad, confirmed via joint_state), then
   re-homing: still went straight to `settled` with **zero real
   motion** (position stayed at 0.5 rad-ish, never returned to zero).
   Root cause in [node.py](ros2_ws/src/tilt_axis_bridge/tilt_axis_bridge/node.py):
   `_tick_state_machine` polls at `poll_rate_hz` (10Hz/100ms default);
   `go_home()` is fire-and-forget over serial, and the very next tick
   (100ms later) checked whether the driver already reported
   `MOTOR_STATUS_HOMING`. If the driver hadn't flipped its own status
   register within that single 100ms window -- plausible serial/driver
   latency, not a real failure -- the old code concluded homing "never
   entered" and, since the motor was still enabled, immediately called
   `set_zero_point()` on whatever position the axis happened to be
   sitting at: a silent false "success" with no real motion, and now a
   wrong zero reference to boot.

   Fixed with a grace window plus a confirmed-start latch, not a bare
   timeout: `HOMING_ENTRY_GRACE_S = 1.0` (several polls' margin), and a
   new `self._homing_confirmed` flag set the first time
   `MOTOR_STATUS_HOMING` is actually observed. A not-yet-Homing reading
   is now only trusted as "stopped" (real completion or real stall) once
   either the grace window has elapsed or homing was already confirmed
   to have started -- so a genuinely fast real home (axis already at
   the switch) still completes promptly, while a slow-to-register one
   isn't mistaken for a failure mid-flight.

   **Verified live, definitively, not just by re-reading the code**:
   moved the axis to 3.5 rad (~200.5deg, confirmed via joint_state),
   deployed the fix (scp'd, `colcon build --packages-select
   tilt_axis_bridge`, `systemctl restart tpl-scanner.service`), then
   re-homed while holding a live rosbridge subscription open through
   the whole run (issuing `call_service` and `subscribe` on the same
   WebSocket connection, so there's no gap between commanding home and
   watching the very next status message -- the earlier CLI-based
   attempts to catch this in flight kept missing it, since separate
   `ros2 service call`/`ros2 topic echo` invocations have enough of
   their own startup latency to miss a homing run that turned out to be
   only ~1s on a small move). Status read `homing` continuously for the
   entire real run this time -- confirmed live for the first 20+
   seconds, well past the old single-tick race window -- and joint_state
   position landed back at ~0.0008 rad once it settled. This is a
   genuinely different, better outcome than the pre-fix behavior (zero
   motion, instant false settle) for the identical scenario. Axis left
   at its homed position afterward, not moved further.

**Found and fixed (2026-09-11): a `git pull` on the Pi's own checkout
left both GUI pages as real 0-byte files on disk -- a corrupted local
git object database, not a code bug.** Surfaced as "both the GUI and
onscreen menu only render a white screen" right after deploying the
live-coverage-feedback commit -- a real regression report, taken
seriously as one, not assumed to be a caching hiccup. First checks
ruled out the obvious: `tpl-gui-http.service`/`tpl-scanner.service`
both active, but `curl`-ing either page returned `HTTP 200` with **0
bytes** -- `ls -la` on the actual files on disk confirmed it, both
`index.html` and `status.html` genuinely empty. Root cause found via
`git status`/`git show HEAD:...` erroring with `error: object file
.git/objects/39/836e371... is empty` -- the commit object for the
just-pulled commit was itself a 0-byte file in the Pi's local object
store. `find .git/objects -type f -size 0` turned up 12 corrupted
objects total (roughly matching that commit's real object count:
4 changed files worth of blobs/trees plus the commit itself), pointing
at the `git fetch` inside that `git pull` having been interrupted or
truncated mid-transfer -- very plausibly related to the same
connectivity flakiness this Pi had shown earlier the same session
(went fully unreachable for a stretch, `tpl-lidar` mDNS still not
resolving even after it came back, IP-only SSH needed) rather than
anything about the commit's own content. **`node.py` was also one of
the zeroed files, but `tpl-scanner.service` kept running completely
unaffected** -- colcon's non-symlink install copies source into
`install/` at build time rather than reading it live, so the already-
built copy there was untouched; only the source tree and the two
directly-served static GUI files actually mattered here. Fixed by
deleting all 12 zero-byte objects (`find .git/objects -type f -size 0
-delete`), then a clean `git fetch origin` (git fsck now silent, no
integrity errors) followed by `git checkout -- <the 4 files>` to
materialize the now-healthy blobs onto disk -- confirmed via `curl`
returning real byte counts for both pages, then relaunched the kiosk's
Chromium (same "it doesn't auto-reload" reasoning as every other
GUI-file deploy this session) and grabbed a real `grim` screenshot
showing the Status tab live again, motor genuinely `idle` (independent
reconfirmation the earlier stall really had cleared). Takeaway for next
time this Pi's connectivity is flaky mid-`git pull`: a `git status`
right after any pull/fetch that happened during a shaky connection is
worth doing on principle, not just when something visibly breaks --
this one only *looked* fine (`git pull` printed a normal-looking
fast-forward summary with real insertion counts, no error surfaced at
the time) until the files were actually opened.

**Found and fixed (2026-09-11): a second sweep scan run right after a
first could skip showing "homing" entirely and start accepting real
data before the axis had actually finished (sometimes even *started*)
moving.** Real user report: "after running one sweep scan, when i run
another it seems to start collecting data right away, even while
homing, i can see it homes, but the ui doesn't report it... and the
preview behaves as if data is being collected while it happens."

Root cause: `self._tilt_status` (node.py) is just whatever
`tilt_axis_bridge`'s own `~/status` last said, cached via
`_on_tilt_status` -- it was never invalidated when a new scan started.
A run ends with the axis settled, so going into the *next* command
`self._tilt_status` is already `"settled"`; the home/move service call
is async (`tilt_axis_bridge`'s own next tick is what actually starts
real motion, not the call itself returning), so there's a real window
-- confirmed live, long enough to matter -- where `_tick`'s
`STATE_SWEEP_HOMING`/`STATE_HOMING`/`STATE_MOVING` branches can run and
see that stale `"settled"` before any fresh status reflecting the new
command has arrived, concluding the wait is already over before the
axis has even started. For `STATE_SWEEP_HOMING` specifically this meant
`_start_sweep()` fired near-instantly: real physical homing motion
continued regardless (confirmed -- `tilt_axis_bridge`'s own state
machine is unaffected by scan_aggregator's premature conclusion), but
scan_aggregator had already moved on to `STATE_SWEEP_SCANNING` and
started accepting clouds as real sweep data during it -- explaining
both halves of the report exactly: no "homing"/"sweep_homing" ever
visible (the whole transition happened within roughly one ~100ms tick,
too fast to register on screen even though the text was technically
correct for that one instant), and the coverage map/preview lighting
up as if real data was already landing during what was, physically,
still the homing move. The exact same race applies to every
`STATE_MOVING` wait too (step-and-stare's own inter-stop moves) -- just
far less visible there, since a genuinely-premature
`STATE_SETTLING_EXTRA` still waits `settle_extra_s` before capturing,
usually (not guaranteed -- a large step could still eat into real
settle time) covering for it.

Fixed with a new `_invalidate_tilt_status()` (sets `self._tilt_status`
to `""`, a sentinel guaranteed to never equal any real status string),
called at all four places that issue a command and later trust
`self._tilt_status == "settled"` to confirm it finished:
`_start_scan_impl`, `_start_sweep_scan_impl`,
`_on_start_mount_calibration` (all three, right before their own
`_home_client.call_async`), and `_advance_to_next_target` (right
before publishing a new `cmd_position` target) -- fixed for
step-and-stare's inter-stop moves too, not just the specifically
reported sweep case, since it's the identical defect.

**Verified live, and the fix surfaced a real, separate finding along
the way.** Two sweep scans run back to back: run #2 now correctly
showed `sweep_homing` continuously for a real ~10s (not skipped) before
transitioning to `sweep_scanning`, both runs completed to `done:` with
real output files, no errors. On an earlier attempt at this same
verification, run #2's homing hit the real 60s `homing_timeout_s` and
aborted -- not a bug in the fix, the opposite: `tilt_axis_bridge`
itself had wedged again (the third time this specific session --
`/tilt_axis_bridge/status` and `/tilt_axis_bridge/joint_state` both
publishing zero messages over a 15s window while `scan_aggregator` kept
publishing fine on the same rosbridge connection, `journalctl` showing
nothing further from `tilt_axis_bridge` after a burst of "MKS command
failed" warnings -- same signature as the two earlier wedges above).
The fix correctly refused to trust stale data through that wedge and
genuinely timed out and reported failure, rather than silently
believing a stale "settled" and proceeding to merge whatever garbage
arrived while the motor was actually stuck -- arguably the OLD buggy
behavior was actively *masking* this exact class of hang every time it
happened during a fresh scan start, since it never really waited long
enough to notice. Recovered with the usual `systemctl restart`, then
the two-scans test above completed cleanly on the retry with no fourth
wedge. Test scan files deleted after verification, `enable_sweep_deskew`
left untouched (this fix and that feature are unrelated).

**Investigated and mitigated (2026-09-11), per explicit request: the
recurring `tilt_axis_bridge` wedge itself** (three occurrences
documented above, same day). Root-caused, and fixed at the only level
Python actually has any power over -- full details below, since this
is a real, nontrivial piece of infrastructure now, not a one-line fix.

**Root cause.** A dedicated investigation read `mks_driver.py`'s entire
send/receive path (`_send()`) and confirmed: a real pyserial
`timeout=0.5s` *is* already set on the serial port (`DEFAULT_TIMEOUT_S`,
`mks_driver.py`) -- this was never a "someone forgot `timeout=`" bug.
The actual gap is one level below that: every wedge was independently
confirmed with the process sampled in kernel state **D**
(uninterruptible sleep) -- a thread blocked inside a kernel syscall
that cannot be interrupted by *anything* in userspace, not a signal,
not Python's own `try`/`except`, and not even that already-real
pyserial timeout, since a userspace timeout is built on top of the
exact same blocking read and never regains control if the underlying
syscall itself never returns (that's the literal definition of
uninterruptible sleep). This is almost certainly happening in the
USB-serial kernel driver/adapter itself, below pyserial entirely -- not
a bug in this project's own protocol code. `tilt_axis_bridge` runs
everything (motion, status, joint_state) on one single-threaded ROS2
executor, so one stuck call freezes the *entire* node -- nothing else
on it can run until the syscall returns, exactly matching every
observed symptom: total silence from just this one node (no status, no
joint_state) while other nodes on the same rosbridge connection kept
working fine, `~/stop`/`~/home` also hanging (they funnel through the
same driver, same lock, same thread), and the only recovery ever being
an external `systemctl restart` (a real process restart, the one thing
that *can* force a stuck fd/thread away, since the OS reclaims
everything when the process dies).

**The fix: `MksDriverWatchdog` (new class, `mks_driver.py`), a
transparent wrapper node.py now constructs instead of `MksDriver`
directly** (one-line change at the constructor call site -- every
public `MksDriver` method proxies straight through via `__getattr__`,
so no other code anywhere needed to change). Since Python fundamentally
cannot force a thread out of an uninterruptible syscall, this doesn't
fix the underlying kernel-level stall -- nothing at the Python level
can -- it makes the *node* resilient to it instead: every driver call
now runs on a fresh, dedicated `threading.Thread(daemon=True)`, and the
calling thread (this node's one ROS2 executor thread) only ever waits
up to a new `WATCHDOG_TIMEOUT_S` (2.0s, a generous margin over the
0.5s nominal command timeout) via `Thread.join(timeout=...)`. If that
elapses, the call -- and the worker thread it's stuck on, and the
`MksDriver`/`serial.Serial` instance whose own internal lock that
thread still holds forever -- is abandoned outright, not joined
further, not force-killed (nothing in Python can do that to a thread
in an uninterruptible syscall). A brand new `MksDriver` is constructed
for every call after that, so the node keeps publishing status/
joint_state and stays responsive to new commands instead of needing an
external restart. A watchdog timeout surfaces to the caller as an
ordinary `MksCommandError` -- the exact same exception type every
other driver failure already raises -- so it flows through `_tick`'s
existing "disconnected -> reconnect loop" handling with zero new
error-handling code needed on the `node.py` side; an ordinary exception
from the call itself (not a timeout) is re-raised exactly as raised,
so normal operation sees no difference at all from calling `MksDriver`
directly.

**A real design mistake, caught before it shipped, not after.** The
first version used `concurrent.futures.ThreadPoolExecutor(max_workers=1)`
instead of a raw thread -- cleaner-looking Python, and wrong for this
specific case in a way that would have made things *worse*, not
better: `ThreadPoolExecutor` registers a global `atexit` hook that
joins *every* worker thread any executor in the process ever created,
unconditionally, before the interpreter is allowed to exit at all.
Verified directly with a throwaway script (a `ThreadPoolExecutor` task
abandoned after its own 1s `result(timeout=...)` expired, script then
tries to exit without calling `shutdown()`): the process did not
actually exit until the full ~30s the abandoned task's own sleep took
to finish, not promptly after the timeout. For this node that would
have turned "one wedged serial call" into "this whole process's clean
shutdown now hangs for however long the kernel-level stall lasts,
possibly forever" -- a real regression for a project that specifically
depends on a fast, reliable SIGINT-triggered shutdown for its own
physical poweroff feature (see the poweroff entry above). Switched to
a plain `threading.Thread(daemon=True)` per call instead -- daemon
threads are explicitly excluded from Python's own interpreter shutdown
sequence by design, confirmed the same way (the equivalent throwaway
script exited promptly, no ~30s wait, with an abandoned daemon thread
still nominally "running" in the background). `destroy_node()`'s own
`self._driver.close()` (previously unable to raise, now able to if the
driver happens to be wedged mid-shutdown) was also wrapped in a
best-effort `try`/`except MksCommandError` so a watchdog timeout there
can't block a clean shutdown either.

**Verified four ways, the last one live and unplanned.** (1) A
standalone test (a fake `MksDriver` monkeypatched in, no real hardware
needed) confirmed: a normal fast call still works untouched; a normal
exception raised by the driver itself (not a timeout) is re-raised
exactly as-is with *no* rebuild; a genuinely hanging call times out at
very close to `WATCHDOG_TIMEOUT_S` and raises a clear `MksCommandError`;
a fresh driver is constructed after that and subsequent calls succeed
against it; and -- the specific regression this whole class exists to
avoid -- the process exits *promptly* even with an abandoned, still-
"running" daemon thread left behind, unlike the `ThreadPoolExecutor`
version. (2) Real hardware: full regression check, `~/home` -- real
physical homing motion, `settled` at the correct position, no
difference from before this change. (3) Real hardware: a real
`cmd_position` move (0.4 rad target, landed at 0.4015 rad, normal
tolerance) -- `settled` -> `moving` -> `settling` -> `settled`, exactly
as before. (4) **A real, live wedge happened mid-testing, entirely
unplanned, and it's the most convincing evidence this actually works**:
mid-homing, the journal showed the watchdog firing exactly as designed
(`read_cumulative_encoder: no response within 2.0s`), followed
immediately by `_tick`'s own existing "Driver communication lost"
handling picking up the resulting `serial.SerialException` from the
freshly-rebuilt-but-not-yet-`open()`'d driver and correctly arming the
reconnect loop -- **the node stayed alive and kept clearly
self-reporting `disconnected` every ~5s reconnect attempt** the entire
time, confirmed via `ps aux` (healthy, responsive process state, stable
memory, no growth from repeated rebuilds) -- a night-and-day difference
from the previous three occurrences' total, silent, unrecoverable-
without-outside-intervention death. In this specific instance the
underlying stall turned out to be persistent enough that even a fresh
`MksDriver`/`serial.Serial` *within the same process* couldn't escape
it (every reconnect attempt's own `set_enable()` call kept re-hitting
the same 2.0s watchdog timeout) -- an honest limit of what this fix
can do, since it operates one level above the actual kernel-level
resource; a full `systemctl restart` (a real new process, a
genuinely fresh fd from the OS's own perspective) was still what
finally cleared it that time, confirmed via a clean "Connected to MKS
driver" log line and `idle` status afterward. So: this fix is a real,
verified, working improvement -- the node no longer goes completely
dark and unrecoverable-except-by-restart on every wedge, and in many
cases (a transient, non-persistent stall) should now self-heal within
about 2 seconds with no restart needed at all -- but it is not a
guarantee against every possible manifestation of a kernel-level USB
stall, some of which may still need a real process restart (or,
if this keeps recurring, genuine hardware investigation: a different
USB-serial adapter, checking USB autosuspend settings, `dmesg` around
wedge timestamps -- none of that was done this pass, per explicit
scope, but is the natural next step if the watchdog's own self-heal
path turns out not to be enough in practice going forward).

**Found and fixed (2026-09-11): the onboard screen's Status tab showed
the wrong port.** `status.html`'s Network line (`pollNetInfo`) read
`SSID · wlan0 IP · :9090` -- 9090 is rosbridge's own internal websocket
port, not something anyone types into a browser. The whole point of
that line is "how do I reach this device from a phone/tablet," which is
`tpl-gui-http.service`'s `python3 -m http.server 8080` (see README.md),
the same port this very page was just loaded from. Changed the
hardcoded suffix to `:8080`. Deployed and confirmed live in a real
browser against the Pi's own `net_info.json`: the Network line now
reads e.g. `LiFi · 192.168.0.115 · :8080`.

## Decisions made this session (context for "why", not just "what")

- **Raspberry Pi 3B field-recording deployment**: investigated in detail
  (cost, RAM budget, setup steps) and real code changes landed in support
  of it (`enable_pointcloud` launch flag, `rosapi_node` elimination saving
  ~80MB RSS) — but the user ultimately chose to **stick with the Windows/WSL2
  setup**, not deploy to a Pi, at the time. **Superseded 2026-08-29**: now
  actively deploying to a Pi 4 (4GB) instead — see "Raspberry Pi 4
  deployment (in progress)" under "Running it". The RAM/launch-flag work
  from this investigation is directly relevant again, not just harmless
  leftover.
- **Position readout switched to degrees** in the GUI header (was radians).
- **PC↔Pico serial baud is now live-configurable** (Motor page's Config
  sub-tab dropdown +
  `serial_baud` parameter), to follow the Pico bridge firmware's own move
  away from a hardcoded UART speed.
- **Reversed from "stay in FOC" earlier this session to switching to Close
  mode, then removing FOC entirely.** User found Close mode gave smoother/
  more precise movement in practice, which outweighed the earlier
  documented reasoning for FOC (flagship three-loop control, runs cooler).
  A real bug muddied the transition for a while: `_try_connect_driver()`
  was unconditionally forcing work_mode back to `BusVFoc` on every
  connect/reconnect, so Close mode kept silently reverting until that was
  found and fixed (see "Live work-mode switches" gotcha below) — most of
  the session's actual runtime was still on FOC even after the user had
  switched. Once Close mode's persistence was confirmed working, the user
  asked to remove FOC and everything FOC-only outright: the Work mode
  dropdown no longer offers `PulseVFoc`/`BusVFoc`, the manual PID panel
  and the entire auto-tune feature (service, GUI panel, `pid_autotune.py`)
  are gone, and `set_pid_vfoc` was dropped from the raw driver-command
  dispatch table. See "PID tuning" above for what tuning looks like now.
- **Motor tab restructured from four sub-tabs to two** (Jog / Config).
  Jog holds only day-to-day operation (Enable/Disable/E-stop, status,
  Homing, Go to degree, Continuous sweep, Step-and-stare); everything else
  (raw motion testing, all driver configuration, diagnostics) moved into
  Config, per explicit request rather than by feature category.
- **GUI reskinned to an industrial "Hardened Control Room" theme** — Windows'
  built-in Bahnschrift face, squared-off corners, beveled panel edges,
  corner rivets (pure CSS background layers, no markup changes), brushed-
  metal header texture, phosphor-green accent. Picked from three mocked-up
  directions (Control Room / Machine Panel / Blueprint) presented as a
  standalone comparison artifact before applying.
- **Added a project-wide hard speed limit (40 RPM)** and a
  `reverse_direction` flip for the whole coordinate system — see the
  gotchas below for both.
- **Added a "Shutdown Everything" GUI button** (`~/shutdown_stack`) that
  SIGINTs the whole `ros2 launch` process tree, for tearing down the
  entire stack from the browser instead of needing terminal access.

## Open items / next steps

From the original brief, still open:
- Empirically measure per-stop settle time to finalize tilt step size / total
  scan time trade-off.
- ~~Calibrate the tilt-axis-to-sensor-origin lever arm (lives in
  `scanner_description`'s URDF once measured).~~ Built: the numbers
  themselves are still uncalibrated (all six default to 0, i.e. CAD's "in
  line with the optical centre" assumption) and still need empirical
  confirmation once real measurement/target-based calibration happens --
  but *how* to enter that calibration changed. It's no longer a URDF edit
  + rebuild; it's six live GUI fields (translation X/Y/Z + roll/pitch/yaw)
  on the Config > VLP-16 page, see "VLP-16 configuration" below.
- ~~Decide targets vs. ICP for multi-station registration; plan field
  overlap.~~ Decided: ICP, done in other (external) software -- out of
  scope for this project entirely. Each run here just needs to keep writing
  one independent, well-formed `.pcd` per station (already how
  `scan_aggregator` works -- one file per run, no cross-run merging
  attempted), so no code changes follow from this. Field overlap between
  stations is the user's job at capture time, not something this codebase
  needs to plan or enforce.

Still to be done:
- Confirm the `tpl` user has passwordless sudo for `shutdown` on the real
  Pi, then verify the new "Shutdown Everything also powers off the Pi"
  behavior end-to-end against real hardware (see the 2026-09-10 entry
  above) -- unconfirmed, no shell access to the deployed device this pass.
- Test with real LiDAR (VLP-16 attached, not just the tilt axis alone) --
  unit has arrived and real-hardware testing is underway. Multiple real
  bugs found and fixed against it across several sessions now (WSL2
  mirrored networking + a Windows Firewall rule needed for the sensor's
  inbound UDP broadcast to reach it at all; a self-defeating
  `lookup_transform` timeout in `_transform_and_accumulate`; a `/mnt/*`
  `output_dir` cloud-drop issue that was open/unexplained for a while and
  has since been resolved; the sweep-turnaround ghost-duplicate artifact,
  see "Scan modes"/"Known gotchas" above for these) -- none were
  reachable by any earlier mocked test.
- Use this unit's actual per-laser calibration instead of the generic
  stock one. Currently `velodyne_transform_node`'s `calibration` parameter
  points at the driver's bundled generic `VLP16db.yaml` (see
  `scanner_bringup/config/vlp16.yaml`'s comment), not this specific
  sensor's real factory calibration -- ROS never fetches it from the
  sensor automatically. `ros-drivers/velodyne`'s `ros2` branch has a
  `velodyne_pointcloud/scripts/gen_calibration.py` that converts the
  manufacturer's per-unit `db.xml` (degrees/cm) into the YAML (radians/m,
  REP-103) the driver actually reads -- confirmed present in that repo's
  source, but **not currently installed** in this project's conda/
  RoboStack ROS2 environment, so it'd need fetching separately. Where to
  actually obtain this unit's `db.xml` (sensor web interface, included
  physical/digital media, or a Velodyne serial-number lookup) is
  unconfirmed -- worth checking once the unit is physically in hand.
  Deliberately not started yet (no file to convert until then).
- ~~Validate scan data (some form of sanity/quality check on a completed
  `.pcd`, not just "the run finished without error").~~ Superseded
  2026-09-10 -- folded into "post-scan completeness" under the "Coming
  soon" tier of the Feature roadmap at the end of this file, same idea,
  now prioritized/scoped against the actual use case.
- ~~Banner/indicator that appears while a scan is actively running, so
  it's obvious at a glance from across the room.~~ Built 2026-08-29: a
  large banner spanning the full page width, sitting directly under the
  header (visible on every tab, not just the Scan tab), driven by
  `scan_aggregator`'s `~/status` text -- shows a real percentage for
  step-and-stare and a duration-bounded sweep, an animated striped
  "indeterminate" fill for homing/an unbounded sweep/saving, red for an
  aborted run, and is hidden entirely while idle. See `index.html`'s
  `parseScanStatus`/`updateScanProgress`. Verified in-browser against all
  of the real status-text shapes `_publish_status` can actually produce.
- ~~Support working from a remote webpage (phone, tablet, etc.), not just
  a browser on the same machine as rosbridge.~~ In progress 2026-08-29 as
  part of the Pi 4 deployment -- see "Wi-Fi hotspot + controlling it from
  a phone/tablet" under "Raspberry Pi 4 deployment" above and
  `README.md`. The GUI-side piece (defaulting the rosbridge address to
  whatever host served the page, not always `localhost`) is done and
  tested; the hotspot/GUI-HTTP-server/systemd-autostart side is
  documented but not yet tested against real Pi hardware. **Extended
  2026-08-29**: documented how to switch `wlan0` between home-WiFi-client
  and hotspot duty without it being a one-way trip -- both NetworkManager
  connection profiles stay configured permanently, and an
  `autoconnect-priority` difference between them (home WiFi higher) lets
  NetworkManager auto-switch on its own (home WiFi when in range,
  hotspot fallback when not), plus two one-line manual-override scripts
  (`tpl-wifi-home`/`tpl-wifi-hotspot`) for forcing either on demand. See
  "Switching between home WiFi and the hotspot" in `README.md`. **Further
  extended same day**: documented full kiosk-mode auto-boot for the case
  where the Pi has its own attached screen (Desktop Autologin via
  `raspi-config`, screen-blanking disabled via Wayfire's `dpms_timeout`,
  and a small wrapper script that waits for `tpl-gui-http.service` to
  actually be serving before launching Chromium in `--kiosk` mode against
  `localhost:8080` -- avoids a race between the desktop session starting
  and that systemd service finishing startup independently). This
  resolves the "onboard screen not yet decided" open question from a
  build-it-both-ways angle rather than picking one: kiosk mode covers the
  attached-touchscreen path, the phone-as-client approach from the WiFi
  hotspot work covers the other, and neither conflicts with the other
  since the GUI server/rosbridge already support multiple simultaneous
  clients. Flagged honestly in `README.md`: a full desktop + Chromium is
  real additional RAM/CPU pressure on top of the ROS2 stack itself, on
  hardware whose headroom for even just the core workload is still
  unconfirmed -- dropping kiosk mode for phone-only is the easy fallback
  if the Pi turns out tight. **Superseded same day, within hours**: the
  actual screen turned out to be a small 380x420 GPIO/SPI panel, not an
  HDMI/DSI monitor -- the whole Chromium-kiosk plan above doesn't apply,
  a panel that small/that connected can't run a real browser at all.
  Replaced with a much narrower design: the panel only ever shows current
  scan/tilt status plus the IP to reach the real web GUI at, all actual
  control stays in that GUI. Also means `rviz2` isn't needed on the Pi at
  all -- removed from step 4's dependency list entirely (one less
  `aarch64`-availability risk) -- and Raspberry Pi OS **Lite** (not
  Desktop) is now step 1's recommendation, since nothing needs a desktop
  environment any more either. New `scripts/pi/status_display.py`: a
  standalone script (not a colcon package, doesn't need its own
  topics/services), subscribes to `/tilt_axis_bridge/status` and
  `/scan_aggregator/status` -- the same topics the web GUI itself already
  reads -- plus a `current_ip()` helper that reads whatever address
  `wlan0` currently has, deliberately not caring whether that's the
  hotspot's fixed `10.42.0.1` or a DHCP-assigned home-WiFi address (works
  unchanged either way, ties directly into the WiFi-switching work).
  Everything except the actual panel-drawing call (a `TODO`, blocked on
  knowing the exact panel model -- new "Open questions" entry) is real,
  tested code, not just planned: an isolated-`ROS_DOMAIN_ID` test
  confirmed the subscription callbacks receive and store real messages
  correctly (throwaway publisher, both topics) and `current_ip()` against
  both a real interface and a missing one. **A real bug this caught**:
  the first cut's `main()` (`except KeyboardInterrupt: pass` then an
  unconditional `rclpy.shutdown()` in `finally`) raised a second, spurious
  "rcl_shutdown already called" error when actually tested under a
  `timeout`-forced SIGTERM -- the same signal systemd sends on every
  service stop/restart, so this would have hit on every single deploy,
  not just Ctrl-C. `rclpy.spin()` surfaces a SIGTERM as
  `ExternalShutdownException`, not `KeyboardInterrupt` -- that exception
  already tears down the context, so the following unconditional
  `rclpy.shutdown()` call doubles up. Fixed: catch
  `ExternalShutdownException` alongside `KeyboardInterrupt`, and guard the
  `finally` block's `rclpy.shutdown()` with `if rclpy.ok()`. Worth noting
  this exact pattern (`except KeyboardInterrupt: ... finally: ...
  rclpy.shutdown()`, no `ExternalShutdownException` handling) is already
  present in every other node's `main()` across this codebase
  (`tilt_axis_bridge`, `scan_aggregator`, `vlp16_config`) -- not fixed
  there as part of this change (out of scope, not what was asked), but
  the same latent noisy-shutdown-log behavior likely applies to all of
  them under `systemctl stop`/`restart` on the Pi too, worth a pass if it
  ever actually causes a problem rather than just log noise.

  **Extended same day, panel identified**: it's an Elecrow RR035 /
  ELEGOO 3.5" GPIO touchscreen (confirmed the same hardware) -- 480x320,
  XPT2046 resistive touch (the earlier "380x420" figure was wrong;
  corrected against the vendor's own product page). Also a real change
  in intent, not just a spec correction: it should function like a real
  HDMI monitor for general debugging (boot messages, login prompt, a
  full shell), not just show a fixed custom status readout -- a
  genuinely different, and better-supported, goal for this class of
  panel. Researched properly (`WebSearch`/`WebFetch`) rather than
  guessed, since a wrong driver recommendation here would send a real
  debugging session down a dead end: current Bookworm ships a mainline
  `piscreen` DRM overlay (`dtoverlay=piscreen,drm,speed=18000000` in
  `/boot/firmware/config.txt`) that a Raspberry Pi engineer confirmed
  working for this exact panel/touch-controller combination in a real
  2026 forum thread -- no third-party driver needed as the first thing
  to try. Elecrow's own current driver repo
  (`Elecrow-keen/Elecrow-LCD35`, distinct from the older generic
  `goodtft/LCD-show` their own wiki still references) is the documented
  fallback, with a real Pi4-specific gotcha found and recorded: `vc4-kms-v3d`
  needs changing to `vc4-fkms-v3d` in `config.txt` first on system images
  after 2021-10-30, or the installer fails to start. Whichever route
  works, the panel becomes a normal Linux console either way -- which
  means `status_display.py`'s "panel-drawing call, TODO, blocked on
  unknown hardware" placeholder from the entry above is now **fully
  resolved, not just narrowed**: no panel-specific graphics library is
  needed anywhere in this project, plain ANSI terminal output (clear
  screen + print) already reaches it like any other console. Replaced
  the placeholder with exactly that -- verified the render call actually
  produces correct output for real published status messages (extending
  the same isolated-`ROS_DOMAIN_ID` test from before), though the escape
  codes themselves obviously haven't been eyeballed against the real
  console yet. Also changed `status_display.py`'s intended role: no
  longer auto-started over the login prompt (would work against the
  "acts like an HDMI monitor for debugging" goal this whole pivot is
  for) -- it's an on-demand tool now, run manually for a quick glance,
  console otherwise stays a normal login/shell. See "Onboard screen
  (Elecrow RR035 / ELEGOO 3.5\" GPIO touchscreen)" in `README.md` for
  the full writeup and sources.
- **Reversed, 2026-09-07, per explicit request**: the panel now *does*
  auto-launch a fullscreen kiosk status display at startup after all --
  the "on-demand tool, not auto-started" call two entries above no
  longer holds. New `web/tilt_axis_gui/status.html`: a purpose-built,
  compact status page (not the full control GUI -- deliberately, the
  480x320 panel has no room for that) showing live network address,
  motor state, VLP-16 reachability, and scan state, via the exact same
  plain-rosbridge-WebSocket-subscription pattern as the main GUI.
  Network info comes from a tiny separate `net_info.json`
  (`scripts/pi/write_net_info.sh` + `tpl-net-info.timer`, since a
  browser page can't query network interfaces itself). Autostart is
  `scripts/pi/labwc_autostart`, installed as `~/.config/labwc/autostart`
  -- confirmed this Pi's actual desktop uses `lightdm` autologin +
  `labwc` (a wlroots/Openbox-alike Wayland compositor), not X11/LXDE as
  the original Bookworm plan assumed; `xrandr` confirmed the panel is
  genuinely the whole display at `480x320`, not a secondary output.
  **Real gotcha found and fixed, not guessed**: Chromium's first launch
  on a fresh desktop tries to create a system keyring via
  `gnome-keyring` and blocks on an interactive "Choose password for new
  keyring" prompt instead of ever showing the kiosk page -- caught by
  actually screenshotting the live panel over SSH (`grim`, the standard
  wlroots screenshot tool: `XDG_RUNTIME_DIR=/run/user/1000
  WAYLAND_DISPLAY=wayland-0 grim out.png`, pulled back and inspected
  directly) rather than trusting "the chromium process exists" as proof
  of correctness. Fixed with `--password-store=basic`. **Verified with
  a full cold reboot and a second real screenshot**: kiosk page comes up
  on its own showing genuinely live data (real SSID/IP, `idle` motor,
  `online` VLP-16, `idle` scan) -- zero manual steps. `status_display.py`
  (the ANSI terminal version from the entry above) is kept as-is, still
  useful as an SSH-only alternative with no screen needed, just no
  longer the primary way status is shown on this device. Full setup
  commands in README's step 10, same section.
- **Real USB-stick I/O failure during a sweep scan, 2026-09-07 --
  genuinely a storage/hardware problem, not a `scan_aggregator` bug, but
  most of the data was recoverable.** User reported the completed scan's
  output file appearing at 0 bytes with no renamed file ever showing up
  (the GUI's post-scan rename popup, see `_on_rename_output_request`).
  Traced through `dmesg` rather than the application code first, since
  the symptom (0-byte file, rename silently not happening) didn't match
  how `_finish_run`/`_write_output_in_background` actually work
  (`_last_output_path` is only set *after* `write_pcd()` returns, so a
  rename request has no completed file to act on if the write itself
  never finished) -- found real block-level errors: `Buffer I/O error on
  dev sda1 ... lost async page write` (several, during the write) and
  `exFAT-fs (sda1): Volume was not properly unmounted. Some data may be
  corrupt.` The mount's own `errors=remount-ro` safety option (see
  README step 6) didn't trip a full read-only remount this time, but the
  interrupted write still left a corrupted lost-cluster chain.

  **Not lost, though**: exFAT's own recovery had already swept the
  orphaned clusters into `/mnt/tpl_usb/FOUND.000/FILE0000.CHK`.
  Confirmed it was genuinely the missing scan by reading its first few
  hundred bytes directly -- a real, well-formed PCD v0.7 header
  (`FIELDS x y z intensity`, binary data). Its header claimed
  `POINTS 17471384`, but the actual binary body was shorter -- computed
  exactly (body length / 16 bytes-per-point): **12,611,571 complete
  points actually present (72% of the original), plus 14 trailing
  partial bytes from a point that was mid-write when it failed.** Wrote
  a small one-off script to rebuild a valid file: patch `WIDTH`/`POINTS`
  in the header to the real count, drop the trailing partial bytes, save
  as `scan_20260907_151601_recovered_partial.pcd` (timestamp taken from
  the recovered blob's own mtime, `_recovered_partial` suffix so it's
  never mistaken for a clean, complete scan later). Verified the
  resulting file's size matches the corrected header exactly.

  **Root cause is most likely the USB stick itself, not anything
  code-side** -- a 150MB write+`sync`+SHA-256-verified round trip
  afterward completed with a correct checksum (data integrity fine for
  that size), but the `sync` alone took ~28s for 150MB (~5.5MB/s
  effective flush speed), notably slow for exFAT on a USB stick and
  consistent with a marginal/slow device that could plausibly fail
  outright under the much larger continuous write a sweep scan produces
  (this one: ~280MB intended, for 17.5M points). **Not yet
  root-caused further** -- worth trying a different/known-reliable USB
  stick if this recurs, especially for sweep-mode scans (larger,
  single, continuous writes) more than step-and-stare (written once at
  the end too, but from a run that's typically far smaller -- the
  earlier real step-and-stare test this same day was 220,095 points,
  ~3.5MB). The filesystem itself is still flagged dirty
  ("not properly unmounted") from this incident -- not fixed in place
  since forcing an unmount+`fsck.exfat` while the desktop's own
  auto-mounter and a live GUI session were both active risked more
  disruption than benefit; recommended fix is the same as this
  project's own designed workflow -- pull the stick and let Windows
  (or a clean Linux `fsck.exfat`) check it next time it's convenient,
  the same way it'd get pulled to grab data anyway.

  **Correction, same day, once the error recurred on a second scan**:
  the hardware theory above was wrong, or at least not the real root
  cause. User reported unplugging the stick *because the GUI said the
  scan had saved successfully* -- the "USB disconnect" `dmesg` had shown
  wasn't a spontaneous hardware fault at all, it was the user
  reasonably trusting a "done" status and then physically removing the
  drive. That reframed the whole incident: the actual bug is in
  `pcd_writer.write_pcd()`, which closed its file handle without ever
  calling `os.fsync()`. Python's `close()` only flushes its own buffer
  into the OS page cache -- it does **not** guarantee the OS has
  actually written the data through to the physical device. For a
  large write to removable USB storage that gap can be substantial:
  `scan_aggregator` marked `STATE_DONE` (and the GUI showed "done") the
  moment `write_pcd()` returned, while a real chunk of the write could
  still be sitting in cache, not yet durable -- exactly consistent with
  both recovered files being truncated (one at 72% of its claimed
  points, the other landing suspiciously close to its own claimed count
  -- recovered a second file from this second incident too, this time
  using the header's own claimed count as authoritative once confirmed
  enough real bytes existed to cover it, rather than trusting a raw
  byte-count that turned out to include exFAT recovery cluster-padding).

  Fixed properly: added `f.flush()` + `os.fsync(f.fileno())` before the
  file closes in `write_pcd()`, so `STATE_DONE`/"done" now only fires
  once the data has genuinely reached the device. **Verified with direct
  proof, not just code review**: attached `strace -f -e trace=fsync` to
  the live `scan_aggregator` process during a real test scan and
  confirmed `fsync(22) = 0` actually firing on the write thread, then
  confirmed the resulting file's size matched its header exactly
  (`157951` points x 16 bytes + header = the file's real byte count).
  Lesson for next time a "device disconnected mid-write" symptom shows
  up again: check *why* it disconnected (dmesg alone doesn't say) before
  assuming hardware -- this one was the software reporting success too
  early, not a bad stick or a bad port after all.

  Unrelated near-miss during this same investigation, worth a one-line
  note: killing a `strace -f` session that was ptrace-attached to an
  18-thread process (via `pkill`) was immediately followed by the whole
  Pi going unreachable on the network (not just SSH refusing -- ICMP
  "Destination Host Unreachable") for about a minute. Turned out to be
  the user power-cycling the Pi for an unrelated reason at roughly the
  same time, not actually caused by the `pkill` -- but given how it
  looked in the moment, prefer `-p <pid>` strace sessions that exit on
  their own (e.g. bound by `timeout`) over ones needing to be killed
  externally, just in case that assumption is ever wrong.
- **This rig's real calibration is now hardcoded, no settings-file
  import needed for a fresh install, 2026-09-07.** Direct follow-up to
  the stall above: importing a backed-up settings file fixed *this*
  install, but every *future* fresh install (new Pi, wiped SD card, lost
  settings file) was still exposed to the identical failure mode. Every
  `declare_parameter` fallback literal across `tilt_axis_bridge`,
  `scan_aggregator`, and `vlp16_config` now matches this rig's actual
  current values (inverts, mount offsets, tilt/sweep ranges, motor
  speed/accel, `reverse_direction`, `output_dir`) instead of generic
  placeholders -- a persisted settings file still overrides all of these,
  this only changes what a *fresh* one starts from. The safety-critical
  piece: `_apply_persisted_mks_driver_settings` previously did nothing at
  all when the settings file's `mks_driver` section was empty/missing --
  exactly the gap that let the original stall happen. Added
  `DEFAULT_MKS_DRIVER_SETTINGS` (this rig's real driver config, including
  the corrected `home_direction`) as a baseline merged under whatever's
  actually persisted, so a fresh install always gets a safe, correct
  driver configuration pushed on connect now, not whatever happened to
  already be sitting in the driver's own EEPROM.

  **Verified for real, not just by code review**: moved the actual
  `~/.lidar_scanner_settings.json` aside entirely (simulating a genuinely
  fresh install) and restarted the stack. Hit a real, unrelated scare
  mid-test -- `tilt_axis_bridge` went unresponsive (silent status topic,
  `ros2 node list` missing several nodes) a few minutes after a clean
  connect. Turned out to be `ros2 node list`/`topic echo`'s own known CLI
  flakiness compounding a genuine but separate hardware hiccup (`ps aux`
  confirmed every process was still alive throughout -- nothing had
  crashed); a physical check + power cycle of the driver plus a fresh
  service restart cleared it. Once clean, confirmed via a direct `rclpy`
  status check (not the flaky CLI) and individual `ros2 param get` calls:
  every hardcoded default -- `invert_x/z_axis`, `tilt_end_deg`,
  `mount_roll_deg`/`mount_pitch_deg`, `reverse_direction`, `output_dir` --
  came up correct with **zero settings file present**, and zero
  "Failed to apply persisted mks_driver.*" warnings, meaning every driver
  command including the corrected `home_direction` applied cleanly.
  Restored the real settings file afterward (it still carries the named
  scan presets, which aren't part of this hardcoded-baseline mechanism)
  and did one final clean restart to confirm nothing regressed.
- **Double-image overlap artifact: root-caused and a calibration tool
  built, 2026-09-07.** User had reported this exact symptom on both the
  Pi and the original laptop setup -- a duplicate/ghosted copy of part
  of the scene, offset along the VLP-16's own spin direction, present in
  only a narrow portion of a scan. Several wrong hypotheses ruled out in
  order with real evidence before landing on the actual cause: not dual
  return mode (user confirmed flat walls were *also* affected, which
  dual-return ghosting wouldn't explain), not the sweep-turnaround
  settle dwell (increasing `settle_time_s` from 0.3s to 2.0s made no
  difference on a real re-test). **Actual cause, found by the user**:
  any scan range past ~150 degrees causes the VLP-16's own 30-degree
  vertical FOV to cover some physical geometry twice, through two
  different (spin azimuth, tilt angle) combinations meant to reconstruct
  onto the same points -- and they don't, because the physical mount
  isn't sitting at exactly the assumed ~45-degree angle
  (`mount_roll_deg`/`mount_pitch_deg`) the whole geometric model assumes.
  Real-world mounting is never perfectly precise, so this was never
  going to be exactly right without either a much more precise physical
  jig or a software calibration pass.

  Built `scripts/calibration/` (two-part, see README's own writeup for
  the exact usage): a small rclpy listener that captures raw
  `/velodyne_points` (sensor frame, pre-mount-transform) tagged with the
  tilt angle at capture time during a real overlap scan, and a separate
  offline numpy-only script that searches for the `mount_roll_deg`/
  `mount_pitch_deg` that makes a user-specified flat surface (caught in
  the overlap) reconstruct as a single sharp plane instead of a doubled
  one -- replaying the exact same `velodyne -> tilt_link -> base_link`
  transform chain `vlp16_config`/the URDF actually use, just batched in
  numpy instead of per-cloud tf2 lookups. No scipy dependency -- a small
  hand-rolled compass-search optimizer instead, matching this project's
  existing preference (see `pcd_writer.py`'s own docstring) for a short
  hand-rolled implementation over a dependency for one thing.

  **`mount_yaw_deg` is deliberately excluded from the search** -- a real
  finding, not a shortcut: confirmed algebraically
  (`Rz(tilt) @ Rz(yaw_error) == Rz(tilt + yaw_error)` for every point,
  since the tilt joint only ever rotates about one fixed axis and yaw
  composes with it additively) and then with a synthetic test, that any
  yaw error is exactly equivalent to a rigid rotation of the *entire*
  output about that axis -- which preserves every internal geometric
  relationship perfectly, so no self-consistency check can ever recover
  it, regardless of how many or how varied the target surfaces are. Also
  tried, before landing on excluding it: a two-non-parallel-wall
  synthetic test specifically to see if that would constrain yaw the way
  it would for a more generic extrinsic calibration problem -- it
  didn't, confirming the degeneracy is structural (specific to this
  single-rotation-axis tilt joint), not just "not enough data." This
  doesn't matter for the reported bug regardless: a yaw error can't
  create or explain internal doubling, only roll/pitch can.

  **Verified end-to-end against synthetic ground-truth data** (a real
  overlap scan wasn't re-run against the finished tool this session):
  generated known-truth points for a flat wall as they'd appear from the
  raw sensor frame under a deliberately-wrong initial guess, confirmed
  the forward-transform round-trips to machine precision, then confirmed
  the optimizer recovers the true roll/pitch exactly (0.0000 degree
  error) starting from a wrong guess, run both as direct function calls
  and as the actual shipped script via a real subprocess with real CLI
  args -- not just reviewed by eye.
- ~~Add VLP-16 configuration (currently only `config/vlp16.yaml` at the file
  level — no GUI exposure).~~ Built: new `vlp16_config` package + GUI tab,
  see "VLP-16 configuration" below. Verified against a mocked sensor/mocked
  rosbridge, not yet against the real unit (still hasn't arrived) --
  specifically, `~/status`'s raw JSON has never been checked against a real
  `/cgi/status.json` response.
- ~~Store scan presets (step-and-stare/sweep parameter sets) for quick
  recall.~~ Built, split the other way from what this note originally
  proposed: Scan presets and Config presets are two independent,
  custom-named preset types (not driver/motor config riding along inside a
  scan preset), each a named dropdown (Load/Save As New/Delete) backed by
  a dict stored inside the main shared settings file -- see "Presets: Scan
  vs Config, independently named/loaded" above.

Maybe to do:
- More industrial-style GUI (visual/theming pass, distinct from the
  functional layout work already done). **Now higher-priority than
  "maybe"** given the Pi hotspot work above -- this GUI was built for a
  laptop browser and has never been checked on a phone-sized screen;
  bigger touch targets and a reflowed layout are likely needed for it to
  be genuinely usable one-handed on the phone/tablet clients the hotspot
  work is specifically for.

## Feature roadmap (software) -- prioritized 2026-09-10

Not implemented yet, any of it -- this is a planning/prioritization pass
following a discussion of the actual use case (see "Use case" below and
the memory file), not a status update. Tiers are the user's own
prioritization, in order; items within a tier aren't further ordered.
Revisit/re-prioritize as work actually starts on each.

**Use case driving this list**: primarily scanning the interiors of
large spaces -- cathedrals, conference centers -- to build a full 3D
model of the space for the entertainment industry (renders/designs,
e.g. virtual production, event design), not a surveying/engineering-
metrology use case. That pushes toward multi-station jobs of big open
interiors (registration/organization across many stations matters more
than for one small room) with render/design pipelines as the actual
downstream consumer of the raw `.pcd` output.

**Next steps:**
- ~~E57 export, instead of (or alongside) PCD, with as much metadata as
  possible (station name, timestamp, instrument/sensor info, pose).~~
  **Built 2026-09-10**, alongside PCD (not instead of it -- a "Export as
  E57" button next to each locally-saved .pcd, .e57 lands as a sibling
  file). New `scan_aggregator/e57_writer.py`: a hand-rolled ASTM E57
  writer (paged/CRC-32C-protected physical file, XML metadata section,
  Float-precision CompressedVector binary point data) plus a PCD reader
  (`read_pcd_xyzi`, accepts both `DATA ascii` and `DATA binary` -- real
  files of both kinds exist on disk from this project's history) to feed
  it. No e57/libE57Format dependency shipped -- same reasoning
  `pcd_writer.py` already gives for hand-rolling PCD instead: no aarch64
  wheels exist for that library, so it wouldn't install on the Pi. E57
  itself is a genuinely intricate binary format though (unlike PCD),
  which made "hand-roll blind from the spec" a real correctness risk
  with no way to validate the result on this project's own hardware --
  see `e57_writer.py`'s own module docstring for the actual approach
  taken: `pye57` (wraps the real libE57Format C++ library) was installed
  here on the dev machine ONLY as a throwaway validation oracle, never a
  runtime dependency of anything that ships, used to reverse-engineer
  the exact byte layout from real reference files rather than trusting
  memory of the spec text alone.

  **A real, non-obvious bug found and fixed via that oracle, not just
  "written and assumed correct":** an early version stored
  `xmlPhysicalOffset` (the file header field pointing at the XML
  section) as a plain content-byte count -- this happened to work for
  every small test file, because nothing before the XML had crossed a
  1020-byte page-CRC boundary yet, making the omitted conversion a
  no-op purely by coincidence. It silently produced unopenable files
  the moment real scan-sized data pushed the XML section past that
  first boundary -- caught by systematically bisecting exactly where a
  growing point count stopped opening in `pye57` (broke between 50 and
  63 points), not by code review, since the bug looked completely
  correct on inspection and even round-tripped successfully against
  the module's own (self-consistently wrong) reader logic. Root cause:
  `xmlPhysicalOffset`/`dataPhysicalOffset` are genuine physical (raw
  file byte) positions, requiring `physical = logical + 4*(logical //
  1020)` -- confirmed by literally searching a reference file's raw
  bytes for the `<?xml` prolog and comparing its true position against
  both the stored field and this formula. Fixed via a new
  `_logical_to_physical_offset` helper, applied everywhere a field is
  actually named "...Physical...".

  **Verified at real production scale, not just small synthetic
  cases**: a battery of edge-case point counts (0, 1, exactly at the
  packet-size threshold, both sides of it, up to 200,000) all open
  correctly and round-trip exact values through `pye57`; a real 20,000-
  point slice of an actual ASCII scan file from this project's own
  `test scans` folder converts and round-trips correctly; and a full
  real 14,907,471-point binary scan (`scan_20260828_184054.pcd`, 238MB)
  converts in ~7s, opens cleanly, and matches the source exactly on
  point count, global cartesian bounds, and a random 2,000-point sample
  (checked at that scale rather than every point, purely to keep the
  verification script itself fast -- not a gap in what the writer
  itself does).

  Wired into `scan_aggregator/node.py`: `~/export_e57_request`/
  `~/export_e57_response` (background thread, same request/response-
  over-topic pattern as `~/export_to_usb_request` -- no progress topic,
  since a single conversion is fast enough at real scan scale not to
  need one). `_on_list_local_scans_request` now also lists `.e57`
  files alongside `.pcd` ones, so a converted file shows up next to its
  source and rides the exact same generic, extension-agnostic Download/
  Export-to-USB/Delete paths every other listed file already uses --
  no new code needed for any of those three. GUI: `index.html`'s Scans
  tab gains an "Export as E57" button per `.pcd` row only (an `.e57`
  row has nothing further to convert). Metadata included: station name
  (from the source filename), a description, and acquisition
  start/end (the source `.pcd`'s own mtime, since `scan_aggregator`
  doesn't currently track a real capture-start timestamp separately --
  see "Session/project grouping" below, not yet built, which is where a
  real one would come from). Pose is written as identity, matching
  `write_pcd`'s own convention that points already arrive in
  `scan_aggregator`'s output frame.

  **Not yet tested against a live ROS/rosbridge connection** -- the
  writer/reader core is thoroughly verified per above, but
  `_on_export_e57_request`'s actual service wiring and the GUI button's
  round trip through a real `scan_aggregator` node were only checked by
  syntax-checking the Python and by rendering the button in a browser
  against synthetic (non-live) row data, not exercised against a
  running node. Added to the testing checklist at the end of this file.

  **Extended 2026-09-10, same day, per explicit request: per-point
  ring/time capture (for future sweep deskewing) and real per-scan
  metadata.**

  *Ring/time*: `node.py`'s `_try_transform_and_accumulate` (the one
  place both step-and-stare and sweep mode funnel every point through)
  now also reads `ring`/`time` off the raw `/velodyne_points` message
  when present, alongside the existing `x`/`y`/`z`/`intensity` --
  `EXTRA_POINT_FIELDS`/`POINT_FIELD_NAMES`, new module constants. Ring
  identifies which of the 16 laser channels produced a point (also
  useful for the still-open per-laser-calibration item, independent of
  deskewing); time is a per-point capture-time offset -- the actual
  ingredient a future deskewing pass needs, since today's transform is
  one tf2 lookup per whole ~0.1s cloud message, not one per point (see
  the new "Distant features" roadmap entry below). **Not yet confirmed
  against real hardware that "ring"/"time" are this project's actual
  driver/version's real field names** -- standard for the
  `ros-drivers/velodyne` ROS2 driver family in general, but never
  checked against this specific installed version. Deliberately
  defensive about that uncertainty rather than assuming it: new
  `_read_points_with_extra_fields` checks `cloud_msg.fields` for both
  names before requesting them, and falls back to NaN-filled ring/time
  columns (not a crash, not a dropped point) if either is missing --
  every array in a run stays the same width regardless, since
  `_write_output_in_background`'s `np.concatenate` needs that. A single
  combined `pc2.read_points(transformed, field_names=POINT_FIELD_NAMES)`
  call reads all six fields together in one `skip_nans` pass, rather
  than a second, separate read for ring/time -- two independently-
  filtered reads risked silently misaligning rows if `skip_nans`
  happened to drop a different point count from each.

  Threaded all the way to both output formats, not just captured and
  dropped: `pcd_writer.write_pcd` widened from a fixed x/y/z/intensity
  signature to a generic `field_names` parameter (still one bulk
  `tofile()` write, still one uniform float32 column type -- ring's
  0-15 values and time's are both exactly representable, so a
  mixed-type PCD was never actually needed); `e57_writer.write_e57`
  likewise widened to accept any `field_names`/column count past the
  required x/y/z/intensity base, encoding ring/time as two more
  `Float precision="single"` CompressedVector fields (not E57's own
  standard names, but neither is anything project-specific ever was --
  a reader that doesn't recognize a field name just doesn't use it,
  confirmed against `pye57`/libE57Format). The live preview publisher
  (`_maybe_publish_preview`/`_make_preview_cloud`) deliberately still
  only ever sends x/y/z/intensity -- slices `merged[:, :4]` before
  building the preview cloud, since a live viewer has no use for
  ring/time and that code's wire format (`point_step=16`) was never
  widened to match.

  **Verified thoroughly offline, the same way the base E57 writer was**:
  a full write_pcd(6 fields) -> read_pcd_points -> write_e57 round trip
  across the same size sweep used before (0, 1, boundary sizes, up to
  200,000 points) -- confirmed via `pye57` that the file opens and via
  direct manual decode of the CompressedVector binary section (`pye57`'s
  own `read_scan_raw` convenience wrapper has a hardcoded whitelist of
  field names it'll surface and silently ignores anything else,
  `ring`/`time` included -- not a real limitation, just needed a lower-
  level check to actually see those two columns' values) that all six
  columns, including ring/time, round-trip to the source data exactly.
  Backward compatibility re-confirmed too: a real, already-on-disk
  4-field `.pcd` (no ring/time) still reads and converts correctly
  through the same, now-generalized `read_pcd_points`/`write_e57` API.

  *Real per-scan metadata*: `_on_export_e57_request` now fetches
  vlp16_config's current mount-calibration parameters
  (`mount_roll_deg`/`pitch`/`yaw`, `mount_x`/`y`/`z`) via the same async
  `GetParameters` client/callback pattern `_finish_mount_calibration`
  already uses for the same service (a blocking call from the
  background thread that does the actual conversion isn't an option --
  a service response only ever arrives via a callback on the executor
  thread), then assembles scan_aggregator's own current scan-config
  parameters (range/step/RPM/sweep settings, invert flags) plus vlp16's
  raw `~/status` passthrough (a new subscription/cache,
  `_on_vlp16_status`/`self._vlp16_status_json` -- passed through as raw
  JSON text rather than parsed into named fields, since that topic's own
  schema is explicitly unconfirmed against the real sensor per
  `vlp16_config`'s own module docstring, and guessing field names inside
  an already-uncertain blob felt like compounding one unverified
  assumption with another). All three land in the E57 as new
  `extra_string_fields` -- `write_e57` gained that parameter, rendering
  arbitrary `{element_name: text}` pairs as sibling String elements next
  to `<description>` (`tplScanConfigJson`/`tplMountCalibrationJson`/
  `tplVlp16StatusJson`), so a future tool could parse them directly
  rather than needing to scrape free text.

  **A real, honestly-stated limitation, not glossed over**: all of this
  reflects whatever the rig is configured as *at export time*, not
  necessarily what was actually active during the original capture --
  export is an on-demand button that can fire well after the scan
  finished, and there's no per-run manifest yet recording historical
  values separately (that's "Session/project grouping" just below, not
  built yet). Stated both here and inside the exported file's own
  `description` field, so it travels with the data. If the
  vlp16_config service doesn't respond in time, the export still
  succeeds (mount calibration is genuinely optional metadata here,
  unlike mount-calibration's own stricter use of the identical service
  call) -- `description` says so explicitly when that happens.

  Verified: the full `extra_string_fields` mechanism (plus the
  ring/time fields together in the same file) via a direct write_e57
  call with representative JSON payloads, confirmed via `pye57` that the
  file still opens and via manual XML extraction (see this file's own
  earlier note on the physical-vs-logical offset bug -- an ad hoc
  verification script reading with the wrong technique falsely looked
  broken here too; re-confirmed against the correct extraction method
  and a real ElementTree well-formedness check) that all three custom
  fields and their real JSON content are present and intact. **Not yet
  tested against a live node** -- same gap as the base ROS/GUI wiring
  above, `_on_export_e57_mount_params_received`'s actual service round
  trip and `_on_vlp16_status`'s real message schema are both new
  surface area added to the same "needs real hardware" list.
- ~~Session/project grouping: automatic file naming per venue + station
  (not today's flat, independently-named-per-run files), plus the
  ability to download an entire job's folder (all stations) at once
  from the GUI rather than one file at a time.~~ **Built 2026-09-11**,
  then **redesigned the same day per explicit request**, before ever
  being committed -- the first pass used a free-typed venue name plus a
  manually-tracked, auto-incrementing station number
  (`<venue>_station<N>_<timestamp>`); the shipped design instead uses a
  **project dropdown + "New Project…" button**, and drops the station
  number entirely, naming files `<project>_<timestamp>` -- each capture
  is told apart by its own timestamp alone, nothing to increment or
  redo. (The old design was real, working code, verified end-to-end on
  real hardware, before being replaced -- see git history if the
  station-number approach is ever wanted back; nothing about the
  redesign was a correctness problem with the first version, purely a
  UX preference.)

  New `scan_aggregator` parameter `project_name` (string, default
  `""`). `_build_output_basename` (node.py) makes `<sanitized
  project>_<timestamp>.<ext>` when set, or the old plain
  `scan_<timestamp>` when blank -- additive, not a forced workflow
  change, same as the first design. `index.html`'s Scan tab gets a
  "Project" fieldset: a `<select id="projectSelect">` plus a "New
  Project…" button (a plain `prompt()`, matching this project's
  existing use of native `confirm()` dialogs elsewhere rather than a
  custom modal for something this simple). There is no separate
  "known projects" registry anywhere -- the dropdown's options are
  derived entirely by parsing real filenames back out of the existing
  `~/list_local_scans_response` (`parseProjectName`, matching
  `_build_output_basename`'s own naming convention exactly), refreshed
  on connect and after every completed run (the same "done:" status
  trigger the post-scan rename popup already uses) -- so a project
  created by any connected client (phone, kiosk, laptop) shows up for
  every other one too, and there's nothing to go stale or fall out of
  sync with what's actually on disk. "New Project…" adds an option
  locally (client-side `sanitizeProjectName`, mirroring node.py's
  `_sanitize_filename_part` exactly, so what's shown/selected
  immediately already matches what the backend will actually name
  files with) and selects it -- it takes effect on the next Start
  Scan/Start Sweep Scan, same push-at-start-time model every other scan
  config field already uses (`sessionParams()`, shared by
  `setScanParams`/`setSweepScanParams`).

  "Download an entire job's folder at once" -- unchanged in approach
  from the first design, just re-pointed at the new naming convention:
  rather than real nested filesystem directories (would have meant
  touching every existing OUTPUT_DIR-flat-assuming handler's path logic
  -- list/export/delete/rename/export_e57, five-plus call sites),
  grouping is a pure naming-convention + client-side parse, zero
  changes needed to any of those. A `~/bundle_project_request`/
  `response` (`_on_bundle_project_request`, background thread, same
  pattern as `export_to_usb`/`export_e57`) zips every `.pcd`/`.e57`
  file belonging to one project into `<project>_bundle.zip` inside
  OUTPUT_DIR (`ZIP_STORED`, no compression -- binary point data isn't
  meaningfully compressible, same reasoning `pcd_writer.py` gives for
  binary over ASCII), matched via a **full regex match**
  (`^<project>_\d{8}_\d{6}\.(pcd|e57)$`), not a `.startswith()` prefix
  check -- the latter would have let a project named "Test" incorrectly
  pull in a different project named "Test2"'s files too, confirmed as
  a real distinguishing test case, not just a theoretical concern.
  Always rebuilt fresh on request rather than cached, since new scans
  can be added to the same project between one download and the next.
  The GUI Scans tab groups listed files by project (most-recently-
  active first, anything not matching the convention listed separately
  below, unchanged from before), each group getting a "Download All
  (zip)" button. `_on_list_local_scans_request`'s file filter still
  recognizes `.zip` so a bundle shows up in the list like anything
  else.

  **Verified thoroughly both offline and against real hardware**:
  `_sanitize_filename_part`/`_build_output_basename` and the GUI's
  matching `sanitizeProjectName`/`parseProjectName` tested directly
  (round-trip correctness, and specifically the "Test" vs "Test2"
  full-match-not-prefix distinction); GUI rendering verified in-browser
  (Project fieldset, "New Project…" with `window.prompt` stubbed for
  the test, and injected synthetic scan-list data producing the
  correct grouped layout). **Then deployed to the real Pi and
  confirmed live**: two real short scans under `project_name="Test
  Project"` produced `Test_Project_<timestamp1>.pcd` and
  `Test_Project_<timestamp2>.pcd` exactly as designed (no station
  number, no collision); a real `~/bundle_project_request` against
  those two actual files produced a valid zip, pulled back and
  confirmed with Python's own `zipfile` module to contain both real
  files, byte-size-exact, `testzip()`-clean. Test artifacts deleted
  from the Pi afterward, `project_name`/scan-range fields restored to
  their real prior values. **Still not clicked from an actual browser
  against the real stack** -- both the naming and the bundle download
  were exercised by publishing directly to the relevant topics/
  services, not through `index.html`'s own Project fieldset or
  "Download All" button in a live browser session; worth doing once for
  real UI-level confidence, though the service-level behavior working
  end-to-end is what was actually in question.
- **Every scan now saves natively as .e57 -- .pcd is no longer a
  user-facing format at all, 2026-09-11, per explicit request.**
  `_finish_run`/`_write_output_in_background` (node.py) call
  `write_e57` directly now, not `write_pcd` -- no PCD file is ever
  created for a new scan, not even as a discarded intermediate; there
  was never a "write PCD then auto-convert" step, since `write_e57`
  already accepts the same points array `write_pcd` did. Real
  per-scan metadata (mount calibration, scan config, sensor status --
  previously only gathered by the on-demand "Export as E57" button,
  see the E57-export entry above) is now gathered for *every* run
  automatically, using the run's own **real captured start/end
  timestamps** (new `self._run_start_time`, set in
  `_start_scan_impl`/`_start_sweep_scan_impl`) rather than a saved
  file's mtime -- genuinely more accurate provenance than the
  on-demand path ever had, not just carried over unchanged.
  `_on_list_local_scans_request` no longer lists `.pcd` at all, so
  there is no download/export/USB-export option for one anywhere in
  the GUI. The "Export as E57" button is gone from `index.html`
  entirely (nothing left to convert from the GUI's own point of view).

  **`_on_export_e57_request` (the legacy .pcd -> .e57 conversion
  service) was deliberately kept, not deleted**, as a migration tool
  for `.pcd` files that already existed on disk before this change
  (several real ones do, e.g. `2.pcd`/`3.pcd`/`test1.pcd` on the real
  Pi) -- reachable directly over ROS, same "escape hatch, no dedicated
  GUI button" precedent this project already has elsewhere (e.g.
  `tilt_axis_bridge`'s `~/driver_command`). Refactored rather than
  duplicated: factored a shared `_fetch_mount_params_then`
  (the async GetParameters-then-continue pattern) and
  `_assemble_e57_metadata` (the actual metadata dict, parameterized by
  station name/description/acquisition times/mount params) out of what
  used to be `_on_export_e57_request`-only code, so the automatic
  save path and the legacy conversion path share the real logic
  instead of two copies drifting apart. `pcd_writer.py` had its now-
  fully-unused `write_pcd`/`DEFAULT_FIELD_NAMES` deleted outright
  (confirmed zero remaining callers anywhere in this workspace first)
  -- only `fsync_durable` (a general durability helper, never
  PCD-specific despite living in that file) remains there.

  **A real bug found and fixed while making this change, not just
  reasoned about**: `_on_rename_output_request` used to unconditionally
  force a `.pcd` suffix onto whatever name a user typed in the post-
  scan rename popup, regardless of the file's actual format -- harmless
  while `.pcd` was the only format ever produced, but would have
  silently mis-renamed every completed run's real `.e57` file into a
  wrongly-suffixed `"<name>.pcd"` once this change landed, if left
  as-is. Fixed to derive the extension from the actual file being
  renamed (`os.path.splitext(self._last_output_path)`) instead of a
  hardcoded literal.

  **Verified end-to-end against real hardware**, not just offline:
  deployed to the real Pi, ran a real scan with no project set --
  produced `scan_<timestamp>.e57` directly (never a `.pcd` at any
  point), pulled it back and confirmed with `pye57` it's a valid,
  well-formed file with `ring`/`time` present and real
  `tplScanConfigJson`/`tplMountCalibrationJson`/`tplVlp16StatusJson`
  metadata, correctly labeled "Native scan_aggregator output" in its
  description (distinguishing it from the legacy-conversion path's
  own wording). Tested the rename fix directly: renamed a real output
  to `"my renamed scan"` and confirmed the file landed as `"my renamed
  scan.e57"`, not `.pcd`. Ran a second real scan with
  `project_name="Final Test"` set, confirming the project-naming and
  automatic-E57 changes compose correctly
  (`Final_Test_<timestamp>.e57`, no station number), then a real
  `~/bundle_project_request` against it produced a correct, valid zip.
  All test artifacts deleted from the Pi afterward, and
  `project_name`/scan-range parameters restored to their real prior
  values.
- ~~Scan time estimate shown before starting a run, computed from the
  configured range/step/RPM/dwell settings.~~ **Built 2026-09-10, sweep
  half only** -- a step-and-stare estimate (`updateScanTimeEstimate`,
  `#scanTimeEstimate`) already existed before this roadmap item was even
  written (stop count x (settle + capture time), read live off the
  VLP-16 tab's RPM field); what was actually missing and got built now
  is the equivalent for Continuous sweep scan. New
  `updateSweepTimeEstimate()`/`#sweepTimeEstimate` in `index.html`,
  wired to the Min/Max/Speed/Duration fields (Accel deliberately not
  included -- same simplification the existing estimate already makes
  for per-stop moves, ignoring accel ramp). Two cases, since sweep's
  `Duration=0` means "unbounded, until Stop Scan is pressed" rather than
  a fixed run length: with `Duration>0` the estimate is just that value
  restated as a duration (not a formula -- `sweep_duration_s` counts
  exactly from when real, non-edge-margin data starts arriving, per the
  "sweep_duration_s now counts from when real data starts" entry above,
  so the configured number already *is* the capture-phase length, not
  something to derive); with `Duration<=0` there's no total to give, so
  it instead shows one-way transit time across the Min/Max range at the
  configured RPM (`speed_rpm * 6` deg/s) -- the same kind of number this
  doc's own turnaround-ghost investigation computed by hand elsewhere
  ("at 2 RPM (12 deg/s) over a 190 deg span, one-way transit takes
  ~15.8s"), now surfaced live in the GUI instead of a one-off
  calculation. Both cases match the existing estimate's "floor, not a
  real prediction" framing -- excludes homing and the initial transit
  from home into the Min/Max range in both cases. Verified in-browser
  (served over a local static file server, no live rosbridge needed --
  matches this project's existing pattern of GUI-only verification
  without hardware): both the bounded and unbounded text render
  correctly on input, the invalid-input guard (`Min >= Max`) falls back
  to `—` same as the pre-existing estimate does, and the readout sits
  correctly in the Continuous sweep scan fieldset, styled consistently
  with the existing one. **Not yet tested against a real sweep run** --
  no hardware available this pass; the math itself (RPM-to-deg/s, the
  `Duration` semantics) was checked against already-documented values in
  this file, not re-derived from a live scan.

  **Follow-up, same day: the step-and-stare estimate really was
  meaningfully inaccurate on real scans, per direct user report ("always
  took much longer than it said it would"), not just theoretically
  incomplete.** Traced one real, concrete, fixable cause plus two real
  ones that still can't be fixed without hardware:
  - **Fixed**: `tilt_axis_bridge`'s own `settle_time_s` (default 0.3s --
    the dwell `STATE_SETTLING` holds after *every* real move, not just
    sweep turnarounds, before it reports "settled") was silently missing
    from the calculation entirely. `scan_aggregator`'s `STATE_MOVING` only
    advances once it sees that "settled" status (`_tick`), so every stop
    pays this dwell on top of the GUI's own `settle_extra_s` ("Extra
    settle" field) -- only the latter was ever counted. Added a new
    `TILT_AXIS_BRIDGE_SETTLE_TIME_S` JS constant (0.3, matching
    `node.py`'s declared default) into the per-stop calculation.
    Hardcoded rather than read live like the VLP-16 RPM field is, because
    unlike that field, `settle_time_s` has no GUI input anywhere at all
    (not in `applyTiltAxisBridgeSettingsToForm`) to read a live value
    from -- only reachable via the raw driver-command escape hatch or a
    hand-edited settings file; if it's ever actually changed from its
    default, this constant needs updating to match by hand. Verified: a
    116-stop test case's estimate moved from 1m21s to 1m56s, exactly
    116 x 0.3s = 34.8s more, matching the math.
  - **Not fixed, deliberately not guessed at**: real per-stop move/travel
    time (accel ramp + actual physical travel, distinct from the settle
    dwell after it lands) and the final write-to-disk time once a run
    ends both remain genuinely unestimated -- both are plausible real
    contributors (homing especially, which can plausibly run tens of
    seconds depending on how far the axis has to search) but neither has
    ever been empirically timed against real hardware (see this doc's own
    "Open items"), so inventing a specific number for either risked
    replacing one kind of inaccuracy with another, less honest one.
    Instead, made the exclusion itself much harder to miss: the readout
    text now says outright "PLUS unestimated homing, per-stop travel, and
    final save time (real total will be longer)" directly in the number
    itself, not only in the hint paragraph underneath it -- the hint was
    already saying this before today, so the earlier inaccuracy report
    suggests a caveat nobody reads past the headline number isn't
    sufficient on its own. **Still open**: an actual empirical measurement
    of real homing/move/save time against hardware, to either fold a real
    number into the estimate or confirm how large the gap actually is --
    blocked on hardware access.
- ~~One Start Scan button with a mode dropdown, added 2026-09-10, per
  explicit spec. Today `index.html`'s Scan tab has two separate buttons
  (`startScanBtn` "Start Scan (step-and-stare)" and `startSweepScanBtn`
  "Start Sweep Scan"), each with its own always-visible field group
  (step-and-stare's Start/End/Step/etc., sweep's Min/Max/Speed/etc.) --
  collapse to one Start button plus a step-and-stare/sweep dropdown,
  showing only the relevant field group for whatever's selected.~~
  **Built 2026-09-11** in `index.html`'s Scan tab. New "Scan mode"
  fieldset with `scanModeSelect` (`Step-and-stare` / `Continuous sweep`,
  defaults to step-and-stare) sits above the two range/capture fieldsets.
  Those fieldsets now carry the existing `.tabpage`/`.tabpage.active`
  show/hide classes (the same pattern already used for the top-level
  Scan/Scans/Config tabs and the Config sub-tabs, not a new mechanism) --
  `applyScanMode()` toggles `active` on `#mode-group-stare` /
  `#mode-group-sweep` to match the dropdown, run once on load and again
  on every `change` event. The Run fieldset's two buttons collapsed into
  one `startScanBtn` "Start Scan"; its click handler reads
  `scanModeSelect.value` and calls `startStareScan()` or
  `startSweepScan()` (the old two click-handler bodies, unchanged, just
  renamed into plain async functions so a single listener can pick
  between them). No other code referenced `startSweepScanBtn` by ID
  (checked), so nothing else needed updating. Verified: syntax-checked
  every `<script>` block by parsing it with Node's `Function`
  constructor, then loaded `index.html` in a real browser (local
  `http.server`, no rosbridge needed for this) and confirmed dropdown
  changes actually toggle which fieldset is visible and that only one
  "Start Scan" button exists. Not yet deployed/tested against real
  hardware -- the parameter-setting and service-call logic itself is
  byte-for-byte the same as the two already-verified handlers it
  replaces, just reached through a branch instead of two buttons, so
  live hardware testing is optional here rather than required, but still
  worth doing next time the Pi's online. Once the onboard-screen tabs
  below exist, `status.html`'s own Scan tab Start/Stop pair should get
  the same treatment for consistency -- not done here, that page has its
  own separate `start_scan_from_panel`/`start_sweep_scan_from_panel`
  buttons untouched by this change.
- ~~Onboard-screen tabs, added 2026-09-10, per explicit spec. `status.html`
  (the 480x320 kiosk panel, see "Onboard kiosk status display" above) is
  currently one flat page -- network/motor/VLP-16/scan status plus a
  controls row all in one view. Split into three tabs (Status/Control/
  Scan), Control gains Home + Release Stall, Scan gets Start/Stop, and
  the panel auto-switches to Scan the moment a run starts.~~ **Built
  2026-09-11.** `status.html` now has a small pill-style `.tabnav`
  (Status/Control/Scan) above a `.tabbody` of `.tabpage` divs, same
  show/hide convention as `index.html`'s own tabs (a separate copy, this
  file stays single-file/no shared imports per its own long-standing
  note) but its own compact CSS sized for a touch panel:
  - **Status tab**: the original four readouts (Network/Motor/VLP-16/
    Scan), unchanged, just wrapped in their own tabpage.
  - **Control tab**: `presetSelect`, `calibrateBtn`, `shutdownStackBtn`
    (all pre-existing) plus two new buttons -- **Home** (new `homeBtn`,
    calls `~/home` on `TILT_NODE`, same service `index.html`'s own
    `homeBtn` already used) and **Release Stall** (new `releaseStallBtn`,
    calls the raw `release_stall` MksDriver command). This page never had
    `index.html`'s generic `[data-cmd]`/`sendDriverCommand` machinery for
    the driver_command/driver_response request-over-topic pattern, so it
    got a small dedicated copy scoped to just this one button rather than
    porting the whole generic system for one call.
  - **Scan tab**: `scanModeSelect` + `startScanBtn` (moved here from the
    old flat controls row, unchanged logic) plus a new `stopScanBtn`
    (`~/stop_scan` on `SCAN_NODE` -- this panel never had a Stop control
    at all before this), and its own live scan-status readout
    (`valScanTab`, mirrors the Status tab's `valScan` off the same
    subscription) so the current state is visible without switching tabs
    away from Start/Stop mid-run.
  - **Auto-switch to the Scan tab** implemented via `lastScanStatusText`
    + `isTerminalScanStatus()`: when the previous status was idle/done/
    aborted/`mount_calibration_done` and the new one isn't, `switchTab
    ('scan')` fires -- driven off `scan_aggregator`'s own status topic
    (same one this page already subscribed to), so it fires whether the
    run was started from this panel, `index.html`, or a phone, matching
    the spec. Verified it does NOT fight a manual tab switch mid-scan
    (checked by forcing a `capturing` update while parked on Control --
    stayed on Control, only the idle/done/aborted -> active *transition*
    switches tabs, not every in-progress update).

  **Live coverage feedback itself** (the separate "Coming soon" item
  below) was not yet built when this entry was first written -- it now
  is, see that item's own entry below for the full writeup, including a
  compact version added to this page's own Scan tab. Initially verified
  via a local
  static serve at the real 480x320 size plus mocked `bridge`/service
  calls confirming: tab clicks switch correctly, the auto-switch fires on
  an idle->active transition and not on an in-progress->in-progress
  update, Start/Stop/Home dispatch to the right services depending on the
  mode dropdown, and Release Stall publishes the exact `driver_command`
  shape `tilt_axis_bridge` expects (`{id, command: "release_stall",
  params: {}}`).

  **That first pass's "all three tabs render/fit correctly" claim was
  wrong -- found from real photos of the physical panel, fixed, and
  re-verified for real.** The Control tab's five controls and the Scan
  tab's three overflowed past the right edge with text visibly cut off
  (user sent real photos of the physical unit showing this directly).
  Root cause: `.controls select`/`.controls button` had `min-width: 0`,
  which defeats `flex-wrap: wrap` -- instead of wrapping to a second line
  once the row can't fit everything, `min-width:0` lets flex items shrink
  indefinitely, and since the text itself is `white-space: nowrap`, the
  *text* overflows the shrunk button rather than the row ever wrapping.
  The earlier local-serve pass genuinely rendered this same way (missed
  on review, a real oversight, not a difference in environment) --
  `grim` screenshots of the physical panel taken right after first
  deploying the tabs also show it, just not scrutinized closely enough
  at the time. Fixed by dropping `min-width:0` (back to the default
  `auto`, which floors shrinking at each item's own content size) and
  switching `flex-basis` from `0` to `auto` so mismatched label lengths
  (`HOME` vs `RELEASE STALL`) each get space proportional to their own
  content first, not forced equal-width. Re-verified for real against
  the physical panel, not just re-read: deployed (scp, no rebuild --
  static HTML), relaunched the kiosk's Chromium with
  `--remote-debugging-port` this one time so `Runtime.evaluate` could
  call the page's own `switchTab()` directly over CDP (this session's
  Browser pane still can't reach the Pi's rosbridge or a remote debug
  port itself, so this ran from a plain Python socket script over SSH,
  same technique as the raw-websocket rosbridge checks elsewhere in this
  doc) -- `grim` screenshots of Control and Scan afterward, cropped and
  zoomed at the pixel level, show `SHUTDOWN` and `STOP` (the two that
  were previously cut off) each with a clean, complete right border and
  fully legible text. Relaunched once more afterward without the debug
  flag, back to the normal kiosk launch command, left on the Status tab.
  Release Stall still not exercised against a real stall condition --
  hard to manufacture on demand, lowest priority, left open. `~/home`
  and `~/stop_scan` are both covered by the dedicated homing-bug entry
  below and the earlier real `stop_scan`/auto-switch verification above.

**Coming soon:**
- ~~Live coverage feedback during a scan -- even something simple like a
  2D angular map of which step/sweep regions have been captured so far,
  so a partial or aborted scan is obvious while still on-site, not
  discovered back at the studio.~~ **Built and confirmed live 2026-09-11,
  both modes, both real gap detection and a full real scan.** The Pi
  went unreachable partway through the original build (`192.168.0.115`:
  destination host unreachable, `tpl-lidar` mDNS also failed to resolve
  -- the device itself, not a networking issue on this side) --
  deliberately left uncommitted until it came back online and could
  actually be verified, rather than let "committed" stop meaning
  "hardware-verified" for this one. Once the Pi was back: deployed via
  scp, rebuilt `scan_aggregator`, restarted the stack (clean, all 7
  nodes up).

  **Step-and-stare, full real run**: `tilt_start_deg=10, tilt_end_deg=30,
  step_deg=2` (11 stops). Held a live rosbridge subscription to both
  `~/coverage` and `~/status` open through the whole run (same technique
  as the homing-fix verification earlier in this doc) and watched the
  coverage array fill in left-to-right, one non-zero count per completed
  stop, in order, finishing as `[4, 2, 4, 2, 4, 2, 2, 4, 3, 2, 4]` --
  all 11 bins non-zero, matching `done: 11 stops -> ...e57`.

  **Sweep, deliberately stopped early**: `sweep_min_deg=10,
  sweep_max_deg=40, sweep_edge_margin_deg=2` (16 bins at 2° resolution).
  Started the sweep, let it run a few seconds, then called `~/stop_scan`
  on purpose partway through. Final coverage:
  `[0, 3, 1, 4, 5, 2, 3, 5, 2, 4, 4, 0, 0, 0, 0, 0]` -- the reached
  portion (roughly 12°-30°) populated, the two edge-margin bins and the
  never-reached tail (32°-40°, the part of the range the sweep hadn't
  gotten to yet when it was stopped) all correctly zero. This is exactly
  the feature doing its actual job: a partial run's own gap is right
  there in the data, not just "it renders something."

  **A real stall happened during this testing, unrelated to the feature
  itself as far as could be told.** Partway through the sweep test above,
  the user reported the motor had stalled -- confirmed independently via
  raw `driver_command` reads (`read_motor_status`=1/Stopped +
  `read_enable_status`=False, the exact stopped-and-disabled stall
  signature, not stopped-and-arrived) even though the high-level
  `~/status` topic still read a stale `idle` (the FSM's stall-detection
  branch only runs from `STATE_MOVING`/`SWEEPING`/`JOGGING`, so a stall
  that develops outside one of those doesn't get caught and surfaced the
  same way). All motion/testing paused immediately, nothing further sent
  until this was understood. First `release_stall` attempt came back
  with a transport-level error (`function 0x3D: expected 5 bytes, got
  0`, matching an already-documented transient serial flakiness pattern
  in this project's own history), not a real rejection -- retried, and
  that attempt both transported cleanly and actually cleared it
  (`read_enable_status` back to `True`, motor genuinely healthy again,
  independently confirmed before trusting the user's own "released"
  report, not after). Root cause of the stall itself wasn't
  investigated/isolated -- the coverage feature's own test sequence sits
  well inside this rig's already-exercised sweep-mode envelope (narrower
  range, same edge-margin mechanism already tested clean earlier this
  session), so there's no strong reason to suspect this specific change
  caused it, but that's circumstantial, not a real root cause, and
  should stay in mind if stalls become a repeat pattern.

  Went with a 1D angular strip (one tile per tilt slice) rather than a
  full 2D tilt x azimuth grid -- the roadmap item's own wording ("even
  something simple like...") left room for this, and there's a real
  architectural reason it fits this project better than true 2D binning
  would: each incoming `/velodyne_points` message is already
  approximately one full revolution's worth of points at whatever tilt
  the axis happens to be at when that message arrives (the VLP-16 spins
  continuously and publishes per-revolution, not per-point), so counting
  *messages* landing in each tilt bin -- via `self._current_tilt_rad` at
  callback time, already tracked for the sweep edge-margin logic -- gives
  a real, useful coverage signal without needing per-point azimuth
  extraction (which would mean computing azimuth from the raw
  sensor-frame cloud before the tilt transform, a real added-complexity
  path since points reaching `scan_aggregator` for accumulation are
  merged rather than kept per-revolution). For step-and-stare this maps
  exactly 1:1 onto stops (`bin_deg = step_deg`, so `_init_coverage`'s bin
  count matches `len(self._targets_rad)` exactly, verified with a plain
  Python arithmetic check outside of ROS -- see below); for sweep, a
  fixed `SWEEP_COVERAGE_BIN_DEG = 2.0` resolution.

  **Backend** (`scan_aggregator/node.py`): new `self._coverage_counts`
  (a numpy int array, one count per tilt bin), `_init_coverage` (called
  from `_start_scan_impl`/`_start_sweep_scan_impl`, sized/reset for the
  scan that's actually starting -- deliberately never called from
  `_on_start_mount_calibration`, so a calibration run leaves whatever
  the previous real scan's map was alone rather than clearing or
  polluting it), `_record_coverage` (called from `_on_pointcloud`'s
  `STATE_CAPTURING` branch and the real-sweep, non-`MODE_CALIBRATE` half
  of its `STATE_SWEEP_SCANNING` branch -- deliberately not the
  calibration half), and `_maybe_publish_coverage` (new `~/coverage`
  topic, JSON `{start_deg, bin_deg, counts}`, throttled by the new
  `coverage_publish_period_s` parameter (default 0.5s), called from
  `_tick` right alongside the existing `_maybe_publish_preview` --
  same "keeps publishing through STATE_DONE/STATE_ABORTED so the final
  state stays reviewable" reasoning). Verified the bin arithmetic with a
  standalone Python script (no ROS involved) mirroring the exact
  `_init_coverage`/`_record_coverage` formulas: a simulated
  278-target step-and-stare scan produced exactly 278 coverage bins,
  hitting every stop twice landed exactly `[2, 2, ..., 2]`; a simulated
  200°-range sweep with a deliberate 100-110° gap left produced exactly
  that gap as zero-count bins in the output, everything else non-zero.

  **Frontend**: both `index.html` (new "Live coverage" fieldset on the
  Scan tab, a `<canvas>` bar plus a text summary like "116/130 tiles
  captured (89%) -- 5.0°..185.0°") and `status.html` (a compact version
  in its own Scan tab, canvas only, no separate summary text -- screen
  space). Both subscribe to `~/coverage` and just draw whatever payload
  they're handed (`renderCoverage`, two independent copies matching this
  project's usual "separate self-contained files" pattern, not a shared
  import) -- brightness per tile scales with revolution count, capped at
  3 (a coverage map, not a density heatmap, so it saturates to "lit" fast
  rather than needing dozens of revolutions to look complete), zero-count
  tiles stay dark. Verified via the same local-static-serve +
  `javascript_tool`-injected-synthetic-data technique used earlier this
  session for other GUI work (no live rosbridge needed for this) --
  confirmed on both pages at their real rendering context (including
  `status.html` at the real 480x320 kiosk size) that a deliberately
  injected gap in the counts array renders as a visibly darker strip,
  the summary text computes correctly, and an empty `counts` array (no
  scan run yet) doesn't error.

  **Redesigned from a bar to a circular compass, 2026-09-11, per
  explicit request** -- same underlying `~/coverage` data, a different
  `renderCoverage` on both pages. Each bin is now a spoke drawn at its
  own real tilt angle around a full 360° circle (0° at the top,
  clockwise) instead of stretched along a fixed-width strip, only drawn
  where the configured/physical range actually reaches -- so the shape
  directly matches where the axis physically points in the room, and
  the untouched arc (e.g. the ~100° a 0-260°-range scan never reaches)
  reads as a genuine visual gap rather than being implied by a
  percentage. Spoke length is revolution count (same 0-3-capped
  intensity scaling as the bar version); zero-count bins draw as a
  short dim stub at the inner radius rather than nothing, so an
  in-range-but-uncaptured position still has a visible mark to
  distinguish it from "outside the configured range entirely" (no
  spoke drawn there at all). `index.html` got a fixed 140px icon
  beside its summary text (replacing the full-width bar + text-below
  layout); `status.html` got a 64px version beside a two-line summary
  in its own Scan tab (previously canvas-only, no summary text --
  switching the coverage row from `flex:1` to a fixed height freed up
  the horizontal room to add one). Verified with the same synthetic-
  data technique as the bar version first (a deliberately injected gap
  renders as a visible short/dim wedge at the right angular position on
  both pages), then confirmed for real against the physical kiosk: ran
  a full 16-stop scan on real hardware and grabbed a `grim` screenshot
  showing the gauge fully lit (16/16, 100%, a complete green fan) --
  real backend data driving the real redesigned frontend on the actual
  device, not just synthetic data in a local browser.

  **Also found while verifying this**: `scan_aggregator` had been
  killed by the kernel's OOM-killer (`dmesg` confirmed it directly --
  ~2.5GB RSS at the time, `exit code -9`) between the previous
  real-hardware test and this one, with `tpl-scanner.service` staying
  "active" the whole time since systemd only tracks the launch parent,
  not each spawned node -- `ros2 node list` was the thing that actually
  caught it (a plain `ros2 param set` against it failed with "Node not
  found," which could just as easily have been read as more of the
  already-documented CLI flakiness if it hadn't been double-checked).
  Very plausibly this session's own doing -- a lot of real scans run
  back-to-back over several hours without a single restart in between,
  each one's `_merged_points`/coverage arrays adding up -- rather than
  a concern for normal single-session field use, but worth keeping in
  mind if a future long-running deployment ever sees the same thing:
  `tpl-scanner.service` staying "active" is not proof every node inside
  it is still alive, `ros2 node list` is the real check. Recovered with
  a plain service restart. Separately, the very first kiosk reconnect
  after that restart didn't recover on its own even though rosbridge
  came back up fine and a *fresh* page load connected immediately --
  not investigated further (relaunching Chromium, which this session
  already does after every GUI deploy anyway, worked around it) but
  flagged here rather than silently worked around, in case the
  reconnect logic has a real edge case worth a closer look someday.

  **Real physical bug found and fixed, 2026-09-11, from direct real-world
  experience with the rig: coverage was silently discarding roughly half
  of what actually gets captured.** The VLP-16 spins its own full 360°
  on every single revolution regardless of where the external tilt axis
  has it pointed -- it has no "front" or "back," it always sees all the
  way around itself. So a cloud captured while the tilt axis sits at
  position X genuinely contains real points at X's mirror too (180°
  around the compass), not just at X -- and a sweep whose real travel
  span is 180° or more (exactly the reported case: 5-195° with a 5°
  edge margin on each side, an effective 180° span) combines with that
  mirroring to cover the *entire* 360° compass, not just its own
  configured range. The coverage map's `_coverage_counts` array was
  sized/indexed to only ever span the configured range, so it was
  structurally incapable of recording that other half at all -- not a
  rendering bug, a real gap in what was ever being tracked.

  Fixed in `_init_coverage`/`_record_coverage` (node.py): the array now
  always spans the full 0-360° compass (`n_bins = round(360/bin_deg)`,
  `start_deg` always `0.0`), and every recorded capture increments both
  its own bin and its mirror bin (`(deg + 180) % 360`). No frontend
  changes needed at all -- `renderCoverage` on both pages already just
  draws whatever's non-zero at each angular position, so fixing the
  backend's own tracking scope was the entire fix. Explicitly assumes
  the VLP-16's own azimuth FOV is left at its default full 360°
  (`view_width` on the VLP-16 tab) -- a deliberately narrowed crop there
  isn't modeled, since checking it would mean reading `vlp16_config`'s
  own live parameters on every single capture for a much rarer
  configuration.

  Verified three ways: (1) a standalone Python script mirroring the
  exact formulas confirmed the user's own reported scenario (5-195°,
  5° edge margins) produces genuine 100% coverage of the full circle
  with zero remaining gaps; (2) the same script confirmed a genuinely
  short/aborted sweep (well under 180° of real travel) still correctly
  shows real gaps rather than being artificially inflated to full by
  the mirroring -- the fix doesn't paper over real incomplete coverage;
  (3) real hardware, the user's exact scenario: watched a live sweep's
  `~/coverage` topic and confirmed real captured bins appearing in the
  195°-360° region -- territory the old code's array didn't even
  include -- genuinely populated with real data, not synthetic. Coverage
  plateaued around 53% during the ~50s window observed rather than
  reaching 100% -- not a mirroring problem (confirmed by checking which
  specific bins were covered: real data was genuinely spread on both
  sides of the compass, not clustered in just the original range), just
  the 2°-bin resolution not yet having been touched by every possible
  message-timing alignment in the time observed, an expected sampling-
  granularity effect of polling `_current_tilt_rad` at each capture
  rather than continuously integrating position. Stopped the sweep
  early out of caution when the plateau initially looked like it might
  be the axis stuck oscillating rather than making progress (uncomfortably
  close to the shape of the earlier stall incident this same session) --
  confirmed via `~/tilt_axis_bridge/status` reading `idle` immediately
  after, motor genuinely healthy throughout, not an actual second
  incident. Test scan files (2, one from an earlier run this same
  session) deleted afterward.
- ~~**Preview Sweep button**, spec'd 2026-09-10, per explicit request, on
  both `index.html` and the onboard screen's Control tab.~~ **Built and
  verified on real hardware, 2026-09-11.** Quickly moves the tilt axis
  to whatever sweep range is currently configured for the selected scan
  (Min/Max) and runs a couple of back-and-forth passes there -- motion
  only, no point-cloud capture -- so the operator can physically watch/
  see where the scanner is about to sweep before committing to a real
  run. Distinct from two things that already exist and could be confused
  for it: `status.html`'s own Start button (`startScanBtn`,
  `scanModeSelect` set to "Sweep") starts a *real* sweep scan
  (`~/start_sweep_scan_from_panel`), and `index.html`'s Motor tab already
  has a raw motion-only "Sweep" jog tool (steps Min->Max with manual
  fields, no capture) -- but that one takes its own independently-typed
  range, not "whatever the current scan is configured to do," and just
  runs until a separate Stop click rather than a bounded couple of
  passes. This is closer to that raw jog tool's mechanism (direct
  `tilt_axis_bridge` motion, no `scan_aggregator`/capture involved) but
  reads its range from the actual scan config and stops itself.

  **Implementation.** `index.html`: new "Preview Sweep"/"Stop Preview"
  buttons in the Scan tab's Continuous sweep scan fieldset (`runPreviewSweep`).
  Reads Min/Max/Speed/Accel straight off that fieldset's own fields (the
  Scan tab's `sweepScanMinDeg`/etc, not the Motor tab's independent
  `sweepMin`/etc), pushes them via the exact same `setSweepParams` +
  `~/sweep_enable` mechanism the Motor tab's own raw jog tool already
  uses, then stops itself automatically. `status.html`: same mechanism
  on the Control tab, except this page has never had local Min/Max/
  Speed/Accel fields of its own (every other control here just acts on
  whatever's live on the nodes, from a preset or from `index.html`), so
  it reads the range live off `scan_aggregator`'s own `sweep_min_deg`/
  `sweep_max_deg`/`sweep_speed_rpm`/`sweep_accel` via `get_parameters`
  instead.

  **"A couple of passes" and how it stops itself.** `tilt_axis_bridge`'s
  own `~/status` publishes its raw state string directly (`_publish_status`
  -- see `mks_driver.py`/`node.py`), which is `"sweeping"` while moving
  between ends and `"settling"` during the post-turnaround dwell (see
  the ghost-duplicate turnaround-settle fix elsewhere in this file).
  Both GUIs already keep a live subscription to this exact string
  (`stState` on `index.html`, `valMotor` on `status.html`, both already
  used for other things). Preview Sweep polls it and counts every
  `sweeping` -> `settling` transition as one real turnaround; after 4 of
  them (`PREVIEW_SWEEP_ENDPOINTS`, i.e. 2 full back-and-forth round
  trips) it publishes `~/sweep_enable = false` and stops -- which is
  exactly what the existing Stop-sweep button already does mid-motion
  (see `_on_sweep_enable` in `node.py`: it force-transitions a currently-
  `sweeping` state into `settling` so the axis always finishes settling
  in place rather than halting abruptly), so Preview Sweep's own
  self-stop behaves identically to a manual stop, not a special case.
  Bounded by a generous 120s deadline as a safety net in case a stall or
  an unexpected status shape ever prevented the expected transitions
  from arriving (same defensive pattern as the existing step-and-stare
  jog tool's own bounded poll loop).

  **Guarded against colliding with a real scan.** `scan_aggregator`
  drives its own sweep scans through this *exact same* `~/sweep_enable`
  topic (see its `_sweep_enable_pub`) -- if Preview Sweep were started
  while a real sweep scan was already running, its own auto-stop would
  cut that real scan short the moment it hit its pass count. Both GUIs
  refuse to start Preview Sweep while a real scan looks active:
  `index.html` reuses its existing `parseScanStatus(...).visible` check
  (the same one already driving the progress banner) against
  `scan_aggregator`'s own status; `status.html` gets an equivalent new
  `isScanActive()` check against the same status text it already tracks
  (`valScan`).

  **Verified for real against actual hardware** (the Pi went briefly
  offline mid-session -- network/power, unrelated to this change --
  confirmed back up before this verification ran). `index.html` was
  deployed and loaded in a real browser against the live rosbridge
  origin: switching Scan mode to "Continuous sweep" correctly revealed
  the new fieldset with both buttons and the expected hint text, laid
  out correctly, Stop Preview correctly starting disabled. The browser
  sandbox used for this session's own automated screenshots couldn't
  complete a live rosbridge *websocket* connection from its context
  (plain HTTP to the page loaded fine; this looks like a sandbox
  networking quirk, not a real regression -- the same rosbridge port was
  independently confirmed listening and reachable), so the actual click-
  through interaction was verified the same way this session's earlier
  wedge-fix work was: a raw Python websocket script driving the exact
  same rosbridge calls the button's own JS makes (`get_parameters`/
  `set_parameters`/topic publish, no library, same technique used
  throughout this session since `ros2 topic echo`/CLI tools have been
  unreliable here). With `scan_aggregator` confirmed idle first (the
  guard's own precondition), it set `sweep_min_rad`/`sweep_max_rad`/
  `sweep_speed_rpm`/`sweep_accel` (10-25deg @ 20RPM), published
  `sweep_enable=true`, and watched `~/status`: `idle -> sweeping ->
  settling` (turnaround 1) `-> sweeping -> settling` (2) `-> sweeping ->
  settling` (3) `-> sweeping -> settling` (4), each leg taking ~1.1s,
  exactly 4 real turnarounds detected as designed. `sweep_enable=false`
  was then published (matching what the GUI does at that point) and the
  axis settled cleanly: `settling -> settled`, no stall, no error. This
  confirms the core turnaround-counting/auto-stop mechanism works
  correctly end to end on the real motor -- both GUIs call the identical
  rosbridge operations this script exercised directly, just from a
  button click instead of a script. The "blocked while a real scan is
  running" guard was verified by code inspection (reuses
  `parseScanStatus`/mirrors its exact idle/done/aborted logic) rather
  than by actually running a conflicting real scan to trigger it.
- **Post-scan completeness check**: a real sanity/quality pass on a
  finished `.pcd` (hole/low-density detection, not just "the run
  finished without error") -- supersedes the older, more generic
  "Validate scan data" item above, same idea now scoped against why it
  actually matters here (re-shoot cost).
- **Calibration staleness tracking** (added 2026-09-10, from a
  discussion of leveling/drift-related calibration gaps -- see that
  discussion for the other ideas raised alongside it, not carried
  forward here per explicit request). `mount_roll_deg`/`mount_pitch_deg`
  (the "Calibrate Mount" tool) only fix the sensor's mounting angle on
  the tilt puck, not tripod leveling or drift since the last time it was
  run -- record when `Calibrate Mount` was last run and against what,
  and surface it in the GUI (e.g. "last calibrated 14 scans / 6 days
  ago") so staleness is visible rather than something an operator has to
  remember to check.

**Future features:** (empty -- GPS data from the VLP-16 was scrapped
2026-09-10, per explicit request, once it was flagged that the sensor
has no onboard GPS receiver of its own to pull this from in the first
place; see git history if it's ever reconsidered)

**Distant features:**
- **Color capture** -- a calibrated camera (e.g. 360°) on the same tilt
  mount, colorizing the cloud via the same kind of extrinsics pipeline
  `mount_roll_deg`/etc. already calibrate. Probably the single highest
  value-add for the entertainment/render use case specifically, but a
  real hardware + calibration project of its own, not a quick add.
- ~~Per-point sweep deskewing, added 2026-09-10, per explicit request --
  the actual reason ring/time capture was added. Today's transform is
  one tf2 lookup per whole cloud message (~0.1s of points), so every
  point in that window gets the same pose even though the tilt axis
  (continuously, in sweep mode) moves within it. Caveat stated up
  front: this might not be practical to actually run on the Pi --
  interpolating a real per-point transform (or doing many more, much
  smaller tf2 lookups) for tens of millions of points is real CPU work
  on hardware already running the whole ROS2 stack live.~~ **Built and
  confirmed live 2026-09-11**, and the "might not be practical" caveat
  turned out to be avoidable rather than a real limit, by not doing
  what it assumed:

  **The design that sidesteps the CPU cost.** Naive per-point deskewing
  means either N tf2 lookups per cloud message or one expensive
  interpolated one -- both scale with point count, tens of thousands of
  points per revolution. But only *one* piece of the full base_link<-
  velodyne transform actually varies within a single cloud message
  during a sweep: the tilt joint's own rotation (base_link<-tilt_link,
  a pure rotation about Z per `scanner.urdf.xacro`'s "tilt_axis" joint
  -- origin xyz/rpy all zero, axis `0 0 1`, no fixed offset to account
  for). The other half, tilt_link<-velodyne (the mount calibration --
  translation + roll/pitch/yaw from `vlp16_config`), is fixed for the
  whole message regardless of deskewing. So the new
  `_try_transform_and_accumulate_deskewed` (node.py) does exactly one
  tf2 lookup per cloud message, same as the non-deskewed path -- for
  the fixed mount part only -- then computes the varying Z-rotation
  itself: each point's own absolute capture time
  (`cloud_msg.header.stamp + that point's own "time" field`) is
  interpolated against a new rolling `_joint_state_history` buffer
  (real `/tilt_axis_bridge/joint_state` samples, appended in
  `_on_joint_state`, `JOINT_STATE_HISTORY_MAXLEN = 300` ~ 6s at the
  50Hz republish rate) via `np.interp`, then every point in the whole
  array gets its own angle applied via plain vectorized `cos`/`sin` --
  not a Python loop, not any per-point tf2 call. New `enable_sweep_deskew`
  parameter, **off by default** (new/not yet field-proven the way the
  plain sweep path is), inert for step-and-stare (stationary during
  capture, nothing to deskew) and `MODE_CALIBRATE`
  (`_accumulate_calibration_cloud` never looks at it). `_pending_transforms`'
  retry queue now carries which transform function a cloud was
  submitted through, so a retried cloud stays deskewed (or not)
  consistently rather than possibly switching mid-retry if the
  parameter changed in between. GUI: new "Per-point deskew" checkbox on
  `index.html`'s Continuous sweep scan fieldset, wired through the same
  param-push/preset-save/load paths as the other sweep settings
  (pushed as `false` unconditionally for mount calibration specifically,
  since it's irrelevant there regardless of the checkbox).

  **Verified four ways.** (1) A standalone script confirmed the Rz
  rotation formula against manually-computed expected coordinates for
  several angles, and confirmed `np.interp` against a synthetic
  joint-history buffer produces the right interpolated/clamped angles.
  (2) Real hardware, step-and-stare, unaffected regression check (this
  path was refactored to share `_finish_accumulate_array` with the new
  deskewed path) -- full 11-stop run completed cleanly. (3) Real
  hardware, sweep with `enable_sweep_deskew` left at its default
  `false` -- confirmed the ordinary path still works unchanged after
  the refactor, full run to `done:`. (4) Real hardware, sweep with
  `enable_sweep_deskew=true` -- ran to completion (74 clouds captured),
  produced a real, similarly-sized `.e57` (42.8MB vs 38.7MB for the
  non-deskewed run moments before, same params) with no errors/
  exceptions in the log. A rigorous geometric "does it actually look
  less smeared" comparison wasn't done this pass (would need pulling
  both files for a real point-level analysis) -- what's confirmed is
  that the new code path runs correctly end-to-end on real hardware and
  produces real output, not that it's been visually validated to
  improve scan quality yet. Reset to `false` (the safe default) and
  test scan files deleted after verification.

  **A real, separate issue surfaced during this testing, unrelated to
  this feature: `tilt_axis_bridge` wedged twice in one session** (kernel
  state D, uninterruptible sleep, blocked in what's almost certainly a
  serial/MKS read with no timeout -- same signature as the earlier
  wedge already documented in "Known gotchas" above). Both times
  recovered via `systemctl restart tpl-scanner.service`; both times
  confirmed unrelated to this feature specifically, since the wedge is
  entirely inside `tilt_axis_bridge`'s own driver code, which this
  feature's changes never touch (they're all in `scan_aggregator`).
  Flagged here because it happened *during* this work and could
  otherwise read as this feature's fault at a glance -- it isn't, but a
  second occurrence in one session is worth someone's attention on its
  own, independent of everything else in this entry.

**Ideas / unlikely to happen:**
- **Rough pre-alignment hints**: embedding operator-entered coarse
  position/heading per station into the session metadata, to give
  external ICP registration (done outside this project, see "Decided:
  ICP" above) a better starting guess than blind pairwise registration.
- **Automatic copy/sync** of finished scans off the device (to a laptop
  or cloud storage) instead of a manual USB-stick pull.

## Testing checklist (once the Pi/hardware is back online)

Added 2026-09-10, per explicit request, to collect everything from a
hardware-less session that was built/fixed and verified as much as
possible without the rig, but still has a real, specific gap only real
hardware can close. Check items off here as they're confirmed; if one
turns up broken, that's a normal HANDOFF.md entry (root cause, fix,
re-verify), not just a box left unchecked.

- [x] **Pi poweroff on "Shutdown Everything" -- real bug found, root-
  caused, fixed, and now confirmed end-to-end for real, all 2026-09-11.**
  Sudo was never the problem (`tpl` already has blanket `NOPASSWD:
  ALL`) -- the real cause was `tpl-scanner.service`'s default
  `KillMode=control-group` killing the old detached-subprocess poweroff
  timer within ~1s of the SIGINT teardown, confirmed live with a
  planted cgroup marker process, every single time, unconditionally
  (not intermittent). Fixed by scheduling via `sudo systemd-run`
  instead (its own independent systemd unit, confirmed live to survive
  the identical SIGINT).

  **Real end-to-end trigger, with the user physically present**: called
  `~/shutdown_stack` for real. The `ros2 service call` CLI itself never
  returned cleanly (expected -- the node handling the request is one of
  the things its own SIGINT tears down, see that handler's own
  docstring on the response being best-effort). Watched the actual
  device over the network afterward rather than trusting the CLI
  hanging as a proxy for anything: SSH remained reachable for a while
  after the scheduled 8s delay had already elapsed (sshd isn't part of
  `tpl-scanner.service`'s cgroup, so it's correctly unaffected by the
  SIGINT/kill mechanics above -- this was never expected to cut SSH
  instantly), then moved through connection-refused (sshd itself
  stopping, as part of the real shutdown sequence progressing) to a
  fully dark state -- both ICMP ping and SSH outright timing out, not
  merely refused, which only happens once the network interface itself
  goes down at power-cut. Total wall-clock from trigger to fully dark
  was longer than the naive "SIGINT + 8s" mental model (real systemd
  shutdown sequences stop every other service, unmount filesystems,
  etc. first -- meaningfully slower on Raspberry Pi SD-card I/O than on
  typical dev hardware), but the end state is unambiguous: the Pi
  genuinely, completely lost power. This closes out the last real
  open question on this entire checklist -- device physically powered
  back on by the user afterward.
- [x] **Sweep mode's new time estimate -- run for real 2026-09-11,
  bounded-Duration case.** `sweep_min_deg=5, sweep_max_deg=40,
  sweep_speed_rpm=5, sweep_duration_s=8` (`sweep_edge_margin_deg=5`,
  already configured): estimate says "8s of capture once sweeping
  starts". Real wall-clock from calling `~/start_sweep_scan` to a
  `done:` status: ~23-27s (start 1789123156.87, `done:` first observed
  at 1789123184.22, previous poll ~4.5s earlier not yet done -- see
  the real, ~1.77M-point output file's own existence as confirmation
  a genuine sweep ran, not just a status string). Consistent with the
  step-and-stare finding just below: the excluded homing+transit+save
  time is real and roughly comparable in size to the counted portion
  for a short test run, not a rounding error. **Real gotcha hit while
  picking test parameters, not a code bug**: an initial attempt used
  `sweep_min_deg=5, sweep_max_deg=15` (10 deg span) with the same 5 deg
  edge margin -- since the margin applies from *both* ends, the entire
  10 deg range fell inside it, `_sweep_data_started` could never
  become true, and the sweep ran indefinitely (status stuck at "moving
  to start" with a climbing drop count) until manually stopped via
  `~/stop_scan` (which worked cleanly, correctly fell back to
  `_abort()`'s existing "no points captured" path, `aborted: run
  completed but no points were captured`). Not a new roadmap item on
  its own, but worth remembering: a range not comfortably wider than
  `2 x sweep_edge_margin_deg` is a real, silent trap, not just a bad
  test choice on my part -- nothing in the GUI or status text warns
  about it today.
- [x] **Step-and-stare time estimate's `settle_time_s` fix -- run for
  real 2026-09-11.** `tilt_start_deg=5, tilt_end_deg=15, step_deg=5`
  (3 stops), `revolutions_per_stop=2`, `settle_extra_s=0.5`,
  `rotation_rate_hz=10` (matching a 600 RPM assumption): estimate
  formula gives `3 x (0.3 + 0.5 + 2/10) = 3.0s`. Real wall-clock from
  calling `~/start_scan` to a `done:` status: ~14-17.5s (start
  1789122697.04, first blank poll at 1789122710.88 i.e. +13.8s,
  `done:` confirmed by 1789122714.45 i.e. +17.4s). The fix itself
  (adding the 0.3s per-stop settle) is correct arithmetic, confirmed
  earlier without hardware -- what real hardware adds here is
  confirming just how large the *remaining*, still-unestimated gap
  actually is: roughly 11-14.5s of homing+per-stop-travel+save on top
  of a 3.0s estimate, for a trivially small 3-stop scan. Real output
  file (`scan_20260911_113151.pcd`, 320,270 points) confirms a genuine
  scan ran.
- [x] **Homing / per-stop travel / final-save time** for the
  step-and-stare estimate -- **still not folded into the estimate as a
  real number** (deliberately -- see the 2026-09-10 entry on why
  guessing felt worse than an honest "unestimated"), but its real
  *magnitude* is no longer a total unknown: both real runs above put it
  at roughly 11-19s for a short test scan/sweep on this rig, i.e.
  comparable to or larger than the estimated portion itself for a small
  run. Worth remembering when reading either estimate: for a short
  scan specifically, the excluded time can dominate the number shown,
  not just pad it.
- [x] **E57 export's ROS/GUI wiring -- confirmed live 2026-09-11.**
  Deployed today's `scan_aggregator` changes to the real Pi (synced the
  modified source files -- they aren't tracked by git yet -- rebuilt
  with `colcon build --packages-select scan_aggregator`, restarted via
  `sudo systemctl restart tpl-scanner.service`, confirmed a single
  clean process tree afterward, no orphans). Published a real
  `~/export_e57_request` for an existing real scan (`test1.pcd`,
  14,983,609 points, saved 2026-09-07) directly over the topic (not yet
  through the actual GUI button/browser -- see the still-open item
  below) and got back `{"error": null}` with a genuine 240MB `.e57`
  written on the Pi's own SD card. Pulled it back and opened it with
  `pye57`: point count matches exactly, `cartesianX` values are real,
  physically plausible scan coordinates, and the XML is well-formed.
  **Not yet clicked from an actual browser against the real stack** --
  the request was published directly, not through `index.html`'s
  "Export as E57" button; worth doing once for real UI-level confidence,
  though the service-level round trip working end-to-end is the part
  that was actually in question.
- [x] **"ring"/"time" are this project's real point-field names --
  confirmed 2026-09-11.** `ros2 topic echo /velodyne_points --field
  fields` on the real running driver: `ring` (uint16, offset 16) and
  `time` (float32, offset 18) are both really there, exactly as
  `EXTRA_POINT_FIELDS` assumed -- no code change needed. Also read the
  actual installed `tf2_sensor_msgs.py` source on the Pi to settle the
  other real open question here: `do_transform_cloud` calls
  `read_points(cloud)` with no `field_names` filter (i.e. reads every
  field, not just x/y/z), only overwrites the x/y/z columns, and
  reserializes with the original `cloud.fields` list -- confirms ring/
  time really do survive the transform step, so reading them from the
  transformed message in one combined call
  (`_read_points_with_extra_fields`) is correct, not just hoped-for.
  **Still open**: no actual scan has been captured since this code was
  deployed (the test above reused a pre-existing 4-field `.pcd` from
  2026-09-07), so ring/time flowing all the way into a fresh `.pcd`/
  `.e57` hasn't been directly observed yet, only the topic-level field
  presence and the transform-preservation logic behind it.
- [x] **E57 export's real per-scan metadata -- confirmed 2026-09-11,
  with real data.** The same `test1.pcd` export above shows genuinely
  live values pulled from the real rig, not placeholders:
  `tplScanConfigJson` (`step_deg: 0.65`, `sweep_duration_s: 180.0`, real
  invert flags), `tplMountCalibrationJson` (`mount_roll_deg: 88.99`,
  `mount_pitch_deg: 135.59`, fetched live from `vlp16_config` via the
  async `GetParameters` call), and `tplVlp16StatusJson` (`"rpm": 601`,
  `"state": "On"` for both motor and laser, GPS PPS absent) -- matches
  what a direct `ros2 topic echo /vlp16_config/status` showed
  independently at the same time. `_on_export_e57_mount_params_received`
  and `_on_vlp16_status` both confirmed working, not just non-crashing.
- [x] **Session/project grouping -- confirmed live 2026-09-11, against
  the redesigned (dropdown + "New Project…", no station number)
  version** -- see that dated entry above for why it was redesigned
  same-day before ever being committed; the *first* design was also
  confirmed live before being replaced, not just reasoned about, so
  this checklist item has genuinely been satisfied twice over, against
  two different real designs. Deployed to the real Pi (rebuilt
  `scan_aggregator`, restarted via `sudo systemctl restart
  tpl-scanner.service`, confirmed a single clean process tree). Set
  `project_name="Test Project"` and ran two real short scans directly
  against the service: produced `Test_Project_20260911_123547.pcd` and,
  a second real scan later, `Test_Project_20260911_123615.pcd` --
  exactly the new naming convention, no station number, no collision
  between the two. Published a real `~/bundle_project_request` for
  "Test Project" against these two actual files: got back
  `Test_Project_bundle.zip` (12,833,646 bytes, matching the two source
  files' combined size plus minimal `ZIP_STORED` overhead), pulled it
  back and confirmed with Python's own `zipfile` module that both real
  files are present, byte-size-exact, and `testzip()`-clean. Test
  artifacts deleted from the Pi afterward, and
  `project_name`/scan-range fields temporarily changed for this test
  restored to their real prior values. **Still not clicked from an
  actual browser against the real stack** -- the naming and bundle
  download were both exercised by publishing directly to the relevant
  topics/services, not through `index.html`'s own Project fieldset,
  "New Project…" button, or "Download All" button in a live browser
  session; worth doing once for real UI-level confidence, though the
  service-level behavior working end-to-end is what was actually in
  question.
- [x] **One-Start-Scan-button mode dropdown -- confirmed live 2026-09-11,
  all four services it can dispatch to, real motion + real output each
  time.** Deployed via `colcon build --packages-select scan_aggregator
  tilt_axis_bridge` (no source changed for either package in this commit,
  ran anyway as routine practice) + `systemctl restart tpl-scanner.service
  tpl-gui-http.service`; all 7 launch processes came up clean. Real UI
  click-through wasn't possible from this environment's own browser
  sandbox -- it can reach the Pi's `tpl-gui-http.service` over plain HTTP
  (loaded `index.html` for real, rosbridge URL auto-filled to the Pi's own
  address) but its outbound network blocks the actual WebSocket upgrade
  to rosbridge on 9090 (code 1006 on every attempt; a raw HTTP/1.1
  Upgrade handshake to the same host:port from this same machine's own
  shell, bypassing the browser sandbox entirely, completed instantly --
  `101 Switching Protocols` -- confirming rosbridge itself was never the
  problem). Fell back to this project's other established real-hardware
  method instead: `ros2 service call` over SSH, directly against the same
  four services the two merged buttons now dispatch to depending on their
  mode dropdown -- `/scan_aggregator/start_scan` and `.../start_sweep_scan`
  (what `index.html`'s button calls) and `.../start_scan_from_panel` and
  `.../start_sweep_scan_from_panel` (what `status.html`'s calls). All four
  ran a real scan to completion with a small quick config (3-stop
  step-and-stare, short bounded sweep) and produced a real `.e57` each,
  confirmed via the `/scan_aggregator/status` topic's own `done:` message
  each time -- test output files deleted afterward, nothing else in that
  scans directory touched. This isn't a full substitute for actually
  clicking the dropdown+button in a live browser session against the
  rig (the browser-side JS branch itself was instead verified earlier by
  mocking `bridge.callService` and confirming the right service name was
  called for each mode -- see roadmap entry above), but between the two
  it closes the real gap: the JS branch picks the right service, and each
  service still does the right real thing on real hardware.
- [x] **Onboard-screen tabs -- confirmed live 2026-09-11, three of four
  real-hardware gaps closed.** Deployed (`git pull` on the Pi's
  checkout, no rebuild needed -- static HTML only) and the kiosk's own
  Chromium relaunched (it doesn't auto-reload) to pick up the new page;
  confirmed via a real `grim` screenshot of the physical 480x320 panel.
  - **Auto-switch to Scan tab**: confirmed live -- triggered a real scan
    over SSH while the kiosk sat on Status, a follow-up screenshot shows
    it jumped to Scan on its own, showing `homing`.
  - **Stop button** (`~/stop_scan`): called for real mid-scan, returned
    `stopped`.
  - **Home button** (`~/home`): confirmed live, see the dedicated
    entry above -- this one surfaced a real pre-existing bug
    (`_tick_state_machine`'s homing race) along the way, now fixed and
    verified with a real large-displacement home.
  - **Release Stall**: still not exercised against a real stall
    condition -- hard to manufacture on demand, lowest priority of the
    four, left open.
  - **Layout**: real photos of the physical panel (sent by the user)
    showed the Control and Scan tabs' buttons overflowing past the right
    edge with text cut off -- a genuine bug (`min-width:0` defeating
    `flex-wrap`) missed by this entry's own earlier "renders/fits
    correctly" claim above. Fixed and re-verified for real via CDP
    (`Runtime.evaluate` calling the page's own `switchTab()` over the
    kiosk's remote-debugging port) plus pixel-level `grim` crops showing
    `SHUTDOWN`/`STOP` fully intact -- see the roadmap entry above for the
    full writeup.
  - **Bezel**: after the layout fix above, the user reported the
    physical enclosure's bezel was *still* covering a strip of the real
    screen's right edge -- a hardware/mounting fact no CSS fix for the
    flex-wrap bug could touch, and not something `grim` could ever have
    caught either (it captures the full software framebuffer, not what
    the bezel physically obscures). Added `main`'s right padding as an
    asymmetric safe margin (10px -> 26px, right side only, see the
    comment left in `status.html` right on that rule) so page content
    stays clear of that zone. Deployed, kiosk relaunched, **user
    confirmed live on the real device it now looks right** -- the exact
    margin value was necessarily picked without being able to verify it
    from this session's own side, so if it ever needs retuning again
    (different enclosure, different unit), that same rule is the one
    to adjust.
- [x] **Preview Sweep button -- confirmed live 2026-09-11.** See the
  roadmap entry above for the full writeup: both GUIs deployed and the
  underlying turnaround-counting/auto-stop mechanism (the same
  `get_parameters`/`set_parameters`/`~/sweep_enable` calls either
  button's own JS makes) driven directly against the real motor via a
  raw websocket script -- 4 real turnarounds detected exactly as
  designed, clean settle afterward, no stall.
- [ ] **Calibration staleness tracking** -- not yet built as of this
  session (still a "Coming soon" roadmap item, see above), listed here
  as a forward pointer so this checklist stays the single place to
  check once it lands, rather than needing a second list started later.
