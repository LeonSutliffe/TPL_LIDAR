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

Everything currently targets the Windows+WSL2 setup described below. **A
Raspberry Pi 4 Model B (4GB) self-contained deployment is now in
progress** (decided 2026-08-29, reversing the earlier "not pursued" call
from the original Pi field-recording investigation -- see "Raspberry Pi 4
deployment (in progress)" further down for current status) -- the
`enable_pointcloud`/RAM work from that original investigation is still
live and directly relevant now, not just harmless leftover.

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
with real hardware, rather than the laptop.** Not yet done: USB output
storage, WiFi hotspot, systemd auto-start, and the onboard screen's
status script haven't been exercised together as one combined
field-ready run (each is individually documented in README, just not
combined yet); a physical jog/motion command also wasn't issued this
session (same caveat as the earlier FTDI bridge verification -- protocol
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
fixed or manually restored in `.wslconfig`. Not investigated further
this session since it wasn't blocking the actual task (Pi verification);
worth root-causing before next relying on the laptop's own USB
passthrough path.

**Extended 2026-08-29: USB output storage, a WiFi hotspot, and web-based
control from a phone are now all planned/documented (not yet tested --
no Pi hardware yet), in `README.md` rather than duplicated here.** Full
detail lives there; summary for this doc's own chronological record:
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
- Validate scan data (some form of sanity/quality check on a completed
  `.pcd`, not just "the run finished without error").
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
