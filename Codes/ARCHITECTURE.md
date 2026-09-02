# Project A7 — Mapping System Architecture

This document is the reference companion to the code in `sensors/ultrasonic/`,
`sensors/vision/`, and `mapping/`. Code docstrings cover *how* each piece
works; this covers *why*, plus the things that only make sense at the
whole-system level.

**Tested, not just written.** Every pure-logic path (coverage planning,
the state machine, side-pass avoidance, object policies, map
serialization) is exercised end-to-end by
`mapping/examples/simulated_mapping_demo.py` with zero real hardware,
and it passes. `sensors/ultrasonic/ultrasonicManager.py` imports
cleanly against the real `gpiozero` library. `sensors/vision/visionManager.py`
is syntax-checked but not import-tested (it needs `opencv-python` +
`ultralytics`, which pull in a multi-GB PyTorch install not worth doing
just to check imports). None of this has run on actual GPIO pins, a
real camera, or a real rover — see "Edge Cases" for what that means
for you before you power anything on.

---

## 1. Overview & design principles

Five rules drove every decision here, taken directly from your brief:

1. **CarController only controls movement** — it never reads a sensor
   or makes a decision. `MappingManager` is the only thing that reads
   sensors *and* commands the car.
2. **ArmController is untouched** — no edits to `controller.py` or
   `ik.py`. Where MappingManager needs to interact with the arm, it
   only calls the existing public `update()` method.
3. **Sensor managers are independent**, constructed outside both
   controllers, and injected wherever they're needed.
4. **MappingManager receives everything through the constructor** — it
   never does `GPSManager()` internally.
5. **MappingManager never touches hardware directly** — no `gpiozero`,
   `cv2`, `serial`, or `smbus2` imports anywhere in `mapping/`. Confirm
   this yourself: `grep -rE "gpiozero|cv2|serial|smbus2" mapping/`
   returns nothing.

Rule 4 and 5 together are what make rule 6 ("reusable for simulation")
free: `mapping/interfaces.py` defines the shape MappingManager expects
(`CarLike`, `ArmLike`, `GPSLike`, `IMULike`, `UltrasonicLike`,
`VisionLike`) as `typing.Protocol` classes, not base classes. Anything
with the right methods satisfies them — a real `GPSManager`, or a
20-line fake in a test file. `MappingManager.__init__` validates every
injected dependency against these Protocols at construction time and
raises a clear `TypeError` if something doesn't fit, rather than
failing confusingly three method calls later.

---

## 2. Suggested package structure

This matches the target tree in your brief. New files from this
session are marked; everything else is exactly what you already have.

```
Robot/
├── Car/
│   ├── carController.py            (existing, untouched)
│   └── btsMotor.py                 (existing, untouched)
│
├── Arm/
│   ├── controller.py                (existing, untouched)
│   ├── detector.py                  (existing, untouched)
│   └── ik.py                        (existing, untouched)
│
├── sensors/
│   ├── __init__.py                  ← new
│   ├── imu/
│   │   ├── imuManager.py            (existing — see migration note below)
│   │   └── mpu6050.py               (existing)
│   ├── gps/
│   │   ├── gpsManager.py            (existing — see migration note below)
│   │   └── neom8n.py                (existing)
│   ├── ultrasonic/
│   │   ├── __init__.py              ← new
│   │   └── ultrasonicManager.py     ← new
│   └── vision/
│       ├── __init__.py              ← new
│       └── visionManager.py         ← new (includes VisionManagerDetectorAdapter)
│
├── mapping/
│   ├── __init__.py                  ← new
│   ├── interfaces.py                ← new — Protocol contracts
│   ├── mapData.py                   ← new — Pose/Obstacle/DetectedObject/RoverMap
│   ├── stateMachine.py              ← new — MappingState + transitions
│   ├── coveragePlanner.py           ← new — boustrophedon planner
│   ├── obstacleAvoidance.py         ← new — side-pass planner
│   ├── objectPolicy.py              ← new — ignore/avoid/pickup strategies
│   ├── localization.py              ← new — GPS+IMU fusion
│   ├── mappingManager.py            ← new — the orchestrator
│   └── examples/
│       └── simulated_mapping_demo.py ← new — runnable, hardware-free proof
│
├── logger/
│   └── logger_manager.py            (existing, untouched)
│
└── serverController.py              (existing, untouched)
```

**Migration note on IMU/GPS:** your actual current code imports these
as `Car.sensors.basic.imu.imuManager` (nested inside `Car/`), but your
target tree (and rule 3) put `sensors/` at the top level, as a sibling
of `Car/`. I didn't move `imuManager.py`/`gpsManager.py` — that's
working code I wasn't asked to touch, and moving it without being able
to test your full tree risked breaking a real import path. If you want
full consistency with the target tree, the move is mechanical: relocate
the files, then in `carController.py` change

```python
from Car.sensors.basic.imu.imuManager import IMUManager
```
to
```python
from sensors.imu.imuManager import IMUManager
```
That's the only call site — nothing else references that path.

---

## 3. Class architecture & relationships (UML-style)

```
                    ┌─────────────────────┐
                    │   <<Protocol>>       │
   depends on  ┌───▶│  CarLike, ArmLike,   │◀───  implemented by
   (typing     │    │  GPSLike, IMULike,   │      (structurally,
   only, no    │    │  UltrasonicLike,     │      no inheritance)
   import)     │    │  VisionLike          │
               │    └─────────────────────┘
               │
   ┌───────────┴─────────────┐        ┌─────────────────────────┐
   │      MappingManager      │──────▶│  RoverMap (composition)  │
   │  (mapping/mappingManager)│        └─────────────────────────┘
   │                          │──────▶ BoustrophedonPlanner
   │  injected:               │──────▶ SidePassPlanner
   │   car, arm, gps, imu,    │──────▶ PositionEstimator (localization)
   │   ultrasonic, vision     │──────▶ MappingStateMachine
   │                          │──────▶ ObjectPolicy (ignore/avoid/pickup)
   └──────────────────────────┘

   Real classes satisfying the Protocols (verified against your files,
   zero changes needed):

     CarController  ⊨ CarLike     (drive, stop, turn_left, turn_right, move_forward)
     Controller     ⊨ ArmLike     (update)
     GPSManager     ⊨ GPSLike     (get_location, get_telemetry, has_fix)
     IMUManager     ⊨ IMULike     (get_orientation, get_telemetry)
     UltrasonicManager ⊨ UltrasonicLike  (get_telemetry, has_obstacle)     [new]
     VisionManager  ⊨ VisionLike  (detect, read_frame)                    [new]
```

Composition inside `mapping/`, one level down:

| Owner | Owns (composition) |
|---|---|
| `UltrasonicManager` | `UltrasonicArray` (gpiozero access) + `UltrasonicTracker` (background thread) |
| `VisionManager` | `VisionDetector` (cv2/YOLO access) + `VisionTracker` (background thread) |
| `MappingManager` | `RoverMap`, `MappingStateMachine`, `PositionEstimator`, one `ObjectPolicy` instance, a `list[Waypoint]` from `BoustrophedonPlanner`, and transient `AvoidancePlan`s from `SidePassPlanner` |

Note what's *not* in this diagram: `mapping/` has zero import
dependency on `sensors/`, `Car/`, or `Arm/`. It only imports from
`logger/` and within itself. That's deliberate — MappingManager's
correctness can be verified (as it was, above) without any hardware
package installed at all.

---

## 4. Data models

All in `mapping/mapData.py`, all plain dataclasses with a `to_dict()`
for JSON:

| Model | Fields | Purpose |
|---|---|---|
| `Pose` | `x_m, y_m, heading_deg, latitude, longitude, timestamp` | One position+heading sample. Used for the trail. |
| `Obstacle` | `x_m, y_m, source, distance_m, timestamp` | One recorded ultrasonic obstacle. |
| `DetectedObject` | `x_m, y_m, class_name, confidence, policy, action_taken, timestamp` | One recorded vision detection + what was done about it. |
| `MapCell` | `row, col, x_m, y_m, visited, visited_at` | One coverage-grid cell. |
| `RoverMap` | `width_m, height_m, cell_size_m, origin, grid[][], trail[], obstacles[], objects[]` | The whole mission. |

`RoverMap.to_json()` / `.save(path)` / `.load(path)` round-trip the
entire mission losslessly (verified in demo 4). For **WiFi
transmission**, `to_dict()` is already a plain JSON-safe dict — the
natural next step is a Flask route returning
`jsonify(mapper.rover_map.to_dict())`, which I did *not* add to
`serverController.py` (see Future Improvements — it needs an actual
`MappingManager` instance wired up with real pins/camera first).

---

## 5. Mapping algorithm (boustrophedon coverage)

`BoustrophedonPlanner.generate()` produces a flat, ordered list of
absolute `Waypoint(x_m, y_m)` — alternating left-to-right and
right-to-left lanes, `lane_spacing_m` apart, each lane broken into
points `waypoint_spacing_m` apart.

`MappingManager._navigate_to()` doesn't know it's tracing a lawnmower
pattern — it just does, every tick:

```
target_heading = bearing from current pose to next waypoint
if heading error > 5°:  turn toward it, return
if distance > tolerance:  drive forward min(distance, waypoint_spacing_m)
else:  mark waypoint reached, advance to the next one
```

The lawnmower *shape* falls entirely out of the waypoint geometry —
within a lane, consecutive waypoints differ only in x, so the natural
heading is ~90° from the lane-to-lane heading. No special-casing
"am I sweeping or turning a corner" anywhere in `MappingManager`.

Tuning: `lane_spacing_m` should come from your actual effective sensor
width (ultrasonic cone + camera FOV) with a safety overlap margin —
this is not derived automatically, because how much overlap you trust
is a judgment call, not a geometry fact.

---

## 6. Obstacle avoidance algorithm (side-pass)

`SidePassPlanner.plan()` (in `mapping/obstacleAvoidance.py`) picks a
side (whichever of left/right ultrasonic reads more clearance,
falling back to `default_side` if both are unusable or too close to
call), then emits a 7-step **relative** maneuver:

```
turn away 90° → forward clearance_m → turn back 90° → forward pass_distance_m
              → turn back 90° → forward clearance_m → turn away 90°
```

Because every turn is relative to the rover's *current* heading (not
a global compass direction), this traces a clean rectangle and
returns the rover to its original heading regardless of what that
heading was — no absolute-angle bookkeeping needed. `MappingManager`
executes the steps one per `tick()` in `AVOIDING_OBSTACLE`, then hands
off to `RETURNING_TO_PATH`, which just re-enters normal waypoint
navigation — see the diagram from earlier in this conversation for why
that's sufficient (waypoints are absolute, so "resume" just works).

`object_policy="avoid"` reuses this exact planner (see below) — an
object you want to steer around and an obstacle you didn't expect
need the same physical response, even though they came from different
sensors.

---

## 7. Object handling workflow

`mapping/objectPolicy.py` implements ignore/avoid/pickup as a strategy
pattern — `ObjectPolicy.handle(detection, context) -> PolicyResult` —
so `MappingManager`'s loop never branches on the policy name; it just
calls whichever policy was configured and reacts to the result.

- **ignore**: `PolicyResult(action_taken=IGNORED, next_state=None)` —
  logged into `RoverMap.objects`, mission continues untouched.
- **avoid**: `next_state=AVOIDING_OBSTACLE` — routes through
  `SidePassPlanner` exactly like an ultrasonic obstacle.
- **pickup**: `next_state=PICKING_OBJECT`. Each `tick()` in that state
  calls `self.arm.update()` (your existing `Controller.update()`,
  unmodified) and treats a non-`None` return as success, with a
  `pickup_timeout_s` (default 20s) escape hatch that records
  `PICKUP_FAILED` and resumes mapping rather than hanging forever.

**Read this before relying on "pickup" for anything real:**
`Controller.update()` only *computes IK joint angles* — nothing in the
files you gave me drives servos or closes the gripper (that logic
lives in whatever external script currently reads `controller.angles`,
referenced in `controller.py`'s own comments as "test.py"). So
`_tick_picking_object()`'s success signal currently means "the arm
knows where to move," not "the arm grabbed the object." This is
flagged in the code with a docstring, not silently assumed — treat
`pickup` as scaffolding until `ArmController` exposes a real
`pick_up_object() -> bool` that drives the actual sequence.

**A second real gotcha, solved:** if `VisionManager` and `ArmController`'s
own `BottleDetector` both open `camera_id=0`, they'll conflict.
`sensors/vision/visionManager.py` ships `VisionManagerDetectorAdapter`,
which wraps a shared `VisionManager` in the exact shape
`Controller(detector=...)` already expects — so:

```python
vision = VisionManager(target_classes={"bottle": 39})
arm = Controller(detector=VisionManagerDetectorAdapter(vision))
mapper = MappingManager(..., vision=vision, arm=arm, ...)
```

gives you one shared camera, zero changes to `controller.py`/`detector.py`.

---

## 8. GPS + IMU fusion strategy

`mapping/localization.py`'s `PositionEstimator`:

- **Heading** comes from IMU yaw, corrected toward the GPS-implied
  direction of travel whenever the rover is moving fast enough for
  that to be trustworthy (a lightweight complementary filter, not a
  Kalman filter) — bounds yaw drift without a magnetometer.
- **Position** is GPS-primary: each new fix reprojects to local
  meters via an equirectangular approximation around the mapping
  origin (accurate well beyond what a single mapping plot needs).
  Between fixes (NEO-M8N updates around 1 Hz, far slower than a
  control loop), position dead-reckons forward from the commanded
  speed, then snaps to the next real fix — a visible but
  coverage-grid-tolerable jump, not true sensor fusion (see Future
  Improvements for the EKF/UKF upgrade path).
- **No fix at all** (indoors, just booted): free-runs on dead
  reckoning alone, which drifts exactly the way `mpu6050.py`'s own
  docstring already warns about. `require_gps_fix=True` (default)
  blocks `start_mapping()` until a fix exists; it does not protect
  against losing the fix mid-mission (see Edge Cases).

**Calibration you must do before trusting navigation, not code can
determine for you:**
1. **Yaw sign convention** — whether a physical right turn increases
   or decreases the IMU's reported yaw depends on which way the gyro
   Z axis is mounted. `localization.py`'s heading math assumes a
   specific convention; verify it on the rover and negate if wrong.
2. **`estimated_speed_mps`** — the real-world speed (m/s) your rover
   achieves at a given `cruise_speed` (0..1 duty cycle) has to be
   measured (time it over a known distance). The default (0.3) is a
   placeholder; using the wrong value makes every distance-based move
   systematically too short or too long.

---

## 9. State machine

See the diagram earlier in this conversation for the visual. Full
transition table (`mapping/stateMachine.py`):

| From | Can go to |
|---|---|
| IDLE | INITIALIZING |
| INITIALIZING | MAPPING, ERROR, IDLE |
| MAPPING | AVOIDING_OBSTACLE, PICKING_OBJECT, COMPLETED, ERROR, IDLE |
| AVOIDING_OBSTACLE | RETURNING_TO_PATH, ERROR, IDLE |
| PICKING_OBJECT | RETURNING_TO_PATH, MAPPING, ERROR, IDLE |
| RETURNING_TO_PATH | MAPPING, ERROR, IDLE |
| COMPLETED | IDLE |
| ERROR | IDLE |

Every in-progress state can reach IDLE directly — that's
`stop_mapping()`, a user abort that has to work regardless of which
sub-state the mission is in. `MappingStateMachine.transition()` raises
`InvalidTransition` on anything not in this table, so a bug that tries
to jump states incorrectly fails loudly at the point of the mistake
instead of leaving the rover in an inconsistent state.

---

## 10. Important edge cases

Roughly in order of "how likely to bite you first":

1. **Camera contention** — solved by the adapter above, but only if
   you actually wire ArmController through it. Two independent
   `cv2.VideoCapture(0)` instances will conflict.
2. **Blocking turns** — `CarController.turn_left/turn_right` block
   until the full angle is rotated and don't check ultrasonic
   mid-turn. `MappingManager` reuses them as-is (safer than
   re-implementing tested rotation logic) rather than duplicating
   IMU-feedback code — the tradeoff is a brief window with no fresh
   obstacle check while actively turning. Keep side-pass turn angles
   small (they're 90° here) and let pre/post checks do the work.
3. **Two calibration constants that WILL be wrong until you measure
   them**: IMU yaw sign convention, and `estimated_speed_mps`. Wrong
   values don't crash anything — they make the rover confidently drive
   the wrong distance or turn the wrong way.
4. **Pickup "success" is a placeholder** — see section 7. Don't ship
   `object_policy="pickup"` as a "the rover actually grabs things"
   feature until `ArmController` has a real completion signal.
5. **GPS fix loss mid-mission** — `require_gps_fix` only gates
   *startup*. If the fix drops during MAPPING, the estimator silently
   falls back to dead reckoning with no alarm raised. A watchdog on
   fix age is a natural next addition (see Future Improvements).
6. **Ultrasonic cross-talk** — three sensors read back-to-back can
   pick up each other's echoes. Staggered with a 10ms delay between
   reads; still worth distrusting three suspiciously-identical
   readings.
7. **`has_obstacle()` fails closed** — returns `True` if no reading
   exists yet, by design (safer than a false "clear"). This means a
   wiring fault looks identical to "there's something 30cm in front of
   me from the moment the rover boots" — if avoidance triggers
   immediately and stays stuck, check wiring before assuming there's
   really an obstacle.
8. **COCO class coverage** — only "bottle" is a real YOLO class from
   your example list (bottle/box/rock/trash). "box"/"rock"/"trash"
   need a custom-trained model — `VisionManager(target_classes=...)`
   is ready for that swap, it just isn't done here.
9. **Object re-detection** — `_recently_handled()` is a simple
   distance+class heuristic to avoid immediately re-triggering
   avoid/pickup on the same still-visible object. It is not real
   object tracking/re-identification; two different bottles 40cm
   apart could be conflated.
10. **Concurrent access from serverController.py** — if you ever call
    `MappingManager.tick()` from one thread while Flask handles a
    `/api/move` request on another, both want the same `car`. Reuse
    `serverController.py`'s existing `car_lock` pattern around both.
11. **Long-mission drift** — local (x_m, y_m) accumulates
    floating-point + dead-reckoning error over very long runtimes or
    very large areas. Fine at 5×7m; worth watching at 50×70m.
12. **Side-pass sizing is fixed, not adaptive** — `clearance_m` and
    `pass_distance_m` don't scale to the actual obstacle's size
    (ultrasonic gives distance, not shape), and a detour can overshoot
    or undershoot the interrupted waypoint. Not a correctness bug
    (navigation re-homes either way), just an efficiency one.
13. **Continuous YOLO inference on a Pi 5 CPU** is a real thermal/CPU
    load next to GPS/IMU/ultrasonic threads and Flask. Profile the
    achievable `poll_hz` on real hardware rather than trusting the
    default (10 Hz).

---

## 11. Future improvements

- **SLAM upgrade path** (requirement #8, already unblocked): because
  `MappingManager` only depends on `GPSLike`/`IMULike` Protocols
  through `PositionEstimator`, a SLAM backend can replace
  `PositionEstimator` behind the same `.pose()`/`.update()` shape
  without touching `MappingManager` itself.
- **Real sensor fusion** — replace the snap-and-drift approach in
  `localization.py` with an EKF/UKF (e.g. via `filterpy`).
- **Wheel encoders** for real odometry between GPS fixes, instead of
  pure IMU dead reckoning.
- **`ArmController.pick_up_object() -> bool`** that actually drives
  the servo/gripper sequence and reports true success — the single
  highest-value change to make `object_policy="pickup"` real.
- **An interruptible turn primitive** on `CarController` (e.g. an
  optional per-iteration callback) so avoidance-grade safety checks
  can run mid-turn, not just between calls.
- **Dynamic `lane_spacing_m`** derived from measured sensor FOV/range
  instead of a fixed constant.
- **Richer object policies** — e.g. `"photograph"`: stop, save an
  image + GPS tag, continue.
- **Multi-rover map merging** — `RoverMap.to_dict()`/`load()` are
  already positioned for this; a merge function stitching two maps
  sharing a GPS origin is the natural next step.
- **Live WiFi telemetry** — a `/api/map` route on `serverController.py`
  returning `mapper.rover_map.to_dict()`, once a real `MappingManager`
  is wired up with actual pins/camera.
- **Custom-trained YOLO model** for box/rock/trash, or a classical-CV
  fallback heuristic for "unknown object" when vision can't classify
  something ultrasonic flagged.
- **Map versioning/diffing** across repeated missions of the same area.
- **Persist `MappingStateMachine.history`** into `RoverMap` for
  post-mission debugging.

---

## 12. Running the demo

```bash
cd your-project-root      # next to your existing logger/ package
python3 -m mapping.examples.simulated_mapping_demo
```

Runs five scenarios with zero real hardware: full coverage completion,
obstacle avoidance mid-mission, `stop_mapping()` from multiple states,
map save/reload, and the `pickup` policy path. All five currently pass.
