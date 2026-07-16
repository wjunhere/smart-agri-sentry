# Fixed-Point Diagnosis Stop Design

## Goal

Allow an operator to configure several map-coordinate locations where the car
stops during autonomous waypoint patrol, shows a selected tomato diagnosis with
a simulated 80% to 90% confidence, then resumes the original Nav2 patrol.

## Scope

- The feature is active only while `mission_control_node` is in `PATROL`.
- A fixed-point rule contains `x`, `y`, `radius`, and `disease_class`.
- The default, editable trigger radius is `0.20` metres.
- The bottom status bar contains an editable fixed-point section immediately
  after the visual-logic controls. The existing bottom-bar vertical scrollbar
  exposes it when the viewport cannot show all controls.
- The operator can add, delete, edit, and save multiple rules before preheat.
- Saving persists rules in the board-side `mission_params.yaml`. Preheat reads
  the saved configuration; an active patrol uses that startup snapshot.
- A rule triggers once per autonomous patrol run. Starting another patrol resets
  the per-run trigger set.
- The selectable categories are the tomato model labels: `late_blight`,
  `healthy`, `early_blight`, `bacterial_spot`, `leaf_mold`,
  `septoria_leaf_spot`, and `tomato_yellow_leaf_curl_virus`.

## Architecture

The browser owns configuration editing only. `web_remote_node` validates and
persists the fixed-point list through a small HTTP endpoint, following the
existing waypoint API pattern. The API returns the saved normalized list so
all browser clients remain in sync after their normal polling refresh.

`mission_control_node` loads the configured list during initialization and
compares its current `/odometry/filtered` map position against every unhandled
rule while in `PATROL`. When the car enters a rule radius, it records the rule
as handled and uses the same stop transition, Nav2 cancellation, pipeline
trigger, timeout, action, and resume mechanism used for an accepted plant
detection. It does not insert, remove, or alter Nav2 waypoints or replace the
active navigation goal.

The fixed-point event carries an override diagnosis class in mission state.
When the vision pipeline reports a diagnosis for that event, mission control
publishes the configured class and a deterministic time-varying confidence in
the inclusive range `[0.80, 0.90]` on `/vision/diagnosis`. The frontend
therefore displays the configured result while the normal diagnosis card and
the existing pipeline lifecycle remain intact. A normal visual plant event has
no override and continues to publish its real diagnosis unchanged.

## Data Contract

`mission_params.yaml` gains the following optional top-level field:

```yaml
fixed_point_stops:
  - x: 1.2
    y: -0.5
    radius: 0.20
    disease_class: early_blight
```

`GET /fixed-point-stops` returns:

```json
{"status":"ok","fixed_point_stops":[{"x":1.2,"y":-0.5,"radius":0.2,"disease_class":"early_blight"}]}
```

`POST /fixed-point-stops` accepts the same list under `fixed_point_stops`.
Validation rejects non-finite coordinates, radii not greater than zero, and
categories outside the supported tomato labels. Invalid requests leave the
saved configuration unchanged and return HTTP 400.

## Mission Flow

1. The operator saves zero or more fixed-point rules before preheat.
2. Automatic patrol starts and initializes its handled-rule set as empty.
3. In `PATROL`, obstacle handling retains priority. Fixed-point checks occur
   only after the existing avoidance-state exclusion; a point cannot interrupt
   backup, turning, arc drive, or rejoin movement.
4. On entering an unhandled radius, mission control logs the rule, stores the
   configured diagnosis override, cancels the active Nav2 task, and transitions
   through the existing visual stop pipeline.
5. The final diagnosis shown to the frontend uses the selected category and a
   confidence that remains between 0.80 and 0.90.
6. The existing resume delay returns the car to `PATROL` and resends the saved
   original waypoint goal. The fixed point stays handled until the next patrol
   run.
7. If the pipeline times out or errors, the existing timeout/recovery behavior
   still resumes patrol; the configured result is not fabricated without a
   successful pipeline event.

## Frontend Behavior

The new `固定点停车` section is a compact editable table. Each row has numeric
`X (m)`, `Y (m)`, and `半径 (m)` inputs, a native select control for the tomato
category, and an icon delete button. The section has an add button and a save
button. It is placed after `视觉逻辑` inside `.env-bar-inner`, which already has
`overflow-y: auto`; the bottom bar remains scrollable rather than growing over
the main dashboard.

The browser fetches saved rules on load and after saving. Save errors are shown
in the existing frontend error/status channel and leave local edits available
for correction. The point list remains editable before preheat and is not
silently rewritten while a patrol is active.

## Testing

- `web_remote_node` tests cover valid persistence, invalid payload rejection,
  and preservation of the existing YAML fields such as `cruise_speed`.
- `mission_control_node` tests cover entering a configured radius in `PATROL`,
  exactly-once handling per patrol, no trigger outside the radius, and no
  trigger in obstacle movement states.
- Mission tests cover publishing an override diagnosis only for a fixed-point
  event, with the selected class and a confidence in `[0.80, 0.90]`; visual
  detections retain real diagnosis behavior.
- Frontend static tests cover the section placement after the visual-logic
  controls, rule fields, HTTP endpoint use, and bottom-bar scrolling style.

## Non-Goals

- No new Nav2 waypoint, behavior-tree action, or navigation-goal mutation.
- No modification of YOLO thresholds, plant-detection stopping, camera control,
  or obstacle-avoidance trajectories.
- No simulated diagnosis in real visual plant detection flows.
