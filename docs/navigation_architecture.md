# Navigation architecture

The autonomous navigation path has three owners:

```text
Mission -> Wombat -> Nav2
```

## Mission

Mission owns the search region, lane sequence, weed tracking, target selection,
handled and unreachable weed bookkeeping, and search-specific recovery. It
requests or returns normal lane and recovery goals through `wombat_next_goal`.

When a new weed target is selected, Mission publishes one complete
`rs1/msg/WombatWeedGoal` message on `/wombat/weed_goal`. The message includes the
map-frame pose, weed ID, and mission generation. A target is published once per
selection, not on every perception update.

Mission publishes `rs1/msg/WombatStatus` on `/wombat/status`. A region update
increments `mission_generation`, invalidates previous goals, and publishes
`region_reset=true`. Mission completion publishes `mission_complete=true`.

## Wombat

Wombat owns the Nav2 `NavigateToPose` action lifecycle. It sends one normal
goal at a time, keeps the lane-end goal active, and explicitly cancels that
goal before sending a weed goal. After the weed goal succeeds, Wombat reports
`WEED_HANDLED` and Mission returns the original search progression.

Wombat publishes its current lifecycle state on `/wombat/state` as
`rs1/msg/WombatState`. The message contains `state` and `mission_generation`;
the state constants are `WAITING_FOR_REGION`, `APPROACHING_START`,
`PRE_LANE_WAIT`, `TURNING`, `SEARCHING`, `RECOVERY`, `COMPLETE`, and `ABORT`.
The topic is transient-local and is refreshed once per second, so a GUI
subscriber receives the current state even if it connects between transitions.

The Glider publishes human-readable processing text on `/glider/status` as
`std_msgs/msg/String`. It also publishes its typed lifecycle state on
`/glider/state` as `rs1/msg/GliderState`. The state constants are `STANDBY`,
`GOTO_POO`, `DESCEND`, `ALIGN`, `FINAL_DESCEND`, `PICKUP`, `CLEAR_EXIT`,
`GOTO_TRUCK`, `DROP_OFF`, `RETURN_HOME`, and `LAND`; the message also reports
whether it is carrying poo. This topic is transient-local and is refreshed by
the control loop.

Wombat owns the Nav2 retry count. Nav2 has no recovery nodes in the active
behavior tree, so a planning or control failure aborts the action. After
retries are exhausted, Wombat reports a lane skip or unreachable weed to
Mission; Mission decides how the search plan advances.

A new mission generation cancels the active Nav2 goal and starts a fresh goal
request. Stale service responses and stale Nav2 results are ignored.

## Nav2

Nav2 owns global planning, local control, obstacle avoidance, goal tolerances,
and path following. The active behavior tree contains only `ComputePathToPose`
and `FollowPath`; it does not clear costmaps, spin, wait, back up, or otherwise
perform recovery. Wombat does not publish velocity commands and does not
implement path following.

Teleoperation may still publish directly to the robot's `cmd_vel` topic for
manual simulation control; that is separate from autonomous Wombat behavior.
