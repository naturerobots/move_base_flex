# Open Issues: MBF Goal Actions Panel

Known thread-safety and robustness issues in `src/mbf_goal_actions_panel.cpp`, found during the
review of the plan refiner integration. They are ranked by how likely they are to occur and how
severe the impact is.

## Threading model

To follow the issues below, keep in mind which code runs on which thread:

- **GUI thread (Qt):** the `update*ActionClient` slots, the stop buttons, property change
  handlers, and lambdas posted through `QMetaObject::invokeMethod`.
- **Executor thread:** everything triggered by `ros_node_`. That includes `newGoalCallback`,
  goal response, feedback, result and cancel callbacks, and parameter client callbacks.
- **Connection check threads (3):** poll `action_server_is_ready()` every 500 ms. When a server
  is not ready, they queue `update*ActionClient` on the GUI thread.

## 1. Stop Controller / Stop Planner can do nothing

*Functional bug. Likely to happen in normal use. Relevant to operator safety.*

A new goal is sent while an older one is still active. `dispatchExePath` and `newGoalCallback`
then cancel the old goal and send the new one. The CANCELED result of the old goal can arrive
after the new goal was accepted. `exePathResultCallback` (or `getPathResultCallback`) then
resets `goal_handle_exe_path_` (or `goal_handle_get_path_`), which at that point holds the new
goal.

Effect: the new goal keeps running, but the stop button reports "No controller goal to stop".
The status text can also show "Cancelled" while the robot is still driving.

Repro: send a goal. While the robot is driving, send a second goal. Then press
Stop Controller.

Fix: check the goal id in the result callback. Ignore results that do not belong to the
current goal handle, as the refiner chain already does with `refine_chain_id_`.

## 2. GUI freezes while an action server is down

*Likely to happen. Recovers when the server is back.*

Each connection check thread queues `update*ActionClient` on the GUI thread every 500 ms. Each
call blocks for about 1–2 s (`wait_for_action_server` 1 s, `wait_for_service` 1 s), so the
queue grows faster than it drains. Three threads now do this (planner, refiner, controller).
Each call also holds its client mutex, so executor callbacks that need the same mutex wait too.

Repro: configure the panel, then kill `move_base_flex`. rviz becomes sluggish or freezes.

Fix: add an atomic "update pending" flag per client, so at most one call is queued at a time.

## 3. Planner and controller goal handles have no lock

*Can crash. Rare.*

`goal_handle_get_path_` and `goal_handle_exe_path_` are written on the executor thread (goal
response, result and cancel callbacks). They are also read and reset on the GUI thread (stop
buttons, `update*ActionClient`). Writing the same `std::shared_ptr` from two threads at once is
a data race and can corrupt the reference count.

This needs a stop or server change at the same moment a callback runs, so the time window is
small.

The not-set branches of `updateGetPathActionClient` and `updateExePathActionClient` have the
same problem. Their cancel callbacks reset `action_client_*` on the executor thread without
the mutex.

Fix: handle all goal and result callbacks on the GUI thread (queued
`QMetaObject::invokeMethod`). Then the goal handles are only used on one thread. This also
fixes #1 and #4.

## 4. The wait loop on server change can freeze rviz for good

*Rare. Permanent when it happens.*

On a server change, `updateGetPathActionClient` and `updateExePathActionClient` spin in
`while(goal_handle_…)` on the GUI thread. They hold the client mutex and have no timeout, and
they wait for the executor thread to deliver the cancel response. rviz hangs forever if:

- the old server dies during the change, so no cancel response ever arrives, or
- the executor thread is blocked in `newGoalCallback` or `sendGetPathGoal` waiting for the same
  mutex. Both threads then wait on each other forever (deadlock).

Fix: add a timeout to the loop, or remove the loop once #3 is done. The refiner client already
works without waiting.

## 5. Exceptions in the parameter callbacks abort rviz

*Depends on the robot config. Always happens when the config matches.*

The parameter callbacks for `planners`, `controllers` and `plan_refiners` have two unguarded
calls:

- `future.get()` throws if the service call fails.
- `as_string_array()` throws if the parameter is declared but not set, or has another type.

The exception reaches the executor thread or the Qt event loop and ends in `std::terminate`.
If a config hits this, rviz crashes on every connect, so it is easy to spot.

Fix: add a try/catch in the three callbacks and show an error status instead.

## 6. Cosmetic issues

- `RCLCPP_WARN(..., "SETTING GOAL HANDLE")` in `sendExePathGoal` is leftover debug output.
- The connection check threads set the server status to "ready" every 500 ms. This overwrites
  intermediate states such as "updating planner list...".

## Recommended order

1. Fix #1 and #2. Both are small and happen in normal use.
2. Fix #3 and #4 together as one refactor: handle the callbacks on the GUI thread.
3. Fix #5 at any time. It is a small, independent change.

## Already fixed

These issues were fixed in the refiner review and are listed here for reference:

- Double free in `updateRefinerProperties`. `removeChildren()` already deletes the child
  properties, so the code must not delete them again.
- `async_cancel_goal` throws `UnknownGoalHandleError` if the goal finished just before the
  cancel. All cancel calls now go through `try_cancel_goal<>`.
- The refine chain state is only accessed under `refine_path_action_client_mutex_`.
- Callbacks of stopped or replaced refine chains are dropped (`refine_chain_id_`). A new plan
  stops the running refiner goal.
- A rejected refine goal ends the chain instead of leaving it stuck.
- `save()` keeps the refiner selection restored by `load()` if the refiner list has not
  arrived yet.
- The wait loop on a planner server change no longer spins forever after a failed cancel.
