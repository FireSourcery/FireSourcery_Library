# Motor Control State Machines, the detail

The design behind [StateMachineOverview.md](StateMachineOverview.md): the engine conventions both machines rely on,
and the decisions, with the options rejected.

## Engine conventions both machines rely on

From [Architecture.md](../Framework/StateMachine/Architecture.md):

- One leaf pointer per machine; the active states are the leaf and its parents. No orthogonal regions, no history.
- A handler returns `NULL` to stay, which may still change context (an internal transition); the current state, to
  re-enter it; or another state, to exit up to the common ancestor and enter down from it.
- Inputs walk from the leaf to the root. An unmapped slot defers to the parent. A child that maps an input and
  returns `NULL` refuses it for its whole branch.
- `NEXT` is the clock transition, run after `LOOP`, leaf first: while a node's `NEXT` returns `NULL`, its parent's
  `LOOP` and `NEXT` run in the same cycle. That is how `SAFE_STOP` runs `SPEED_ZERO`'s loop.
- Actions (`P_ACTION_TABLE`) reach the leaf only; a child that needs its parent's action shares the table.
- Sub-states are `const State_T` in the file that owns them, with `P_TOP` set to a published root. The machine
  definition does not list them.
- `EXIT` is compiled out (`STATE_MACHINE_EXIT_FUNCTION_ENABLE` is not defined), so transition work is in `ENTRY`.
  Returning from a leaf to its own root does not re-run the root's `ENTRY`.
- A sub-state id is unique only among its siblings; `*_GetPathId` packs the whole path, 4 bits per level.

## Design decisions

### State design rules

- A new state is justified by a different safety envelope, not a different purpose.
- One-shot automated procedures get a state; interactive parameter adjustment does not, unless it changes the
  safety envelope: `MOTOR_TUNING` admits PID writes with the motors running, and its exits keep or reload the live
  gains.
- Orthogonal concerns get orthogonal mechanisms: what control structure is running is a separate concern from
  what input source is connected. So the feedback mode is data (`Motor_FeedbackMode_T`), not a state, and so are the
  input source (`Config.InputMode`) and the `STANDBY` exit policy (`StandbyExitMode`).
- `LOCK` is for operations the operator must commit to and acknowledge as risky, outside normal operation.
- The operator's use case does not conflate with the system's state.

### Controller

- **The root does not know app states or inputs.** An app mounts its sub-states under `MC_STATE_MAIN` through
  `MotorController_App_T`: `ENTER_MAIN` picks the entry sub-state, `PROC_ANALOG_USER` maps the analog inputs,
  `INIT` loads its config. Its inputs share one id, `MC_STATE_INPUT_APP_USER`, whose value is the app's own enum.
  The app runs in the controller's machine rather than a nested one, so its sub-states inherit `MAIN`'s handlers
  for state commands and faults. A nested machine on a narrower context (`Motor_Table_T`) would give the app its
  own input alphabet; it remains the alternative. The app is chosen at compile time (`P_APP`), in a shape that
  allows a runtime choice later.
- **One parked state.** `STANDBY` belongs to the root, so every app shares it.
- **Inputs are buffered, not events.** The user may select a direction while parked; the controller stores it, and
  entering `MAIN` uses it. A direction selection does not cause a transition. `STANDBY` accepts state commands
  only: motor and app inputs are buffered by their callers and consumed when `MAIN` is entered.
- **Safe landings.** `LOCK` exit and fault clear always land in `STANDBY`, never in live `MAIN`, and `MAIN` is left
  only with the motors stopped.
- **Exit policy as data.** The `STANDBY` exit policy, `StandbyExitMode`, is runtime data resolved at init from the
  input topology. That keeps the input mode and the Park input out of the machine. An explicit park latches
  `MANUAL`, so a host that parked holds `STANDBY`; a reboot restores the resolved mode.
- **Write admission belongs to the controller** ([0001](../Docs/Decisions/0001_ParameterWriteAdmission.md)).
  Whether a write is safe now is lifecycle reasoning that applies to any parameter, so the controller decides it
  from its own state. The motor machine gates outputs, the electrical hazard, and nothing else.

### Motor

- **Direction does not drive transitions.** A layer above must be able to store a direction while it holds its own
  safe state: a state is left on the loss of any one condition, but entering one can take more than one. So setting
  a direction never starts the motor; `PWM` does, given a direction and feedback. `DEACTIVATED` resets the
  direction. `NULL` could double as a disable, but no other value could act as an enable, so it does not.
- **Intervention is a motor-level supervisor**: active control on its own path, with the user's command taken out
  of the loop. Its states are named by the quantity forced to zero, not by the mechanism; `SPEED_RAMP_ZERO` was
  rejected because the ramp's slope is a parameter. The nearest standard stop functions:

| Here | Nearest standard stop function |
|---|---|
| `PASSIVE`, floating | Safe Torque Off (IEC 61800-5-2), without a safety-rated channel |
| `PASSIVE` or `FAULT` holding V0 above the freewheel limit | active short circuit |
| `SPEED_ZERO` | CiA 402 Halt: decelerate, stay enabled, resumable |
| `SAFE_STOP` | SS1-d (IEC 61800-5-2), CiA 402 Quick Stop option 1; no safety-rated monitoring channel |
| `TORQUE_ZERO` | none: it never converges, it inhibits motoring |

- **The freewheel limit is the rated speed**, where the back-EMF reaches the nominal voltage. Below it a floating
  bridge is safe; above it the back-EMF rectifies into the bus, uncontrolled. So above it a release decelerates
  through `SPEED_ZERO`, and `PASSIVE` and `FAULT` hold V0.
- **Fault is an ordinary root**, not the engine's fault-state template. Its input carries the set and clear deltas
  in one value (`Motor_FaultCmd_T`, `MotorController_FaultCmd_T`), so no caller reads, modifies and passes back the
  whole flag word. Flags alone are inert outside `INIT`: the transition into `FAULT` is what stops the motor, so a
  procedure that finds a fault returns `FAULT` itself. A clear re-polls the conditions before leaving.
- **Each driving state writes its own duty**, in its `LOOP`, right after computing it. Hold states write nothing,
  and the PWM thread writes no duty. A thread-level write had overwritten V0 with the last vector, so a `FAULT`
  entry above rated speed re-applied the last `RUN` voltage; deciding in the thread by reading the duty back is
  racy, since a duty written in the same ISR loads only at the next load point.
- **Bumpless entry.** `RUN` and `INTERVENTION` resolve their limits first, then match the loops to the present
  output. A feedback-mode change re-enters `RUN` instead of passing through `PASSIVE`, so the loops re-match
  without coasting.
- **Feedback mode as data.** `RUN` reads the feedback mode; open loop is a branch of its own (`OPEN_LOOP`). `RUN`
  runs the current loop in every mode, so voltage modes are not sent.

### Inputs and threads

- **One input per subtree.** A sub-state entered from many sources is reached through its subtree's input, which
  carries a `State_T *` (`OPEN_LOOP`, `CALIBRATION`, `INTERVENTION`). The tree does the acceptance: sources that
  share a parent take the parent's handler, which checks `P_TOP`, and an exception overrides at the child
  (`SAFE_STOP` refuses). A target already on the active branch is a no-op, so repeated requests hold without
  re-entry; the brake hold sends `Motor_ApplySpeedZero` with every brake value. `NULL` is the branch's exit, taken
  inside the branch only; every handler outside it refuses `NULL`. Rejected:
  - engine-iterated source lists of `StateMachine_TransitionCmd_T`: acceptance moves out of the states, exclusions
    must be listed, and every handler needs a repeat guard;
  - a request enum: it only renames the sub-states. Inside the firmware the pointer is the id; enums belong at the
    wire, as the var ids;
  - a whitelist of pointers: it loses to the structural `P_TOP` check;
  - one generic target-state input for every subtree: it brings the whitelist back into `PASSIVE` and
    `DEACTIVATED`.
- **Transition commands for single sources.** A `StateMachine_TransitionCmd_T` names its start state, acts only
  while that state is on the active path, and needs no input id. `Motor_Enable`, `Motor_Disable`, and every
  procedure started from inside its branch use one.
- **The controller's inputs run on its proc thread**, so they need no lock. The motor's inputs cross from the main
  thread to the PWM thread, so they apply under the engine's lock: `STATE_MACHINE_ASYNC_CRITICAL` masks IRQs for
  the handler and the transition, and the PWM ISR never runs between an exit and an entry. `Motor_StateMachine.h`
  refuses a lock-free build: the engine runs the new entry before it moves the leaf, so the old `LOOP` could run once
  more and overwrite a `FAULT` entry's V0 with the last `RUN` duty. A fault set from the controller's 1 ms timer
  thread can be lost to a transition it interrupted; the 1 s re-send of the latched flags covers it.
- **Motors are polled, not notified.** A motor stops on its own fault at once; the controller's poll only moves the
  system state.

### Layering

- **The app sends the motor signed commands in the motor's frames and never sets its state.** The motor owns
  continuity through zero speed: its direction limits the voltage to a half-plane, which blocks plugging. How much
  plugging to allow near zero is the motor's to decide, behind the same contract.
