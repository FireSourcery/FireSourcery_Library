# Hierarchical State Machine Architecture for Motor Control
## Vehicle Application Layer Architecture

### Overview
This document describes a hierarchical state machine architecture that separates concerns between the **Motor Layer** (low-level motor control) and the **Application Layer** (vehicle-specific logic). The design uses a multi-layer state machine approach with clear abstraction boundaries.

---

## Architecture Layers

### Layer 1: Motor Layer (`Motor_StateMachine`)
**Purpose**: Low-level motor control - commutation, feedback, safety
**Scope**: Single motor hardware control
**Thread Context**: High-priority PWM thread (real-time)

#### Motor States
```
INIT → STOP → PASSIVE → RUN
         ↓      ↓         ↑
       FAULT ← CALIBRATION
         ↓
    OPEN_LOOP
```

| State | Description | Phase Output | Feedback |
|-------|-------------|--------------|----------|
| **INIT** | System initialization, sensor verification | Float | Off |
| **STOP** | Motor at zero speed, safe configuration state | Float/V0 | Off |
| **PASSIVE** | Freewheeling with direction set, no active control | Float | Off (observation only) |
| **RUN** | Active closed-loop control with feedback | PWM | On |
| **OPEN_LOOP** | Startup, alignment, or manual torque control | PWM | Partial |
| **CALIBRATION** | Sensor calibration, parameter identification | V0/PWM | Test |
| **FAULT** | Safety state, all outputs disabled | Float | Off |

#### Motor State Inputs
- `MSM_INPUT_FAULT` - Enter/exit fault state
- `MSM_INPUT_PHASE_OUTPUT` - Control phase output mode (Float/V0/PWM)
- `MSM_INPUT_FEEDBACK_MODE` - Select feedback type (encoder, sensorless, etc.)
- `MSM_INPUT_DIRECTION` - Set rotation direction (CW/CCW/NULL)
- `MSM_INPUT_OPEN_LOOP` - Enter open-loop control mode
- `MSM_INPUT_CALIBRATION` - Enter calibration procedures

#### Key Characteristics
- **Direction Management**: Direction (CW/CCW/NULL) is set at motor level
  - `MOTOR_DIRECTION_NULL` → STOP state (no direction set)
  - `MOTOR_DIRECTION_CW/CCW` → PASSIVE state (ready to run)
- **Speed-based Transitions**: Many transitions check `Motor_IsSpeedZero()`
- **Independent Operation**: Each motor operates independently
- **No Vehicle Context**: Layer has no knowledge of Park/Neutral/Drive concepts

---

### Layer 2: Application Layer (`MotorController_StateMachine`)
**Purpose**: System-level orchestration, multi-motor coordination
**Scope**: Overall system state and configuration
**Thread Context**: Main thread (lower priority)

#### MotorController States
```
INIT → PARK ⇄ MAIN → LOCK
        ↓      ↓       ↓
      FAULT ← ← ← ← ← ←
```

| State | Description | Motor States | Use Case |
|-------|-------------|--------------|----------|
| **INIT** | System boot, wait for sensor stabilization | STOP | Startup |
| **PARK** | Stationary safe state, system standby | STOP | Parking brake, standby |
| **MAIN** | Active operation mode (contains substates) | STOP/PASSIVE/RUN | Normal operation |
| **LOCK** | Blocking operations (calibration, NVM save) | STOP | Maintenance |
| **FAULT** | System-level fault protection | FAULT (motors) | Error handling |

#### Main State Modes (Substates)
- `MAIN_MOTOR_CMD` - Direct motor command mode
- `MAIN_VEHICLE` - Vehicle drive mode (see Layer 3)

#### MotorController Inputs
- `MCSM_INPUT_FAULT` - System fault handling
- `MCSM_INPUT_LOCK` - Enter locked/calibration operations
- `MCSM_INPUT_STATE_COMMAND` - System state commands (Park/Stop/Start)
- `MCSM_INPUT_MAIN_MODE` - Switch between Main substates

---

### Layer 3: Vehicle Drive Layer (`Vehicle_StateMachine`)
**Purpose**: Vehicle-specific drive logic (PRNDL equivalent)
**Scope**: Drive modes and direction management
**Thread Context**: Main thread

#### Vehicle States
```
NEUTRAL ⇄ DRIVE (Forward)
    ↕          ⇅
    → ← DRIVE (Reverse)
```

| State | Description | Motor Direction | Motor Control States | User Input |
|-------|-------------|-----------------|---------------------|------------|
| **NEUTRAL** | Vehicle in neutral, motors freewheeling or braking | Unchanged from previous | PASSIVE/RUN (brake only) | Brake only |
| **DRIVE** | Vehicle actively driving | FORWARD or REVERSE | PASSIVE/RUN | Throttle + Brake |

#### Vehicle Direction Mapping
```
Vehicle State      → Motor Direction    → Motor State
──────────────────────────────────────────────────────
NEUTRAL            → CW/CCW (retained)  → PASSIVE
DRIVE + FORWARD    → CW (FORWARD)       → RUN
DRIVE + REVERSE    → CCW (REVERSE)      → RUN
PARK (upper layer) → NULL               → STOP
```

#### Vehicle Inputs
- `VEHICLE_STATE_INPUT_DIRECTION` - Forward/Reverse/None
- `VEHICLE_STATE_INPUT_CMD_START` - Throttle/Brake/Release commands

#### Vehicle Commands
```c
typedef enum Vehicle_Cmd {
    VEHICLE_CMD_RELEASE,   // Freewheel (Float output)
    VEHICLE_CMD_BRAKE,     // Active braking
    VEHICLE_CMD_THROTTLE   // Active propulsion
} Vehicle_Cmd_T;
```

---

## Vehicle Use Case: Park/Neutral/Forward/Reverse Architecture

### State Hierarchy Overview
```
MotorController (System Level)
    │
    ├─ PARK State ────────────► Motors: STOP, Direction: NULL
    │                            Output: V0 or Float
    │                            Use: Parking brake, standby
    │
    └─ MAIN State
        │
        └─ VEHICLE Substate (Vehicle Level)
            │
            ├─ NEUTRAL ───────► Motors: PASSIVE, Direction: CW/CCW (retained)
            │                   Output: Float (or brake if commanded)
            │                   Use: Clutch disengaged, ready to engage
            │
            └─ DRIVE ─────────► Motors: RUN, Direction: CW or CCW
                │               Output: PWM (active control)
                ├─ Forward Mode  Direction: CW
                └─ Reverse Mode  Direction: CCW
```

### Detailed State Descriptions

#### PARK State (MotorController Layer)
**Purpose**: Ultimate safe state, vehicle parked
- **Entry Actions**:
  - Set all motors to `MOTOR_DIRECTION_NULL`
  - Transition motors to `MOTOR_STATE_STOP`
  - Apply mechanical brake if available
  - Optionally apply V0 (short circuit braking)

- **Characteristics**:
  - Speed must be zero to enter
  - No motor rotation allowed
  - Lowest power consumption state
  - Safe for configuration changes

- **Exit Conditions**:
  - User input (start command)
  - System validation checks pass
  - Transition to MAIN state

- **Motor State**: `STOP` (Direction = NULL)

#### NEUTRAL State (Vehicle Layer)
**Purpose**: Vehicle in neutral, motors not providing propulsion
- **Entry Actions**:
  - Motors remain in current direction (CW/CCW)
  - Activate `PHASE_OUTPUT_FLOAT` (coast)
  - Clear throttle commands

- **Characteristics**:
  - Motor direction is **preserved** from previous DRIVE state
  - Motors in `PASSIVE` state (freewheeling)
  - Can accept brake commands only
  - Vehicle can coast or be pushed
  - Speed monitoring continues (back-EMF observation)

- **Accepted Inputs**:
  - Brake: Transition motor to `RUN` state with regenerative/friction braking
  - Forward/Reverse: Check speed, then transition to DRIVE
  - Release: Maintain freewheeling

- **Transition to DRIVE**:
  1. Check if speed is zero
  2. If speed = 0: Set new direction, transition to DRIVE
  3. If speed ≠ 0 and direction matches: Transition to DRIVE
  4. If speed ≠ 0 and direction opposite: Remain in NEUTRAL until speed = 0

- **Motor State**: `PASSIVE` (Direction = CW or CCW retained)

#### DRIVE State (Vehicle Layer)
**Purpose**: Active vehicle propulsion
- **Entry Actions**:
  - Verify motor direction matches desired drive direction
  - Transition motors to `RUN` state
  - Enable active control (PWM output)
  - Match feedback state to current conditions

- **Substates**:
  - **Forward**: `MotorDirection = MOTOR_USER_DIRECTION_FORWARD (CW)`
  - **Reverse**: `MotorDirection = MOTOR_USER_DIRECTION_REVERSE (CCW)`

- **Characteristics**:
  - Full throttle and brake control active
  - Closed-loop feedback control
  - Regenerative braking available
  - Speed/torque limits enforced

- **Accepted Inputs**:
  - Throttle: Adjust motor speed/torque setpoint
  - Brake: Apply regenerative/friction braking (motor stays in RUN)
  - Release: Transition to freewheel or regen based on `ZeroMode`
  - Neutral: Transition to NEUTRAL state

- **Zero Throttle Behavior** (configurable):
  - `ZERO_MODE_FLOAT`: Coast (output float)
  - `ZERO_MODE_CRUISE`: Maintain speed
  - `ZERO_MODE_REGEN`: Light regenerative braking

- **Motor State**: `RUN` (Direction = CW or CCW)

---

## Transition Rules and Safety

### Direction Change Rules
```
Current State    Command        Speed    Action
────────────────────────────────────────────────────────────────
PARK            → Forward       = 0      Set Direction=CW, Go to DRIVE
PARK            → Reverse       = 0      Set Direction=CCW, Go to DRIVE

NEUTRAL         → Forward       = 0      Set Direction=CW, Go to DRIVE
NEUTRAL         → Forward       > 0      If Dir=CW: Go to DRIVE
                                        If Dir=CCW: Stay NEUTRAL (wait for stop)

NEUTRAL         → Reverse       = 0      Set Direction=CCW, Go to DRIVE
NEUTRAL         → Reverse       > 0      If Dir=CCW: Go to DRIVE
                                        If Dir=CW: Stay NEUTRAL (wait for stop)

DRIVE(Fwd)      → Reverse       any      Go to NEUTRAL, wait for stop
DRIVE(Rev)      → Forward       any      Go to NEUTRAL, wait for stop

DRIVE           → Neutral       any      Go to NEUTRAL (retain direction)
NEUTRAL         → Park          any      If speed=0: Go to PARK
```

### Safety Interlocks
1. **Speed Zero Check**: Direction changes require `Motor_IsSpeedZero() == true`
2. **Direction Mismatch**: Opposing direction at speed → forced NEUTRAL
3. **Fault Propagation**: Motor fault → MotorController FAULT → All motors stop
4. **Park Entry**: Can only enter PARK when all motors are in STOP state

---

## State Responsibilities

### Motor Layer Responsibilities
- ✓ Commutation timing and PWM generation
- ✓ Current/voltage feedback loops
- ✓ Speed estimation and control
- ✓ Motor-level fault detection (overcurrent, overheat)
- ✓ Direction management (CW/CCW/NULL)
- ✗ Does NOT know about Park/Neutral/Drive concepts

### MotorController Layer Responsibilities
- ✓ Multi-motor coordination
- ✓ System-level fault handling
- ✓ Park state management (parking brake)
- ✓ Configuration and calibration modes
- ✓ NVM operations (blocking)
- ✓ Boot sequence
- ✗ Does NOT implement vehicle drive logic directly

### Vehicle Layer Responsibilities
- ✓ Park/Neutral/Drive state machine
- ✓ Forward/Reverse direction logic
- ✓ Throttle/Brake command mapping
- ✓ Zero-throttle behavior (cruise/coast/regen)
- ✓ Direction change validation (speed checks)
- ✓ Motor state transitions (STOP→PASSIVE→RUN)

---

## Implementation Notes

### State Machine Features
1. **Hierarchical Structure**: Substates inherit from parent states
2. **Transition Tables**: Each state defines allowed input transitions
3. **Entry/Loop/Exit Actions**: Standard state callbacks
4. **Synchronous vs Asynchronous**:
   - Motor state proc: High-priority PWM thread
   - Application state proc: Main thread (lower priority)

### Thread Safety
- Motor layer inputs use critical sections for thread-safe access
- MotorController and Vehicle run on main thread (no contention)
- State transitions check motor state before proceeding

### Direction Change Flow Example
```c
// User shifts from Forward to Reverse at speed
1. Vehicle receives REVERSE input
2. Check: MotMotors_IsEvery(&motors, Motor_IsSpeedZero)
3. If FALSE: Remain in NEUTRAL, monitor speed
4. If TRUE:
   a. Set MotMotors_ApplyUserDirection(MOTOR_USER_DIRECTION_REVERSE)
   b. Motor transitions: PASSIVE → Direction=CCW
   c. Vehicle transitions: NEUTRAL → DRIVE(Reverse)
   d. Motor transitions: PASSIVE → RUN (on throttle input)
```

### Configuration Options
```c
// Zero throttle behavior in DRIVE state
typedef enum {
    VEHICLE_ZERO_MODE_FLOAT,   // Coast
    VEHICLE_ZERO_MODE_CRUISE,  // Maintain speed
    VEHICLE_ZERO_MODE_REGEN    // Regenerative braking
} Vehicle_ZeroMode_T;
```

---

## Advantages of This Architecture

### Separation of Concerns
- **Motor Layer**: Reusable across different vehicle types
- **MotorController Layer**: Adaptable to different motor counts/configurations
- **Vehicle Layer**: Vehicle-specific, easily customized

### Safety
- Multiple layers of fault checking
- Direction changes validated at multiple levels
- Park state enforced at system level

### Flexibility
- Can run motors without Vehicle (direct motor command mode)
- Easy to add new vehicle modes (e.g., Crawl, Sport, Eco)
- Configuration options for different vehicle behaviors

### Testability
- Each layer can be tested independently
- Mock interfaces between layers
- Clear state invariants

---

## Example State Sequences

### Startup Sequence
```
1. Power On
   └─ MotorController: INIT
       └─ Motors: INIT
           - Verify sensors
           - Load calibration
           - Check faults

2. Init Complete (no faults)
   └─ MotorController: PARK
       └─ Motors: STOP (Direction=NULL)

3. User presses "Start"
   └─ MotorController: MAIN → VEHICLE
       └─ Vehicle: NEUTRAL
           └─ Motors: PASSIVE (Direction=NULL → set on demand)
```

### Forward Drive Sequence
```
1. From NEUTRAL
   └─ User selects Forward
       ├─ Check speed = 0? YES
       ├─ Set Motor Direction = CW
       └─ Vehicle: NEUTRAL → DRIVE(Forward)
           └─ Motors: PASSIVE (Dir=CW)

2. User presses Throttle
   └─ Vehicle sends throttle command
       └─ Motors: PASSIVE → RUN
           - Enable PWM output
           - Start closed-loop control

3. User releases Throttle
   └─ Behavior depends on ZeroMode:
       - FLOAT: Motor → PASSIVE (coast)
       - CRUISE: Motor stays RUN (speed hold)
       - REGEN: Motor stays RUN (light braking)
```

### Emergency Stop Sequence
```
1. Fault Detected (any layer)
   └─ Motor: FAULT (immediate)
       - Phase Float
       - Disable PWM

2. Fault Propagates Up
   └─ MotorController: FAULT
       - All motors → FAULT
       - Beep alarm
       - Log fault

3. User Clears Fault
   └─ Check fault conditions cleared
       - If OK: → PARK state
       - If NOT: Remain FAULT
```

---

## API Examples

### Motor Layer
```c
// Set motor direction (from Vehicle)
Motor_FOC_SetDirection(p_motor, MOTOR_USER_DIRECTION_FORWARD);

// Check if ready for direction change
bool isZero = Motor_IsSpeedZero(p_motor);

// Transition motor states (via state machine inputs)
StateMachine_ApplyInput(&motor->STATE_MACHINE,
                        MSM_INPUT_DIRECTION,
                        MOTOR_DIRECTION_CW);
```

### Vehicle Layer
```c
// User selects Forward
Vehicle_StateMachine_ApplyDirection(p_drive, MOTOR_USER_DIRECTION_FORWARD);

// Get current drive state
Motor_Direction_T dir = Vehicle_StateMachine_GetDirection(p_drive);

// Check state
if (StateMachine_IsActiveStateId(p_drive->STATE_MACHINE,
                                 VEHICLE_STATE_ID_NEUTRAL))
```

### MotorController Layer
```c
// Enter park state
MotorController_EnterPark(p_controller);

// Check if in park
bool inPark = MotorController_IsState(p_controller,
                                                    MCSM_STATE_ID_PARK);
```

---

## Future Enhancements

### Additional Vehicle States
- **Crawl Mode**: Ultra-low speed for precise maneuvering
- **Sport Mode**: Aggressive throttle response
- **Eco Mode**: Efficiency-optimized control
- **Hill Hold**: Automatic brake on inclines

### Advanced Features
- **Traction Control**: Individual motor speed adjustment
- **Torque Vectoring**: Differential torque for cornering
- **Auto Park**: Automated parking sequence
- **Launch Control**: Optimized acceleration from standstill

---

## Summary

This architecture provides a clean, hierarchical separation between:
1. **Motor Layer**: Hardware control and safety
2. **MotorController Layer**: System coordination
3. **Vehicle Layer**: Vehicle behavior

The design allows for:
- Safe operation with multiple interlock levels
- Flexible vehicle behavior customization
- Reusable motor control code
- Clear state invariants and transition rules

The Park/Neutral/Forward/Reverse states are implemented at the appropriate layers, with direction management bridging between the vehicle concepts (P/N/D/R) and motor concepts (NULL/CW/CCW).
