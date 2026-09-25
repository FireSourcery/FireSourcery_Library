# Conventional Hierarchical State Machine Architecture for Motor Control
## Industry-Standard Approach for Vehicle Applications

### Overview
This document presents a conventional, industry-standard approach to hierarchical state machines for motor control in vehicle applications. It follows established patterns from automotive, industrial automation, and robotics domains.

---

## Conventional Architecture Patterns

### UML Statechart Approach
The conventional approach follows **UML Statecharts** (Harel Statecharts) principles:
- Hierarchical (nested) states
- Concurrent (orthogonal) regions
- History states
- Entry/Exit actions
- Internal transitions

---

## Layer Architecture

### Conventional Three-Layer Model

```
┌─────────────────────────────────────────────────┐
│         Vehicle/Application Layer               │
│    (Gear Selector: P/R/N/D/L/S)                │
│    - User intent interpretation                 │
│    - Safety interlocks                          │
│    - Mode selection                             │
└────────────────┬────────────────────────────────┘
                 │ Commands: Park, Drive_Fwd, Drive_Rev, Coast
                 │ Status: Gear, Speed, Ready
                 ▼
┌─────────────────────────────────────────────────┐
│         Motion Control Layer                    │
│    (Drive Controller State Machine)            │
│    - Trajectory planning                        │
│    - Multi-motor coordination                   │
│    - Torque/Speed arbitration                   │
└────────────────┬────────────────────────────────┘
                 │ Commands: Enable, Speed_Ref, Torque_Ref
                 │ Status: State, Actual_Speed, Faults
                 ▼
┌─────────────────────────────────────────────────┐
│         Servo/Motor Layer                       │
│    (Motor Drive State Machine)                 │
│    - Commutation                                │
│    - Current control                            │
│    - Position/Speed feedback                    │
└─────────────────────────────────────────────────┘
```

---

## Standard Motor Drive States (Servo Layer)

### IEC 61800-7 / PLCopen Standard States

The industry standard (PLCopen Motion Control) defines these states:

```
                    ┌──────────────┐
                    │  NOT_READY   │ (Power-up, initialization)
                    └──────┬───────┘
                           │ Power OK + Init Complete
                           ▼
    ┌──────────────────────────────────────┐
    │      SWITCH_ON_DISABLED              │ (Power stage disabled)
    └──────┬───────────────────────────────┘
           │ Enable Command
           ▼
    ┌──────────────────────────────────────┐
    │         READY_TO_SWITCH_ON           │ (Power stage ready)
    └──────┬───────────────────────────────┘
           │ Switch On
           ▼
    ┌──────────────────────────────────────┐
    │         SWITCHED_ON                  │ (Power on, no drive)
    └──────┬───────────────────────────────┘
           │ Enable Operation
           ▼
    ┌──────────────────────────────────────┐
    │      OPERATION_ENABLED               │ (Active control)
    └──────┬───────────────────────────────┘
           │
           ├─ Quick Stop ──→ QUICK_STOP_ACTIVE
           │
           └─ Fault ──────→ FAULT
                            │
                            └─ Fault Reset → SWITCH_ON_DISABLED
```

### State Descriptions (PLCopen/IEC Standard)

| State | Code | Description | Power Stage | Control |
|-------|------|-------------|-------------|---------|
| **NOT_READY** | 0 | Initialization, self-test | Off | Disabled |
| **SWITCH_ON_DISABLED** | 1 | Ready for power-on | Disabled | Disabled |
| **READY_TO_SWITCH_ON** | 2 | Pre-operational | Enabled | Disabled |
| **SWITCHED_ON** | 3 | Power applied, ready for motion | Enabled | Standby |
| **OPERATION_ENABLED** | 4 | Normal operation | Enabled | Active |
| **QUICK_STOP_ACTIVE** | 5 | Controlled emergency stop | Enabled | Stopping |
| **FAULT_REACTION** | 6 | Fault detected, safe shutdown | Enabled | Stopping |
| **FAULT** | 7 | Fault state, awaiting reset | Disabled | Disabled |

---

## Conventional Vehicle Application States

### Standard Transmission States (PRNDL)

```
                    ┌──────────┐
                    │   PARK   │ (Mechanical lock)
                    └────┬─────┘
                         │
          ┌──────────────┼──────────────┐
          │              │              │
          ▼              ▼              ▼
    ┌─────────┐    ┌─────────┐    ┌─────────┐
    │ REVERSE │◄───│ NEUTRAL │───►│ DRIVE   │
    └─────────┘    └─────────┘    └────┬────┘
                         ▲              │
                         │              ▼
                         │         ┌─────────┐
                         └─────────│  LOW    │ (Low gear)
                                   └─────────┘
```

### Conventional State Definitions

#### PARK (P)
**Purpose**: Vehicle immobilized, safe for exit
- **Characteristics**:
  - Mechanical parking pawl engaged (if available)
  - Motor drive: SWITCH_ON_DISABLED or SWITCHED_ON
  - Zero torque command
  - Brake applied (electronic parking brake)
  - Highest safety level

- **Entry Conditions**:
  - Vehicle speed = 0
  - Brake pedal pressed (or EPB activated)
  - Ignition can be turned off

- **Exit Conditions**:
  - Brake pedal pressed
  - Valid drive request
  - Ignition on

#### REVERSE (R)
**Purpose**: Backward motion
- **Characteristics**:
  - Motor direction: Reverse
  - Motor state: OPERATION_ENABLED
  - Speed limited (typically 15-25 km/h)
  - Backup sensors/camera active
  - Audible warning may be active

- **Entry Conditions**:
  - From NEUTRAL
  - Vehicle speed = 0 (or very low forward speed with safety check)
  - Brake pressed during shift

- **Exit Conditions**:
  - Shift to N/D
  - Fault detected
  - Speed must be zero to shift to forward

#### NEUTRAL (N)
**Purpose**: No power transmission, free rolling
- **Characteristics**:
  - Motor state: SWITCHED_ON (ready but not commanding torque)
  - Zero torque command
  - Motors can freewheel
  - Brake control available
  - Used for towing, coasting, or waiting at lights

- **Entry Conditions**:
  - From any gear at any speed (emergency capability)
  - User selects neutral

- **Exit Conditions**:
  - Brake pressed + valid gear selection
  - Speed requirements met for target gear

#### DRIVE (D)
**Purpose**: Forward motion, normal driving
- **Characteristics**:
  - Motor direction: Forward
  - Motor state: OPERATION_ENABLED
  - Full torque/speed range available
  - Regenerative braking active
  - Most commonly used state

- **Entry Conditions**:
  - From NEUTRAL or PARK
  - Vehicle speed = 0 (or low reverse speed with safety check)
  - Brake pressed during shift

- **Exit Conditions**:
  - Shift to N/R/P
  - Fault detected

#### LOW (L) / SPORT (S) - Optional
**Purpose**: Modified drive characteristics
- **LOW**: Increased regenerative braking, lower speed limit
- **SPORT**: Higher performance, reduced efficiency focus
- These are typically sub-modes of DRIVE

---

## Hierarchical State Structure (Conventional)

### Composite State Pattern

```
┌─────────────────────────────────────────────────────────────┐
│                    SYSTEM (Top Level)                        │
│  ┌───────────┐  ┌──────────────────────────────────────┐   │
│  │   INIT    │──│         OPERATIONAL                   │   │
│  └───────────┘  │  ┌────────────────────────────────┐  │   │
│                 │  │        PARK/STANDBY             │  │   │
│                 │  └────────────────────────────────┘  │   │
│                 │  ┌────────────────────────────────┐  │   │
│                 │  │         DRIVE_READY             │  │   │
│                 │  │  ┌──────────┐  ┌─────────────┐ │  │   │
│                 │  │  │ NEUTRAL  │  │   DRIVING   │ │  │   │
│                 │  │  └──────────┘  │ ┌─────────┐ │ │  │   │
│                 │  │                │ │ FORWARD │ │ │  │   │
│                 │  │                │ ├─────────┤ │ │  │   │
│                 │  │                │ │ REVERSE │ │ │  │   │
│                 │  │                │ └─────────┘ │ │  │   │
│                 │  │                └─────────────┘ │  │   │
│                 │  └────────────────────────────────┘  │   │
│                 └──────────────────────────────────────┘   │
│  ┌───────────┐                                             │
│  │   FAULT   │◄────────────────────────────────────────────│
│  └───────────┘                                             │
└─────────────────────────────────────────────────────────────┘
```

### Inheritance of State Behavior
- Child states inherit entry/exit actions from parents
- Can override parent behaviors
- Parent state's guards apply to all children

---

## Conventional Transition Rules

### Standard Safety Interlocks

#### Speed-Based Interlocks
```
Transition          Speed Condition         Additional Conditions
─────────────────────────────────────────────────────────────────
P → R               v = 0                   Brake pressed
P → N               v = 0                   Brake pressed
P → D               v = 0                   Brake pressed

N → R               v ≤ v_threshold_rev     Brake pressed
N → D               v ≤ v_threshold_fwd     Brake pressed
N → P               v = 0                   Brake pressed

D → R               v = 0 *                 Safety timeout
R → D               v = 0 *                 Safety timeout

D → N               Any speed               User command
R → N               Any speed               User command

* Industry standard: Direct D↔R requires v=0
  Some systems allow "shift-on-fly" with torque interruption
```

#### Brake Interlocks (Shift Lock)
- **Shift Lock**: Brake pedal must be pressed to shift out of Park
- **Ignition Interlock**: Some shifts require ignition in "RUN" position
- **Seat Belt Interlock**: Optional safety feature

#### Direction Change Protection
Conventional systems use one of these approaches:

1. **Hard Interlock** (Most Common):
   - Require complete stop (v = 0) before opposite direction
   - System rejects command until stopped
   - Safer but less responsive

2. **Torque Reduction with Speed Check**:
   - If opposite direction commanded at speed:
     - Reduce torque to zero
     - Apply moderate braking
     - Wait for v < threshold (e.g., 5 km/h)
     - Then allow direction change

3. **Soft Interlock** (Performance vehicles):
   - Allow direction change at very low speeds (< 3 km/h)
   - Perform torque blending during transition
   - Higher complexity, requires careful tuning

---

## State Machine Implementation Patterns

### Pattern 1: Switch-Case State Machine (Simple)
```c
typedef enum {
    STATE_INIT,
    STATE_PARK,
    STATE_NEUTRAL,
    STATE_DRIVE,
    STATE_REVERSE,
    STATE_FAULT
} VehicleState_t;

void vehicle_state_machine(Event_t event) {
    static VehicleState_t current_state = STATE_INIT;
    VehicleState_t next_state = current_state;

    switch (current_state) {
        case STATE_PARK:
            next_state = park_state_handler(event);
            break;
        case STATE_NEUTRAL:
            next_state = neutral_state_handler(event);
            break;
        // ... other states
    }

    if (next_state != current_state) {
        state_exit(current_state);
        current_state = next_state;
        state_entry(current_state);
    }
}
```

### Pattern 2: State Table (Flexible)
```c
typedef struct {
    VehicleState_t current_state;
    Event_t event;
    VehicleState_t next_state;
    bool (*guard)(void);  // Transition condition
    void (*action)(void); // Transition action
} StateTransition_t;

const StateTransition_t transition_table[] = {
    // From     Event        To        Guard           Action
    {STATE_PARK, EVT_SELECT_D, STATE_DRIVE, check_speed_zero, engage_drive},
    {STATE_PARK, EVT_SELECT_N, STATE_NEUTRAL, check_brake, disengage_park},
    // ... more transitions
};
```

### Pattern 3: Function Pointer State (Object-Oriented)
```c
typedef struct State State_t;
struct State {
    void (*entry)(void);
    void (*exit)(void);
    State_t* (*handle_event)(Event_t);
    State_t* parent; // For hierarchical states
};

State_t state_park = {
    .entry = park_entry,
    .exit = park_exit,
    .handle_event = park_handle_event,
    .parent = &state_operational
};
```

### Pattern 4: Hierarchical State Machine (QP Framework Style)
```c
typedef enum {
    RET_HANDLED,
    RET_SUPER,     // Defer to parent
    RET_TRAN       // State transition
} StateReturn_t;

StateReturn_t park_state(Vehicle_t *me, Event_t *e) {
    switch (e->sig) {
        case ENTRY_SIG:
            engage_parking_brake();
            return RET_HANDLED;

        case EXIT_SIG:
            release_parking_brake();
            return RET_HANDLED;

        case SELECT_DRIVE_SIG:
            if (speed_is_zero() && brake_pressed()) {
                return TRANSITION(&drive_state);
            }
            return RET_HANDLED;

        default:
            return RET_SUPER(&operational_state); // Defer to parent
    }
}
```

---

## Motor-Level State Machine (Conventional)

### Standard Servo Drive States

```c
typedef enum {
    MOTOR_NOT_READY,
    MOTOR_DISABLED,
    MOTOR_READY,
    MOTOR_ENABLED,
    MOTOR_FAULT,
    MOTOR_QUICK_STOP
} MotorState_t;

typedef enum {
    MOTOR_CMD_SHUTDOWN,
    MOTOR_CMD_SWITCH_ON,
    MOTOR_CMD_ENABLE,
    MOTOR_CMD_QUICK_STOP,
    MOTOR_CMD_DISABLE,
    MOTOR_CMD_FAULT_RESET
} MotorCommand_t;
```

### Control Word Pattern (CANopen/EtherCAT Standard)
```c
// Following DS402 (CiA) standard control word
typedef union {
    uint16_t word;
    struct {
        uint16_t switch_on         : 1; // Bit 0
        uint16_t enable_voltage    : 1; // Bit 1
        uint16_t quick_stop        : 1; // Bit 2
        uint16_t enable_operation  : 1; // Bit 3
        uint16_t fault_reset       : 1; // Bit 7
        // ... other bits
    } bits;
} ControlWord_t;

// Status word for feedback
typedef union {
    uint16_t word;
    struct {
        uint16_t ready_to_switch_on : 1;
        uint16_t switched_on        : 1;
        uint16_t operation_enabled  : 1;
        uint16_t fault              : 1;
        uint16_t voltage_enabled    : 1;
        uint16_t quick_stop         : 1;
        // ... other bits
    } bits;
} StatusWord_t;
```

---

## Direction Management (Conventional Approaches)

### Approach 1: Separate Direction State
```c
typedef enum {
    DIR_NONE,      // Not moving, no direction set
    DIR_FORWARD,
    DIR_REVERSE
} Direction_t;

typedef struct {
    VehicleState_t gear_state;
    Direction_t direction;
    MotorState_t motor_state;
    float speed_cmd;
    float torque_cmd;
} VehicleControl_t;

// Direction is independent from gear state
// Neutral can have a direction "memory"
```

### Approach 2: Direction Embedded in Gear State
```c
typedef enum {
    GEAR_PARK,
    GEAR_NEUTRAL_NONE,    // No direction memory
    GEAR_NEUTRAL_FWD,     // Neutral but last was forward
    GEAR_NEUTRAL_REV,     // Neutral but last was reverse
    GEAR_DRIVE,
    GEAR_REVERSE
} GearState_t;

// State encodes both gear and direction history
```

### Approach 3: Two Orthogonal State Machines
```c
// Concurrent regions (UML statechart)
typedef struct {
    // Region 1: Gear selection
    enum {PARK, NEUTRAL, DRIVE} gear_region;

    // Region 2: Direction/Motion
    enum {STOPPED, MOVING_FWD, MOVING_REV} motion_region;
} VehicleState_t;

// Both regions operate simultaneously
// Coordination through events and guards
```

---

## Conventional Safety Architecture

### Fault Handling Hierarchy

```
┌─────────────────────────────────────────┐
│         Application Safety Layer        │
│  - Validate user inputs                 │
│  - Check interlock conditions           │
│  - Coordinate fault responses           │
└─────────────────┬───────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────┐
│      Motion Control Safety Layer        │
│  - Speed limit enforcement              │
│  - Torque limit arbitration             │
│  - Trajectory monitoring                │
│  - Emergency stop coordination          │
└─────────────────┬───────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────┐
│       Motor Drive Safety Layer          │
│  - Overcurrent protection               │
│  - Overvoltage/undervoltage             │
│  - Overtemperature                      │
│  - Hardware safety shutdown             │
└─────────────────────────────────────────┘
```

### Fault Response States
- **WARNING**: Continue operation with reduced capability
- **FAULT_REACTION**: Controlled shutdown in progress
- **FAULT**: Safe state reached, requires manual reset
- **SAFE_TORQUE_OFF (STO)**: Hardware-level safety disconnect

---

## Timing and Execution

### Conventional Task Structure

```
High Priority (Fast Loop - 100μs to 1ms)
├─ Motor current control
├─ PWM generation
├─ Commutation updates
└─ Hardware fault detection

Medium Priority (Medium Loop - 5ms to 10ms)
├─ Motor state machine
├─ Speed/Position control
├─ Motor-level fault handling
└─ Sensor processing

Low Priority (Slow Loop - 50ms to 100ms)
├─ Vehicle state machine
├─ User input processing
├─ Mode management
├─ Communication
└─ Logging

Background (Asynchronous)
├─ Diagnostics
├─ Calibration
├─ NVM operations
└─ HMI updates
```

---

## Design Best Practices

### 1. Clear State Definitions
- Each state should have a single, clear purpose
- Avoid "catch-all" states
- Document entry/exit conditions explicitly

### 2. Deterministic Transitions
- No ambiguous transition conditions
- Priority order for simultaneous events
- Timeout handling for stuck states

### 3. Safety by Design
- Default to safe state on uncertainty
- Multiple layers of protection
- Hardware-level last resort (STO)

### 4. Testability
- Each state testable in isolation
- Transition coverage testing
- Fault injection testing

### 5. Maintainability
- State diagrams match code structure
- Consistent naming conventions
- Clear separation of concerns

---

## Common Pitfalls to Avoid

### 1. **State Explosion**
- Too many states makes system unmanageable
- Solution: Use hierarchical states and orthogonal regions

### 2. **Race Conditions**
- Multiple threads modifying state
- Solution: Atomic state transitions, message queues

### 3. **Missing Fault Paths**
- Not all states have fault exit path
- Solution: Global fault transition from every state

### 4. **Unclear State Ownership**
- Multiple layers trying to control same motor state
- Solution: Clear command/status interface between layers

### 5. **Hard-Coded Delays**
- Timeout delays as state machine logic
- Solution: Event-driven design, explicit timeout states

---

## Industry Standards and References

### Relevant Standards
- **IEC 61800-7**: Adjustable speed electrical power drive systems (generic interface)
- **PLCopen Motion Control**: Standardized function blocks for motion
- **CANopen DS402**: Device profile for drives and motion control
- **ISO 26262**: Functional safety for automotive
- **IEC 61508**: Functional safety of electrical/electronic systems

### Common Frameworks
- **Quantum Platform (QP)**: Hierarchical event-driven framework
- **UML-based tools**: SCXML, Yakindu, Rhapsody
- **Industrial PLCs**: IEC 61131-3 state machines
- **Automotive AUTOSAR**: Mode management

---

## Comparison: Conventional vs Other Approaches

| Aspect | Conventional (PLCopen/IEC) | Simplified | Complex (Aerospace) |
|--------|---------------------------|------------|---------------------|
| **State Count** | 6-8 core states | 3-4 states | 15+ states |
| **Safety Focus** | High (certified patterns) | Medium | Very High (redundancy) |
| **Complexity** | Medium | Low | High |
| **Flexibility** | Good | Limited | Excellent |
| **Typical Use** | Industrial, Automotive | Consumer products | Safety-critical |

---

## Conclusion

The conventional approach emphasizes:

1. **Standardization**: Follow PLCopen/IEC standards for motor states
2. **Safety**: Multiple layers with clear fault handling
3. **Hierarchy**: Use parent/child state relationships
4. **Separation**: Clear interfaces between vehicle, motion, and motor layers
5. **Determinism**: Predictable, testable state transitions

This architecture is proven in:
- Electric vehicles (Tesla, Rivian, traditional OEMs)
- Industrial automation (servo drives, CNC machines)
- Robotics (collaborative robots, AGVs)
- Material handling (forklifts, conveyor systems)

The key is matching the complexity to your application requirements while maintaining safety and maintainability.
