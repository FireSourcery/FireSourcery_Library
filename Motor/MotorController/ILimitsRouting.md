##

```mermaid
flowchart LR
    %% Live limit sources
    SYS(["Heat (MOSFETs), OptDin"])
    USER(["User Set"])
    PRO(["Protocol / app<br/>CiA 0x60E0/0x60E1, 0x6072"])
    VB(["<b>VBus voltage</b><br/>UV/OV ramps"])

    %% Stored limits
    CFG[("Motor Config<br/>ILimitMotoring / ILimitGenerating")]
    IDC[("<b>VBus IdcLimits</b><br/>IDischargeLimit / IChargeLimit")]

    %% Frame conversions
    PB{{"<b>IBus operating point [Cw/Ccw]</b><br/>power balance by v_q (exact)<br/>or phase bound by motion<br/>reads FOC state"}}
    VD{{"<b>IBus Approximate [Cw/Ccw]</b><br/>UV ramp × Mot, OV ramp × Gen<br/>by Direction, the V half-plane<br/>least accurate, cheapest, least coupled"}}
    GET{{"<b>Motor_ResolveILimits [Cw/Ccw]</b><br/><b>Merge</b><br/>intersect of the three"}}

    %% Merge points
    ARB["<b>System arbitration [Ratio Mot/Gen]</b><br/>_MotorController_IDerate: hand-composed min<br/>one ratio for both sides; LimitArray not built"]
    PAIR["<b>ILimit [Mot/Gen]</b><br/>min of sources<br/>key: Direction"]
    DIR["<b>ILimit [Fwd/Rev]</b><br/>min of sources<br/>key: DirectionForward"]
    WIN["<b>ISupplyLimits [Cw/Ccw]</b><br/>key: power flow"]

    %% Active clamp
    SLOT["<b>Active clamp</b><br/>TorqueRamp limits + stored Target<br/>PidSpeed output limits"]

    %% Limit data flow
    SYS --> ARB
    USER --> ARB
    VB -->|V_DERATE| VD --> WIN
    VB -->|"POWER_BALANCE / PHASE_BOUND"| PB --> WIN
    IDC --> PB
    ARB -->|"ratio pair × Config"| PAIR
    CFG --> PAIR
    CFG --> VD
    PRO --> DIR
    PAIR --> GET
    DIR --> GET
    WIN --> GET
    GET --> SLOT

    %% Speed limit path, drawn for the shared 1 s call site only
    SPD(["VBus speed derate, OptDin speed limit"])
    SARB["<b>_MotorController_SpeedDerate</b><br/>hand-composed min, one ratio"]
    SSLOT["<b>Speed limit [Fwd/Rev]</b><br/>Motor_SetSpeedLimitDerate, stored<br/>Motor_ResolveSpeedLimits on Run_Entry and ON_SPEED"]
    SPD --> SARB -->|"ratio × Config"| SSLOT

    %% Call sites
    C1S[["MC main 1 s<br/>MotorController_ProcILimits → Motor_SetILimitDerates<br/>MotorController_ProcSpeedLimits → Motor_SetSpeedLimitDerate"]]
    ENT[["Run_Entry, Intervention_Entry<br/>Motor_FOC_ResolveISupplyLimitsOnEntry<br/>then Motor_ResolveILimits"]]
    TICK[["Speed tick 1 kHz<br/>Run (Current), TorqueZero, SpeedZero<br/>Motor_FOC_ResolveISupplyLimitsOnSpeed<br/>then Motor_ResolveILimits"]]
    CVB[["VBus thread 1 ms"]]
    MC10[["MC main 1 ms"]]
    RES[["Motor_ResolveILimits<br/>each push"]]
    AL[["Motor_ResolveAlignILimits"]]
    VM[["Run_Entry, voltage mode"]]

    C1S -.-> ARB
    C1S -.-> SARB
    ENT -.-> WIN
    ENT -.-> GET
    TICK -.->|ON_SPEED| WIN
    TICK -.-> GET
    CVB -.->|ON_VBUS| WIN
    MC10 -.->|ON_MAIN| WIN
    RES -.-> GET
    AL -.->|writes| SLOT
    VM -.->|writes| SLOT

    classDef source fill:#e8f5e9,stroke:#2e7d32,color:#1b1b1b
    classDef storage fill:#eceff1,stroke:#455a64,color:#1b1b1b
    classDef convert fill:#ede7f6,stroke:#5e35b1,color:#1b1b1b
    classDef merge fill:#e3f2fd,stroke:#1565c0,color:#1b1b1b
    classDef clamp fill:#ffebee,stroke:#c62828,color:#1b1b1b,stroke-width:3px
    classDef callsite fill:#fff3e0,stroke:#ef6c00,color:#1b1b1b,stroke-dasharray:4 3
    class SYS,PRO,VB,SPD source
    class CFG,IDC storage
    class PB,VD convert
    class ARB,PAIR,DIR,WIN,GET,SARB merge
    class SLOT,SSLOT clamp
    class C1S,ENT,TICK,CVB,MC10,RES,AL,VM callsite
```

Green stadium: live limit source. Grey cylinder: stored config limit. Purple hexagon: frame conversion. Blue box: merge point. Red box: active clamp in the hot path. Amber dashed box: call site, with dotted edges meaning "invokes". Solid edges carry limit values.

One of [MOTOR_I_SUPPLY_LIMITS_ON_SPEED], [MOTOR_I_SUPPLY_LIMITS_ON_VBUS], [MOTOR_I_SUPPLY_LIMITS_ON_MAIN] selects the thread that resolves [ISupplyLimits] periodically, with [Motor_FOC_ResolveISupplyLimits]: the speed tick, the 1 ms VBus thread, or the 1 ms block of [MotorController_Main_Thread]. There is no default. Entry resolves it under every selection, so a state starts on a window keyed by its own Direction. With none selected nothing resolves it, and [ISupplyLimits] stays open from [Motor_Init].

The system arbitration is a hand-composed min ([_MotorController_IDerate]: the MOSFET heat derate and the OptDin torque limit); the LimitArray form is commented out in [MotLimits.h] and [MotorController.c]. The speed derate ([_MotorController_SpeedDerate]: the bus derate and the OptDin speed limit) rides the same 1 s call and reaches [Motor_ResolveSpeedLimits] on [Run_Entry] and the speed tick. It bounds the speed, not the current, and is drawn for the shared call site only.

