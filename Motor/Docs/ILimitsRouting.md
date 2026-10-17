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
    VD{{"<b>IBus Approximate [Ratio Mot/Gen]</b><br/>UV ramp → Mot, OV ramp → Gen<br/>least accurate, cheapest, least coupled"}}
    GET{{"<b>Motor_ResolveILimits [Cw/Ccw]</b><br/><b>Merge</b><br/>intersect of the three"}}

    %% Merge points
    ARB["<b>System arbitration [Ratio Mot/Gen]</b><br/>hand-composed min or LimitArray<br/>unsigned ratio pair"]
    PAIR["<b>ILimit [Mot/Gen]</b><br/>min of sources<br/>key: Direction"]
    DIR["<b>ILimit [Fwd/Rev]</b><br/>min of sources<br/>key: DirectionForward"]
    WIN["<b>ISupplyLimits [Cw/Ccw]</b><br/>key: power flow"]

    %% Active clamp
    SLOT["<b>Active clamp</b><br/>TorqueRamp limits + stored Target<br/>PidSpeed output limits"]

    %% Limit data flow
    SYS --> ARB
    USER --> ARB
    VB -->|V_DERATE| VD --> ARB
    VB -->|"POWER_BALANCE / PHASE_BOUND"| PB --> WIN
    IDC --> PB
    ARB -->|"ratio pair × Config"| PAIR
    CFG --> PAIR
    PRO --> DIR
    PAIR --> GET
    DIR --> GET
    WIN --> GET
    GET --> SLOT

    %% Call sites
    C1S[["MC main 1 s<br/>MotorController_ApplyILimits<br/>→ Motor_SetILimitDerates"]]
    CVB[["VBus thread, V_DERATE<br/>push on update"]]
    TICK[["Speed tick 1 kHz<br/>Run (Current), TorqueZero, RampDown<br/>MOTOR_I_SUPPLY_LIMITS_SPEED_LOOP"]]
    MC10[["MC main ~10 ms (default)<br/>Motor_ResolveISupplyLimits"]]
    RES[["Motor_ResolveILimits<br/>each push, Motor_SetDirection,<br/>Run_Entry, Intervention_Entry"]]
    AL[["Motor_FOC_SetAlignILimits"]]
    VM[["Run_Entry, voltage mode"]]

    C1S -.-> ARB
    CVB -.-> ARB
    TICK -.->|or| PB
    MC10 -.->|or| PB
    RES -.-> GET
    AL -.->|writes| SLOT
    VM -.->|writes| SLOT

    classDef source fill:#e8f5e9,stroke:#2e7d32,color:#1b1b1b
    classDef storage fill:#eceff1,stroke:#455a64,color:#1b1b1b
    classDef convert fill:#ede7f6,stroke:#5e35b1,color:#1b1b1b
    classDef merge fill:#e3f2fd,stroke:#1565c0,color:#1b1b1b
    classDef clamp fill:#ffebee,stroke:#c62828,color:#1b1b1b,stroke-width:3px
    classDef callsite fill:#fff3e0,stroke:#ef6c00,color:#1b1b1b,stroke-dasharray:4 3
    class SYS,PRO,VB source
    class CFG,IDC storage
    class PB,VD convert
    class ARB,PAIR,DIR,WIN,GET merge
    class SLOT clamp
    class C1S,CVB,TICK,MC10,RES,AL,VM callsite
```

Green stadium: live limit source. Grey cylinder: stored config limit. Purple hexagon: frame conversion. Blue box: merge point. Red box: active clamp in the hot path. Amber dashed box: call site, with dotted edges meaning "invokes". Solid edges carry limit values.

##
```mermaid
flowchart LR
    SC["Speed cmd"] --> SL["clamp: speed limits"] --> RFG["Speed ramp (RFG)<br/>acc/dec, hold, set, bypass, quick stop"] --> PI["Speed PI"]
    TC["Torque cmd"] --> TCL["clamp: effective limits"] --> TR["Torque ramp<br/>slope"]
    PI --> SEL["mode select"]
    TR --> SEL
    SEL --> LIM["Torque limitation<br/>[max of lowers : min of uppers]<br/>+ active-source status"] --> CC["Current loop"]
    SRC["Sources: fixed/variable limits,<br/>power motoring/regen, Vdc controller,<br/>current, thermal"] --> LIM
    LIM -. "effective limits" .-> PI
    LIM -. "effective limits" .-> TCL
    LIM -. "at limit: tracking" .-> RFG
```

