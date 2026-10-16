# Motor Header Dependencies

Include graph of the `Motor/Motor` and `Motor/MotorController` headers. Arrows point from a header to the headers it includes; edges implied by a longer path are omitted. Module-level dependencies are in [ARCHITECTURE.md](../../ARCHITECTURE.md).

## Motor

```mermaid
---
config:
  flowchart:
    wrappingWidth: 600
---
flowchart TB
    Include["Motor_Include.h"]
    Table["Extension/Motor_Table.h"]
    Thread["Motor_Thread.h"]
    Var["Motor_Var.h"]
    Sensor["Sensor/Motor_Sensor.h"]
    User["Motor_User.h"]
    Sub["StateMachine/<br/>Motor_Calibration.h · Motor_OpenLoop.h<br/>Motor_Intervention.h"]
    MSM["Motor_StateMachine.h"]
    FOC["Motor_FOC.h"]
    Config["Motor_Config.h"]
    Analog["Analog/Motor_Analog.h"]
    Motor["Motor.h"]
    Parts["Phase · Phase_Input · VBus<br/>Sensor/RotorSensor · Math/FOC<br/>Motor_Clock · Motor_Electrical"]

    Include --> Table & Thread & Var & Sensor
    Table --> User & Analog
    Thread --> MSM & Analog
    User --> Sub
    Sub --> MSM
    MSM --> FOC
    FOC --> Motor
    Var --> Config
    Config --> Motor
    Analog --> Motor
    Sensor --> Motor
    Motor --> Parts
```

## MotorController

```mermaid
---
config:
  flowchart:
    wrappingWidth: 600
---
flowchart TB
    Include["MotorController_Include.h"]
    Thread["MotorController_Thread.h"]
    Proto["MotorController_MotProtocol.h"]
    Analog["MotorController_Analog.h"]
    User["MotorController_User.h"]
    Var["MotorController_Var.h"]
    SM["MotorController_StateMachine.h"]
    MC["MotorController.h"]
    App["MotorController_App.h"]
    Subs["MotAnalogUser · Shifter · OptDin<br/>MotNvm · MotBuzzer"]
    Engines["MotProtocol/<br/>MotPacket.h · MotProtocol.h · MotVarId.h"]
    MotorInclude["Motor/Motor_Include.h"]
    MotorThread["Motor/Motor_Thread.h"]
    Motor["Motor/<br/>Motor_Table.h · Motor_User.h · Motor_Config.h<br/>Motor_StateMachine.h · VBus.h"]

    Include --> Thread & Proto & MotorInclude
    Thread --> User & MotorThread
    Proto --> Var & Engines
    Analog --> SM
    User --> SM
    Var --> MC & Engines
    SM --> MC
    MC --> App & Subs & Motor
```
