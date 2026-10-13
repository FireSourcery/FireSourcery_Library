# Motor Dependency Graph

Arrows point from a dependency to its consumer. Each Mermaid fence renders independently. A diagram expands only its own group; dependencies outside that group appear as a single module node. Dotted connectors identify compile-time optional dependencies.

## Module Overview

```mermaid
flowchart TB
    Peripheral["Peripheral"] --> Motor["Motor"]
    Transducer["Transducer"] --> Motor
    Framework["Framework"] --> Motor
    Math["Math"] --> Motor
    System["System"] --> Motor

    Motor --> MotorController["MotorController"]
    MotProtocol["MotProtocol"] --> MotorController
    Peripheral --> MotorController
    Transducer --> MotorController
    Framework --> MotorController
    Math --> MotorController
    System --> MotorController
    Type["Type"] --> MotorController
```
### Framework And System

```mermaid
block
    StateMachine
    StateMachineThread
    Timer
    LimitArray
    Critical
    SysTime
```

## Motor

### Motor Module

```mermaid
flowchart TB
    MotorInclude["Motor_Include aggregate"]

    MotorCore["Motor core"]
    MotorTable["Motor table"]
    MotorConfig["Motor configuration"]
    MotorCommutation["Motor commutation"]
    MotorSensor["Motor sensor"]

    MotorCore --> MotorFoc["Motor FOC"]
    MotorCore --> MotorState["Motor state machine"]
    MotorCore --> MotorAnalog["Motor analog callbacks"]
    MotorCore --> MotorThread["Motor thread"]
    MotorCommutation --> MotorState
    MotorDebug["Motor debug"] --> MotorThread
    MotorAnalog --> MotorThread
    MotorState --> MotorThread

    MotorState --> MotorUser["Motor user API"]
    MotorCalibration["Motor calibration"] --> MotorUser
    MotorOpenLoop["Motor open loop"] --> MotorUser
    MotorFoc --> MotorUser

    MotorTable --> MotorInclude
    MotorThread --> MotorInclude
    MotorUser --> MotorInclude
    MotorVar["Motor variable API"] --> MotorInclude
    MotorConfig --> MotorInclude
    MotorCore --> MotorInclude
    MotorSensor --> MotorInclude
    MotorAnalog --> MotorInclude

```

### Motor Control Domains

```mermaid
flowchart TB
    Peripheral["Peripheral"] --> Phase["Phase output"]
    Peripheral --> PhaseInput["Phase input"]
    Transducer["Transducer"] --> RotorSensor["Rotor sensor"]
    Math["Math"] --> FocMath["FOC math"]

    Phase --> FocMath
    Phase
    PhaseInput
    VBus["VBus"]
    RotorSensor
    FocMath
    FocExtension["FOC extension"]
    FocSensorless["FOC sensorless"]
```


## MotorController

### MotorController Module

```mermaid
flowchart TB
    ControllerInclude["MotorController_Include aggregate"]

    Motor["Motor"] --> ControllerInclude
    MotProtocol["MotProtocol"] --> ControllerInclude

    ControllerApp["MotorController app"] --> ControllerCore["MotorController core"]
    MotAnalogUser["MotAnalogUser"] --> ControllerCore
    MotNvm["MotNvm"] --> ControllerCore
    MotLimits["MotLimits"] --> ControllerCore
    MotBuzzer["MotBuzzer"] --> ControllerCore
    ControllerCore --> ControllerState["MotorController state machine"]

    ControllerState --> ControllerUser["MotorController user API"]
    ControllerUser --> ControllerThread["MotorController thread"]
    ControllerState --> ControllerThread
    ControllerVar["MotorController variable API"] --> ControllerProtocol["MotorController protocol adapter"]

    ControllerApp --> ControllerInclude
    ControllerThread --> ControllerInclude
    ControllerProtocol --> ControllerInclude
    ControllerUser --> ControllerInclude
    ControllerInclude --> MotorController["MotorController"]
    ControllerCore --> MotorController
```




