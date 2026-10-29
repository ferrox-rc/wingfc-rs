# WingFC-rs System Architecture & Control Flowcharts

This document compiles the complete architectural and dataflow flowcharts for `wingfc-rs`, serving as a visual reference for system boot, real-time sensor processing, cascaded attitude stabilization, arming safety logic, and hardware actuation.

---

## 1. System Boot & Initialization Sequence

```mermaid
flowchart TD
    Start([Power On / Hardware Reset]) --> Pins[Initialize BoardPins Container]
    Pins --> PwrIMU[Assert P1.08 IMU Power Rail with HighDrive]
    PwrIMU --> Actuators[Initialize PWM Timers: PWM0 Motors & PWM1/2 Servos]
    Actuators --> UART[Initialize EasyDMA Buffered UARTE0 for RC Receiver]
    UART --> SpawnRx[Spawn Asynchronous RC Receiver Task]
    
    SpawnRx --> CheckCalib{Throttle > 1900us at Boot?}
    CheckCalib -- Yes --> ESCCalib[Enter ESC Throttle Calibration Mode<br/>Fast-Blink Yellow LED<br/>Direct Throttle Passthrough until Throttle <= 1050us]
    CheckCalib -- No --> IMUInit[Initialize LSM6DS3TR-C IMU via TWISPI0 @ 400kHz]
    
    ESCCalib --> IMUInit
    IMUInit --> GyroCalib[Stationary Gyroscope Calibration<br/>Average 250 stationary samples<br/>Fast-Blink White LED]
    
    GyroCalib --> Watchdog[Initialize 2000ms Hardware Watchdog Timer]
    Watchdog --> Ready([Enter Hardware Event-Driven Flight Loop])
```

---

## 2. Multi-Rate Decoupled Flight Loop (1666.6 Hz IMU -> 208.33 Hz Loop)

```mermaid
flowchart TD
    DRDY([LSM6DS3 INT1 DRDY Interrupt @ 1666.6 Hz]) --> Accum[8:1 Sinc / Boxcar Accumulator<br/>Average 8 consecutive samples]
    Accum -- 8th Sample Emitted --> PetWDT[Pet 2000ms Hardware Watchdog]
    PetWDT --> FixedDT[Deterministic dt = 0.0048s / 208.33 Hz]
    FixedDT --> DrainRC[Drain Latest RC Channels from Buffered Receiver Channel]
    DrainRC --> CheckFailsafe{Duration since last packet > 500ms?}
    
    CheckFailsafe -- Yes --> ForceFailsafe[Force ArmingStatus::Failsafe<br/>Slow-Blink Red LED]
    CheckFailsafe -- No --> ModeMatrix[Evaluate Flight Mode Activation Matrix<br/>CH5 Arm, CH6 Mode, Throttle Interlocks]
    
    ModeMatrix --> IngestExt[Ingest External Sensors Watch @ 20 Hz<br/>Baro & Calibrated Compass]
    ForceFailsafe --> IngestExt
    
    IngestExt --> HealthCheck{Sensor Health Check<br/>Plausible Mag & Timely Baro?}
    HealthCheck -- Mag Healthy --> Mag9DoF[Enable 9-DoF AHRS & Heading Hold]
    HealthCheck -- Mag Degraded / Timeout --> Mag6DoF[Fallback to 6-DoF AHRS & Angle Mode]
    HealthCheck -- Baro Degraded / Timeout --> BaroFallback[Fallback AltHold to Angle Mode]
    
    Mag9DoF --> CheckArmStatus{Current Flight Status?}
    Mag6DoF --> CheckArmStatus
    BaroFallback --> CheckArmStatus
    
    CheckArmStatus -- Disarmed --> Passthrough[Zero PIDs<br/>Direct Stick Passthrough to Servos<br/>Throttle Locked to 1000us<br/>Solid Green LED]
    CheckArmStatus -- Failsafe --> Neutralize[Motor Cut to 1000us<br/>Flight Surfaces Neutralized to 1500us]
    CheckArmStatus -- ArmedAcro --> AcroLoop[PT1 Gyro Filter -> Angular Rate PID + Feedforward with Fixed dt]
    CheckArmStatus -- ArmedAngle --> AngleLoop[PT1 Gyro Filter -> VQF AHRS -> Cascaded Angle P + Rate PID with Fixed dt]
    CheckArmStatus -- ArmedHeadingHold --> HeadingLoop[Tilt-Compensated Heading -> Bank Angle Cascaded Controller]
    CheckArmStatus -- ArmedAltHold --> AltLoop[Vertical Kalman Filter -> Cascaded Altitude / Pitch Controller]
    
    Passthrough --> Mixer[Airframe Mixer]
    Neutralize --> Mixer
    AcroLoop --> Mixer
    AngleLoop --> Mixer
    HeadingLoop --> Mixer
    AltLoop --> Mixer
    
    Mixer --> Clamping[Per-Servo Clamping, Inversion, Subtrim]
    Clamping --> Dispatch[Write Duty Cycles to Hardware PWM0 & PWM1/2]
    Dispatch --> TelemetryCheck{loop_counter % divider == 0?}
    TelemetryCheck -- Yes --> EmitTelemetry[Send CRSF Telemetry Snapshot @ ~50 Hz]
    TelemetryCheck -- No --> Sleep([Sleep in WFE awaiting next Hardware DRDY Interrupt])
    EmitTelemetry --> Sleep
```


---

## 3. Signal Dataflow: Where IMU Meets Stick Input (Armed Angle Mode)

```mermaid
flowchart TD
    subgraph IMU_Pipeline["1. Inertial Sensor Pipeline (400 kHz EasyDMA I2C)"]
        SENS[LSM6DS3TR-C 6-DoF Sensor] -->|12-byte Burst Read| RAW[Raw Gyro & Accel Readings]
        RAW --> SCALE[Scale: Gyro -> rad/s, Accel -> m/s²]
        SCALE --> BIAS[Subtract Stationary Gyro Bias]
        BIAS --> ORIENT[Apply 8-Way Board Mounting Orientation Matrix]
        ORIENT --> LPF[PT1 Low-Pass Filter: Reject Prop / Motor Harmonics]
    end

    subgraph RC_Pipeline["2. Pilot RC Stick Pipeline (Buffered UARTE0)"]
        RADIO[Transmitter Sticks] -->|CRSF @ 420k / iBUS @ 115.2k| PKT[Raw Channel Pulses 1000..2000 us]
        PKT --> MAP[Normalize Sticks: Pitch, Roll, Yaw to -1.0 .. +1.0]
    end

    subgraph Outer_Loop["3. Outer Attitude Loop (Self-Leveling)"]
        ORIENT -->|Accel| VQF[VQF AHRS Quaternion Fusion]
        LPF -->|Filtered Gyro| VQF
        VQF --> EULER[Extract Euler Angles: est_pitch, est_roll]
        
        MAP -->|pitch_stick, roll_stick| TARGET[target_pitch = pitch_stick * MAX_PITCH_ANGLE<br/>target_roll = roll_stick * MAX_ROLL_ANGLE]
        
        TARGET & EULER --> ERR_CALC[target_pitch_rate = target_pitch - est_pitch * ANGLE_P_GAIN<br/>target_roll_rate = target_roll - est_roll * ANGLE_P_GAIN]
    end

    subgraph Inner_Loop["4. Inner Angular Rate PID Loop"]
        ERR_CALC --> PID_BLOCK[PID Controller with FeedForward & Dynamic D-Term Filtering]
        LPF -->|gyro.x, gyro.y, gyro.z| PID_BLOCK
        PID_BLOCK --> MIX_DEMANDS[Normalized Control Demands: pitch_mix, roll_mix, yaw_mix]
    end

    subgraph Mixing_Output["5. Multi-Topology Airframe Mixer & Actuation"]
        MIX_DEMANDS --> MIX_TOPOLOGY[AirframeMixer: Elevon, Diff Thrust, T-Tail, V-Tail]
        MIX_TOPOLOGY --> SERVOS_NORM[Normalized Surface Commands in -1.0 .. +1.0]
        SERVOS_NORM --> SERVO_CFG[Apply Min/Max Endpoints, Subtrim, and Servo Reversing]
        SERVO_CFG --> PWM_HARDWARE[ServoPwmDriver & EscPwmDriver 1000..2000 us]
        PWM_HARDWARE --> FLIGHT_SURFACES[Physical Elevons & Motor ESCs]
    end
```

---

## 4. Arming Safety State Machine & Flight Mode Matrix

```mermaid
stateDiagram-v2
    [*] --> Disarmed: Boot Completed & Gyro Calibrated
    
    Disarmed --> RefusedSwitchArmedAtBoot: Arm Switch HIGH at Boot
    RefusedSwitchArmedAtBoot --> Disarmed: Arm Switch Moved to LOW
    
    Disarmed --> RefusedThrottleRaised: Arm Switch HIGH but Throttle > 1050us
    RefusedThrottleRaised --> Disarmed: Arm Switch Moved to LOW
    
    Disarmed --> ArmedAcro: Arm Switch HIGH & Throttle <= 1050us & Mode == Acro
    Disarmed --> ArmedAngle: Arm Switch HIGH & Throttle <= 1050us & Mode == Angle
    
    ArmedAcro --> ArmedAngle: Mode Switch toggled to Angle range (CH6 > 1300us)
    ArmedAngle --> ArmedAcro: Mode Switch toggled to Acro range (CH6 < 1300us)
    
    ArmedAcro --> Disarmed: Arm Switch toggled LOW (Immediate Motor Cut)
    ArmedAngle --> Disarmed: Arm Switch toggled LOW (Immediate Motor Cut)
    
    ArmedAcro --> Failsafe: RC Packet Loss > 500ms
    ArmedAngle --> Failsafe: RC Packet Loss > 500ms
    Disarmed --> Failsafe: RC Packet Loss > 500ms
    
    Failsafe --> Disarmed: RC Link Restored & Arm Switch LOW & Throttle <= 1050us
```

---

## 5. Visual Telemetry LED Status Logic

```mermaid
flowchart TD
    State{Flight Status}
    
    State -- Stationary Calibrating --> LED_White[RGB: Fast Blink White 5Hz]
    State -- ESC Throttle Calib --> LED_YellowFast[RGB: Fast Blink Yellow 5Hz]
    State -- Arm Refused / Safety Lockout --> LED_YellowDbl[RGB: Double Blink Yellow 1Hz]
    State -- Disarmed & Safe --> LED_Green[RGB: Solid Green]
    State -- Armed Acro --> LED_Blue[RGB: Solid Blue]
    State -- Armed Angle --> LED_Cyan[RGB: Solid Cyan]
    State -- Failsafe / Link Lost --> LED_Red[RGB: Slow Blink Red 2Hz]
    
    subgraph UserLED["Discrete User LED (P0.17)"]
        U1{Link Health}
        U1 -- Signal Lost --> U_Off[User LED: OFF]
        U1 -- Packets Active --> U_Pulse[User LED: Micro-Pulse on Frame]
        U1 -- Link Healthy Idle --> U_Solid[User LED: Solid ON]
    end
```
