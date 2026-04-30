# Final PID Pilot

This package is a self-contained PIDF tuning framework for FTC mechanisms. It is designed to let a team stand up a tuning OpMode quickly, expose the right controls in FTC Dashboard, and get live telemetry that is specific enough to explain what the controller is doing rather than just whether it "works."

The package currently supports two major control families:

- Velocity tuning for mechanisms such as flywheels and shooters
- Position tuning for mechanisms such as arms, elevators, slides, turrets, and servo-driven axes

It also includes:

- A reusable PIDF controller core
- A base `LinearOpMode` runner that handles live config refresh and mode switching
- A tuning mode enum (`REV_UP` vs `MAINTAIN`)
- Three example OpModes that show how to wire the tuners to real hardware

This README is intentionally long. The goal is not just to describe what each class is called, but to explain the actual engineering model behind the code, how the systems fit together, and what each major method is responsible for.

## Package Contents

Files in this package:

- `PIDFController.java`
- `PIDFTunerOpMode.java`
- `PIDFTuningMode.java`
- `VelocityPIDFTuner.java`
- `PositionPIDFTuner.java`
- `TuneFlywheelNew.java`
- `TuneArm.java`
- `TuneElevator.java`

## Design Goals

The package is built around a few strong design choices:

- The tuners own the outer control loop themselves.
- FTC Dashboard is treated as a live tuning surface, not just a logging sink.
- `REV_UP` and `MAINTAIN` are separate behaviors with separate gains.
- Telemetry is designed to explain the controller's internal terms, not just output a setpoint and a measurement.
- Feedforward and feedback are separated cleanly enough that teams can reason about them.
- Velocity tuning uses raw `ticks/s` units instead of hidden normalization.
- Position tuning supports multiple actuator families behind one consistent tuning interface.

Those choices explain much of the structure in the code.

## High-Level Architecture

At a high level, the package works like this:

1. A concrete OpMode such as `TuneFlywheelNew` or `TuneArm` returns a `Config` object.
2. `PIDFTunerOpMode` instantiates the matching tuner and enters the main loop.
3. On every loop:
   - The OpMode asks `configureVelocity()` or `configurePosition()` for a fresh config
   - The tuner refreshes itself from that config
   - The tuner reads sensors
   - The tuner computes feedforward and PID terms
   - The tuner applies output to hardware
   - The tuner publishes rich telemetry to Driver Station and Dashboard
4. The driver can toggle between `REV_UP` and `MAINTAIN` using `gamepad1.x`.

That means the package is not based on static startup configuration only. It is designed for live tuning, live Dashboard edits, and live mode swaps.

## Shared Concepts

Before diving into each class, it helps to understand the common ideas used across the package.

### 1. `REV_UP` vs `MAINTAIN`

The package treats "get there quickly" and "hold steady under disturbances" as two different control problems.

- `REV_UP`
  - Prioritizes fast approach
  - Often uses no integral or very little integral
  - May use motion profiling in the position tuner
  - Accepts some overshoot if it improves speed

- `MAINTAIN`
  - Prioritizes zero steady-state error and disturbance rejection
  - Commonly uses integral
  - Is the mode used for disruption sampling
  - Is the mode you care about once the mechanism is already near target

This is why both tuners maintain two gain sets instead of one.

### 2. Feedforward vs feedback

The code tries to keep these roles distinct:

- Feedforward handles the expected baseline effort
- PID handles the leftover error

Examples:

- In velocity control, `kF` is a real physical feedforward in motor-power per `ticks/s`
- In position control, `kF` is not a velocity term; it is a static trim or friction kick
- Gravity and cosine compensation in the position tuner are explicit additional feedforward terms

### 3. Dashboard-driven configuration

All tuners are annotated with `@Config`, so static fields appear in FTC Dashboard.

This means:

- Default gains are visible and editable
- Sample OpModes can expose their targets and operating mode live
- Config objects can blend Dashboard defaults with method overrides

### 4. Telemetry as a control diagnostic surface

The telemetry is not only:

- target
- actual
- error

It also exposes:

- `pTerm`, `iTerm`, `dTerm`, `fTerm`
- filtered derivative measurements
- feedforward breakdown
- disruption metrics
- relay tuning metrics
- active gains
- warnings and advisories

That is deliberate. It lets a team diagnose whether the issue is bad feedforward, too much proportional action, not enough damping, integral windup, or a mechanism-specific limitation.

## `PIDFController.java`

This is the shared controller core used by both tuners.

### Responsibility

`PIDFController` does the raw control math:

- proportional term
- integral accumulation and clamping
- derivative on measurement
- derivative low-pass filtering
- optional `kF * setpoint`
- final output clamping

It does not know anything about:

- motors vs servos
- velocity vs position semantics
- characterization
- relay tuning
- motion profiles
- disruption tests

That separation is important. The tuners own mechanism logic. The controller owns control math.

### Why derivative on measurement?

The controller computes derivative from the measurement, not from direct error difference.

That avoids derivative kick when the setpoint changes suddenly. In practice, this matters a lot for FTC mechanisms because many moves are step changes. The filtered derivative still reacts to actual motion, but it does not explode just because the target jumped.

### Internal state

The class stores:

- `integralSum`
- `previousMeasurement`
- `rawMeasurementRate`
- `filteredMeasurementRate`
- `pTerm`, `iTerm`, `dTerm`, `fTerm`
- `lastOutput`
- `lastError`
- `lastErrorRate`
- `hasMeasurement`

This lets the tuners publish internals directly without duplicating control math.

### Important methods

- `calculate(setpoint, measurement, dt)`
  - Main control update
  - Clamps `dt`
  - Computes all terms
  - Updates state
  - Returns the clamped final output

- `reset()`
  - Clears integral memory, derivative history, and output state
  - Called on mode changes and phase transitions where old controller memory would be misleading

- `setGains(kP, kI, kD, kF)`
  - Lets tuners swap between `REV_UP` and `MAINTAIN` immediately

### Why `integralSumMax` lives in accumulator units

The controller clamps the integral accumulator itself, not the `iTerm` directly. That keeps the meaning of the cap tied to the stored error history. The tuners sometimes compute this cap from actuator headroom, and that is why they convert from output-space intuition back into accumulator-space values.

## `PIDFTuningMode.java`

This is a small enum with a large design role.

Values:

- `REV_UP`
- `MAINTAIN`

Each value also carries a human-readable description.

This enum is used in:

- the OpMode loop
- both tuner classes
- Dashboard-facing sample OpModes
- control branching for gain selection
- disruption-phase eligibility
- motion-profile behavior

It is the common language that lets the entire package treat "approach" and "hold" as different controller personalities.

## `PIDFTunerOpMode.java`

This is the base runner for all tuning OpModes.

### Responsibility

`PIDFTunerOpMode` handles the runtime shell around the actual tuners:

- selecting whether velocity or position tuning is active
- validating that only one tuner is configured at a time
- fetching Dashboard telemetry when available
- driving the main loop
- refreshing config every loop
- handling mode toggling with `gamepad1.x`
- clamping loop timing

### Why config is refreshed every loop

This is one of the most important design choices in the package.

Instead of building a config once at startup and freezing it, the OpMode re-calls `configureVelocity()` or `configurePosition()` every loop. That makes Dashboard edits take effect immediately. It also means any hardware mode assumptions made by the tuner must be reasserted each loop if needed.

This is exactly why the velocity tuner now reasserts `RUN_WITHOUT_ENCODER` in `refreshFrom()`.

### Main methods

- `configureVelocity()`
  - Override this in a velocity tuning OpMode
  - Return a `VelocityPIDFTuner.Config`

- `configurePosition()`
  - Override this in a position tuning OpMode
  - Return a `PositionPIDFTuner.Config`

- `runOpMode()`
  - Ensures only one tuner is active
  - Dispatches to either velocity or position runner

- `runVelocityTuner(initialConfig)`
- `runPositionTuner(initialConfig)`
  - Main live tuning loops
  - Re-fetch config every cycle
  - Track timing
  - Handle X-button mode toggling

- `getDashboardTelemetry()`
  - Uses reflection to avoid a hard runtime dependency
  - Gracefully falls back if Dashboard is unavailable

### Telemetry helpers

The class also provides:

- `addLine(...)`
- `addData(...)`
- `updateTelemetry(...)`

These mirror output to both Driver Station and Dashboard telemetry when Dashboard is present.

## `VelocityPIDFTuner.java`

This is the most sophisticated class in the package. It is a full velocity tuning workflow, not just a thin PID wrapper.

### Core responsibility

It tunes and runs external velocity PIDF control for one or more motors using raw `ticks/s` units.

That means:

- measurements are motor velocities in `ticks/s`
- `kP` is motor-power per `ticks/s` of error
- `kI` is motor-power per `(ticks/s * s)`
- `kD` is motor-power per `ticks/s^2`
- `kF` is motor-power per `ticks/s`

### Why raw units matter

This tuner explicitly avoids hidden normalization. That is a major design decision.

Benefits:

- Dashboard values correspond to real physical units
- feedforward is physically interpretable
- gain math is honest

Tradeoff:

- gain numbers often look numerically "small"
- default gains cannot be safely guessed across radically different mechanisms

That tradeoff is exactly why the tuner now characterizes the mechanism first and computes
model-based starting gains from the measured response.

### Major subsystems inside the class

The class is best understood as several interacting systems:

1. Gain and mode management
2. Feedforward source selection
3. Startup characterization
4. Model-based PIDF recommendation
5. Optional relay auto-tuning
6. Closed-loop running control
7. Disruption measurement
8. Telemetry and status rendering
9. Builder/config resolution

### Velocity tuner lifecycle

The high-level lifecycle is:

1. Construction
   - caches telemetry
   - stores motor array
   - sets motors to `RUN_WITHOUT_ENCODER`
   - zeroes power once for safety

2. `refreshFrom()`
   - reasserts `RUN_WITHOUT_ENCODER`
   - reloads config values
   - resolves gain sources
   - updates feedforward mode

3. `update(loopTimeSeconds)`
   - reads velocity
   - computes current error
   - handles startup phases or normal closed-loop control
   - runs disruption logic
   - pushes telemetry

### Velocity tuner phases

The class uses a `TunerPhase` enum:

- `CHARACTERIZING`
- `SETTLING`
- `RELAY_TUNING`
- `RELAY_COMPLETE`
- `RUNNING`
- `DISRUPTION`
- `COMPLETE`

Each phase has different output behavior.

#### `CHARACTERIZING`

Purpose:

- drive the mechanism at full power
- estimate settled full-power velocity
- estimate the first-order response time constant
- compute a physical `kF`
- compute initial `REV_UP` and `MAINTAIN` PIDF recommendations

Behavior:

- applies `MAX_POWER`
- samples the final portion of the run
- computes `kF = 1 / settledVelocity`
- records the response curve and estimates `tau` from the 63.2 percent rise time
- computes conservative raw-unit feedback gains from the measured model

Why:

- velocity control benefits enormously from feedforward
- without a reasonable `kF`, the PID loop has to do too much work
- forcing relay oscillation on every run is noisy and can be hard on FTC mechanisms

#### `SETTLING`

Purpose:

- stop output briefly after characterization
- reset controller memory before closed-loop work begins

Behavior:

- output is zero
- controller is reset
- after the pause, the tuner enters normal running with model-computed gains unless relay tuning was explicitly enabled

#### `RELAY_TUNING`

Purpose:

- optionally refine mechanism-specific gains by forcing oscillation

Behavior:

- PID gains are effectively zeroed
- the mechanism runs on feedforward plus a hysteretic relay signal
- the relay forces a stable oscillation around the target
- amplitude and period are measured
- ultimate gain and period are estimated
- conservative `MAINTAIN` and `REV_UP` gains are computed

Relay tuning is off by default. Enable it with `useRelayTuning()` only when the model-based
result is not good enough and the mechanism can safely oscillate around the target.

#### `RELAY_COMPLETE`

Purpose:

- show the computed relay results clearly before normal PID takes over

Behavior:

- holds for a short time
- keeps telemetry visible
- then transitions into running mode

#### `RUNNING`

Purpose:

- standard external velocity PIDF control

Behavior:

- computes profiled target if ramping is enabled
- applies active gains
- computes PID output
- adds feedforward
- writes power directly

#### `DISRUPTION`

Purpose:

- measure disturbance recovery for `MAINTAIN`

Behavior:

- watches for the mechanism to stabilize
- waits for a disturbance
- times recovery
- collects multiple samples

#### `COMPLETE`

Purpose:

- show finished disruption summary
- expose final copyable values

### Feedforward handling

The tuner can obtain `kF` from several sources:

- a one-time characterization result
- a manual override via `skipCharacterization(manualKF)`
- a nonzero `kF` explicitly embedded in either gain set

This is managed by:

- `resolveActivePhysicalKf()`
- `resolveModePhysicalKf(...)`
- `usesManualKf()`
- `usesConfiguredGainKf()`
- `syncFeedforwardMode()`

One subtle point:

- feedforward selection and phase routing are related but not identical
- `syncFeedforwardMode()` now only changes phase during pre-decision states
- once relay tuning or normal running is active, changing the feedforward source should not destroy the active state machine

### Why `ensureMotorMode()` exists

The velocity tuner is an external power controller. It must keep motors in `RUN_WITHOUT_ENCODER`.

Because the surrounding OpMode re-creates config every loop, some user code may call `setMode(RUN_USING_ENCODER)` inside `configureVelocity()`. If that happened and the tuner did not reassert `RUN_WITHOUT_ENCODER`, then `setPower()` would stop being raw power and would start acting like an internal velocity command through the SDK controller. That would make the SDK's inner loop fight the tuner's outer loop.

`ensureMotorMode()` prevents that nondeterminism.

### Integral and proportional sanity logic

The tuner has helper logic that turns feedforward and target velocity into warnings:

- `updateDerivedGainState()`

This method:

- derives `integralSumMax` from available output headroom when not overridden
- warns if `kF` consumes all output headroom
- warns if `kP * target` is much larger than remaining headroom

This is not just cosmetic. It teaches the user when a gain is physically unreasonable for the current operating point.

### Relay auto-tuning subsystem

The inner `RelayAutoTuner` class is a complete subsystem with its own states:

- `WAITING_FOR_TARGET`
- `OSCILLATING`
- `COMPUTING`
- `COMPLETE`

It also uses a crossing-state enum:

- `ABOVE`
- `BELOW`

#### What it measures

- oscillation amplitude
- oscillation period
- `Ku`
- `Pu`

#### What it computes

- conservative `MAINTAIN` gains
- conservative `REV_UP` gains

#### Why hysteresis is used

Without hysteresis, encoder noise near the setpoint would cause false switching and fake zero crossings. The relay would chatter instead of producing a useful oscillation.

### Disruption subsystem

The inner `DisruptionPhase` class is a second state machine used after the controller is already functioning.

Stages:

- `WAITING`
- `ARMED`
- `DETECTING`
- `RECOVERING`
- `COMPLETE`

Purpose:

- quantify how quickly the mechanism recovers from a real disturbance

This is especially valuable for flywheels. Step response alone is not enough if the mechanism must survive ring loading or other match disturbances.

### Velocity tuner telemetry

The velocity tuner publishes several categories:

- setpoints
- measurements
- errors
- term breakdown
- active gains
- characterization data
- relay auto-tune metrics
- disruption metrics
- warnings
- copyable final values

This is why the class is large. It is not only a controller; it is also a teaching and diagnostics surface.

### Velocity tuner method map

Important public methods:

- `VelocityPIDFTuner(Config config)`
  - constructs the tuner, performs initial motor safety setup, then loads config

- `refreshFrom(Config config, PIDFTuningMode forcedMode)`
  - re-reads live config every loop
  - reasserts external motor mode
  - resolves gains and feedforward source

- `setMode(PIDFTuningMode newMode)`
  - swaps active gain family and clears controller/disruption memory when needed

- `getMode()`
  - returns current logical mode

- `update(double loopTimeSeconds)`
  - main loop entry point for all velocity behavior

- `pushFinalSummary()`
  - prints final summary lines after the OpMode loop exits

Important internal methods by responsibility:

- Gain application and derived limits
  - `applyActiveGains()`
  - `updateDerivedGainState()`
  - `applyRelayComputedGains()`
  - `applyRelayHeadroomGuard(...)`

- Feedforward routing
  - `computeFeedforwardTerm(...)`
  - `resolveActivePhysicalKf()`
  - `resolveModePhysicalKf(...)`
  - `usesManualKf()`
  - `feedforwardReadyForDerivedLimits()`
  - `computeFeedforwardAtTarget()`
  - `usesConfiguredGainKf()`
  - `syncFeedforwardMode()`

- Phase and transition helpers
  - `shouldRunRelayTuning()`
  - `transitionToPostFeedforwardPhase()`
  - `resolveRelaySkipNote()`
  - `appendRelayTuneNote(...)`
  - `resolveOperationalPhase()`

- Relay-phase helpers
  - `runRelayTuningLoop(...)`
  - `runRelayCompleteLoop(...)`
  - `getDisplayedRelayMaintainGains()`
  - `getDisplayedRelayRevUpGains()`

- Characterization helpers
  - `startCharacterization()`
  - `runCharacterizationLoop()`
  - `finishCharacterization()`
  - `computeCharacterizedKf(...)`

- Hardware and signal helpers
  - `ensureMotorMode()`
  - `applyPower(...)`
  - `readAverageVelocity()`
  - `resolveProfiledTarget(...)`
  - `getPhaseElapsedSeconds()`
  - `moveToward(...)`
  - `clip(...)`

- Telemetry and UI helpers
  - `pushTelemetry(...)`
  - `buildStatusBlock(...)`
  - `resolveDisruptionInstruction()`
  - `formatTelemetryDouble(...)`

### Velocity tuner inner helper types

- `GainSet`
  - immutable holder for `kP`, `kI`, `kD`, `kF`

- `DisruptionPhase`
  - state machine that times disturbance recovery
  - owns sample counts, timeouts, and summary generation

- `RelayAutoTuner`
  - self-contained relay auto-tuning controller
  - owns relay state, zero-crossing detection, oscillation metrics, and computed gains

- `Stage`
  - disruption-stage enum

- `RelayState`
  - relay auto-tune stage enum

- `CrossingState`
  - relay zero-crossing side enum

- `TunerPhase`
  - top-level velocity tuner phase enum

### `VelocityPIDFTuner.Config`

The config class is the public API for building a velocity tuning session.

Builder methods:

- `target(double)`
  - required
  - velocity setpoint in `ticks/s`

- `tuningMode(PIDFTuningMode)`
  - initial active mode

- `withMotors(DcMotorEx...)`
  - required
  - sets the controlled motors

- `withRunUsingEncoderVelocityMotors(DcMotorEx...)`
  - alias for documentation intent only
  - the tuner still runs them externally

- `revUpGains(kP, kI, kD, kF)`
  - optional manual `REV_UP` gain set

- `maintainGains(kP, kI, kD, kF)`
  - optional manual `MAINTAIN` gain set

- `skipRelayTuning()`
  - explicit no-op for the default path, where relay tuning is already off

- `useRelayTuning()`
  - enables the optional relay oscillation pass after model-based characterization

- `relayAmplitude(double)`
  - sets relay strength

- `relayHysteresisBandPct(double)`
  - sets relay deadband fraction

- `relayDetune(double)`
  - multiplies the relay-computed gains

- `skipCharacterization(double manualKF)`
  - bypasses startup characterization and uses this physical `kF`

- `integralSumMax(double)`
  - manual anti-windup override

- `derivativeAlpha(double)`
  - derivative filter tuning

- `velocityRampTicksPerSecPerSec(double)`
  - slew limits `REV_UP` target changes

- `runDisruptionPhase(boolean)`
  - enables recovery timing

- `disruptionSamples(int)`
- `disruptionReadyStableMs(long)`
- `disruptionDetectTimeoutMs(long)`
- `disruptionRecoveryTimeoutMs(long)`
- `disruptionReadyBandPct(double)`
- `disruptionDropThresholdPct(double)`
  - tune disruption behavior

- `realDisruptionRefineIterations(int)`
- `realDisruptionRefineSamples(int)`
  - currently reserved placeholders

- `telemetry(Telemetry)`
  - required

Validation checks:

- target present
- motors present
- telemetry present

## `PositionPIDFTuner.java`

The position tuner is broader than the velocity tuner because it supports multiple actuator families.

### Core responsibility

It tunes position control for:

- DC motors
- CR servos with feedback encoders
- standard servos in open-loop mapping mode
- standard servos in closed-loop mode using external encoder or analog feedback

This class is effectively a multi-backend position control framework under one API.

### Major subsystems inside the class

1. Actuator-family abstraction
2. Feedback-source abstraction
3. Position normalization
4. Optional trapezoidal motion profiling
5. Static, gravity, and cosine feedforward
6. Optional hard position constraints
7. At-target qualification
8. Disruption testing
9. Telemetry and diagnostics

### Actuator modes

The tuner uses `ActuatorMode` internally:

- `MOTOR`
- `SERVO_OPEN_LOOP`
- `SERVO_WITH_EXTERNAL_ENCODER`
- `CR_SERVO`

These modes determine:

- how measurements are read
- what output API is used
- whether a motion profile is meaningful
- whether disruption logic is available

### Servo feedback modes

For standard servos, feedback may be:

- `NONE`
- `MOTOR_ENCODER`
- `ANALOG_INPUT`

This allows the same tuner to cover:

- pure open-loop servo mapping
- servo with external encoder
- servo with analog position sensor

### Why position is normalized internally

Unlike the velocity tuner, the position tuner scales setpoint and measurement by a move-specific distance scale:

- `moveScaleTicks`

This keeps gains from feeling wildly different for tiny moves vs large moves. If the controller used raw ticks directly for every move, a very large arm move and a tiny trim move would feel like completely different controllers unless the gains were adjusted constantly.

The normalization scale is based on:

- actual move distance
- tolerance floor

That balance makes small moves less twitchy without hiding units completely from the rest of the system.

### Position tuner update flow

The core `update()` logic is:

1. handle `SERVO_OPEN_LOOP` separately if needed
2. read feedback position
3. update motion profile
4. apply active gains
5. normalize target and measurement
6. compute PID output
7. compute feedforward
8. apply actuator-specific output
9. update at-target counter
10. update disruption phase
11. push telemetry

### Standard servo open-loop mode

This is the simplest mode.

Behavior:

- no feedback control
- no PID correction
- target is mapped directly into servo position space

Purpose:

- useful when you just want safe endpoint mapping and a direct command path

Because there is no feedback loop:

- `isAtTarget()` is forced true
- disruption sampling is unavailable

### Motor and CR servo closed-loop modes

These are the more traditional position-control cases.

Behavior:

- mechanism position is measured from encoders
- PID runs in normalized position space
- output is power-like actuator command

Differences:

- motors use `setPower()` directly
- CR servos use `setPower()` scaled by `servoOutputScale`

### Standard servo closed-loop mode

This mode blends:

- a direct target-to-servo-position mapping
- a PID correction layered on top

Process:

1. map target ticks to base servo position
2. compute PID correction in normalized position space
3. add feedforward
4. add correction on top of base position
5. clamp into `[0.0, 1.0]`

This is useful when the servo is position-controlled by hardware internally but you still want better accuracy, external sensor feedback, or compensation terms.

### Motion profiling

The position tuner can use a trapezoidal profile in `REV_UP` mode.

Purpose:

- avoid slamming a mechanism with an instantaneous position step
- provide smoother, more controlled motion

Enabled by:

- `useMotionProfile(maxVelocity, maxAcceleration)`

Implemented by:

- `updateProfile(...)`
- `calculateMotionState(...)`
- `MotionState`

If disabled:

- the profiled target simply equals the requested target

### Position feedforward model

The position tuner has three feedforward-like pieces:

- static trim from `kF`
- constant gravity term from `kG`
- angle-dependent cosine term from `kCos`

These are assembled by:

- `computeNormalizedFeedforward()`

#### Static trim

`computeStaticTrimTerm()` applies:

- `kF * sign(error)`

This is intentionally not a velocity feedforward. It is a friction-breakaway or stiction-help term.

#### Gravity constant

Useful for:

- elevators
- vertical slides

It adds a constant bias regardless of position.

#### Cosine compensation

Useful for:

- arms
- turrets or joints where gravity torque varies with angle

It computes:

- `kCos * cos((position - zeroTicks) / ticksPerRadian)`

### Position constraints

The tuner now supports optional hard position constraints:

- `positionBounds(minTicks, maxTicks)`

This system does two things:

1. clamps requested targets into the safe window
2. suppresses any further outward command if the mechanism is already at a limit

This matters because merely clamping the target is not always enough. If the actuator is already physically at the hard stop and the controller still computes an outward output, the mechanism can keep pushing against the stop and damage itself.

Telemetry exposes:

- clamped vs requested target
- whether bounds are enabled
- current bound values
- a human-readable constraint status message

### At-target logic

The tuner does not declare success from one lucky loop. It requires several consecutive in-band loops:

- `REQUIRED_AT_TARGET_LOOPS = 5`

This reduces false "stable" declarations caused by noise or transient crossings.

### Position disruption subsystem

The position tuner also has a disruption measurement system.

Purpose:

- quantify recovery after the mechanism is pushed or disturbed away from the setpoint

It is disabled in `SERVO_OPEN_LOOP` because there is no true feedback control there.

Stages:

- `WAITING`
- `ARMED`
- `DETECTING`
- `RECOVERING`
- `COMPLETE`

### Position telemetry

The position tuner publishes:

- requested target
- clamped target
- profiled target, velocity, acceleration
- measured position
- PID terms
- feedforward breakdown
- active mode
- actuator mode
- feedback mode
- actuator command
- constraint status
- disruption metrics

That makes it possible to debug not only gains, but also motion profile behavior, servo mapping, and safety-bound enforcement.

### Position tuner method map

Important public methods:

- `PositionPIDFTuner(Config config)`
  - constructs the tuner and loads initial config

- `refreshFrom(Config config, PIDFTuningMode forcedMode)`
  - reloads actuator selection, feedback mode, gains, target, bounds, and profile settings

- `setMode(PIDFTuningMode newMode)`
  - swaps between `REV_UP` and `MAINTAIN`

- `getMode()`
  - returns the current logical tuning mode

- `isAtTarget()`
  - returns true only after the in-band loop counter is satisfied

- `update(double loopTimeSeconds)`
  - main control loop for all non-open-loop cases

- `pushFinalSummary()`
  - prints the final disruption summary

Important internal methods by responsibility:

- Open-loop path
  - `runStandardServoOpenLoop()`

- Hardware preparation and gain application
  - `prepareConfiguredHardware()`
  - `applyActiveGains()`

- Feedback and target generation
  - `readPositionMeasurement()`
  - `updateProfile(...)`
  - `calculateMotionState(...)`

- Feedforward assembly
  - `computeNormalizedFeedforward()`
  - `computeStaticTrimTerm()`
  - `resolveActivePositionKf()`

- Settling and disruption logic
  - `updateAtTargetCounter()`
  - `updateDisruptionPhase()`

- Telemetry and status
  - `pushTelemetry()`

- Output writers
  - `applyMotorPower(...)`
  - `applyCRServoPower(...)`
  - `applyStandardServoPosition(...)`

- Geometry, mapping, and safety helpers
  - `averageMotorPositions(...)`
  - `mapTicksToServoPosition(...)`
  - `constrainTargetTicks(...)`
  - `constrainClosedLoopOutput(...)`
  - `clip(...)`

### Position tuner inner helper types

- `ActuatorMode`
  - selects motor, CR servo, standard-servo-open-loop, or standard-servo-closed-loop behavior

- `ServoFeedbackMode`
  - selects no feedback, encoder feedback, or analog feedback for standard servos

- `GainSet`
  - immutable position gain holder

- `MotionState`
  - immutable holder for profile position, velocity, and acceleration at a time slice

- `DisruptionPhase`
  - position disturbance recovery timing state machine

- `Stage`
  - disruption stage enum

### `PositionPIDFTuner.Config`

This is the public surface for building a position tuning session.

Builder methods:

- `target(double)`
  - required
  - target in the same units as your feedback source

- `tuningMode(PIDFTuningMode)`
  - initial active mode

- `withMotors(DcMotorEx...)`
  - use motor position control

- `withServos(Servo...)`
  - use standard servo control

- `withServoFeedback(DcMotorEx)`
  - closed-loop standard servo using encoder feedback

- `withServoFeedbackAnalog(AnalogInput, double voltageToTicksScale)`
  - closed-loop standard servo using analog feedback

- `withServoOpenLoopRange(double minTicks, double maxTicks)`
  - defines target-to-servo-position mapping
  - required for standard servo modes

- `withCRServos(CRServo servo, DcMotorEx... feedbackEncoders)`
- `withCRServos(CRServo[] servos, DcMotorEx... feedbackEncoders)`
  - CR servo control with encoder feedback

- `servoOutputScale(double)`
  - scales final command for standard or CR servos

- `revUpGains(kP, kI, kD, kF)`
- `maintainGains(kP, kI, kD, kF)`
  - manual gain sets

- `integralSumMax(double)`
  - integral contribution cap in output units

- `derivativeAlpha(double)`
  - derivative filter alpha

- `useMotionProfile(double maxVelocity, double maxAcceleration)`
  - enables trapezoidal profile in `REV_UP`

- `positionToleranceTicks(double)`
  - in-band tolerance

- `positionBounds(double minTicks, double maxTicks)`
  - optional hard mechanical limits

- `feedforwardGravityConstant(double)`
  - constant gravity compensation

- `feedforwardCosineConstant(double)`
  - angle-based gravity compensation

- `cosineFeedforwardReference(double zeroTicks, double ticksPerRadian)`
  - encoder-angle mapping for cosine compensation

- `runDisruptionPhase(boolean)`
- `disruptionSamples(int)`
- `disruptionReadyStableMs(long)`
- `disruptionDetectTimeoutMs(long)`
- `disruptionRecoveryTimeoutMs(long)`
- `disruptionReadyBandPct(double)`
- `disruptionDropThresholdPct(double)`
  - disruption configuration

- `telemetry(Telemetry)`
  - required

Validation checks include:

- target present
- exactly one actuator family selected
- telemetry present
- motion profile limits valid
- optional position bounds valid
- servo output scale nonnegative
- cosine reference present when `kCos` is nonzero
- standard servo mapping range valid
- only one standard-servo feedback source chosen
- analog feedback scale nonzero when used
- CR servo feedback encoders provided and count-valid

## Sample OpModes

The package includes three sample tuners.

### `TuneFlywheelNew.java`

Purpose:

- demonstrates velocity tuning for a two-motor flywheel

Configuration highlights:

- exposes `TARGET_VELOCITY`
- exposes `TUNING_MODE`
- uses two outtake motors
- reverses the left motor
- enables disruption sampling with five samples

This is the main example for the velocity tuner.

### `TuneArm.java`

Purpose:

- demonstrates motor position tuning for an arm

Configuration highlights:

- exposes target position
- uses `REV_UP` by default
- enables motion profiling
- enables cosine feedforward
- provides cosine reference calibration

This is the main example for angle-dependent gravity compensation.

### `TuneElevator.java`

Purpose:

- demonstrates motor position tuning for an elevator or slide

Configuration highlights:

- exposes target position
- uses `MAINTAIN` by default
- enables constant gravity feedforward

This is the simplest example of a vertical load-holding position controller.

## Typical Engineering Workflows

### Velocity mechanism workflow

Recommended flow:

1. Start with a sample OpMode like `TuneFlywheelNew`
2. Set target speed
3. Let characterization compute `kF`, `tau`, and model-based PIDF
4. Let the tuner run with those computed starting gains
5. Test `REV_UP`
6. Test `MAINTAIN`
7. Enable `useRelayTuning()` only if the model-based result still needs refinement
8. Run disruption sampling if the mechanism must survive disturbances
9. Copy final values into the mechanism class that will use them in production

### Position mechanism workflow

Recommended flow:

1. Pick the actuator family
2. Make sure units are meaningful and consistent
3. Set target and tolerance
4. Add gravity or cosine feedforward if needed
5. Add motion profile if step commands are too violent
6. Add position bounds if the mechanism has hard end stops
7. Tune `REV_UP` for approach behavior
8. Tune `MAINTAIN` for holding behavior
9. Use disruption testing if hold robustness matters

## Important Differences Between the Two Tuners

Velocity tuner:

- uses raw `ticks/s`
- has characterization
- has relay auto-tuning
- treats `kF` as physical feedforward
- measures disturbance recovery in velocity space

Position tuner:

- uses normalized move scaling internally
- supports motors and servos
- supports open-loop and closed-loop servo flows
- supports motion profiles
- supports gravity and cosine compensation
- treats `kF` as static trim, not velocity feedforward
- supports optional hard position bounds

## Common Extension Points

Teams extending this package will most likely touch:

- `Config` builder surfaces
- telemetry keys and status blocks
- sample OpModes
- relay formulas or acceptance criteria
- motion profile shape
- additional safety constraints
- final summary formatting

Good extension candidates:

- package-level helper methods for writing tuned gains back to mechanism classes
- more sample OpModes
- automatic export of final values
- richer position summaries
- more actuator-specific safety hooks

## Practical Notes and Caveats

- The velocity tuner expects to own motor power control completely.
- The velocity tuner must keep motors in `RUN_WITHOUT_ENCODER`.
- A nonzero `kF` in the position tuner is not the same concept as velocity `kF`.
- Standard-servo closed-loop behavior depends heavily on the quality of external feedback.
- Motion profiling only affects `REV_UP` in the position tuner.
- Disruption phases are meaningful only when feedback exists.
- Dashboard defaults are only starting points. Mechanism-specific tuning still matters.

## Summary

`finalPIDPilot` is not just a pair of PID classes. It is a tuning framework built around:

- real-time Dashboard interaction
- explicit `REV_UP` vs `MAINTAIN` behavior
- strong telemetry
- physically meaningful feedforward separation
- mechanism-specific tuning paths
- safety features like position bounds and forced external motor control

If you understand:

- `PIDFController`
- `PIDFTunerOpMode`
- `VelocityPIDFTuner`
- `PositionPIDFTuner`

then you understand the package. The three sample OpModes simply show how to bind those generic systems to real robot hardware.
