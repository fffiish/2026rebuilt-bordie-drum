# `frc.robot.lib.subsystem` — the abstracted mechanism library

The team's own layer sitting between WPILib command-based and the vendor APIs. It exists so that
a new mechanism — turret, hood, flywheel, spindexer, kicker, intake pivot, elevator, climber — is
**a config object and nothing else**. No new subsystem class, no new IO class, no new simulation
model, no new logging or tuning plumbing.

This is the part of the codebase that should survive a game change. `subsystems/` gets rewritten
every January; `lib/` shouldn't.

Related: `advantagekit.md` §2 (the IO pattern this implements), `wpilib/command-based.md` §4–5
(the `Subsystem`/`Command` contracts it fulfills).

---

## 1. Shape of the library

```
lib/subsystem/
├── RegisteredSubsystem.java      abstract base: implements Subsystem, self-registers
├── VirtualSubsystem.java         RegisteredSubsystem with no hardware of its own
├── DeviceConnectedStatus.java    (connected, canId) pair, struct-serializable
│
├── angular/                      rotating mechanisms — position in Angle, velocity in AngularVelocity
│   ├── AngularIO                 interface + @AutoLog inputs
│   ├── AngularIOTalonFX          real: Phoenix 6, master + followers, optional remote CANcoder
│   ├── AngularIOSim              sim: PivotSim + ProfiledPIDControllers + current-limit model
│   ├── AngularSubsystem          the Subsystem: command factories, tuning, tolerance, alerts
│   ├── AngularSubsystemConfig    control-level config (gains, tolerances, log key, bus)
│   ├── AngularIOTalonFXConfig    hardware config (CAN IDs, ratios, limits, soft limits)
│   ├── AngularIOSimConfig        physics config (motor, MOI, arm length, travel limits)
│   ├── AngularIOOutputMode       what the IO is doing now
│   ├── AngularSubsystemOutputMode  what was asked of the subsystem
│   └── SignalIOManager           static per-bus Phoenix signal refresh
│
├── linear/                       extending mechanisms — position in Distance, velocity in LinearVelocity
│   └── (exact mirror of angular/, minus velocity control and remote sensors)
│
└── sensor/
    ├── canrange/                 CANrange ToF distance sensor → thresholded Trigger
    └── currentsensor/            derived sensor: current draw over a threshold → Trigger
```

`angular` and `linear` are deliberately **parallel, not shared**. The duplication buys type
safety: an `AngularSubsystem` cannot be handed a `Distance`, and units are enforced by the
compiler rather than by convention. See §9 for where they diverge.

---

## 2. The three-layer split

Every mechanism is described by **three separate config objects**, and this separation is the
core design idea:

| Config | Answers | Lives as long as |
|---|---|---|
| `*SubsystemConfig` | *How should this thing be controlled?* — gains, tolerances, log key, CAN bus | the season |
| `*IOTalonFXConfig` | *What hardware is it?* — CAN IDs, followers, gear ratios, current limits, soft limits, inversion | the robot |
| `*IOSimConfig` | *What physics is it?* — motor type, count, MOI or carriage mass, travel limits, arm length | the model |

`RobotContainer` picks the IO implementation per runtime mode and pairs it with the shared
subsystem config:

```java
// real
new AngularSubsystem(new AngularIOTalonFX(TurretConstants.kTalonFXConfig),
                     TurretConstants.kSubsystemConfigReal);
// sim
new AngularSubsystem(new AngularIOSim(TurretConstants.kSimConfig, currentDrawCalculatorSim),
                     TurretConstants.kSubsystemConfigSim);
// replay / hardware absent
new AngularSubsystem(new AngularIO() {}, TurretConstants.kSubsystemConfigReal);
```

That last line is why every `AngularIO` / `LinearIO` method has a **default no-op body**: an
anonymous empty implementation is a complete, valid IO layer. It's what runs in replay (where
inputs come from the log) and on a partially-assembled robot.

All configs are Lombok `@Builder @Getter` with `@Builder.Default` on everything optional, so a
mechanism declaration reads as a list of only the properties that actually differ from sane
defaults. Fields that need to be retuned live carry `@Setter`; genuinely fixed hardware facts
(CAN IDs, gear ratios, inversion) are `final` with no setter — **the type system encodes what is
safe to change at runtime.**

⚠️ The subsystem config's gains, tolerances, and constraints are `@Setter` because
`AngularSubsystem` writes tuner values back into them. The IO configs duplicate the gain fields
because the IO also needs a mutable copy to rebuild vendor configuration from. Both copies are
kept in sync by `setPIDVG` / `setPIDG`; **there is no single source of truth for gains at
runtime**, which is worth knowing when debugging a value that "won't change."

---

## 3. `RegisteredSubsystem` and `VirtualSubsystem`

```java
public abstract class RegisteredSubsystem implements Subsystem {
  protected RegisteredSubsystem() { this.register(); }
}
public class VirtualSubsystem extends RegisteredSubsystem {}
```

Two lines that do real work. Implementing the bare `Subsystem` interface (rather than extending
`SubsystemBase`) skips the `Sendable`/SmartDashboard machinery that AdvantageKit makes redundant,
but it also skips `SubsystemBase`'s automatic `register()` call — without which `periodic()` never
runs. `RegisteredSubsystem` restores that in the constructor.

The distinction between the two names is intent, not behavior:

- **`RegisteredSubsystem`** — owns hardware. `AngularSubsystem`, `LinearSubsystem`,
  `CANRangeSubsystem`.
- **`VirtualSubsystem`** — owns no hardware; coordinates other subsystems or derives values. Gets
  a `periodic()` slot in the scheduler and participates in requirement arbitration, but its
  actuators belong to the mechanism subsystems it composes.

This is the pattern that makes a two-tier structure work: a `VirtualSubsystem` (say a shooter)
holds three `AngularSubsystem`s (turret/hood/flywheel), computes setpoints in its `periodic()`,
and lets each mechanism's **default command** track those setpoints through a supplier. The
mechanism subsystems are the scheduler's resource unit; the virtual subsystem is the semantic
unit. Commands that need only the flywheel don't block the turret.

⚠️ Consequence: a `VirtualSubsystem` and the `AngularSubsystem`s it owns are *separate*
requirements. Requiring the virtual one does not reserve the mechanisms, and vice versa. That's
usually what you want, but it means nothing stops another command from grabbing the flywheel
while the shooter thinks it owns it.

---

## 4. `AngularIO` / `LinearIO` — the logged boundary

```java
public interface AngularIO {
  default void updateInputs(AngularIOInputs inputs) {}

  @AutoLog
  class AngularIOInputs {
    public AngularIOOutputMode IOOutputMode = kNeutral;
    public Angle    angle;      public AngularVelocity velocity;
    public AngularAcceleration acceleration;
    public Voltage  appliedVolts;
    public Current  supplyCurrent, statorCurrent;
    public NeutralModeValue neutralMode;
    public double[] motorTemperatures;
    public DeviceConnectedStatus[] deviceConnectedStatuses;
    public Angle goalPos;  public AngularVelocity goalVel;      // what was commanded
    public Angle referencePos; public AngularVelocity referenceVel;  // where the profile is now
  }

  default void setAngle(Angle angle) {}
  default void setAngle(Angle angle, Voltage feedforward) {}
  default void setVelocity(AngularVelocity velocity) {}
  default void setOpenLoop(Voltage voltage) {}
  default void stop() {}
  default void resetAngle() {}         default void resetAngle(Angle angle) {}
  default void setPIDVG(double kP, double kI, double kD, double kV, double kG) {}
  default void setConstraints(AngularVelocity cruiseVelocity, AngularAcceleration acceleration) {}
  default void setNeutralMode(NeutralModeValue neutralMode) {}
  default void setLogKey(String logKey) {}
}
```

**Setters are plain methods; inputs are one struct.** That asymmetry is AdvantageKit's rule —
outputs are commands (fire-and-forget, recreated in replay by the same logic), inputs are the
one thing that must be captured and re-injected.

Three details worth calling out:

**Goal vs. reference.** `goalPos`/`goalVel` is what user code asked for. `referencePos`/
`referenceVel` is where the motion profile currently *is* — Phoenix's closed-loop reference, or
the `ProfiledPIDController` setpoint in sim. Logging both means a plot immediately distinguishes
"the profile is too slow" from "the mechanism isn't tracking the profile," which is the first
question in every tuning session.

**`IOOutputMode` vs `SubsystemOutputMode`.** Two enums, deliberately kept separate (both files
carry a javadoc note saying so):

| `AngularIOOutputMode` (what the hardware is doing) | `AngularSubsystemOutputMode` (what was asked) |
|---|---|
| `kNeutral`, `kClosedLoop`, `kOpenLoop`, `kVelocity` | `kClosedLoop`, `kOpenLoop`, `kVelocity`, `kHoldAtCall`, `kHoldAtGoal` |

`kHoldAtCall` and `kHoldAtGoal` both reach hardware as `kClosedLoop`; the distinction only exists
at the subsystem level, and only for intent. The IO enum is an *input* (logged, replayed); the
subsystem enum is derived state. Keeping them apart avoids leaking a subsystem-level concept into
the replay boundary.

**Units in the interface, not doubles.** Every method takes `Angle`/`AngularVelocity`/`Voltage`.
⚠️ Remember AdvantageKit serializes `Measure` **inputs** in base units regardless of the unit you
construct with — so these all log as radians/rad·s⁻¹ no matter what the IO implementation uses
internally. Deliberate: two IO implementations can't produce differently-scaled logs.

`setLogKey` exists solely so the IO can put a mechanism name into its `Alert` text. It's called
once from the subsystem constructor.

---

## 5. `AngularSubsystem` / `LinearSubsystem` — the command layer

Constructor does four things, in order:

```java
this.io = io; this.config = config; this.logKey = config.getLogKey();
setTunable();               // build every LoggedTunableNumber
io.setLogKey(logKey);
io.resetAngle();            // zero to the configured reset position
setDefaultCommand(holdAtCall());
```

⚠️ **The default command is `holdAtCall()`** — on construction and any time nothing else requires
the mechanism, it latches the *current* measured position and holds it closed-loop. Not zero
voltage, not stop. A mechanism with no active command actively resists being moved. That's the
right default for an arm or hood; be aware of it for a flywheel or roller, which is why those get
an explicit default command from their owning subsystem.

### Command factory catalogue

Every factory returns a **new command each call** (the pattern `wpilib/command-based.md` §8
recommends), and each is built as:

```java
parallel(<the action>, setOutputMode(<mode>))
```

so the subsystem's `outputMode` field is updated as a side effect of scheduling.

| Factory | Sets | Behavior |
|---|---|---|
| `angle(Angle)` / `length(Distance)` | `kClosedLoop` | set **once**, then `idle()` forever |
| `angle(Supplier<Angle>)` | `kClosedLoop` | set **every loop** from the supplier |
| `angle(Supplier<Angle>, Supplier<Voltage>)` | `kClosedLoop` | ditto, plus an arbitrary feedforward voltage |
| `velocity(AngularVelocity)` / `velocity(Supplier<>)` | `kVelocity` | angular only |
| `openLoop(Voltage)` / `openLoop(Supplier<Voltage>)` | `kOpenLoop` | raw voltage |
| `holdAtCall()` | `kHoldAtCall` | capture position at schedule time, hold it |
| `holdAtGoal(Supplier<Angle>)` | `kHoldAtGoal` | continuously track a supplied setpoint |
| `holdAtGoal(Supplier<Angle>, Supplier<Voltage>)` | `kHoldAtGoal` | tracking + feedforward |
| `stop()` | — | `io.stop()`, instant |
| `resetAngle()` / `resetAngle(Angle)` / `resetAngle(Supplier<Angle>)` | — | re-zero the encoder, instant |
| `setNeutralModeBrake()` / `setNeutralModeCoast()` / `setNeutralMode(...)` | — | instant |

**The once-vs-every-loop distinction is the one to internalize.** `angle(Angle)` is
`sequence(runOnce(set), idle())` — one CAN frame, then the command occupies the subsystem doing
nothing so no default command takes over. `angle(Supplier<Angle>)` is `run(set)` — a CAN frame
every 20 ms. Use the constant form for a fixed setpoint (cheaper on the bus); use the supplier
form when the target moves.

`holdAtGoal(supplier)` is just `parallel(angle(supplier), setOutputMode(kHoldAtGoal))` — the same
tracking behavior with a different label. As a default command it's the standard idiom: the
owning `VirtualSubsystem` writes a target state in its `periodic()`, and the mechanism chases it
without any command ever being explicitly scheduled.

### `periodic()`

Fixed order, and the order matters:

1. `io.updateInputs(inputs)` → `Logger.processInputs("AngularSubsystems/<logKey>", inputs)`
   — the AdvantageKit boundary. Everything after this reads `inputs`, never `io`.
2. Four `LoggedTunableNumber.ifChanged(...)` blocks — gains, motion constraints, position
   tolerance, velocity tolerance. Each writes the new value into `config` **and** pushes it to the
   IO via `setPIDVG`/`setConstraints`.
3. Recompute `isAtAngle` (below) and log it.
4. Scan `deviceConnectedStatuses`; raise a WPILib `Alert` naming the disconnected CAN IDs and bus.
5. Accumulate timing into static counters.

Step 2 is why tuning works without a redeploy: change a number in AdvantageScope, and the next
`periodic()` reconfigures the Talon. ⚠️ `ifChanged` is keyed on `hashCode()`, so each instance
tracks its own tunables — correct, but it means the four blocks share one key per subsystem and
each block must list every tunable it depends on.

### Tolerance / `isAtAngle`

```java
if (outputMode == kOpenLoop)      isAtAngle = false;                       // meaningless open-loop
else if (outputMode == kVelocity) isAtAngle = |goalVel - velocity| < velocityTolerance;
else                              isAtAngle = |goalPos - angle| < positionTolerance
                                           && |velocity| < velocityTolerance;
```

Note the position case requires **both** near-setpoint **and** near-zero velocity — "at angle"
means settled, not merely passing through. Tolerances default to `+∞`, i.e. always at goal, so a
mechanism that never declares tolerances will report ready immediately. Exposed as
`isAtAngle()` and as a `Trigger` via `atAngle()`.

`LinearSubsystem.isAtLength` is the same, expressed as `outputMode != kOpenLoop && near(goal) &&
near(0, velocity)` — equivalent, since linear has no velocity mode.

### Diagnostics for free

Every mechanism instantiated through this layer automatically gets:

- **`AngularSubsystems/<logKey>/*`** — the full input struct, replayable.
- **`AngularSubsystems/<logKey>/AtAngle`** — settled flag.
- **`AngularSubsystems/<logKey>/{KP,KI,KD,KV,KG,KS,CruiseVelocity…,PositionTolerance…}`** — live
  tunables.
- **A disconnection `Alert`** naming exact CAN IDs and bus, surfaced on the driver dashboard and
  logged by AdvantageKit automatically.
- **Aggregate timing** — `recordAndResetTiming()` is a *static* method called once per loop from
  `Robot.robotPeriodic()`, publishing `Timing/AngularSubsystems/InputUpdateMS` and
  `.../SubsystemCodeMS` summed across **all** instances. That split — time spent waiting on CAN
  versus time spent in this code — is exactly the split you need when chasing a loop overrun.

Getters pass through the cached inputs: `getAngle`, `getGoalPos`, `getVelocity`, `getGoalVelocity`,
`getAcceleration`, `getSupplyCurrent`, `getStatorCurrent`, `areAllDevicesConnected`.

---

## 6. `AngularIOTalonFX` — the real implementation

**Construction:** build master + followers on the configured `CANBus` → clear sticky faults →
`setControl(new Follower(masterId, Aligned|Opposed))` on each follower → build and apply a
`TalonFXConfiguration` through `PhoenixUtils.tryUntilOk` → raise a config-failure `Alert` if any
apply failed.

Followers get the **same** configuration as the master **except soft limits, which are explicitly
disabled** on followers. Correct: the follower mirrors the master's output, and a soft limit
tripping independently on a follower would fight it.

### The gearing model

Two independent ratios, and confusing them is the most likely source of a mechanism that moves
the wrong distance:

- **`motorRotationsPerOutputRotations`** → Phoenix `Feedback.SensorToMechanismRatio`. The gearbox.
  Handled *inside* the Talon.
- **`outputAnglePerOutputRotation`** (default `Rotation.of(1.0)`) → applied *in software*, on
  every value crossing the IO boundary. Converts the mechanism's own rotations into whatever
  angular unit the subsystem talks in.

Everything is scaled by the second factor in both directions — positions, velocities,
accelerations, references on the way in; targets, cruise velocity, acceleration on the way out —
**and so are the gains** (`kP`, `kI`, `kD`, `kV`, `kA` are multiplied by it before being written
to `Slot0`; `kS` is not, correctly, since it's a static-friction constant in volts). This is what
lets you tune in the mechanism's natural units and have the numbers mean the same thing after a
gearing change.

⚠️ **`kG` is scaled in `setPIDVG` but not in `getMasterConfig()`.** At construction `Slot0.kG =
deviceConfig.getKG()`; after any live tune, `masterConfig.Slot0.kG = kG * outputAnglePerOutputRotation`.
If that factor isn't 1, gravity feedforward changes magnitude the first time anyone touches a
tuner value. Worth fixing.

### Control requests

Three pre-allocated request objects, reused (allocating per-loop would be GC pressure):

| Method | Request | Sets `outputMode` |
|---|---|---|
| `setAngle(angle[, ff])` | `MotionMagicVoltage.withPosition(...).withFeedForward(ff)` | `kClosedLoop` |
| `setVelocity(v)` | `MotionMagicVelocityVoltage.withVelocity(...)` | `kVelocity` |
| `setOpenLoop(v)` | `VoltageOut.withOutput(v)` | `kOpenLoop` |
| `stop()` | `master.stopMotor()` | `kNeutral` |

All position/velocity control is **Motion Magic**, i.e. the profile is generated on the motor
controller, not the RIO. `sensorOffset` is added to the target inside `setAngle` — meant for
remote-sensor setups where the sensor zero and mechanism zero differ.

Everything is commanded on the **master only**; followers track via the `Follower` request.

### Signals and `SignalIOManager`

The IO subscribes to position, applied volts, supply/stator current, velocity, acceleration,
closed-loop reference, reference slope, and one temperature per motor — then hands them all to a
static registry keyed by bus name:

```java
public class SignalIOManager {
  private static final Map<String, List<BaseStatusSignal>> signalsByBus = new HashMap<>();
  public static void update() {
    for (var entry : signalsByBus.entrySet())
      BaseStatusSignal.refreshAll(entry.getValue());   // one batched call per bus
  }
  public static void addSignals(String busName, BaseStatusSignal... signals) { ... }
}
```

`Robot.robotPeriodic()` calls `SignalIOManager.update()` **once, before
`CommandScheduler.run()`**. One batched `refreshAll` per CAN bus per loop instead of a refresh per
signal per subsystem — a large CAN-utilization and loop-time win, and it guarantees every
subsystem's `updateInputs` in a given cycle reads the **same synchronized snapshot**, which is an
AdvantageKit determinism requirement, not just an optimization.

Update rates: 50 Hz for the logged signals, with `master.getMotorVoltage()` bumped to
`statusFrameUpdateRate` (default 100 Hz) — the comment says "so that the followers update at a
higher rate," since followers track the master's voltage frame.

### Connection status

`DeviceConnectedStatus(connected, canId)` is struct-serializable so it can be an `@AutoLog` input
field. The master is judged by `BaseStatusSignal.isAllGood(...)` over all its signals; followers
only by their temperature signal, since that's the only one subscribed for them. Arrays are
**reused across cycles** (allocated only on first call or size change) — deliberate GC avoidance,
and the reason the code mutates the existing objects with `setConnected` rather than replacing
them.

### ⚠️ Two known traps in this file

**Soft limits are enabled by inverted conditions:**

```java
configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable =
    deviceConfig.getSoftMaxAngle().gte(Radians.of(Double.POSITIVE_INFINITY));
configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable =
    deviceConfig.getSoftMinAngle().lte(Radians.of(Double.NEGATIVE_INFINITY));
```

This enables the forward soft limit only when the max angle **is** infinite, and the reverse limit
only when the min **is** negative-infinite — the exact opposite of the intent. Any mechanism
configuring a finite `softMinAngle`/`softMaxAngle` gets **no soft limits at all**. The conditions
should be `!isInfinite`, i.e. `lt(+∞)` and `gt(−∞)`. `LinearIOTalonFX` has the identical bug.
Mechanisms currently rely on the subsystem-level clamping done by their owning code instead.

**`resetAngle(Angle)` ignores its argument on remote-CANcoder setups:**

```java
if (masterConfig.Feedback.FeedbackSensorSource == RemoteCANcoder) {
  cancoder.get().setPosition(cancoder.get().getAbsolutePosition().getValueAsDouble());
  return;   // requested angle discarded
}
```

Reasonable for an absolute sensor (the absolute reading *is* the truth), but it means
`resetAngle(someAngle)` silently does something different depending on the sensor configuration.

---

## 7. `AngularIOSim` — the physics implementation

Not a stub. It reproduces enough of the real behavior that tuning in sim transfers:

**Plant:** `PivotSim` (linear: `LinearExtensionSim`), both custom `LinearSystemSim<N2,N1,N2>`
subclasses in `lib/sim/`. `PivotSim` extends WPILib's arm model with an optional
`realAngleFromSubsystemAngleZero` supplier — the mechanism's zero can be offset by another
mechanism's live angle, which is how a hood mounted on a moving pivot gets correct gravity torque.
`setArmLength(Distance)` allows a variable-length arm, and `estimateMOI(length, mass)` is provided
for the common thin-rod case.

**Control:** two `ProfiledPIDController`s, mirroring Motion Magic on the RIO side.

- Position: PID + `setpoint.velocity * kV` + gravity FF + caller feedforward, clamped to ±12 V.
  Gravity is `kG * cos(angle)` when `kgArm` is set (arm — torque varies with angle), else a
  constant `kG` (elevator-style).
- Velocity: constraints are `(acceleration, 1e9)` — the profile's "velocity" limit is the
  acceleration limit and jerk is unbounded, i.e. a first-order profile on velocity. Slightly
  confusing but correct for spinning up a flywheel.
- ⚠️ Both controllers share the same `kP/kI/kD`. There is one gain set per mechanism, so a
  mechanism used in both position and velocity mode can't tune them independently in sim.

**Controller reset on mode change:** `setAngle` calls `posController.reset(position, velocity)`
**only if the mode wasn't already `kClosedLoop`**. Without that guard, re-issuing a setpoint each
loop would restart the profile from a standstill every cycle; with it, a supplier-driven
`holdAtGoal` produces continuous motion. Same idea in `setVelocity`.

**Current limiting** (the block commented "Current limiting by Nishant") — the part that makes sim
worth trusting. Rather than clamping torque, it models the electrical circuit:

```
backemf   = ω · gearing / Kv
desiredI  = (V_applied − backemf) / R
```

then applies the **stator** limit by clamping `desiredI`, and the **supply** limit by solving
the quadratic that falls out of `I_supply = I_stator · V_applied / V_bat`:

```
R·I² + backemf·I − I_supply_limit·V_bat = 0
```

taking the physical root, recomputing `V_applied = backemf + I·R`, clamping to ±12 V, and feeding
that to the plant. The result is a mechanism that browns out and slows under load in sim the way
it will on the robot — a saturated mechanism behaves correctly instead of magically hitting its
setpoint.

Supply current is reported back to `CurrentDrawCalculatorSim` (a `VirtualSubsystem` that sums
registered current suppliers and drives simulated battery voltage), which closes the loop: heavy
mechanisms sag `RobotController.getBatteryVoltage()`, which feeds back into every other
mechanism's supply limit.

`CustomDCMotor` supplies `DCMotor` models WPILib doesn't ship — `getKrakenX44()` and
`getKrakenX44Foc()`.

⚠️ Sim reports `acceleration = 0` always, and `motorTemperatures`/`deviceConnectedStatuses` as
empty arrays — so the disconnection alert never fires in sim (empty array trivially satisfies
`allMatch`).

⚠️ Sim advances the plant by the **fixed** `RobotConstants.kDt`, not measured elapsed time. Right
choice for determinism; it means sim physics assume an on-time loop.

---

## 8. Sensor subsystems

Both follow the same recipe: a raw measurement, a **tunable threshold**, a **tunable debounce**,
and a `Trigger` as the public API. Nothing consuming them deals in raw numbers.

### `CANRangeSubsystem`

Full IO pattern over a CTRE CANrange time-of-flight sensor.

- `CANRangeIOInputs` — `isDetected`, `distance`, `connected`.
- `CANRangeIOCANRangeConfig` — CAN id, bus, `fovRange` (default 6.75°), `updateMode` (default
  `ShortRange100Hz`), `updateFrequency` (default 100 Hz).
- `CANRangeSubsystemConfig` — `logKey`, `threshold` (default −1 in, i.e. never tripped),
  `debounce`.
- Exposes `withinThreshold`, a debounced `Trigger`, annotated
  `@AutoLogOutput(key = "CANRanges/{logKey}/WithinThreshold")` — the `{logKey}` templating
  substitutes the instance's field so multiple sensors don't collide.
- Raises a disconnection `Alert` naming the sensor.

### `CurrentSensorSubsystem`

A `VirtualSubsystem` with **no IO layer at all** — it derives a boolean from another subsystem's
already-logged current. Config is a pair of suppliers plus threshold and debounce, with two
convenience constructors:

```java
CurrentSensorSubsystemConfig.fromAngularSubsystem(subsystem, threshold, debounce, logKey);
CurrentSensorSubsystemConfig.fromLinearSubsystem (subsystem, threshold, debounce, logKey);
```

which wire `connectedSupplier = subsystem::areAllDevicesConnected` and
`currentSupplier = subsystem::getSupplyCurrent`. This is the cheap way to detect stalls, hard
stops, or game-piece contact without adding hardware — and because it consumes an already-logged
input rather than touching hardware, it is replay-safe for free.

`exceedsThreshold` requires **both** over-threshold **and** connected, so a dead CAN device reads
as "not stalled" rather than latching a false trip.

⚠️ Changing the debounce tunable **rebuilds the `Trigger` object** (`exceedsThreshold = new
Trigger(...)`). Any binding made against the old instance keeps the old debounce. Fine for live
tuning, not fine if you re-tune after binding and expect the change to take.

---

## 9. Where angular and linear differ

| | angular | linear |
|---|---|---|
| Position / velocity types | `Angle`, `AngularVelocity` | `Distance`, `LinearVelocity` |
| Velocity control mode | ✅ `kVelocity`, `MotionMagicVelocityVoltage` | ❌ position and open-loop only |
| Gains pushed to IO | `setPIDVG(kP,kI,kD,kV,kG)` | `setPIDG(kP,kI,kD,kG)` — no `kV` |
| Remote sensor | ✅ optional CANcoder (`sensorId`, `rotorRotationsPerSensorRotation`, `sensorOffset`) | ❌ integrated sensor only |
| Software unit scaling | `outputAnglePerOutputRotation` | `outputDistancePerOutputRotation` |
| Sim plant | `PivotSim` (+ MOI, arm length, angle-offset supplier) | `LinearExtensionSim` (+ carriage mass) |
| Sim gravity | `kG·cos θ` when `kgArm`, else constant | constant `kG` |
| Sim controllers | two (position, velocity) | one (position) |
| Status-frame rate config | ✅ `statusFrameUpdateRate` | ❌ fixed |
| Tunable log units | radians / rad·s⁻¹ | **inches / in·s⁻¹ / ft·s⁻²** |

That last row is a real inconsistency: angular tunables publish in radians, linear tunables in
inches, feet-per-second-squared, and inches-per-second. Both are internally consistent, but a
mixed-mechanism robot has a dashboard where the units differ per mechanism type.

`kA` exists in `AngularSubsystemConfig` and `AngularIOTalonFXConfig` but is **not** exposed as a
tunable and **not** included in `setPIDVG` — it's applied once at construction and can't be
changed live.

---

## 10. Adding a mechanism

The whole point: this is the complete list.

1. **Write a constants class** with three static configs:

```java
public class HoodConstants {
  public static final AngularIOTalonFXConfig kTalonFXConfig = AngularIOTalonFXConfig.builder()
      .masterId(21).bus(kCanivore)
      .inverted(InvertedValue.Clockwise_Positive)
      .motorRotationsPerOutputRotations(60.0)
      .outputAnglePerOutputRotation(Rotations.of(1.0))
      .supplyCurrentLimit(Amps.of(40)).statorCurrentLimit(Amps.of(80))
      .softMinAngle(Degrees.of(20)).softMaxAngle(Degrees.of(50))
      .resetAngle(Degrees.of(20))
      .kP(...).kD(...).kS(...).kV(...).kG(...).gravityType(Optional.of(Arm_Cosine))
      .cruiseVelocity(...).acceleration(...)
      .build();

  public static final AngularIOSimConfig kSimConfig = AngularIOSimConfig.builder()
      .motor(DCMotor.getKrakenX60(1)).numMotors(1)
      .moi(PivotSim.estimateMOI(Inches.of(8), Pounds.of(3)))
      .motorRotationsPerOutputRotations(60.0)
      .physicalMinAngle(Degrees.of(20)).physicalMaxAngle(Degrees.of(50))
      .resetAngle(Degrees.of(20)).kgArm(true)
      .kP(...).kV(...).kG(...).cruiseVelocity(...).acceleration(...)
      .supplyCurrentLimit(Amps.of(40)).statorCurrentLimit(Amps.of(80))
      .build();

  public static final AngularSubsystemConfig kSubsystemConfigReal = AngularSubsystemConfig.builder()
      .logKey("Hood").bus(kCanivore)
      .positionTolerance(Degrees.of(1.0)).velocityTolerance(DegreesPerSecond.of(5.0))
      .kP(...).kD(...).kS(...).kV(...).kG(...)
      .cruiseVelocity(...).acceleration(...)
      .build();
  // ...and kSubsystemConfigSim, usually differing only in gains
}
```

2. **Instantiate in `RobotContainer`**, switching on runtime mode (real / sim / replay+absent).

3. **Give it a default command** if `holdAtCall()` isn't right — usually
   `hood.setDefaultCommand(hood.holdAtGoal(() -> targetState.getHood()))` from the owning
   `VirtualSubsystem`.

4. **Nothing else.** Logging, replay, tuning, tolerance checking, disconnection alerts, timing
   instrumentation, current-limited simulation, and the whole command surface come with it.

---

## 11. Design notes to preserve

Reasons this structure is worth keeping across a game change:

- **A mechanism is data, not code.** New mechanisms cost a config, not a class. Most FRC codebases
  re-derive the same subsystem boilerplate per mechanism per year; this doesn't.
- **Three configs, three lifetimes.** Control tuning, hardware wiring, and physics modeling change
  at different rates and for different reasons; separating them means a gearbox change doesn't
  touch gains and a re-tune doesn't touch CAN IDs.
- **No-op defaults on every IO method.** `new AngularIO() {}` is a complete implementation. That
  one decision is what makes replay mode and partial-hardware robots work without conditionals
  scattered through subsystem code.
- **One batched signal refresh per bus per loop**, ahead of the scheduler. Both a performance win
  and the thing that keeps the cycle's data consistent.
- **Everything tunable is tunable live**, and every tunable is logged. Tuning at an event never
  requires a redeploy.
- **Sim models saturation, not just kinematics.** A sim that always reaches its setpoint teaches
  you nothing; this one browns out.
- **Two-tier subsystems.** Mechanism subsystems are the scheduler's resource unit; virtual
  subsystems are the semantic unit. Fine-grained requirements without fine-grained button
  bindings.
- **Triggers as the sensor API.** Threshold and debounce live with the sensor, so consumers write
  intent (`gamePieceDetected`) rather than arithmetic.

### Carried-forward fix list

- `SoftwareLimitSwitch.*Enable` conditions are inverted in **both** `AngularIOTalonFX` and
  `LinearIOTalonFX` — soft limits are effectively never on (§6).
- `kG` scaled by `outputAnglePerOutputRotation` in `setPIDVG` but not at construction (§6).
- `kA` configured but neither tunable nor live-settable (§9).
- Sim shares one gain set between position and velocity controllers (§7).
- Angular tunables publish in radians, linear in inches/feet (§9).
- Re-tuning a sensor's debounce replaces the `Trigger`, orphaning existing bindings (§8).
- Gains exist in two mutable copies (subsystem config and IO config) with no single source of
  truth at runtime (§2).
