# AdvantageKit — notated reference

**Version in this repo's build:** AdvantageKit `26.0.0` (`vendordeps/AdvantageKit.json`).
**Raw source:** `_src/advantagekit/docs/docs/` (Mechanical-Advantage/AdvantageKit, `main`).
**Upstream:** https://docs.advantagekit.org

> Read this with `advantagescope.md` (the viewer) and `wpilib/telemetry.md` (the alternative,
> simpler WPILib logging). AdvantageKit is *logging + deterministic replay*; AdvantageScope is
> *viewing*, and neither requires the other.

---

## 1. The one idea

Ordinary FRC logging records **a chosen subset of values** the code decides to publish. The
eternal failure mode is "if only we'd logged one more field."

AdvantageKit instead records **every input flowing into the robot code** — every sensor value,
every joystick byte, every NT read — once per loop cycle. After a match you replay that log
through the same code in a simulator. Because every input is byte-identical and every timestamp
is replayed, **all internal logic re-executes exactly**. You can then add `recordOutput` calls
after the fact and see values you never thought to log during the match.

That property is called **deterministic replay**, and it is the entire reason the IO-layer
boilerplate exists. Everything below is in service of it.

**Consequence to internalize:** anything that can reach the robot code without passing through
the logger breaks replay. The rules in §5 are not style preferences — they are correctness
constraints.

---

## 2. Structure: the IO layer

A traditional subsystem has three parts: public interface, control logic, hardware interface.
AdvantageKit splits the hardware interface into a separate object, and logs at that seam.

```
[ public interface ]  ← commands from the rest of the robot
[ control logic    ]  ← replayed exactly
------ LOG BOUNDARY ------
[ IO layer         ]  ← vendor libraries, never replayed
```

**Interface** — defines outputs as plain methods, inputs as a single struct filled by
`updateInputs`:

```java
public interface ExampleIO {
    @AutoLog
    class ExampleIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
    }
    default void updateInputs(ExampleIOInputs inputs) {}   // default impl = no hardware
    default void setVoltage(double volts) {}
}
```

**Subsystem** — holds an IO instance and an inputs instance, and once per cycle:

```java
io.updateInputs(inputs);
Logger.processInputs("ExampleSubsystem", inputs);   // records on real, INJECTS from log on replay
```

Everything downstream reads `inputs.*`, never the IO object directly. Two guarantees fall out:
the logger sees all input data, and the cache never changes *mid-cycle*, so replayed data looks
identical to real data.

**Selection** — `RobotContainer` picks the implementation; a bare `new ExampleIO() {}` anonymous
class is the do-nothing/replay implementation.

> **In this codebase:** `lib/subsystem/angular/AngularIO` + `AngularIOTalonFX` / `AngularIOSim`
> is exactly this pattern, generalized into a reusable `AngularSubsystem` that any rotating
> mechanism (turret, hood, flywheel, spindexer, kicker, intake pivot/rollers) is instantiated
> from. `RobotContainer`'s `switch (Constants.currentMode)` is the selection step, plus the
> `Constants.*HardwareExists` flags for partially-assembled robots.

The docs are explicit that the IO layer is a *recommendation*, not a requirement — any structure
works as long as all input data flows through an object implementing `LoggableInputs`
(`toLog`/`fromLog`).

---

## 3. Recording inputs

### `@AutoLog`

Annotate the inputs class; AdvantageKit generates `<ClassName>AutoLogged` with `toLog`/`fromLog`
written for you. **Instantiate the generated `...AutoLogged` class, not the annotated one.**
Nested loggable inputs are allowed as fields. Every supported type works except mechanism states.

```java
@AutoLog
public class MyInputs {
    public double myNumber = 0.0;
    public Pose2d  myPose  = new Pose2d();
    public MyEnum  myEnum  = MyEnum.VALUE;
}
// generates MyInputsAutoLogged with table.put(...) / table.get(...) for each field
```

**Units on inputs:** either name the field with the unit suffix (`myDistanceMeters`) or use a
`Measure` type (`Distance myDistance = Meters.of(0)`). ⚠️ **Inputs always serialize in the base
unit** regardless of what unit you constructed with — deliberate, so that two IO implementations
using different units produce identical logs. (Outputs behave differently; see §4.)

### Dashboard / NetworkTables inputs

**NT data is hardware.** `SmartDashboard.getNumber(...)` read directly will *not* replay.

Two correct approaches:
- Coprocessor data (Limelight, PhotonVision) → wrap in an IO layer like any other hardware.
- Dashboard values (auto chooser, tuning numbers) → use the AdvantageKit wrappers, which handle
  the logging/replay round-trip:
  - `LoggedDashboardChooser<T>` — drop-in for `SendableChooser`
  - `LoggedNetworkNumber` / `LoggedNetworkString` / `LoggedNetworkBoolean`

`LoggedDashboardChooser` can wrap an existing `SendableChooser`, which is how PathPlanner's
`AutoBuilder.buildAutoChooser()` is made replay-safe:

```java
autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
```

> **In this codebase:** `RobotContainer` uses exactly this line. `lib/LoggedTunableNumber` and
> `lib/LoggedInterpolatingTable` are the team's own equivalents of `LoggedNetworkNumber` — check
> that they are built on the AdvantageKit network classes and not raw NT, or tuning values will
> silently break replay.

For AdvantageScope's tuning mode, tunables must be published under the **`/Tuning`** table.

---

## 4. Recording outputs

Outputs are anything derivable in simulation: odometry pose, motor voltages, computed setpoints,
internal state. They cost nothing at replay time and are the primary debugging tool — *add
logging calls anywhere, they cannot perturb the replayed control logic.*

```java
Logger.recordOutput("Flywheel/Setpoint", setpointSpeed);
Logger.recordOutput("Drive/Pose", odometryPose);
Logger.recordOutput("FeederState", FeederState.RUNNING);
```

Slashes make subtables. Everything lands under `RealOutputs` (real/sim) or `ReplayOutputs`
(replay), side by side, which is what lets you diff real vs. replayed behavior.

**Structured types** — geometry and kinematics classes serialize via WPILib structs; varargs and
arrays both work:

```java
Logger.recordOutput("MyPose3dArray", poseA, poseB);
Logger.recordOutput("MySwerveModuleStates", stateA, stateB, stateC, stateD);
```

**Units on outputs** (note the contrast with inputs — outputs keep *your* unit):

```java
Logger.recordOutput("MyDistance", 3.14, "meters");   // unit as metadata string
Logger.recordOutput("MyDistance", 3.14, Meters);     // unit object
Logger.recordOutputMeasure("MyDistance", Meters.of(3.14));
Logger.recordOutput("MyDistanceMeters", 3.14);       // suffix convention, also works
```

### `@AutoLogOutput`

Logs a field or getter periodically, private members included. Key is inferred
(`ClassName/fieldName`) or set explicitly.

```java
@AutoLogOutput                       // "Example/myPose"
private Pose2d myPose = new Pose2d();

@AutoLogOutput(key = "Custom/Speeds")
public double[] getSpeeds() {...}
```

- **Key templating** — `key = "Module{index}/Speed"` substitutes another field of the same class,
  read once on the first cycle. Purpose-built for the four-swerve-module case.
- **`unit = "meters"`** — attaches unit metadata; `Measure` fields carry theirs automatically.
- **`forceSerializable = true`** — serialize an enum as a struct/protobuf instead of a name string.

**Constraints that bite:** the annotated class must be reachable by a recursive field search from
`Robot` **and** instantiated within the first loop cycle. Classes outside `Robot`'s package need
`AutoLogOutputManager.addPackage("frc.lib")`; odd cases need `addObject(this)` during init.
Anything not fitting those rules must call `Logger.recordOutput` periodically instead.

---

## 5. Determinism rules (the part that actually breaks things)

### Timestamps

WPILib normally reads the FPGA clock on *every* `Timer.getTimestamp()` call, so every call in a
cycle returns something slightly different — unloggable, unreplayable. AdvantageKit reads the
clock **once at the top of each loop** and injects that value into WPILib.

| Use this | For |
|---|---|
| `Timer.getTimestamp()`, `RobotController.getTime()` | all control logic — deterministic, replayed |
| `Timer.getMonotonicTimestamp()`, `RobotController.getMonotonicTime()` | **only** inside IO impls and performance profiling — real FPGA time, never replayed |

Where you genuinely need sub-cycle precision, the right answer is not the raw clock — it is to
**record the measurement's own timestamp as an input**. Both NT subscriptions and Phoenix 6
`StatusSignal`s expose per-sample timestamps; feed those into odometry/pose estimation. Measuring
when the *sample* was taken beats measuring when the RIO happened to read it.

Deterministic timestamps can be disabled globally, but only if all three hold: logic depends on
intra-cycle timing, the sensors can't carry their own timestamps, and the IO is low-latency
enough that RIO-side precision matters (CAN devices generally are *not*). Escape hatch:

```java
if (!Logger.hasReplaySource()) {
    RobotController.setTimeSource(RobotController::getMonotonicTime);
}
```

### Multithreading

Robot logic must be single-threaded — thread scheduling can't be reproduced, especially across
machines. Prefer 50 Hz in the main loop or motor-controller closed loop. If a thread is truly
needed it must live **entirely inside an IO implementation**, publishing its results as inputs.

`Logger.recordOutput` and `Logger.processInputs` are **not thread-safe** — main thread only.

> **In this codebase:** `subsystems/drive/PhoenixOdometryThread` is the sanctioned form of this —
> a high-frequency sampling thread confined to the drive IO layer.

### Uninitialized inputs

Before `Logger.start()`, DS data and timestamps are **not** deterministic and must not be read.
Practical rule: **construct `RobotContainer` at the end of the `Robot` constructor**, after
`Logger.start()`. A subsystem instantiated as a field initializer runs too early.

Also: inputs are only fresh after the first `periodic()`. Don't read `inputs.*` in a constructor
expecting real values.

> **In this codebase:** `Robot()` does metadata → receivers → `Logger.start()` → `new
> RobotContainer(...)`. Correct order.

### Other non-deterministic sources to watch

- Raw FPGA timestamps (`getMonotonicTimestamp`) in logic.
- NT reads outside an IO layer / dashboard classes.
- Monolithic vendor libraries that touch hardware directly — **YAGSL** and **CTRE's Phoenix 6
  swerve API** are named explicitly. Use the AdvantageKit swerve template instead.
- RIO filesystem reads (treat the file contents as input data).
- `Math.random()` and friends.
- Iteration over unordered collections (`HashMap`, `HashSet`) — ordering differs between runs.
- `DriverStation.waitForDsConnection()` — outright incompatible.

**Testing discipline:** run replay regularly during development and confirm `ReplayOutputs`
matches `RealOutputs`. Catching a determinism break in the shop is cheap; catching it at an
event is not.

---

## 6. Supported types

Keys are slash-delimited strings. **All logged values are persistent** — a value keeps appearing
on later cycles until it is updated (same semantics as NetworkTables).

- **Simple:** `boolean, int, long, float, double, String`; their arrays; and their 2D arrays
  (plus `byte[]` / `byte[][]`).
- **Structured:** WPILib struct/protobuf types — `Translation2d`, `Pose3d`, `SwerveModuleState`,
  … — as values, arrays, or 2D arrays, usable as inputs or outputs.
- **Records:** custom Java records log as structs. Fields must be primitives, enums,
  struct-compatible types, or nested records — **no array fields**; use multiple top-level record
  arrays instead. Record class names must be globally unique or they conflict.
- **Enums:** stored as `name()` strings.
- **Colors:** WPILib `Color` → hex triplet string.
- **Suppliers (output only):** `BooleanSupplier`/`IntSupplier`/`LongSupplier`/`DoubleSupplier`
  can stand in for values — which is how a `Trigger` (it extends `BooleanSupplier`) gets logged.
- **Mechanisms (output only):** must use `LoggedMechanism2d` (+ `LoggedMechanismRoot2d`,
  `LoggedMechanismObject2d`, `LoggedMechanismLigament2d`), not the stock `Mechanism2d`.
  `generate3dMechanisms()` converts to a `Pose3d[]` for AdvantageScope articulated components.

⚠️ **First-log latency:** the *first* protobuf or record value of a given type can take **>100 ms**
to log. Subsequent ones are fast. **Log every such type once while disabled** or you will eat a
loop overrun at the worst moment. Struct logging (the default for built-in types) is exempt.

ℹ️ Measures declared as the interface type (`Measure<AngleUnit>`) need the explicit
`putMeasure()` / `getMeasure()` / `recordOutputMeasure()` methods; concrete record measures
(`Angle`, `Distance`) work with the generic ones.

---

## 7. Built-in logging (free, no setup)

Available during replay with guaranteed accuracy — safe to use directly in code:

- **Timestamp** — see §5.
- **Driver Station** — everything from `DriverStation` and the HID classes, under `DriverStation`.
- **Dashboard inputs** — when read via the AdvantageKit classes, under `NetworkInputs`.

Logged as outputs (informational, not replay-injected):

- **Alerts** — WPILib persistent alerts, viewable on AdvantageScope's line graph.
- **Console** — under `Console`; native-code output is missing in sim.
- **Radio status** — VH-109 stats every ~5 s, under `RadioStatus`.
- **Power distribution** — per-channel current under `PowerDistribution`. Works automatically for
  CTRE PDP (ID 0) / REV PDH (ID 1). Non-default IDs:
  `LoggedPowerDistribution.getInstance(50, ModuleType.kRev)` before `Logger.start()`.
  **Not supported:** CTRE PDP 2.0, AndyMark PDB.
- **System stats** — battery voltage, rails, CAN status, NT clients, under `SystemStats`.
- **Performance** — the fields to check first on a loop-overrun hunt:

| Field | Meaning |
|---|---|
| `LoggedRobot/FullCycleMS` | all periodic code; must stay under the loop period (20 ms) |
| `LoggedRobot/UserCodeMS` | your periodic code |
| `LoggedRobot/LogPeriodicMS` | AdvantageKit's own periodic cost |
| `LoggedRobot/GCTimeMS` | GC time in the cycle |
| `LoggedRobot/GCCount` | collections in the cycle |
| `Logger/QueuedCycle` | cycles backed up waiting on data receivers |
| `Logger/...MS` | per-step breakdown of AdvantageKit periodic |

> **In this codebase:** `Constants.kEnableLoopTimingLogs` gates hand-rolled `Timing/*MS` outputs
> in each subsystem's `periodic()`. Those complement `LoggedRobot/UserCodeMS` — the built-in
> field tells you *that* you're over budget, the custom ones tell you *which subsystem*.

---

## 8. Replay workflows

### Traditional replay

Config (the template's `simMode = REPLAY` path):

```java
setUseTiming(false);                       // run as fast as possible; does NOT hurt accuracy
String logPath = LogFileUtil.findReplayLog();
Logger.setReplaySource(new WPILOGReader(logPath));
Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
```

`findReplayLog()` resolution order: `AKIT_LOG_PATH` env var → the file currently open in
AdvantageScope → interactive prompt.

Launch it as a normal simulation. **The sim GUI must be disabled** (already off in the
templates). Output lands in `ReplayOutputs` beside the untouched `RealOutputs`.

> **In this codebase:** `Robot.java`'s `REPLAY` branch is exactly this, including
> `setUseTiming(false)`. To use it, flip `Constants.simMode` to `Mode.REPLAY`.

### Replay watch

`./gradlew replayWatch` (needs the custom Gradle task from the templates) re-runs replay on every
`src` change and reopens the result in AdvantageScope, preserving your time range and layout.
Built for tight tuning loops — pose estimation, vision filtering. Best on **short logs** and
fast single-core CPUs. Each iteration overwrites the previous replay log; the original is never
touched.

### The replay bubble

The hard limit worth understanding: **modified outputs cannot affect replayed inputs.** Adding
logging is unconditionally safe. *Changing* logic in replay only tells you what the new code
would have computed given the old inputs — the real robot's sensors would have responded
differently to different actuator commands. Tuning a filter on recorded vision data: valid.
Predicting how a changed drive controller would have driven: not valid.

---

## 9. SysId with AdvantageKit

Since subsystems already log their sensor data, the routine sets `logConsumer` to `null` and just
records the test state as an output:

```java
var sysIdRoutine = new SysIdRoutine(
  new SysIdRoutine.Config(null, null, null,
    (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
  new SysIdRoutine.Mechanism(
    (voltage) -> subsystem.runVolts(voltage.in(Volts)),
    null,                    // AdvantageKit is already logging the data
    subsystem));
```

⚠️ **AdvantageKit logs cannot be fed to the SysId analyzer directly.** AdvantageKit only records
field *changes* (reconstructing full timestamps at replay via the cycle list); SysId expects an
explicit sample per timestep. Conversion:

1. Open the log in AdvantageScope ≥ 3.0.2 → **File ▸ Export Data…**
2. Format **WPILOG**, timestamps **AdvantageKit Cycles**. On big logs, restrict to the prefixes
   you need.
3. Save, then open in the SysId analyzer (VSCode ▸ *WPILib: Start Tool* ▸ SysId ▸ "Open data log
   file…").

Alternative data sources that skip this entirely: AdvantageScope's **URCL** (REV) or CTRE's
**signal logger** — follow their own docs if you use them.

> **In this codebase:** `Drive.sysIdQuasistatic/sysIdDynamic` are wired into the auto chooser
> alongside two hand-rolled characterizations (`DriveCommands.wheelRadiusCharacterization`,
> `feedforwardCharacterization`).

---

## 10. Templates (reference implementations worth reading)

Under `_src/advantagekit/docs/docs/getting-started/template-projects/`: KitBot, differential
drive, **Spark swerve**, **TalonFX(S) swerve**, **vision**, skeleton. The swerve and vision
templates are the canonical IO-layer examples and are visibly the ancestors of this repo's
`subsystems/drive/` and `subsystems/vision/`.

**High-frequency odometry** (in the swerve templates): sampling drive motors/encoders/gyro faster
than the 50 Hz loop. 6328's 16-run test, 50 Hz vs 250 Hz:

| | Mean error (m) | Std dev (m) |
|---|---|---|
| 50 Hz | 0.388 | 0.180 |
| 250 Hz | 0.297 | 0.028 |

Read that carefully: mean error improved 23%, **standard deviation improved 84%**. High-frequency
odometry doesn't make autos much more *accurate* — it makes them dramatically more *consistent*.
CANivore timesync (Phoenix Pro) adds a further ~15% error / ~34% std-dev improvement per CTRE.

---

## 11. Cross-references

- Simpler alternative, no replay: WPILib data logging / Epilogue → `wpilib/telemetry.md`
- Viewer, tuning mode, export → `advantagescope.md`
- Python equivalent: PyKit (Team 1757), not used here
- Non-deterministic alternative: CTRE Hoot Replay — vendor-locked to CTRE, single CAN bus,
  accuracy degrades with replay speed. AdvantageKit's comparison page is partisan but the
  determinism argument (butterfly effect: one dropped vision frame → diverged pose → diverged
  command state → useless replay) is technically sound.
