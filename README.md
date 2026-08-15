# Robot2026 Template

A working FRC 2026 robot project with **everything except the mechanisms**. Swerve drive, vision,
logging, simulation, and the team's subsystem library are all here and running. Your job is to build
the mechanism subsystems on top.

It builds and it drives in simulation right now. Start by confirming that, then start adding.

---

## Setup

WPILib 2026 must be installed. The JDK that ships with it is the only one that works — the system
default `java` on most machines is too old and gradle will fail with a confusing message.

```bash
# Windows (git bash / WSL)
export JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"

./gradlew build          # should be BUILD SUCCESSFUL
./gradlew simulateJava   # opens the sim GUI
```

In VS Code with the WPILib extension installed, the JDK is picked up automatically and you can use
the command palette instead (`WPILib: Simulate Robot Code`).

To see the robot: open **AdvantageScope**, `File > Connect to Simulator`. The robot pose, all
subsystem inputs, and every `Logger.recordOutput` call show up as live fields.

---

## What's already here

| Area | Location | State |
| --- | --- | --- |
| Subsystem library | `lib/subsystem/` | **Done. Read this first.** |
| Swerve drive | `subsystems/drive/` | Done — real, sim, and replay IO |
| Vision | `subsystems/vision/` | Done — Limelight + PhotonVision sim, 4 cameras |
| Logging / replay | `Robot.java` | Done — AdvantageKit receivers wired for all 3 modes |
| Field constants | `constants/VisionConstants.java` | Done — 2026 AprilTag layouts |
| Drive commands | `commands/DriveCommands.java` | Done — joystick drive, trench align, SysId |
| Mechanism subsystems | `subsystems/` | **Empty. This is your work.** |
| Autos | `deploy/pathplanner/` | **Empty.** Paths were stripped; draw new ones. |

Two things you'll need that are **deliberately absent**: any `.path`/`.auto`/`.traj` files, and the
AdvantageScope layout. Both are geometry-specific and get remade every season.

---

## The architecture, briefly

Read `docs/lib-subsystem.md` for the full version. The short form:

**A mechanism is not a motor. It's a state machine over an abstracted subsystem.**

`lib/subsystem/angular/` and `lib/subsystem/linear/` give you `AngularSubsystem` and
`LinearSubsystem` — generic, already-logged, already-simulated controllers for "a thing that
rotates" and "a thing that extends." You do not write motor code. You write three config objects and
a state machine.

**The three-config model.** Every mechanism needs:

- `*SubsystemConfig` — what the mechanism *is*: gear ratio, limits, tolerance, gains. Two of these,
  one for real and one for sim, because the gains differ.
- `*IOTalonFXConfig` — how it's *wired*: CAN IDs, bus, inversion, current limits, followers.
- `*IOSimConfig` — how it's *modelled*: moment of inertia, motor type, sim gains.

These live in `constants/<mechanism>/`. Copy the shape from the drive constants and from the doc.

**The state machine.** Each subsystem owns a `*State` class enumerating the positions/velocities it
can be in, and exposes `set(State)` as a **command factory**. Bindings then read as intent:

```java
driverController.leftTrigger.whileTrue(intake.set(IntakeState.kIntaking));
```

Not `intake.setRollerVoltage(6.0)`. The subsystem decides what `kIntaking` means; the binding just
says what the driver wants. This is the pattern the template exists to teach — if you find yourself
writing motor commands in `RobotContainer`, back up.

**Two subsystem tiers.** `RegisteredSubsystem` is a real WPILib subsystem — it can be *required* by
commands, so only one command can own it at a time. `VirtualSubsystem` gets a `periodic()` every loop
but requires nothing; use it for things that only observe (visualizers, alliance checkers).

**IO layers exist for replay.** Every subsystem talks to hardware through an `IO` interface with
`@AutoLog` inputs. AdvantageKit records those inputs; in `REPLAY` mode it feeds them back and reruns
your exact logic against a real match's data. This only works if all hardware reads go through the
IO layer. See `docs/advantagekit.md`.

---

## Where to write code

Search the project for `TODO(template)`. Every insertion point is marked. There are four files you
need to touch, in this order:

1. **`constants/<mechanism>/*Constants.java`** — new folder per mechanism. The three configs.
2. **`subsystems/<mechanism>/`** — the subsystem, its `*State` class, and its `set(...)` factories.
3. **`RobotContainer.java`** — declare the field, construct it in all three of REAL / SIM / REPLAY,
   bind the buttons. Missing the REPLAY branch is the usual mistake; it breaks log replay silently.
4. **`commands/RobotSuperstructure.java`** — anything needing two subsystems to agree, plus
   `registerAutoCommands()` for anything an auto path calls by name.

Add a `Constants.<mechanism>HardwareExists` flag for each one. Setting it false swaps in the blank-IO
constructor, which is how you run the rest of the robot on a half-built chassis.

---

## Gotchas

- **`JAVA_HOME`.** See above. It is the cause of most "gradle is broken" reports.
- **Sim controller axes are not the real axes.** `lib/controller/XboxControllerSim.java` uses a
  non-standard axis mapping (`kLeftX=0, kLeftY=1, kRightX=2, kRightY=3, kRightTrigger=4,
  kLeftTrigger=5`) because the sim GUI enumerates them differently. If a stick does the wrong thing
  in sim but is fine on the robot, that's why.
- **`Vision.periodic()` runs ~27ms in sim** and will trigger loop-overrun warnings. That's the
  PhotonVision sim doing camera raycasts, not your code. It does not happen on the real robot.
- **`AutoBuilder.buildAutoChooser()` reads the deploy directory at construction time.** Every name
  an `.auto` file references must already be registered by `registerAutoCommands()`, which is why
  that call comes first in `RobotContainer`'s constructor.
- **`.asProxy()` on auto commands.** If a command requires a subsystem the path-following command
  doesn't, and you don't proxy it, the scheduler cancels the path. Silent and confusing.

---

## Docs

- `docs/lib-subsystem.md` — the subsystem library, in full. The important one.
- `docs/advantagekit.md` — logging, `@AutoLog`, replay, tuning workflow.
