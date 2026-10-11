// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // These variables affect only the REAL runmode (on an actual robot).
  // They allow you to prevent subsystems that don't exist from being initialized,
  // skipping over looking for hardware that doesn't exist.
  public static boolean driveHardwareExists = true;
  // Off: Limelight heading corrections are suspected of pulling field-centric off. Turn back on
  // once the cameras' mounting poses are set correctly in the Limelight web UI.
  public static boolean visionHardwareExists = false;
  public static boolean intakeHardwareExists = true;
  public static boolean indexerHardwareExists = true;
  public static boolean shooterHardwareExists = true;
  // TODO(template): add one flag per mechanism subsystem as you build it, and gate its REAL-branch
  // construction in RobotContainer on the flag. Flipping one to false substitutes the blank-IO
  // constructor, which lets you run the rest of the robot on a partially-assembled chassis.

  // For interplut & tunable double
  public static boolean kTuningMode = true;

  // Timing instrumentation gate. Keep false in normal operation to avoid loop-time overhead.
  public static final boolean kEnableLoopTimingLogs = true;
}
