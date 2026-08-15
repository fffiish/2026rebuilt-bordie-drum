package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.lib.subsystem.VirtualSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

/**
 * Publishes a side-view {@link LoggedMechanism2d} of the superstructure to AdvantageScope so you
 * can see what the mechanisms are doing without looking at the robot.
 *
 * <p>The convention is to construct <em>two</em> of these from {@link frc.robot.RobotContainer} —
 * one fed by each subsystem's <em>measured</em> state and one by its <em>target</em> state, in
 * {@link frc.robot.constants.RobotConstants#kMeasuredStateColor} and {@code kTargetStateColor}
 * respectively. Overlaying them in AdvantageScope makes tracking error visible at a glance, which
 * is by far the fastest way to tune a mechanism.
 *
 * <p>This extends {@link VirtualSubsystem}, not WPILib's {@code SubsystemBase}: it has no hardware
 * and requires nothing, so it must never participate in command requirements. {@link
 * VirtualSubsystem} self-registers and gets {@link #periodic()} called every loop anyway.
 *
 * <p>TEMPLATE NOTE: the mechanism is empty. Add a {@code LoggedMechanismRoot2d} plus a {@code
 * LoggedMechanismLigament2d} per joint, take a {@code Supplier<YourState>} per subsystem in the
 * constructor, and update the ligaments in {@link #periodic()}.
 */
public class SuperstructureVisualizer extends VirtualSubsystem {
  private static final double PADDING =
      22.0; // inches, so mechanisms outside the frame stay visible
  private static final double FRAME_WIDTH = 27.0; // inches

  private final LoggedMechanism2d mechanism =
      new LoggedMechanism2d(
          Inches.of(PADDING + FRAME_WIDTH + PADDING).in(Meters), Inches.of(95.0).in(Meters));

  // TODO(template): declare roots and ligaments here, e.g.
  // private final LoggedMechanismRoot2d pivotRoot;
  // private final LoggedMechanismLigament2d pivot;

  private final Supplier<Pose2d> robotPose;
  private final String logKey;

  /**
   * @param robotPose used to place 3D component poses on the field
   * @param logKey "Measured" or "Target"
   * @param mechColor color for every ligament in this instance
   */
  public SuperstructureVisualizer(Supplier<Pose2d> robotPose, String logKey, Color8Bit mechColor) {
    this.robotPose = robotPose;
    this.logKey = logKey;

    // TODO(template): build the mechanism, e.g.
    // pivotRoot = mechanism.getRoot("PivotRoot", xMeters, yMeters);
    // pivot = pivotRoot.append(
    //     new LoggedMechanismLigament2d("Pivot", lengthMeters, angleDegrees, 4.0, mechColor));
  }

  @Override
  public void periodic() {
    double start = 0.0;
    if (Constants.kEnableLoopTimingLogs) {
      start = Timer.getFPGATimestamp();
    }

    // TODO(template): push the current state into the ligaments, e.g.
    // pivot.setAngle(intakeState.get().getPivot().in(Degrees));

    Logger.recordOutput(String.format("Superstructure/%s", logKey), mechanism);
    Logger.recordOutput(String.format("Superstructure/%sRobotPose", logKey), robotPose.get());

    if (Constants.kEnableLoopTimingLogs) {
      double end = Timer.getFPGATimestamp();
      Logger.recordOutput("Timing/SuperstructureVisualizerMS", (end - start) * 1000);
    }
  }
}
