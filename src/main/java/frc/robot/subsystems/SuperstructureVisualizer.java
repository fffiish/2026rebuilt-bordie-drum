package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.lib.subsystem.VirtualSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Draws a side-on stick figure of the superstructure and publishes it to AdvantageScope.
 *
 * <p>Two of these get built in {@link frc.robot.RobotContainer}: one fed by each mechanism's
 * <em>measured</em> state, one by its <em>target</em> state, in different colours. Overlaid in
 * AdvantageScope, the gap between them <em>is</em> the tracking error — a lagging arm, an
 * overshooting hood, a mechanism that never arrives. Reading that off a picture is far faster than
 * reading it off a number, which is why this is the main tool during the tuning pass.
 *
 * <p>Extends {@link VirtualSubsystem}, not {@code SubsystemBase}: it owns no hardware and must
 * never take part in command requirements, but still needs a {@link #periodic()} every loop.
 *
 * <p><b>The geometry below is placeholder.</b> It is cosmetic — wrong numbers make a wrong-looking
 * picture, not a wrong robot — but the picture is only useful if it resembles the machine. Measure
 * the five {@code TODO(bringup)} values in CAD and the drawing becomes accurate.
 */
public class SuperstructureVisualizer extends VirtualSubsystem {
  private static final double PADDING = 22.0; // in, so mechanisms outside the frame stay visible
  private static final double FRAME_WIDTH = 27.0; // in TODO(bringup): your frame
  private static final double CANVAS_HEIGHT = 95.0; // in

  // ---------------------------------------------------------------------------------------------
  // Placeholder geometry. All five are measured against two physical datums you can find on the
  // real robot or in CAD, so nobody has to think in canvas coordinates:
  //
  //     "behind front"  = inches back from the FRONT frame rail (the intake end)
  //     "above floor"   = inches up from the ground
  //
  // Wrong numbers draw a wrong-looking picture, not a wrong robot — but the picture is only useful
  // if it resembles the machine.
  // ---------------------------------------------------------------------------------------------

  /** Intake arm pivot axis. */
  private static final double ARM_PIVOT_BEHIND_FRONT = 2.0; // TODO(bringup): measure

  private static final double ARM_PIVOT_ABOVE_FLOOR = 10.0; // TODO(bringup): measure

  /** Pivot axis to the far end of the intake rollers. */
  private static final double ARM_LENGTH = 14.0; // TODO(bringup): measure

  /** Hood pivot axis. */
  private static final double HOOD_PIVOT_BEHIND_FRONT = 23.0; // TODO(bringup): measure

  private static final double HOOD_PIVOT_ABOVE_FLOOR = 30.0; // TODO(bringup): measure

  private static final double HOOD_LENGTH = 10.0;

  /** Height of the drawn frame rail above the floor — cosmetic only. */
  private static final double FRAME_RAIL_ABOVE_FLOOR = 2.0;

  /**
   * A ligament's angle is measured counter-clockwise from "pointing right". The arm's own zero is
   * deployed-at-the-floor, so this offset turns mechanism degrees into drawing degrees.
   */
  private static final double ARM_DRAW_OFFSET_DEG = 180.0;

  private static final double HOOD_NEAR_DEG = 20.0; // TODO(bringup)
  private static final double HOOD_FAR_DEG = 45.0; // TODO(bringup)

  private final LoggedMechanism2d mechanism =
      new LoggedMechanism2d(
          Inches.of(PADDING + FRAME_WIDTH + PADDING).in(Meters),
          Inches.of(CANVAS_HEIGHT).in(Meters));

  private final LoggedMechanismLigament2d arm;
  private final LoggedMechanismLigament2d hood;

  private final Supplier<Pose2d> robotPose;
  private final Supplier<Angle> armAngle;
  private final DoubleSupplier hoodTravel;
  private final String logKey;

  /**
   * @param robotPose used to place 3D component poses on the field
   * @param logKey "Measured" or "Target"
   * @param mechColor colour for every ligament in this instance
   * @param armAngle intake pivot angle — measured or target, depending on the instance
   * @param hoodTravel hood travel, 0.0 retracted to 1.0 extended
   */
  public SuperstructureVisualizer(
      Supplier<Pose2d> robotPose,
      String logKey,
      Color8Bit mechColor,
      Supplier<Angle> armAngle,
      DoubleSupplier hoodTravel) {
    this.robotPose = robotPose;
    this.logKey = logKey;
    this.armAngle = armAngle;
    this.hoodTravel = hoodTravel;

    // A static outline of the frame, so the moving parts have something to be relative to.
    LoggedMechanismRoot2d frameRoot =
        mechanism.getRoot(
            "Frame", Inches.of(PADDING).in(Meters), Inches.of(FRAME_RAIL_ABOVE_FLOOR).in(Meters));
    frameRoot.append(
        new LoggedMechanismLigament2d(
            "FrameRail",
            Inches.of(FRAME_WIDTH).in(Meters),
            0.0,
            3.0,
            new Color8Bit(100, 100, 100)));

    LoggedMechanismRoot2d armRoot =
        mechanism.getRoot(
            logKey + "ArmRoot",
            Inches.of(PADDING + ARM_PIVOT_BEHIND_FRONT).in(Meters),
            Inches.of(ARM_PIVOT_ABOVE_FLOOR).in(Meters));
    arm =
        armRoot.append(
            new LoggedMechanismLigament2d(
                logKey + "Arm", Inches.of(ARM_LENGTH).in(Meters), 0.0, 6.0, mechColor));

    LoggedMechanismRoot2d hoodRoot =
        mechanism.getRoot(
            logKey + "HoodRoot",
            Inches.of(PADDING + HOOD_PIVOT_BEHIND_FRONT).in(Meters),
            Inches.of(HOOD_PIVOT_ABOVE_FLOOR).in(Meters));
    hood =
        hoodRoot.append(
            new LoggedMechanismLigament2d(
                logKey + "Hood", Inches.of(HOOD_LENGTH).in(Meters), HOOD_NEAR_DEG, 6.0, mechColor));
  }

  @Override
  public void periodic() {
    double start = 0.0;
    if (Constants.kEnableLoopTimingLogs) {
      start = Timer.getFPGATimestamp();
    }

    arm.setAngle(ARM_DRAW_OFFSET_DEG - armAngle.get().in(Degrees));

    // Interpolating means the drawing shows the hood mid-travel, not just at its end points.
    double travel = MathUtil.clamp(hoodTravel.getAsDouble(), 0.0, 1.0);
    hood.setAngle(HOOD_NEAR_DEG + travel * (HOOD_FAR_DEG - HOOD_NEAR_DEG));

    Logger.recordOutput(String.format("Superstructure/%s", logKey), mechanism);
    Logger.recordOutput(String.format("Superstructure/%sRobotPose", logKey), robotPose.get());

    if (Constants.kEnableLoopTimingLogs) {
      double end = Timer.getFPGATimestamp();
      Logger.recordOutput("Timing/SuperstructureVisualizerMS", (end - start) * 1000);
    }
  }
}
