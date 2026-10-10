package frc.robot.subsystems.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

/**
 * IO implementation for a navX2.
 *
 * <p>Replaces GyroIOPigeon2, which cannot work on this robot: a Pigeon 2 is a CAN device read
 * through Phoenix, and the navX hangs off the roboRIO's MXP port instead.
 *
 * <p>Two consequences follow from that, both shared with {@link ModuleIOSpark}:
 *
 * <ul>
 *   <li><b>No timestamped signal queue.</b> {@link PhoenixOdometryThread} works by registering
 *       Phoenix {@code StatusSignal}s and reading them with CANivore timesync; an MXP device has
 *       nothing to register. Yaw is therefore sampled once per main loop and reported as a
 *       single-element odometry array, matching what the Spark modules do.
 *   <li><b>Yaw sign is used as reported.</b> A right-side-up navX reports clockwise-positive, but
 *       this one is mounted upside down, which makes it counter-clockwise-positive like WPILib.
 *       Negating it here made field-oriented control rotate the wrong way as the robot turned.
 * </ul>
 */
public class GyroIONavX implements GyroIO {
  // TODO(bringup): confirm the connection. kMXP_SPI is the usual roboRIO header mounting; switch to
  // kUSB1 if the navX is plugged into the RIO's USB port instead.
  private final AHRS navX = new AHRS(NavXComType.kMXP_SPI);

  public GyroIONavX() {
    navX.reset();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navX.isConnected();
    // Not negated: the navX is mounted upside down, so its yaw is already
    // counter-clockwise-positive.
    inputs.yawPosition = Rotation2d.fromDegrees(navX.getAngle());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(navX.getRate());

    inputs.odometryYawTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryYawPositions = new Rotation2d[] {inputs.yawPosition};
  }
}
