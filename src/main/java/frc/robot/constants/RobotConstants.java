package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.util.Color8Bit;

public final class RobotConstants {
  /** loop time in seconds */
  public static final double kDt = 0.02;

  public static final double kMaxTimeoutMS = 1;
  public static final int kMaxAttempts = 5;

  public static final CANBus kCanivoreBus = new CANBus("Drivetrain");
  public static final CANBus kRioBus = new CANBus("rio");

  public static final Color8Bit kMeasuredStateColor = new Color8Bit(255, 0, 0);
  public static final Color8Bit kTargetStateColor = new Color8Bit(0, 0, 255);

  public static final double kLoopOverrunWarningTimeout = 0.2;
}
