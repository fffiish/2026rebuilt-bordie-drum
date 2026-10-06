package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.constants.shooter.ShooterConstants;

/**
 * Hood IO for a pair of PA-14P linear actuators on PWM motor controllers, with their potentiometers
 * on roboRIO analog inputs.
 *
 * <p>The actuators are 12 V brushed DC with a built-in, non-adjustable limit switch at each end, so
 * driving them into a stop is safe and no software cutoff is needed. Output is full-scale in one
 * direction or the other; there is no closed loop, because there are only two positions to reach.
 *
 * <p>The controller type is the one thing likely to change: {@link PWMSparkMax} is a stand-in for
 * whatever PWM controller the actuators land on. Every WPILib PWM controller shares the {@link
 * PWMMotorController} interface, so swapping to {@code VictorSP} or {@code Talon} is a one-line
 * change in {@link #HoodIOPWM()}.
 *
 * <p><b>Both sides are read separately on purpose.</b> Two actuators drive one hood surface, so if
 * one binds or stalls the surface racks. Averaging the two potentiometers would hide exactly that,
 * so the average is reported as position while the disagreement is checked against {@link
 * ShooterConstants#kHoodSideDisagreement} and, when exceeded, reported as a lost sensor — which
 * drops {@link Hood} back to its timer rather than letting it trust a racked hood.
 */
public class HoodIOPWM implements HoodIO {
  /** Below this the analog input is almost certainly unplugged rather than reading a real wiper. */
  private static final double kDisconnectedVolts = 0.05;

  private final PWMMotorController leftActuator;
  private final PWMMotorController rightActuator;
  private final AnalogInput leftPot;
  private final AnalogInput rightPot;

  private boolean extendCommanded = false;

  public HoodIOPWM() {
    // TODO(bringup): swap these for the controller actually used, if not a REV Spark.
    leftActuator = new PWMSparkMax(ShooterConstants.kHoodLeftPwmChannel);
    rightActuator = new PWMSparkMax(ShooterConstants.kHoodRightPwmChannel);
    leftPot = new AnalogInput(ShooterConstants.kHoodLeftAnalogChannel);
    rightPot = new AnalogInput(ShooterConstants.kHoodRightAnalogChannel);

    setExtended(false);
  }

  /** Maps a raw wiper voltage onto 0.0 retracted - 1.0 extended. */
  private static double normalize(double volts) {
    double span =
        ShooterConstants.kHoodSensorVoltsExtended - ShooterConstants.kHoodSensorVoltsRetracted;
    if (Math.abs(span) < 1e-6) {
      return 0.0;
    }
    return MathUtil.clamp((volts - ShooterConstants.kHoodSensorVoltsRetracted) / span, 0.0, 1.0);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    double leftVolts = leftPot.getVoltage();
    double rightVolts = rightPot.getVoltage();

    double left = normalize(leftVolts);
    double right = normalize(rightVolts);

    boolean bothPresent = leftVolts > kDisconnectedVolts && rightVolts > kDisconnectedVolts;
    boolean agree = Math.abs(left - right) <= ShooterConstants.kHoodSideDisagreement;

    inputs.connected = bothPresent && agree;
    inputs.positionNormalized = (left + right) / 2.0;
    inputs.sensorVolts = (leftVolts + rightVolts) / 2.0;
    inputs.extendCommanded = extendCommanded;
    inputs.appliedVolts = leftActuator.get() * RobotController.getBatteryVoltage();
    // PWM controllers report no current; the PDH would have to be queried for that.
    inputs.currentAmps = 0.0;
  }

  @Override
  public void setExtended(boolean extended) {
    extendCommanded = extended;
    double output = extended ? 1.0 : -1.0;
    leftActuator.set(output);
    rightActuator.set(output);
  }

  @Override
  public void stop() {
    leftActuator.set(0.0);
    rightActuator.set(0.0);
  }
}
