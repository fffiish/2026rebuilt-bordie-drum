package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.constants.shooter.ShooterConstants;

/**
 * Hood IO for a SPARK MAX on CAN driving the pair of PA-14P actuators, with their potentiometers on
 * roboRIO analog inputs.
 *
 * <p><b>Brushed mode.</b> A PA-14 is a 2-wire brushed DC motor, so the controller is constructed
 * with {@link MotorType#kBrushed}. A SPARK MAX in brushless mode will not drive it, and brushed
 * mode means there is no controller-side encoder — position comes entirely from the potentiometers.
 *
 * <p>Running on CAN rather than PWM buys three things this mechanism actually wants: a real current
 * limit on a motor that stalls against its own end-stops every time it finishes travelling, current
 * reporting, and a device-presence check. {@link HoodIOPWM} and {@link HoodIORelay} remain for the
 * other wirings but give up all three.
 *
 * <p>Both actuators are driven from this one controller. Their potentiometers are still read
 * separately, because two actuators moving one hood surface can rack if one binds, and averaging
 * the sensors would hide precisely that.
 */
public class HoodIOSparkMax implements HoodIO {
  /** Below this the analog input is almost certainly unplugged rather than reading a real wiper. */
  private static final double kDisconnectedVolts = 0.05;

  private final SparkMax actuators;
  private final AnalogInput leftPot;
  private final AnalogInput rightPot;

  private boolean extendCommanded = false;

  public HoodIOSparkMax() {
    actuators = new SparkMax(ShooterConstants.kHoodSparkMaxId, MotorType.kBrushed);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit((int) ShooterConstants.kHoodCurrentLimit.in(Amps));

    actuators.clearFaults();
    actuators.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

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
    inputs.appliedVolts = actuators.getAppliedOutput() * actuators.getBusVoltage();
    inputs.currentAmps = actuators.getOutputCurrent();
  }

  @Override
  public void setExtended(boolean extended) {
    extendCommanded = extended;
    actuators.set(extended ? 1.0 : -1.0);
  }

  @Override
  public void stop() {
    actuators.stopMotor();
  }
}
