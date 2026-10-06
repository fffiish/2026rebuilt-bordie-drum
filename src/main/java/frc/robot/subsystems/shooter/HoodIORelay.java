package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Relay;
import frc.robot.constants.shooter.ShooterConstants;

/**
 * Hood IO for a pair of PA-14P actuators driven by relay modules (Spike or equivalent), with their
 * potentiometers on roboRIO analog inputs.
 *
 * <p>This is the "no motor controller, just on or off" wiring. A relay module cannot vary speed,
 * only polarity — which is all a two-position hood needs. Note the physical difference from {@link
 * HoodIOPWM}: a Spike lands on the roboRIO's <b>Relay</b> header, not the PWM header.
 *
 * <p>Polarity reversal is the thing to confirm with whoever wired it. Driving the hood out and back
 * requires reversing current through the actuator, so a single on/off channel is not enough; the
 * relay must be wired for both directions ({@link Relay.Direction#kBoth}, which is the default).
 *
 * <p>The actuators' built-in limit switches stop them at each end, so leaving a relay energised
 * against a stop is safe. {@link #stop()} is still worth calling once the hood has arrived, to
 * avoid holding current indefinitely.
 *
 * <p>Potentiometer handling matches {@link HoodIOPWM}: both sides are read separately so that one
 * actuator binding shows up as disagreement rather than being averaged away.
 */
public class HoodIORelay implements HoodIO {
  /** Below this the analog input is almost certainly unplugged rather than reading a real wiper. */
  private static final double kDisconnectedVolts = 0.05;

  private final Relay leftActuator;
  private final Relay rightActuator;
  private final AnalogInput leftPot;
  private final AnalogInput rightPot;

  private boolean extendCommanded = false;

  public HoodIORelay() {
    leftActuator = new Relay(ShooterConstants.kHoodLeftRelayChannel);
    rightActuator = new Relay(ShooterConstants.kHoodRightRelayChannel);
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
    // A relay reports neither applied voltage nor current.
    inputs.appliedVolts = 0.0;
    inputs.currentAmps = 0.0;
  }

  @Override
  public void setExtended(boolean extended) {
    extendCommanded = extended;
    Relay.Value value = extended ? Relay.Value.kForward : Relay.Value.kReverse;
    leftActuator.set(value);
    rightActuator.set(value);
  }

  @Override
  public void stop() {
    leftActuator.set(Relay.Value.kOff);
    rightActuator.set(Relay.Value.kOff);
  }
}
