package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.shooter.ShooterConstants;
import frc.robot.lib.subsystem.VirtualSubsystem;
import frc.robot.lib.subsystem.angular.AngularSubsystem;
import org.littletonrobotics.junction.Logger;

/** Four-motor flywheel. Range comes from flywheel RPM. */
public class Shooter extends VirtualSubsystem {
  private static final ShooterState kDefaultState = ShooterState.kIdle;

  private final AngularSubsystem flywheel;
  private final BangBangController controller = new BangBangController();
  private double bangBangOutput;
  private double requestedVolts;

  private ShooterState targetState = kDefaultState;

  public Shooter(AngularSubsystem flywheel) {
    this.flywheel = flywheel;

    flywheel.setDefaultCommand(
        flywheel.openLoop(this::calculateFlywheelVoltage).finallyDo(() -> stopFlywheel()));
  }

  /** Holds {@code state} while scheduled, then falls back to idle. Bind with {@code whileTrue}. */
  public Command set(ShooterState state) {
    return Commands.startEnd(
        () -> targetState = state,
        () -> {
          targetState = kDefaultState;
          stopFlywheel();
        },
        this);
  }

  /** Latches {@code state} and finishes immediately. For auto sequences. */
  public Command setPersistent(ShooterState state) {
    return Commands.runOnce(() -> targetState = state);
  }

  public ShooterState getTargetState() {
    return targetState;
  }

  public void stopImmediately() {
    targetState = kDefaultState;
    stopFlywheel();
  }

  private boolean feedbackHealthy() {
    return flywheel.isVelocityFeedbackHealthy(ShooterConstants.kFlywheelFeedbackMaxAgeSeconds);
  }

  private Voltage calculateFlywheelVoltage() {
    double target = targetState.getFlywheelVelocity().in(RadiansPerSecond);
    bangBangOutput =
        DriverStation.isEnabled() && target > 0.0 && Double.isFinite(target) && feedbackHealthy()
            ? controller.calculate(flywheel.getVelocity().in(RadiansPerSecond), target)
            : 0.0;
    requestedVolts = bangBangOutput * ShooterConstants.kFlywheelBangBangVoltage.in(Volts);
    logControl();
    return Volts.of(requestedVolts);
  }

  private void stopFlywheel() {
    bangBangOutput = 0.0;
    requestedVolts = 0.0;
    flywheel.stopImmediately();
    logControl();
  }

  private void logControl() {
    Logger.recordOutput("Shooter/ControlType", "WPILibBangBang");
    Logger.recordOutput("Shooter/TargetRPM", targetState.getFlywheelVelocity().in(RPM));
    Logger.recordOutput("Shooter/MeasuredRPM", flywheel.getVelocity().in(RPM));
    Logger.recordOutput(
        "Shooter/ErrorRPM",
        targetState.getFlywheelVelocity().in(RPM) - flywheel.getVelocity().in(RPM));
    Logger.recordOutput("Shooter/BangBangOutput", bangBangOutput);
    Logger.recordOutput("Shooter/RequestedVolts", requestedVolts);
    Logger.recordOutput("Shooter/FeedbackHealthy", feedbackHealthy());
  }

  /** Every connected motor must reach 97% of target before feeding starts. */
  public Trigger atSpeed() {
    return new Trigger(
        () -> {
          double target = targetState.getFlywheelVelocity().in(RadiansPerSecond);
          return target > 0.0
              && feedbackHealthy()
              && flywheel.areAllMotorsAtLeast(
                  RadiansPerSecond.of(ShooterConstants.kFlywheelReadyFraction * target));
        });
  }

  @Override
  public void periodic() {
    logControl();
    Logger.recordOutput("Shooter/TargetState", targetState.toString());
    Logger.recordOutput("Shooter/AtSpeed", atSpeed().getAsBoolean());
    Logger.recordOutput("Shooter/ReadyToFire", readyToFire().getAsBoolean());
  }

  /** The flywheel is where the current state asks it to be. */
  public Trigger readyToFire() {
    return atSpeed();
  }
}
