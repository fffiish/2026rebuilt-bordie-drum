package frc.robot.lib.subsystem.angular;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.lib.subsystem.angular.AngularSubsystemOutputMode.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.subsystem.DeviceConnectedStatus;
import frc.robot.lib.subsystem.RegisteredSubsystem;
import java.util.Arrays;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class AngularSubsystem extends RegisteredSubsystem {
  private static double accumulatedInputUpdateMS = 0.0;
  private static double accumulatedSubsystemCodeMS = 0.0;

  private final AngularIO io;
  private final AngularIOInputsAutoLogged inputs = new AngularIOInputsAutoLogged();

  private final AngularSubsystemConfig config;

  private LoggedTunableNumber kPTunable;
  private LoggedTunableNumber kITunable;
  private LoggedTunableNumber kDTunable;
  private LoggedTunableNumber kVTunable;
  private LoggedTunableNumber kGTunable;
  private LoggedTunableNumber kSTunable;
  private LoggedTunableNumber cruiseVelocityTunable;
  private LoggedTunableNumber accelerationTunable;
  private LoggedTunableNumber positionToleranceTunable;
  private LoggedTunableNumber velocityToleranceTunable;

  @Getter private AngularSubsystemOutputMode outputMode = kHoldAtCall;

  @Getter private boolean isAtAngle = false;

  private final Trigger atAngle = new Trigger(this::isAtAngle);

  @SuppressWarnings("FieldCanBeLocal")
  private final Alert motorDisconectedAlert =
      new Alert("Motor disconnected!", Alert.AlertType.kError);

  private final String logKey;

  public AngularSubsystem(AngularIO io, AngularSubsystemConfig config) {
    this.io = io;
    this.config = config;
    this.logKey = config.getLogKey();

    setTunable();

    io.setLogKey(logKey);
    io.resetAngle();
    setDefaultCommand(holdAtCall());
  }

  private void setTunable() {
    kPTunable =
        new LoggedTunableNumber(String.format("AngularSubsystems/%s/KP", logKey), config.getKP());
    kITunable =
        new LoggedTunableNumber(String.format("AngularSubsystems/%s/KI", logKey), config.getKI());
    kDTunable =
        new LoggedTunableNumber(String.format("AngularSubsystems/%s/KD", logKey), config.getKD());
    kVTunable =
        new LoggedTunableNumber(String.format("AngularSubsystems/%s/KV", logKey), config.getKV());
    kGTunable =
        new LoggedTunableNumber(String.format("AngularSubsystems/%s/KG", logKey), config.getKG());
    kSTunable =
        new LoggedTunableNumber(String.format("AngularSubsystems/%s/KS", logKey), config.getKS());
    cruiseVelocityTunable =
        new LoggedTunableNumber(
            String.format("AngularSubsystems/%s/CruiseVelocityRadiansPerSecond", logKey),
            config.getCruiseVelocity().in(RadiansPerSecond));
    accelerationTunable =
        new LoggedTunableNumber(
            String.format("AngularSubsystems/%s/AccelerationRadiansPerSecondPerSecond", logKey),
            config.getAcceleration().in(RadiansPerSecondPerSecond));
    positionToleranceTunable =
        new LoggedTunableNumber(
            String.format("AngularSubsystems/%s/PositionToleranceRadians", logKey),
            config.getPositionTolerance().in(Radians));
    velocityToleranceTunable =
        new LoggedTunableNumber(
            String.format("AngularSubsystems/%s/VelocityToleranceRadiansPerSecond", logKey),
            config.getVelocityTolerance().in(RadiansPerSecond));
  }

  @Override
  public void periodic() {
    double start = 0.0;
    if (Constants.kEnableLoopTimingLogs) {
      start = Timer.getFPGATimestamp();
    }
    io.updateInputs(inputs);
    Logger.processInputs(String.format("AngularSubsystems/%s", logKey), inputs);
    double afterIO = 0.0;
    if (Constants.kEnableLoopTimingLogs) {
      afterIO = Timer.getFPGATimestamp();
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          config.setKP(kPTunable.get());
          config.setKI(kITunable.get());
          config.setKD(kDTunable.get());
          config.setKV(kVTunable.get());
          config.setKG(kGTunable.get());
          config.setKS(kSTunable.get());
          io.setPIDVG(
              kPTunable.get(), kITunable.get(), kDTunable.get(), kVTunable.get(), kGTunable.get());
        },
        kPTunable,
        kITunable,
        kDTunable,
        kVTunable,
        kGTunable);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          config.setCruiseVelocity(RadiansPerSecond.of(cruiseVelocityTunable.get()));
          config.setAcceleration(RadiansPerSecondPerSecond.of(accelerationTunable.get()));
          io.setConstraints(
              RadiansPerSecond.of(cruiseVelocityTunable.get()),
              RadiansPerSecondPerSecond.of(accelerationTunable.get()));
        },
        cruiseVelocityTunable,
        accelerationTunable);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> config.setPositionTolerance(Radians.of(positionToleranceTunable.get())),
        positionToleranceTunable);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> config.setVelocityTolerance(RadiansPerSecond.of(velocityToleranceTunable.get())),
        velocityToleranceTunable);

    if (outputMode == kOpenLoop) {
      isAtAngle = false;
    } else if (outputMode == kVelocity) {
      isAtAngle =
          MathUtil.isNear(
              inputs.goalVel.in(RadiansPerSecond),
              inputs.velocity.in(RadiansPerSecond),
              config.getVelocityTolerance().in(RadiansPerSecond));
    } else {
      isAtAngle =
          MathUtil.isNear(
                  inputs.goalPos.in(Radians),
                  inputs.angle.in(Radians),
                  config.getPositionTolerance().in(Radians))
              && MathUtil.isNear(
                  0.0,
                  inputs.velocity.in(RadiansPerSecond),
                  config.getVelocityTolerance().in(RadiansPerSecond));
    }

    Logger.recordOutput(String.format("AngularSubsystems/%s/AtAngle", logKey), isAtAngle);

    if (!Arrays.stream(inputs.deviceConnectedStatuses)
        .allMatch(DeviceConnectedStatus::isConnected)) {
      Stream<String> disconnectedDevicesIds =
          Arrays.stream(inputs.deviceConnectedStatuses)
              .filter(d -> !d.isConnected())
              .map(d -> String.valueOf(d.getId()));
      motorDisconectedAlert.setText(
          String.format(
              "Motors: %s; on bus: %s disconnected!",
              disconnectedDevicesIds.collect(Collectors.joining(", ")), config.getBus().getName()));
      motorDisconectedAlert.set(true);
    } else {
      motorDisconectedAlert.set(false);
    }

    if (Constants.kEnableLoopTimingLogs) {
      double end = Timer.getFPGATimestamp();
      accumulatedInputUpdateMS += (afterIO - start) * 1000.0;
      accumulatedSubsystemCodeMS += (end - afterIO) * 1000.0;
    }
  }

  public static void recordAndResetTiming() {
    if (Constants.kEnableLoopTimingLogs) {
      Logger.recordOutput("Timing/AngularSubsystems/InputUpdateMS", accumulatedInputUpdateMS);
      Logger.recordOutput("Timing/AngularSubsystems/SubsystemCodeMS", accumulatedSubsystemCodeMS);
    }
    accumulatedInputUpdateMS = 0.0;
    accumulatedSubsystemCodeMS = 0.0;
  }

  public Command angle(Angle angle) {
    // Only set angle once, run until canceled.
    return parallel(
        sequence(runOnce(() -> io.setAngle(angle)), idle()), setOutputMode(kClosedLoop));
  }

  public Command velocity(AngularVelocity angVel) {
    // Only set angle once, run until canceled.
    return parallel(
        sequence(runOnce(() -> io.setVelocity(angVel)), idle()), setOutputMode(kVelocity));
  }

  public Command angle(Supplier<Angle> angle) {
    // Set angle every loop, run until canceled.
    return parallel(run(() -> io.setAngle(angle.get())), setOutputMode(kClosedLoop));
  }

  public Command angle(Supplier<Angle> angle, Supplier<Voltage> feedforward) {
    // Set angle every loop, run until canceled.
    return parallel(
        run(() -> io.setAngle(angle.get(), feedforward.get())), setOutputMode(kClosedLoop));
  }

  public Command velocity(Supplier<AngularVelocity> angVel) {
    // Set angle every loop, run until canceled.
    return parallel(run(() -> io.setVelocity(angVel.get())), setOutputMode(kVelocity));
  }

  public Command openLoop(Voltage voltage) {
    // Only set duty cycle once, run until canceled.
    return parallel(
        sequence(runOnce(() -> io.setOpenLoop(voltage)), idle()), setOutputMode(kOpenLoop));
  }

  public Command openLoop(Supplier<Voltage> voltage) {
    // Set duty cycle every loop, run until canceled.
    return parallel(run(() -> io.setOpenLoop(voltage.get())), setOutputMode(kOpenLoop));
  }

  public Command stop() {
    return runOnce(io::stop);
  }

  public Command holdAtCall() {
    return parallel(
        sequence(runOnce(() -> io.setAngle(getAngle())), idle()), setOutputMode(kHoldAtCall));
  }

  public Command holdAtGoal(Supplier<Angle> goal) {
    return parallel(angle(goal), setOutputMode(kHoldAtGoal));
  }

  public Command holdAtGoal(Supplier<Angle> goal, Supplier<Voltage> feedforward) {
    return parallel(angle(goal, feedforward), setOutputMode(kHoldAtGoal));
  }

  public Command resetAngle() {
    return Commands.runOnce(io::resetAngle);
  }

  public Command resetAngle(Angle angle) {
    return resetAngle(() -> angle);
  }

  public Command resetAngle(Supplier<Angle> angle) {
    return Commands.runOnce(() -> io.resetAngle(angle.get()));
  }

  public Command setNeutralModeBrake() {
    return setNeutralMode(NeutralModeValue.Brake);
  }

  public Command setNeutralModeCoast() {
    return setNeutralMode(NeutralModeValue.Coast);
  }

  public Command setNeutralMode(NeutralModeValue neutralMode) {
    return setNeutralMode(() -> neutralMode);
  }

  public Command setNeutralMode(Supplier<NeutralModeValue> neutralMode) {
    return Commands.runOnce(() -> io.setNeutralMode(neutralMode.get()));
  }

  private Command setOutputMode(AngularSubsystemOutputMode outputMode) {
    return Commands.runOnce(() -> this.outputMode = outputMode);
  }

  public Trigger atAngle() {
    return atAngle;
  }

  public Angle getAngle() {
    return inputs.angle;
  }

  public Angle getGoalPos() {
    return inputs.goalPos;
  }

  public AngularVelocity getVelocity() {
    return inputs.velocity;
  }

  public AngularVelocity getGoalVelocity() {
    return inputs.goalVel;
  }

  public AngularAcceleration getAcceleration() {
    return inputs.acceleration;
  }

  public Current getSupplyCurrent() {
    return inputs.supplyCurrent;
  }

  public Current getStatorCurrent() {
    return inputs.statorCurrent;
  }

  public boolean areAllDevicesConnected() {
    return Arrays.stream(inputs.deviceConnectedStatuses)
        .allMatch(DeviceConnectedStatus::isConnected);
  }
}
