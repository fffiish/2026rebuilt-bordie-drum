package frc.robot.lib.subsystem.linear;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.lib.subsystem.linear.LinearSubsystemOutputMode.*;

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

public class LinearSubsystem extends RegisteredSubsystem {
  private static double accumulatedInputUpdateMS = 0.0;
  private static double accumulatedSubsystemCodeMS = 0.0;

  private final LinearIO io;
  private final LinearIOInputsAutoLogged inputs = new LinearIOInputsAutoLogged();

  private final LinearSubsystemConfig config;

  private LoggedTunableNumber kPTunable;
  private LoggedTunableNumber kITunable;
  private LoggedTunableNumber kDTunable;
  private LoggedTunableNumber kGTunable;
  private LoggedTunableNumber cruiseVelocityTunable;
  private LoggedTunableNumber accelerationTunable;
  private LoggedTunableNumber positionToleranceTunable;
  private LoggedTunableNumber velocityToleranceTunable;

  @Getter private LinearSubsystemOutputMode outputMode = kHoldAtCall;

  @Getter private boolean isAtLength = false;

  private final Trigger atLength = new Trigger(this::isAtLength);

  @SuppressWarnings("FieldCanBeLocal")
  private final Alert motorDisconectedAlert =
      new Alert("Motor disconnected!", Alert.AlertType.kError);

  private final String logKey;

  public LinearSubsystem(LinearIO io, LinearSubsystemConfig config) {
    this.io = io;
    this.config = config;
    this.logKey = config.getLogKey();

    setTunable();

    io.setLogKey(logKey);
    io.resetLength();
    setDefaultCommand(holdAtCall());
  }

  private void setTunable() {
    kPTunable =
        new LoggedTunableNumber(String.format("LinearSubsystems/%s/KP", logKey), config.getKP());
    kITunable =
        new LoggedTunableNumber(String.format("LinearSubsystems/%s/KI", logKey), config.getKI());
    kDTunable =
        new LoggedTunableNumber(String.format("LinearSubsystems/%s/KD", logKey), config.getKD());
    kGTunable =
        new LoggedTunableNumber(String.format("LinearSubsystems/%s/KG", logKey), config.getKG());
    cruiseVelocityTunable =
        new LoggedTunableNumber(
            String.format("LinearSubsystems/%s/CruiseVelocityInchesPerSecond", logKey),
            config.getCruiseVelocity().in(InchesPerSecond));
    accelerationTunable =
        new LoggedTunableNumber(
            String.format("LinearSubsystems/%s/AccelerationFeetPerSecondPerSecond", logKey),
            config.getAcceleration().in(FeetPerSecondPerSecond));
    positionToleranceTunable =
        new LoggedTunableNumber(
            String.format("LinearSubsystems/%s/PositionToleranceInches", logKey),
            config.getPositionTolerance().in(Inches));
    velocityToleranceTunable =
        new LoggedTunableNumber(
            String.format("LinearSubsystems/%s/VelocityToleranceInchesPerSecond", logKey),
            config.getVelocityTolerance().in(InchesPerSecond));
  }

  @Override
  public void periodic() {
    double start = 0.0;
    if (Constants.kEnableLoopTimingLogs) {
      start = Timer.getFPGATimestamp();
    }
    io.updateInputs(inputs);
    Logger.processInputs(String.format("LinearSubsystems/%s", logKey), inputs);
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
          config.setKG(kGTunable.get());
          io.setPIDG(kPTunable.get(), kITunable.get(), kDTunable.get(), kGTunable.get());
        },
        kPTunable,
        kITunable,
        kDTunable,
        kGTunable);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          config.setCruiseVelocity(InchesPerSecond.of(cruiseVelocityTunable.get()));
          config.setAcceleration(FeetPerSecondPerSecond.of(accelerationTunable.get()));
          io.setConstraints(
              InchesPerSecond.of(cruiseVelocityTunable.get()),
              FeetPerSecondPerSecond.of(accelerationTunable.get()));
        },
        cruiseVelocityTunable,
        accelerationTunable);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> config.setPositionTolerance(Inches.of(positionToleranceTunable.get())),
        positionToleranceTunable);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> config.setVelocityTolerance(InchesPerSecond.of(velocityToleranceTunable.get())),
        velocityToleranceTunable);

    isAtLength =
        outputMode != kOpenLoop
            && MathUtil.isNear(
                inputs.goal.in(Meters),
                inputs.length.in(Meters),
                config.getPositionTolerance().in(Meters))
            && MathUtil.isNear(
                0.0,
                inputs.velocity.in(MetersPerSecond),
                config.getVelocityTolerance().in(MetersPerSecond));

    Logger.recordOutput(String.format("LinearSubsystems/%s/AtLength", logKey), isAtLength);

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
      Logger.recordOutput("Timing/LinearSubsystems/InputUpdateMS", accumulatedInputUpdateMS);
      Logger.recordOutput("Timing/LinearSubsystems/SubsystemCodeMS", accumulatedSubsystemCodeMS);
    }
    accumulatedInputUpdateMS = 0.0;
    accumulatedSubsystemCodeMS = 0.0;
  }

  public Command length(Distance length) {
    // Only set length once, run until canceled.
    return parallel(
        sequence(runOnce(() -> io.setLength(length)), idle()), setOutputMode(kClosedLoop));
  }

  public Command length(Supplier<Distance> length) {
    // Set length every loop, run until canceled.
    return parallel(run(() -> io.setLength(length.get())), setOutputMode(kClosedLoop));
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
        sequence(runOnce(() -> io.setLength(getLength())), idle()), setOutputMode(kHoldAtCall));
  }

  public Command holdAtGoal(Supplier<Distance> goal) {
    return parallel(length(goal), setOutputMode(kHoldAtGoal));
  }

  public Command resetLength() {
    return Commands.runOnce(io::resetLength);
  }

  public Command resetLength(Distance length) {
    return resetLength(() -> length);
  }

  public Command resetLength(Supplier<Distance> length) {
    return Commands.runOnce(() -> io.resetLength(length.get()));
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

  private Command setOutputMode(LinearSubsystemOutputMode outputMode) {
    return Commands.runOnce(() -> this.outputMode = outputMode);
  }

  public Trigger atLength() {
    return atLength;
  }

  public Distance getLength() {
    return inputs.length;
  }

  public Distance getGoal() {
    return inputs.goal;
  }

  public LinearVelocity getVelocity() {
    return inputs.velocity;
  }

  public LinearAcceleration getAcceleration() {
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
