package frc.robot.lib.subsystem.sensor.canrange;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.subsystem.RegisteredSubsystem;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CANRangeSubsystem extends RegisteredSubsystem {
  private final CANRangeIO io;
  private final CANRangeIOInputsAutoLogged inputs = new CANRangeIOInputsAutoLogged();

  private final CANRangeSubsystemConfig config;

  private final LoggedTunableNumber thresholdTunable;
  private final LoggedTunableNumber debounceTunable;

  private boolean isWithinThreshold = false;

  @AutoLogOutput(key = "CANRanges/{logKey}/WithinThreshold")
  private Trigger withinThreshold;

  @SuppressWarnings("FieldCanBeLocal")
  private final Alert disconnectedAlert =
      new Alert("CANRange disconnected!", Alert.AlertType.kError);

  private final String logKey;

  public CANRangeSubsystem(CANRangeIO io, CANRangeSubsystemConfig config) {
    this.io = io;
    this.config = config;
    this.logKey = config.getLogKey();

    withinThreshold =
        new Trigger(() -> isWithinThreshold).debounce(config.getDebounce().in(Seconds));

    thresholdTunable =
        new LoggedTunableNumber(
            String.format("CANRanges/%s/ThresholdInches", config.getLogKey()),
            config.getThreshold().in(Inches));
    debounceTunable =
        new LoggedTunableNumber(
            String.format("CANRanges/%s/DebounceSeconds", config.getLogKey()),
            config.getDebounce().in(Seconds));

    disconnectedAlert.setText(String.format("CANRange %s disconnected!", logKey));

    io.setLogKey(logKey);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(String.format("CANRanges/%s", logKey), inputs);

    isWithinThreshold =
        inputs.distance.lt(config.getThreshold()) && inputs.connected && inputs.isDetected;

    LoggedTunableNumber.ifChanged(
        hashCode(), () -> config.setThreshold(Inches.of(thresholdTunable.get())), thresholdTunable);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          config.setDebounce(Seconds.of(debounceTunable.get()));
          withinThreshold =
              new Trigger(() -> isWithinThreshold).debounce(config.getDebounce().in(Seconds));
        },
        debounceTunable);

    Logger.recordOutput(String.format("CANRanges/%s/WithinThreshold", logKey), isWithinThreshold());
    Logger.recordOutput(
        String.format("CANRanges/%s/ThresholdInches", logKey), config.getThreshold().in(Inches));
    Logger.recordOutput(
        String.format("CANRanges/%s/DistanceInches", logKey), getDistance().in(Inches));

    disconnectedAlert.set(!inputs.connected);
  }

  public boolean isWithinThreshold() {
    return withinThreshold.getAsBoolean();
  }

  public Trigger withinThreshold() {
    return withinThreshold;
  }

  public Optional<Distance> getThreshold() {
    return config.getThreshold().lt(Inches.of(0.0))
        ? Optional.empty()
        : Optional.of(config.getThreshold());
  }

  public Time getDebounce() {
    return config.getDebounce();
  }

  public Distance getDistance() {
    return inputs.distance;
  }

  public boolean isDetected() {
    return inputs.isDetected;
  }

  public boolean isConnected() {
    return inputs.connected;
  }
}
