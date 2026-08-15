package frc.robot.lib.subsystem.sensor.currentsensor;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.subsystem.VirtualSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CurrentSensorSubsystem extends VirtualSubsystem {
  private final CurrentSensorSubsystemConfig config;

  private final LoggedTunableNumber thresholdTunable;
  private final LoggedTunableNumber debounceTunable;

  private boolean doesExceedsThreshold = false;

  @AutoLogOutput(key = "CurrentSensors/{logKey}/ExceedsThreshold")
  private Trigger exceedsThreshold;

  private final String logKey;

  public CurrentSensorSubsystem(CurrentSensorSubsystemConfig config) {
    this.config = config;
    this.logKey = config.getLogKey();

    exceedsThreshold =
        new Trigger(() -> doesExceedsThreshold).debounce(config.getDebounce().in(Seconds));

    thresholdTunable =
        new LoggedTunableNumber(
            String.format("CurrentSensors/%s/ThresholdAmps", config.getLogKey()),
            config.getThreshold().in(Amps));
    debounceTunable =
        new LoggedTunableNumber(
            String.format("CurrentSensors/%s/DebounceSeconds", config.getLogKey()),
            config.getDebounce().in(Seconds));
  }

  @Override
  public void periodic() {
    doesExceedsThreshold =
        config.getCurrentSupplier().get().gt(config.getThreshold())
            && config.getConnectedSupplier().getAsBoolean();

    LoggedTunableNumber.ifChanged(
        hashCode(), () -> config.setThreshold(Amps.of(thresholdTunable.get())), thresholdTunable);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          config.setDebounce(Seconds.of(debounceTunable.get()));
          exceedsThreshold =
              new Trigger(() -> doesExceedsThreshold).debounce(config.getDebounce().in(Seconds));
        },
        debounceTunable);

    Logger.recordOutput(
        String.format("CurrentSensors/%s/ExceedsThreshold", logKey), doesExceedsThreshold());
    Logger.recordOutput(
        String.format("CurrentSensors/%s/CurrentAmps", logKey), getCurrent().in(Amps));
    Logger.recordOutput(
        String.format("CurrentSensors/%s/ThresholdAmps", logKey), getThreshold().in(Amps));
    Logger.recordOutput(String.format("CurrentSensors/%s/Connected", logKey), isConnected());
  }

  public boolean doesExceedsThreshold() {
    return exceedsThreshold.getAsBoolean();
  }

  public Trigger exceedsThreshold() {
    return exceedsThreshold;
  }

  public Current getThreshold() {
    return config.getThreshold();
  }

  public Time getDebounce() {
    return config.getDebounce();
  }

  public Current getCurrent() {
    return config.getCurrentSupplier().get();
  }

  public boolean isConnected() {
    return config.getConnectedSupplier().getAsBoolean();
  }
}
