package frc.robot.lib.subsystem.sensor.canrange;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.RobotConstants.kMaxTimeoutMS;
import static frc.robot.lib.utils.PhoenixUtils.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import java.util.concurrent.atomic.AtomicBoolean;

public class CANRangeIOCANRange implements CANRangeIO {
  private final CANRangeIOCANRangeConfig deviceConfig;
  private final CANrangeConfiguration canRangeConfig;

  private final CANrange CANrange;

  private final StatusSignal<Boolean> isDetected;
  private final StatusSignal<Distance> distance;

  private final Alert configurationsNotAppliedAlert =
      new Alert("Configurations for CANRange not applied!", Alert.AlertType.kError);

  public CANRangeIOCANRange(CANRangeIOCANRangeConfig config) {
    this.deviceConfig = config;
    this.CANrange = new CANrange(config.getId(), config.getBus());

    isDetected = CANrange.getIsDetected();
    distance = CANrange.getDistance();

    canRangeConfig = getCanRangeConfig();
    AtomicBoolean applySuccess =
        new AtomicBoolean(
            tryUntilOk(() -> CANrange.getConfigurator().apply(canRangeConfig, kMaxTimeoutMS)));
    configurationsNotAppliedAlert.set(!applySuccess.get());
  }

  private CANrangeConfiguration getCanRangeConfig() {
    final CANrangeConfiguration configuration = new CANrangeConfiguration();

    configuration.FovParams.FOVRangeX = deviceConfig.getFovRange().in(Degrees);
    configuration.FovParams.FOVRangeY = deviceConfig.getFovRange().in(Degrees);

    configuration.ToFParams.UpdateMode = deviceConfig.getUpdateMode();
    configuration.ToFParams.UpdateFrequency = deviceConfig.getUpdateFrequency().in(Hertz);

    return configuration;
  }

  @Override
  public void updateInputs(CANRangeIOInputs inputs) {
    isDetected.refresh(false);
    distance.refresh(false);

    inputs.isDetected = isDetected.getValue();
    inputs.distance = distance.getValue();
    inputs.connected = isDetected.getStatus().isOK() && distance.getStatus().isOK();
  }

  @Override
  public void setFovRange(Angle fov) {
    canRangeConfig.FovParams.FOVRangeX = fov.in(Degrees);
    canRangeConfig.FovParams.FOVRangeY = fov.in(Degrees);
    CANrange.getConfigurator().apply(canRangeConfig, 0.0);
  }

  @Override
  public void setUpdateMode(UpdateModeValue updateMode) {
    canRangeConfig.ToFParams.UpdateMode = updateMode;
    CANrange.getConfigurator().apply(canRangeConfig, 0.0);
  }

  @Override
  public void setUpdateFrequency(Frequency updateFrequency) {
    canRangeConfig.ToFParams.UpdateFrequency = updateFrequency.in(Hertz);
    CANrange.getConfigurator().apply(canRangeConfig, 0.0);
  }

  @Override
  public void setLogKey(String logKey) {
    configurationsNotAppliedAlert.setText(
        String.format("Configurations for CANRange %s not applied!", logKey));
  }
}
