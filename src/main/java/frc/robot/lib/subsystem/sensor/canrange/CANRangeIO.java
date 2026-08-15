package frc.robot.lib.subsystem.sensor.canrange;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface CANRangeIO {
  default void updateInputs(CANRangeIOInputs inputs) {}

  @AutoLog
  class CANRangeIOInputs {
    public boolean isDetected = false;
    public Distance distance = Meters.of(0.0);
    public boolean connected = false;
  }

  default void setFovRange(Angle fov) {}

  default void setUpdateMode(UpdateModeValue updateMode) {}

  default void setUpdateFrequency(Frequency updateFrequency) {}

  default void setLogKey(String logKey) {}
}
