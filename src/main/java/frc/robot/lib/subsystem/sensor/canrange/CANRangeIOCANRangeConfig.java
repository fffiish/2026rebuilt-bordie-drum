package frc.robot.lib.subsystem.sensor.canrange;

import static com.ctre.phoenix6.signals.UpdateModeValue.ShortRange100Hz;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.units.measure.*;
import lombok.Builder;
import lombok.Getter;

@Builder
@Getter
public class CANRangeIOCANRangeConfig {
  private int id;
  private CANBus bus;
  @Builder.Default private Angle fovRange = Degrees.of(6.75);
  @Builder.Default private UpdateModeValue updateMode = ShortRange100Hz;
  @Builder.Default private Frequency updateFrequency = Hertz.of(100.0);
}
