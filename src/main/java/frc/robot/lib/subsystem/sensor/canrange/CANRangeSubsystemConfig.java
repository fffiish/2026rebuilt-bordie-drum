package frc.robot.lib.subsystem.sensor.canrange;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;

@Builder
@Getter
public class CANRangeSubsystemConfig {
  private final String logKey;
  @Builder.Default @Setter private Distance threshold = Inches.of(-1.0);
  @Builder.Default @Setter private Time debounce = Seconds.of(0.0);
}
