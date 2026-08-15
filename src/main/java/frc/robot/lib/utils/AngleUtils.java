package frc.robot.lib.utils;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;

public final class AngleUtils {

  public static Angle normalize(Angle angle) {
    double radians = angle.in(Radians);

    radians = radians % (2 * Math.PI); // [-2pi, 2pi]
    if (radians > Math.PI) {
      radians -= 2 * Math.PI;
    } else if (radians < -Math.PI) {
      radians += 2 * Math.PI;
    }

    return Radians.of(radians);
  }
}
