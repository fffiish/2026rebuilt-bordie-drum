package frc.robot.lib.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class CustomDCMotor {
  public static DCMotor getKrakenX44(int numMotors) {
    return new DCMotor(
        12, 4.05, 275, 1.4, Units.rotationsPerMinuteToRadiansPerSecond(7530), numMotors);
  }

  public static DCMotor getKrakenX44Foc(int numMotors) {
    // Estimate because there is no data about the effects of FOC on X44 online.
    return new DCMotor(
        12,
        4.05 * 1.15,
        275,
        1.4 * 0.85,
        Units.rotationsPerMinuteToRadiansPerSecond(7530 * 1.05),
        numMotors);
  }
}
