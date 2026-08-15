package frc.robot.lib.alliancecolor;

import edu.wpi.first.wpilibj.DriverStation;

public class AllianceColor {

  public static boolean isAllianceRed() {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  public static boolean isAllianceBlue() {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }
}
