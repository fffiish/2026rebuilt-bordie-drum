package frc.robot.lib.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;

/** Class for automatic computation of shift timings and such. */
public class ShiftUtils {

  public static Optional<Alliance> getAutoWinner() {
    String winner = DriverStation.getGameSpecificMessage();

    if (winner.equals("R")) {
      return Optional.of(Alliance.Red);
    } else if (winner.equals("B")) {
      return Optional.of(Alliance.Blue);
    } else {
      return Optional.empty();
    }
  }

  public enum Shift {
    RED,
    BLUE,
    ALL
  }

  // 3 seconds ahead of main trigger to work earlier
  public static Trigger getHubActivePreemptTrigger(Alliance alliance, Alliance autoWinner) {
    return new Trigger(() -> isActiveAlliance(Timer.getMatchTime() + 3, alliance, autoWinner));
  }

  public static Trigger getHubActiveTrigger(Alliance alliance, Alliance autoWinner) {
    return new Trigger(() -> isActiveAlliance(Timer.getMatchTime(), alliance, autoWinner));
  }

  // kind of wasteful and pointless, fix later
  public static boolean isActiveAlliance(
      double teleopSeconds, Alliance alliance, Alliance autoWinner) {
    Shift shift = getActiveShift(teleopSeconds, autoWinner);
    return shift == Shift.ALL || shift == allianceToShift(alliance);
  }

  public static Shift getActiveShift(double teleopSeconds, Alliance autoWinner) {
    // auto to tele transition
    if (teleopSeconds < 10.0) {
      return Shift.ALL;
    }
    // shifts
    else if (teleopSeconds < 35.0) {
      return allianceToShift(invertAlliance(autoWinner));
    } else if (teleopSeconds < 50.0) {
      return allianceToShift(autoWinner);
    } else if (teleopSeconds < 75.0) {
      return allianceToShift(invertAlliance(autoWinner));
    } else if (teleopSeconds < 100.0) {
      return allianceToShift(autoWinner);
    }
    // endgame (all remaining time)
    else {
      return Shift.ALL;
    }
  }

  public static Alliance invertAlliance(Alliance alliance) {
    return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
  }

  public static Shift allianceToShift(Alliance alliance) {
    return alliance == Alliance.Blue ? Shift.BLUE : Shift.RED;
  }
}
