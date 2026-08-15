package frc.robot.lib.alliancecolor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.subsystem.VirtualSubsystem;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class AllianceChecker extends VirtualSubsystem {
  private final List<AllianceUpdatedObserver> observers = new ArrayList<>();
  private Optional<Alliance> alliance = DriverStation.getAlliance();

  public void registerObserver(AllianceUpdatedObserver observer) {
    observers.add(observer);
  }

  public void registerObservers(AllianceUpdatedObserver... addObservers) {
    Collections.addAll(observers, addObservers);
  }

  public void periodic() {
    double start = 0.0;
    if (Constants.kEnableLoopTimingLogs) {
      start = Timer.getFPGATimestamp();
    }

    alliance = DriverStation.getAlliance();
    alliance.ifPresent(color -> observers.forEach(observer -> observer.onAllianceFound(color)));
    updateMatchTimer();
    publishMatchTimer();

    if (Constants.kEnableLoopTimingLogs) {
      double end = Timer.getFPGATimestamp();
      Logger.recordOutput("Timing/AllianceCheckerMS", (end - start) * 1000);
    }
  }

  public void publishMatchTimer() {
    SmartDashboard.putNumber("ShiftTime", time);
    SmartDashboard.putBoolean("HubActive", hubActive);
  }

  private boolean hubActive;
  private double time;

  public void updateMatchTimer() {

    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      hubActive = false;
      time = 0;
      return;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      hubActive = true;
      time = -1;
      return;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      hubActive = false;
      time = -2;
      return;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its likely early in
    // teleop.
    if (gameData.isEmpty()) {
      hubActive = false;
      time = -3;
      return;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        hubActive = true;
        time = matchTime;
        return;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active =
        switch (alliance.get()) {
          case Red -> !redInactiveFirst;
          case Blue -> redInactiveFirst;
        };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      hubActive = true;
      time = matchTime - 130;
    } else if (matchTime > 105) {
      // Shift 1
      hubActive = shift1Active;
      time = matchTime - 105;
    } else if (matchTime > 80) {
      // Shift 2
      hubActive = !shift1Active;
      time = matchTime - 80;
    } else if (matchTime > 55) {
      // Shift 3
      hubActive = shift1Active;
      time = matchTime - 55;
    } else if (matchTime > 30) {
      // Shift 4
      hubActive = !shift1Active;
      time = matchTime - 30;
    } else {
      hubActive = true;
      time = matchTime;
    }
  }
}
