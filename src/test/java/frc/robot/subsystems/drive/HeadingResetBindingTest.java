package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import org.junit.jupiter.api.Test;

class HeadingResetBindingTest {
  @Test
  void startResetsOnlyHeadingForEveryAllianceWhileEnabledOrDisabled() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStationSim.resetData();
    DriverStation.refreshData();
    LiveWindow.setEnabled(false);
    boolean originalTuning = Constants.kTuningMode;
    Constants.kTuningMode = false;
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.cancelAll();
    scheduler.getDefaultButtonLoop().clear();
    scheduler.enable();
    Drive drive =
        new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    CommandXboxController controller = new CommandXboxController(0);
    XboxControllerSim controllerSim = new XboxControllerSim(controller.getHID());
    RobotContainer.bindHeadingReset(controller.start(), drive);
    try {
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setAutonomous(false);
      DriverStationSim.setTest(false);
      for (boolean enabled : new boolean[] {true, false}) {
        DriverStationSim.setEnabled(enabled);
        for (AllianceStationID station :
            new AllianceStationID[] {
              AllianceStationID.Blue1, AllianceStationID.Red1, AllianceStationID.Unknown
            }) {
          DriverStationSim.setAllianceStationId(station);
          controllerSim.setStartButton(false);
          tick(scheduler);
          assertEquals(enabled, DriverStation.isEnabled());
          assertEquals(station == AllianceStationID.Unknown, DriverStation.getAlliance().isEmpty());

          Pose2d initial = new Pose2d(2.4, 5.1, Rotation2d.fromDegrees(63));
          drive.setPose(initial);
          tick(scheduler);
          assertEquals(initial, drive.getPose(), "No reset without a Start press");

          controllerSim.setStartButton(true);
          tick(scheduler);
          double expectedHeading = station == AllianceStationID.Red1 ? 180.0 : 0.0;
          assertEquals(initial.getX(), drive.getPose().getX(), 1e-9);
          assertEquals(initial.getY(), drive.getPose().getY(), 1e-9);
          assertEquals(expectedHeading, drive.getPose().getRotation().getDegrees(), 1e-9);

          Pose2d moved = new Pose2d(3.7, 1.2, Rotation2d.fromDegrees(-42));
          drive.setPose(moved);
          tick(scheduler);
          assertEquals(moved, drive.getPose(), "Holding Start must not continuously reset pose");

          controllerSim.setStartButton(false);
          tick(scheduler);
          controllerSim.setStartButton(true);
          tick(scheduler);
          assertEquals(moved.getX(), drive.getPose().getX(), 1e-9);
          assertEquals(moved.getY(), drive.getPose().getY(), 1e-9);
          assertEquals(expectedHeading, drive.getPose().getRotation().getDegrees(), 1e-9);
        }
      }
    } finally {
      scheduler.cancelAll();
      scheduler.getDefaultButtonLoop().clear();
      scheduler.unregisterSubsystem(drive);
      Constants.kTuningMode = originalTuning;
      DriverStationSim.resetData();
      DriverStation.refreshData();
      SimHooks.resumeTiming();
    }
  }

  private static void tick(CommandScheduler scheduler) {
    DriverStationSim.notifyNewData();
    SimHooks.stepTiming(0.02);
    scheduler.run();
  }
}
