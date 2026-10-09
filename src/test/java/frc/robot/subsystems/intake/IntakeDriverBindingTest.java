package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.constants.intake.IntakeConstants;
import frc.robot.subsystems.indexer.IndexerState;
import org.junit.jupiter.api.Test;

/** Enabled Teleop and Test tests of the Xbox-trigger binding used by RobotContainer. */
class IntakeDriverBindingTest {
  private static final class DriverFixture implements AutoCloseable {
    final IntakeRollerCommandTest.Fixture robot = new IntakeRollerCommandTest.Fixture();
    final CommandXboxController xbox = new CommandXboxController(0);
    final XboxControllerSim xboxSim = new XboxControllerSim(xbox.getHID());

    DriverFixture() {
      robot.superstructure.bindIntakeTrigger(xbox.leftTrigger(0.5));
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setAutonomous(false);
      DriverStationSim.setTest(false);
      DriverStationSim.setEnabled(true);
      axis(0);
      assertTrue(DriverStation.isTeleopEnabled());
    }

    void axis(double value) {
      xboxSim.setLeftTriggerAxis(value);
      tick();
    }

    void tick() {
      DriverStationSim.notifyNewData();
      // Default commands are scheduled at the end of a cycle; run enough cycles to execute them.
      for (int i = 0; i < 3; i++) robot.scheduler.run();
    }

    void assertIntaking() {
      assertEquals(IntakeState.kIntaking, robot.intake.getTargetState());
      assertEquals(
          IntakeConstants.kPivotDeployed.in(Radians), robot.pivotIO.requestedAngleRadians, 1e-9);
      assertEquals(
          IntakeConstants.kIntakeRollerIntaking.in(RotationsPerSecond),
          robot.pickupIO.velocityRps,
          1e-9);
      assertEquals(
          IntakeConstants.kFeederIntaking.in(RotationsPerSecond), robot.feederIO.velocityRps, 1e-9);
      assertTrue(robot.pickupIO.velocityRps > 0);
      assertTrue(robot.feederIO.velocityRps > 0);
      assertEquals(IndexerState.kIntaking, robot.indexer.getTargetState());
    }

    void assertStowed() {
      assertEquals(IntakeState.kStowed, robot.intake.getTargetState());
      assertEquals(
          IntakeConstants.kPivotStowed.in(Radians), robot.pivotIO.requestedAngleRadians, 1e-9);
      assertEquals(0, robot.pickupIO.velocityRps, 1e-9);
      assertEquals(0, robot.feederIO.velocityRps, 1e-9);
      assertEquals(IndexerState.kIdle, robot.indexer.getTargetState());
    }

    @Override
    public void close() {
      robot.close();
    }
  }

  @Test
  void actualXboxWorksInEnabledTestAfterLiveWindowDisablesTheScheduler() {
    try (var driver = new DriverFixture()) {
      DriverStationSim.setTest(true);
      DriverStationSim.notifyNewData();
      assertTrue(DriverStation.isTestEnabled());
      LiveWindow.setEnabled(true);
      driver.robot.scheduler.disable();
      Robot.enableTestControls();
      assertFalse(LiveWindow.isEnabled());
      driver.axis(0.49);
      driver.assertStowed();
      driver.axis(0.8);
      driver.assertIntaking();
      driver.tick();
      driver.assertIntaking();
      driver.axis(0);
      driver.assertStowed();
      assertEquals(1, driver.robot.pickupIO.stops);
      assertEquals(1, driver.robot.feederIO.stops);
      driver.axis(0.9);
      driver.assertIntaking();
      driver.axis(0);
      driver.assertStowed();
    }
  }

  @Test
  void actualXboxLeftTriggerDeploysRunsBothRollerGroupsAndStowsOnReleaseAndRepress() {
    try (var driver = new DriverFixture()) {
      driver.assertStowed();
      driver.axis(0.49);
      driver.assertStowed();
      driver.axis(0.5);
      driver.assertStowed();
      driver.axis(0.8);
      driver.assertIntaking();
      driver.tick();
      driver.assertIntaking();
      driver.axis(0);
      driver.assertStowed();
      assertEquals(1, driver.robot.pickupIO.stops, "Release must directly stop pickup");
      assertEquals(1, driver.robot.feederIO.stops, "Release must directly stop feeder");
      driver.axis(0.9);
      driver.assertIntaking();
      driver.axis(0);
      driver.assertStowed();
      assertEquals(2, driver.robot.pickupIO.stops);
      assertEquals(2, driver.robot.feederIO.stops);
    }
  }

  @Test
  void disableCancelsIntakeStopsRollersAndRequiresAnotherTriggerEdge() {
    try (var driver = new DriverFixture()) {
      driver.axis(0.9);
      driver.assertIntaking();
      DriverStationSim.setEnabled(false);
      driver.tick();
      assertEquals(IntakeState.kStowed, driver.robot.intake.getTargetState());
      assertEquals(0, driver.robot.pickupIO.velocityRps);
      assertEquals(0, driver.robot.feederIO.velocityRps);
      DriverStationSim.setEnabled(true);
      driver.tick();
      driver.assertStowed();
      driver.axis(0);
      driver.axis(0.9);
      driver.assertIntaking();
    }
  }

  @Test
  void triggerDeploysAfterPreviousRollersOnlyReleaseSuppressedDefaultVelocities() {
    try (var driver = new DriverFixture()) {
      var rollersOnly = driver.robot.intake.runRollers();
      rollersOnly.schedule();
      driver.tick();
      rollersOnly.cancel();
      driver.tick();
      driver.assertStowed();
      driver.axis(0.9);
      driver.assertIntaking();
      driver.axis(0);
      driver.assertStowed();
    }
  }
}
