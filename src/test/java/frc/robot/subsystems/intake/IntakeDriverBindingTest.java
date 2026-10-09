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
import java.util.List;
import org.junit.jupiter.api.Test;

/** Enabled Xbox controls, deployment mode transitions, and the explicit Test deploy command. */
class IntakeDriverBindingTest {
  private static final class DriverFixture implements AutoCloseable {
    final IntakeRollerCommandTest.Fixture robot = new IntakeRollerCommandTest.Fixture();
    final CommandXboxController xbox = new CommandXboxController(0);
    final XboxControllerSim xboxSim = new XboxControllerSim(xbox.getHID());

    DriverFixture() {
      xboxSim.setLeftTriggerAxis(0);
      DriverStationSim.notifyNewData();
      robot.superstructure.bindIntakeTrigger(xbox.leftTrigger(0.5));
      axis(0);
    }

    void axis(double value) {
      xboxSim.setLeftTriggerAxis(value);
      robot.tick();
    }

    void assertRollers(boolean running) {
      assertEquals(
          running ? IntakeRollerState.kIntaking : IntakeRollerState.kOff,
          robot.intake.getRollerState());
      assertEquals(
          running ? IntakeConstants.kIntakeRollerIntaking.in(RotationsPerSecond) : 0,
          robot.pickupIO.velocityRps,
          1e-9);
      assertEquals(
          running ? IntakeConstants.kFeederIntaking.in(RotationsPerSecond) : 0,
          robot.feederIO.velocityRps,
          1e-9);
      assertEquals(IndexerState.kIdle, robot.indexer.getTargetState());
    }

    @Override
    public void close() {
      robot.close();
    }
  }

  @Test
  void xboxRollersRetainStowedOrDeployedPivotAndStopOnReleaseAndRepress() {
    try (var driver = new DriverFixture()) {
      for (var state :
          new IntakePivotState[] {IntakePivotState.kStowed, IntakePivotState.kDeployed}) {
        driver.robot.intake.setPersistentPivot(state).initialize();
        driver.axis(0.49);
        driver.assertRollers(false);
        driver.axis(0.5);
        driver.assertRollers(false);
        driver.axis(0.8);
        driver.assertRollers(true);
        assertEquals(state, driver.robot.intake.getPivotState());
        int stops = driver.robot.pickupIO.stops;
        driver.axis(0);
        driver.assertRollers(false);
        assertEquals(state, driver.robot.intake.getPivotState());
        assertEquals(stops + 1, driver.robot.pickupIO.stops);
        driver.axis(0.9);
        driver.assertRollers(true);
        driver.axis(0);
        driver.assertRollers(false);
      }
    }
  }

  @Test
  void enabledTestUsesNormalXboxControlsWithoutAutomaticallyDeploying() {
    try (var driver = new DriverFixture()) {
      driver.robot.superstructure.bindDeploymentTriggers();
      DriverStationSim.setTest(true);
      DriverStationSim.notifyNewData();
      LiveWindow.setEnabled(true);
      driver.robot.scheduler.disable();
      Robot.enableTestControls();
      assertTrue(DriverStation.isTestEnabled());
      assertFalse(LiveWindow.isEnabled());
      driver.axis(0.8);
      driver.assertRollers(true);
      assertEquals(IntakePivotState.kStowed, driver.robot.intake.getPivotState());
      driver.axis(0);
      driver.assertRollers(false);
    }
  }

  @Test
  void disableStopsRollersAndRequiresAnotherTriggerEdge() {
    try (var driver = new DriverFixture()) {
      driver.robot.intake.setPersistentPivot(IntakePivotState.kDeployed).initialize();
      driver.axis(0.9);
      DriverStationSim.setEnabled(false);
      driver.robot.tick();
      driver.assertRollers(false);
      assertEquals(IntakePivotState.kDeployed, driver.robot.intake.getPivotState());
      DriverStationSim.setEnabled(true);
      driver.robot.tick();
      driver.assertRollers(false);
      driver.axis(0);
      driver.axis(0.9);
      driver.assertRollers(true);
    }
  }

  @Test
  void deployRequestsEightyThenSixtyAfterMeasuredArrival() {
    try (var f = new IntakeRollerCommandTest.Fixture()) {
      var deploy = f.intake.deployOnce();
      deploy.schedule();
      f.tick();
      assertEquals(List.of(80.0), f.pivotIO.currentLimits);
      assertTrue(deploy.isScheduled());
      assertEquals(IntakePivotState.kDeployed, f.intake.getPivotState());
      f.pivotIO.measuredDegrees = 0;
      f.tick();
      assertFalse(deploy.isScheduled());
      assertEquals(List.of(80.0, 60.0), f.pivotIO.currentLimits);
    }
  }

  @Test
  void deployRestoresSixtyAfterTwoSecondTimeoutWithoutArrival() {
    try (var f = new IntakeRollerCommandTest.Fixture()) {
      var deploy = f.intake.deployOnce();
      deploy.schedule();
      f.ticks(90);
      assertTrue(deploy.isScheduled());
      assertEquals(List.of(80.0), f.pivotIO.currentLimits);
      f.ticks(15);
      assertFalse(deploy.isScheduled());
      assertEquals(List.of(80.0, 60.0), f.pivotIO.currentLimits);
      assertTrue(f.intake.isDeployed());
    }
  }

  @Test
  void cancelledDeployKeepsExistingEightyAmpRequestAndDeployedRestingState() {
    try (var f = new IntakeRollerCommandTest.Fixture()) {
      var deploy = f.intake.deployOnce();
      deploy.schedule();
      f.tick();
      deploy.cancel();
      f.ticks(110);
      assertEquals(List.of(80.0), f.pivotIO.currentLimits);
      assertTrue(f.intake.isDeployed());
      var repeat = f.intake.deployOnce();
      repeat.schedule();
      f.tick();
      assertFalse(repeat.isScheduled());
      assertEquals(List.of(80.0), f.pivotIO.currentLimits);
    }
  }

  @Test
  void autonomousThenTeleopDeploysOnlyOnceAndPracticeTeleopAlsoDeploys() {
    for (boolean runAuto : new boolean[] {true, false}) {
      try (var f = new IntakeRollerCommandTest.Fixture()) {
        DriverStationSim.setEnabled(false);
        f.tick();
        f.superstructure.bindDeploymentTriggers();
        f.pivotIO.measuredDegrees = 0;
        DriverStationSim.setAutonomous(runAuto);
        DriverStationSim.setEnabled(true);
        f.tick();
        f.tick();
        assertTrue(f.intake.isDeployed());
        assertEquals(List.of(80.0, 60.0), f.pivotIO.currentLimits);
        if (runAuto) {
          DriverStationSim.setEnabled(false);
          f.tick();
          DriverStationSim.setAutonomous(false);
          DriverStationSim.setEnabled(true);
          f.tick();
          assertEquals(List.of(80.0, 60.0), f.pivotIO.currentLimits);
        }
      }
    }
  }

  @Test
  void explicitTestDeployRequiresEnabledTestWithoutSelectedDiagnostic() {
    try (var f = new IntakeRollerCommandTest.Fixture()) {
      boolean[] diagnostic = {false};
      var deploy = f.superstructure.deployIntakeForTest(() -> diagnostic[0]);
      deploy.schedule();
      f.tick();
      assertFalse(f.intake.isDeployed(), "Teleop cannot use the Test dashboard command");
      DriverStationSim.setTest(true);
      DriverStationSim.setEnabled(false);
      f.tick();
      deploy.schedule();
      f.tick();
      assertFalse(f.intake.isDeployed());
      DriverStationSim.setEnabled(true);
      diagnostic[0] = true;
      f.tick();
      deploy.schedule();
      f.tick();
      assertFalse(f.intake.isDeployed());
      diagnostic[0] = false;
      deploy.schedule();
      f.tick();
      assertTrue(f.intake.isDeployed());
      assertEquals(List.of(80.0), f.pivotIO.currentLimits);
    }
  }

  @Test
  void persistentStowAllowsLaterExplicitRedeployWithoutChangingRollers() {
    try (var f = new IntakeRollerCommandTest.Fixture()) {
      f.intake.setPersistentPivot(IntakePivotState.kDeployed).initialize();
      var rollers = f.superstructure.intakeFuel();
      rollers.schedule();
      f.tick();
      f.intake.setPersistentPivot(IntakePivotState.kStowed).initialize();
      assertEquals(IntakePivotState.kStowed, f.intake.getPivotState());
      assertEquals(IntakeRollerState.kIntaking, f.intake.getRollerState());
      var deploy = f.intake.deployOnce();
      deploy.schedule();
      f.tick();
      assertEquals(IntakePivotState.kDeployed, f.intake.getPivotState());
      assertTrue(rollers.isScheduled());
    }
  }
}
