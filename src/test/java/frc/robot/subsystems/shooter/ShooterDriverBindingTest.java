package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.RobotSuperstructure;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.shooter.ShooterConstants;
import frc.robot.lib.subsystem.DeviceConnectedStatus;
import frc.robot.lib.subsystem.angular.AngularIO;
import frc.robot.lib.subsystem.angular.AngularSubsystem;
import frc.robot.lib.subsystem.angular.AngularSubsystemConfig;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakePivotState;
import frc.robot.subsystems.intake.IntakeRollerState;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class ShooterDriverBindingTest {
  private static final class RecordingIO implements AngularIO {
    double requestedRps;
    double measuredRps;
    double[] motorMeasuredRps = {};
    boolean allConnected = true;
    double requestedAngleRadians;
    int stops;

    @Override
    public void setAngle(Angle angle) {
      requestedAngleRadians = angle.in(Radians);
    }

    @Override
    public void stop() {
      stops++;
      requestedRps = 0;
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
      requestedRps = velocity.in(RotationsPerSecond);
    }

    @Override
    public void updateInputs(AngularIOInputs inputs) {
      inputs.velocity = RotationsPerSecond.of(measuredRps);
      inputs.goalVel = RotationsPerSecond.of(requestedRps);
      inputs.motorVelocitiesRadiansPerSecond =
          java.util.Arrays.stream(motorMeasuredRps).map(value -> value * 2.0 * Math.PI).toArray();
      inputs.deviceConnectedStatuses =
          new DeviceConnectedStatus[] {new DeviceConnectedStatus(allConnected, 39)};
    }
  }

  private static final class Fixture implements AutoCloseable {
    final boolean originalTuning = Constants.kTuningMode;
    final CommandScheduler scheduler = CommandScheduler.getInstance();
    final RecordingIO flywheelIO = new RecordingIO();
    final RecordingIO indexerIO = new RecordingIO();
    final RecordingIO pivotIO = new RecordingIO();
    final RecordingIO pickupIO = new RecordingIO();
    final RecordingIO feederIO = new RecordingIO();
    boolean hoodConnected = true;
    final AngularSubsystem pivot;
    final AngularSubsystem pickup;
    final AngularSubsystem feeder;
    final Intake intake;
    final RobotSuperstructure superstructure;
    final AngularSubsystem flywheel;
    final AngularSubsystem indexerRollers;
    final Hood hood;
    final Shooter shooter;
    final Indexer indexer;
    final CommandXboxController xbox;
    final XboxControllerSim xboxSim;

    Fixture(boolean testMode) {
      HAL.initialize(500, 0);
      SimHooks.pauseTiming();
      DriverStationSim.resetData();
      DriverStation.refreshData();
      Constants.kTuningMode = false;
      scheduler.cancelAll();
      scheduler.getDefaultButtonLoop().clear();
      flywheel =
          new AngularSubsystem(
              flywheelIO,
              AngularSubsystemConfig.builder()
                  .logKey("ShooterBindingFlywheel")
                  .bus(RobotConstants.kRioBus)
                  .velocityTolerance(RotationsPerSecond.of(0.5))
                  .build());
      indexerRollers =
          new AngularSubsystem(
              indexerIO, AngularSubsystemConfig.builder().logKey("ShooterBindingIndexer").build());
      hood =
          new Hood(
              new HoodIO() {
                @Override
                public void updateInputs(HoodIOInputs inputs) {
                  inputs.connected = hoodConnected;
                  inputs.controllerConnected = hoodConnected;
                  inputs.positionNormalized = 0;
                }
              });
      shooter = new Shooter(flywheel, hood);
      indexer = new Indexer(indexerRollers, null, null);
      pivot =
          new AngularSubsystem(
              pivotIO, AngularSubsystemConfig.builder().logKey("ShooterBindingPivot").build());
      pickup =
          new AngularSubsystem(
              pickupIO, AngularSubsystemConfig.builder().logKey("ShooterBindingPickup").build());
      feeder =
          new AngularSubsystem(
              feederIO, AngularSubsystemConfig.builder().logKey("ShooterBindingFeeder").build());
      intake = new Intake(pivot, pickup, feeder);
      superstructure = new RobotSuperstructure(intake, indexer, shooter);
      xbox = new CommandXboxController(0);
      xboxSim = new XboxControllerSim(xbox.getHID());
      xboxSim.setLeftTriggerAxis(0);
      xboxSim.setRightTriggerAxis(0);
      xboxSim.setRightBumperButton(false);
      DriverStationSim.notifyNewData();
      xbox.rightTrigger(0.5).whileTrue(superstructure.shoot());
      superstructure.bindIntakeTrigger(xbox.leftTrigger(0.5));
      xbox.rightBumper().whileTrue(superstructure.outtake());
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setAutonomous(false);
      DriverStationSim.setTest(testMode);
      DriverStationSim.setEnabled(true);
      if (testMode) {
        LiveWindow.setEnabled(true);
        Robot.enableTestControls();
      } else {
        LiveWindow.setEnabled(false);
        scheduler.enable();
      }
      // Establish idle readiness before the first shooting request.
      flywheelIO.measuredRps = ShooterConstants.kFlywheelIdle.in(RotationsPerSecond);
      axis(0);
      assertTrue(testMode ? DriverStation.isTestEnabled() : DriverStation.isTeleopEnabled());
      assertTrue(shooter.readyToFire().getAsBoolean(), "Establish stale idle-ready condition");
    }

    void tick() {
      advanceTicks(3);
    }

    void advanceTicks(int count) {
      DriverStationSim.notifyNewData();
      for (int i = 0; i < count; i++) {
        SimHooks.stepTiming(0.02);
        scheduler.run();
      }
    }

    void axis(double value) {
      xboxSim.setRightTriggerAxis(value);
      tick();
    }

    @Override
    public void close() {
      scheduler.cancelAll();
      scheduler.getDefaultButtonLoop().clear();
      scheduler.unregisterAllSubsystems();
      Constants.kTuningMode = originalTuning;
      LiveWindow.setEnabled(false);
      scheduler.enable();
      DriverStationSim.resetData();
      DriverStation.refreshData();
      SimHooks.resumeTiming();
    }
  }

  private void verifyRightTrigger(boolean testMode) {
    try (var robot = new Fixture(testMode)) {
      robot.axis(0.49);
      assertEquals(ShooterState.kIdle, robot.shooter.getTargetState());
      robot.axis(0.8);
      assertEquals(ShooterState.kShootingNear, robot.shooter.getTargetState());
      assertEquals(
          30, robot.shooter.getTargetState().getFlywheelVelocity().in(RotationsPerSecond), 1e-9);
      assertEquals(30, robot.flywheelIO.requestedRps, 1e-9);
      assertFalse(robot.shooter.atSpeed().getAsBoolean(), "Flywheel is still stopped");
      assertEquals(IndexerState.kIdle, robot.indexer.getTargetState());
      assertEquals(0, robot.indexerIO.requestedRps, 1e-9);
      assertEquals(0, robot.feederIO.requestedRps, 1e-9);
      assertEquals(0, robot.pickupIO.requestedRps, 1e-9);
      robot.flywheelIO.measuredRps = 29.49;
      robot.tick();
      assertFalse(robot.shooter.atSpeed().getAsBoolean());
      assertEquals(0, robot.indexerIO.requestedRps, 1e-9);
      assertEquals(0, robot.feederIO.requestedRps, 1e-9);
      robot.flywheelIO.measuredRps = ShooterConstants.kFlywheelShooting.in(RotationsPerSecond);
      robot.tick();
      assertEquals(IndexerState.kFeeding, robot.indexer.getTargetState());
      assertEquals(32, robot.indexerIO.requestedRps, 1e-9);
      assertEquals(15, robot.feederIO.requestedRps, 1e-9);
      robot.xboxSim.setRightTriggerAxis(0);
      robot.advanceTicks(1);
      assertEquals(ShooterState.kIdle, robot.shooter.getTargetState());
      assertEquals(IndexerState.kIdle, robot.indexer.getTargetState());
      assertEquals(
          ShooterConstants.kFlywheelIdle.in(RotationsPerSecond),
          robot.flywheelIO.requestedRps,
          1e-9);
      assertEquals(0, robot.indexerIO.requestedRps, 1e-9);
      assertEquals(0, robot.feederIO.requestedRps, 1e-9);
      assertEquals(0, robot.pickupIO.requestedRps, 1e-9);
    }
  }

  @Test
  void xboxRightTriggerWaitsForThirtyRpsThenFeedsAndReleasesInTeleop() {
    verifyRightTrigger(false);
  }

  @Test
  void xboxRightTriggerWaitsForThirtyRpsThenFeedsAndReleasesInTest() {
    verifyRightTrigger(true);
  }

  @Test
  void rightTriggerStartsBottomRollersAndFeederEvenWhenHoodIsNotReady() {
    try (var robot = new Fixture(false)) {
      robot.hoodConnected = false;
      robot.axis(0.8);
      assertFalse(robot.shooter.hoodAtTarget().getAsBoolean());
      assertFalse(robot.shooter.readyToFire().getAsBoolean());
      assertEquals(30, robot.flywheelIO.requestedRps, 1e-9);
      assertEquals(0, robot.indexerIO.requestedRps, 1e-9);
      assertEquals(0, robot.feederIO.requestedRps, 1e-9);
      robot.flywheelIO.measuredRps = 30;
      robot.tick();
      assertFalse(robot.shooter.hoodAtTarget().getAsBoolean());
      assertEquals(32, robot.indexerIO.requestedRps, 1e-9);
      assertEquals(15, robot.feederIO.requestedRps, 1e-9);
      robot.axis(0);
      assertEquals(0, robot.flywheelIO.requestedRps, 1e-9);
      assertEquals(0, robot.indexerIO.requestedRps, 1e-9);
      assertEquals(0, robot.feederIO.requestedRps, 1e-9);
    }
  }

  @Test
  void aStoppedOrDisconnectedFollowerKeepsTheShotWaiting() {
    try (var robot = new Fixture(false)) {
      robot.flywheelIO.measuredRps = 30;
      robot.flywheelIO.motorMeasuredRps = new double[] {30, 30, 0, 30};
      robot.axis(0.8);
      assertFalse(robot.shooter.atSpeed().getAsBoolean());
      assertEquals(0, robot.indexerIO.requestedRps, 1e-9);
      assertEquals(0, robot.feederIO.requestedRps, 1e-9);
      robot.flywheelIO.motorMeasuredRps = new double[] {30, 30, 30, 30};
      robot.flywheelIO.allConnected = false;
      robot.tick();
      assertFalse(robot.shooter.atSpeed().getAsBoolean());
      assertEquals(0, robot.indexerIO.requestedRps, 1e-9);
      robot.flywheelIO.allConnected = true;
      robot.tick();
      assertTrue(robot.shooter.atSpeed().getAsBoolean());
      assertEquals(32, robot.indexerIO.requestedRps, 1e-9);
      assertEquals(15, robot.feederIO.requestedRps, 1e-9);
    }
  }

  @Test
  void flywheelEncoderPolarityDoesNotBlockFeedingAtTargetSpeed() {
    for (double followerSign : new double[] {1, -1}) {
      try (var robot = new Fixture(false)) {
        // The actual run reached target with positive raw RPM on all four encoders, even
        // though 26/29 are configured as opposed followers. Either encoder sign is valid.
        robot.flywheelIO.measuredRps = 30.072;
        robot.flywheelIO.motorMeasuredRps =
            new double[] {30.072, 30.064, followerSign * 30.087, followerSign * 30.055};
        robot.axis(0.8);
        assertTrue(robot.shooter.atSpeed().getAsBoolean());
        assertEquals(32, robot.indexerIO.requestedRps, 1e-9);
        assertEquals(15, robot.feederIO.requestedRps, 1e-9);
        robot.axis(0);
        assertEquals(0, robot.indexerIO.requestedRps, 1e-9);
        assertEquals(0, robot.feederIO.requestedRps, 1e-9);
      }
    }
  }

  @Test
  void backwardsLeaderOrUnderSpeedFollowerKeepsFeedingOff() {
    try (var robot = new Fixture(false)) {
      robot.flywheelIO.measuredRps = -30;
      robot.flywheelIO.motorMeasuredRps = new double[] {-30, -30, 30, 30};
      robot.axis(0.8);
      assertFalse(robot.shooter.atSpeed().getAsBoolean());
      assertEquals(0, robot.indexerIO.requestedRps, 1e-9);
      assertEquals(0, robot.feederIO.requestedRps, 1e-9);
      robot.flywheelIO.measuredRps = 30;
      robot.flywheelIO.motorMeasuredRps = new double[] {30, 30, -29.49, -30};
      robot.tick();
      assertFalse(robot.shooter.atSpeed().getAsBoolean());
      assertEquals(0, robot.indexerIO.requestedRps, 1e-9);
      assertEquals(0, robot.feederIO.requestedRps, 1e-9);
    }
  }

  @Test
  void releasingDuringSpinUpCancelsThePendingShot() {
    try (var robot = new Fixture(false)) {
      robot.axis(0.8);
      robot.axis(0);
      robot.flywheelIO.measuredRps = 30;
      robot.tick();
      assertEquals(ShooterState.kIdle, robot.shooter.getTargetState());
      assertEquals(IndexerState.kIdle, robot.indexer.getTargetState());
      assertEquals(0, robot.flywheelIO.requestedRps, 1e-9);
      assertEquals(0, robot.indexerIO.requestedRps, 1e-9);
      assertEquals(0, robot.feederIO.requestedRps, 1e-9);
    }
  }

  @Test
  void simultaneousTriggersShootImmediatelyWhileArmAgitatesAndIntakeContinues() {
    try (var robot = new Fixture(false)) {
      robot.intake.setPersistentPivot(IntakePivotState.kDeployed).initialize();
      robot.xboxSim.setLeftTriggerAxis(0.9);
      robot.axis(0.9);
      assertEquals(IntakeRollerState.kIntaking, robot.intake.getRollerState());
      assertEquals(ShooterState.kShootingNear, robot.shooter.getTargetState());
      assertEquals(30, robot.flywheelIO.requestedRps, 1e-9);
      assertEquals(IntakePivotState.kDeployed, robot.intake.getPivotState());
      robot.flywheelIO.measuredRps = 30;
      robot.tick();
      assertEquals(IndexerState.kFeeding, robot.indexer.getTargetState());
      assertEquals(32, robot.indexerIO.requestedRps, 1e-9);
      assertEquals(15, robot.feederIO.requestedRps, 1e-9);
      assertEquals(IntakePivotState.kAgitateHigh, robot.intake.getPivotState());
      List<IntakePivotState> states = new ArrayList<>();
      states.add(robot.intake.getPivotState());
      for (int i = 0; i < 140; i++) {
        robot.advanceTicks(1);
        var state = robot.intake.getPivotState();
        if (states.get(states.size() - 1) != state) states.add(state);
      }
      assertEquals(
          List.of(
              IntakePivotState.kAgitateHigh,
              IntakePivotState.kAgitateLow,
              IntakePivotState.kAgitateHigh,
              IntakePivotState.kAgitateLow,
              IntakePivotState.kAgitateHigh,
              IntakePivotState.kAgitateLow,
              IntakePivotState.kRaised),
          states);
      assertEquals(IntakeRollerState.kIntaking, robot.intake.getRollerState());
      assertTrue(robot.pickupIO.requestedRps > 0);
      assertTrue(robot.feederIO.requestedRps > 0);
      robot.axis(0);
      assertEquals(IntakePivotState.kDeployed, robot.intake.getPivotState());
      assertEquals(IntakeRollerState.kIntaking, robot.intake.getRollerState());
      assertEquals(IndexerState.kIdle, robot.indexer.getTargetState());
      robot.xboxSim.setLeftTriggerAxis(0);
      robot.tick();
      assertEquals(0, robot.pickupIO.requestedRps);
      assertEquals(0, robot.feederIO.requestedRps);
      robot.axis(0.9);
      assertEquals(IntakePivotState.kAgitateHigh, robot.intake.getPivotState());
      robot.axis(0);
      assertEquals(IntakePivotState.kDeployed, robot.intake.getPivotState());
    }
  }

  @Test
  void shootingWithoutDeploymentSpinsFlywheelAndWaitsBeforeMovingArm() {
    try (var robot = new Fixture(true)) {
      assertFalse(robot.intake.isDeployed());
      robot.axis(0.9);
      assertEquals(IntakePivotState.kStowed, robot.intake.getPivotState());
      assertEquals(30, robot.flywheelIO.requestedRps, 1e-9);
      assertEquals(0, robot.indexerIO.requestedRps, 1e-9);
      assertEquals(0, robot.feederIO.requestedRps, 1e-9);
      robot.flywheelIO.measuredRps = 30;
      robot.tick();
      assertEquals(IntakePivotState.kAgitateHigh, robot.intake.getPivotState());
      robot.axis(0);
      assertEquals(IntakePivotState.kStowed, robot.intake.getPivotState());
    }
  }

  @Test
  void rightBumperInterruptsBothTriggersAndReleaseStopsOuttake() {
    try (var robot = new Fixture(false)) {
      robot.intake.setPersistentPivot(IntakePivotState.kDeployed).initialize();
      robot.xboxSim.setLeftTriggerAxis(0.9);
      robot.axis(0.9);
      robot.xboxSim.setRightBumperButton(true);
      robot.tick();
      assertEquals(ShooterState.kIdle, robot.shooter.getTargetState());
      assertEquals(IntakePivotState.kDeployed, robot.intake.getPivotState());
      assertEquals(IntakeRollerState.kEjecting, robot.intake.getRollerState());
      assertEquals(IndexerState.kUnjamming, robot.indexer.getTargetState());
      robot.xboxSim.setRightBumperButton(false);
      robot.tick();
      assertEquals(IntakeRollerState.kOff, robot.intake.getRollerState());
      assertEquals(IndexerState.kIdle, robot.indexer.getTargetState());
      assertEquals(0, robot.pickupIO.requestedRps);
      assertEquals(0, robot.feederIO.requestedRps);
      assertEquals(ShooterState.kIdle, robot.shooter.getTargetState());
    }
  }

  @Test
  void diagnosticCancellationStopsHeldCommandsAndRetainsDirectDiagnosticAccess() {
    try (var robot = new Fixture(true)) {
      robot.xboxSim.setLeftTriggerAxis(0.9);
      robot.axis(0.9);
      // Robot cancels all commands before entering its existing isolated diagnostic path.
      robot.scheduler.cancelAll();
      robot.intake.diagnosticStopRollers();
      robot.indexer.stopImmediately();
      robot.shooter.stopImmediately();
      assertEquals(IntakeRollerState.kOff, robot.intake.getRollerState());
      assertEquals(IntakePivotState.kStowed, robot.intake.getPivotState());
      assertEquals(0, robot.pickupIO.requestedRps);
      assertEquals(0, robot.feederIO.requestedRps);
      assertEquals(0, robot.indexerIO.requestedRps);
      assertEquals(0, robot.flywheelIO.requestedRps);
      assertSame(robot.pivot, robot.intake.getDiagnosticPivot());
    }
  }
}
