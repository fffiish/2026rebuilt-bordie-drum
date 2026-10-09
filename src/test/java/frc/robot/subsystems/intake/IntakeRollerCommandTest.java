package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.RobotSuperstructure;
import frc.robot.constants.intake.IntakeConstants;
import frc.robot.lib.subsystem.angular.AngularIO;
import frc.robot.lib.subsystem.angular.AngularSubsystem;
import frc.robot.lib.subsystem.angular.AngularSubsystemConfig;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerState;
import java.util.Set;
import org.junit.jupiter.api.Test;

class IntakeRollerCommandTest {
  static final class RecordingIO implements AngularIO {
    double velocityRps;
    int angleCommands;
    int stops;
    double requestedAngleRadians = Double.NaN;

    @Override
    public void setVelocity(AngularVelocity velocity) {
      velocityRps = velocity.in(RotationsPerSecond);
    }

    @Override
    public void setAngle(Angle angle) {
      angleCommands++;
      requestedAngleRadians = angle.in(Radians);
    }

    @Override
    public void stop() {
      stops++;
      velocityRps = 0;
    }
  }

  static final class Fixture implements AutoCloseable {
    final boolean originalTuning = Constants.kTuningMode;
    final CommandScheduler scheduler = CommandScheduler.getInstance();
    final RecordingIO pivotIO = new RecordingIO();
    final RecordingIO pickupIO = new RecordingIO();
    final RecordingIO feederIO = new RecordingIO();
    final AngularSubsystem pivot;
    final AngularSubsystem pickup;
    final AngularSubsystem feeder;
    final AngularSubsystem indexerRollers;
    final Intake intake;
    final Indexer indexer;
    final RobotSuperstructure superstructure;

    Fixture() {
      HAL.initialize(500, 0);
      DriverStationSim.resetData();
      DriverStation.refreshData();
      Constants.kTuningMode = false;
      scheduler.cancelAll();
      pivot = angular(pivotIO, "RollerCommandPivot");
      pickup = angular(pickupIO, "RollerCommandPickup");
      feeder = angular(feederIO, "RollerCommandFeeder");
      indexerRollers = angular(new RecordingIO(), "RollerCommandIndexer");
      intake = new Intake(pivot, pickup, feeder);
      indexer = new Indexer(indexerRollers, null, null);
      superstructure = new RobotSuperstructure(intake, indexer, null);
    }

    private AngularSubsystem angular(RecordingIO io, String key) {
      return new AngularSubsystem(io, AngularSubsystemConfig.builder().logKey(key).build());
    }

    @Override
    public void close() {
      scheduler.cancelAll();
      scheduler.getDefaultButtonLoop().clear();
      scheduler.unregisterSubsystem(pivot, pickup, feeder, indexerRollers, intake, indexer);
      Constants.kTuningMode = originalTuning;
      DriverStationSim.resetData();
      DriverStation.refreshData();
    }
  }

  @Test
  void rollerCommandOwnsOnlyBothRollerGroupsAndStopsWithoutChangingPivot() {
    try (var f = new Fixture()) {
      for (var state : new IntakeState[] {IntakeState.kStowed, IntakeState.kDeployed}) {
        f.intake.setPersistent(state).initialize();
        var angle = f.intake.getTargetPivotAngle();
        var command = f.intake.runRollers();
        assertEquals(Set.of(f.pickup, f.feeder), command.getRequirements());
        var scheduled = command.ignoringDisable(true);
        scheduled.schedule();
        f.scheduler.run();
        assertEquals(
            IntakeConstants.kIntakeRollerIntaking.in(RotationsPerSecond), f.pickupIO.velocityRps);
        assertEquals(
            IntakeConstants.kFeederIntaking.in(RotationsPerSecond), f.feederIO.velocityRps);
        assertEquals(state, f.intake.getTargetState());
        assertEquals(angle, f.intake.getTargetPivotAngle());
        assertEquals(0, f.pivotIO.angleCommands);
        scheduled.cancel();
        assertEquals(0, f.pickupIO.velocityRps);
        assertEquals(0, f.feederIO.velocityRps);
        assertEquals(state, f.intake.getTargetState());
        assertEquals(angle, f.intake.getTargetPivotAngle());
        assertEquals(0, f.pivotIO.angleCommands);
        assertEquals(0, f.pivotIO.stops);
      }
      assertEquals(2, f.pickupIO.stops);
      assertEquals(2, f.feederIO.stops);
    }
  }

  @Test
  void triggerReleaseStopsRollersAndIndexerRetainsDeployingAutoCommand() {
    try (var f = new Fixture()) {
      f.intake.setPersistent(IntakeState.kDeployed).initialize();
      var driverCommand = f.superstructure.intakeFuelWithoutDeploy().ignoringDisable(true);
      assertEquals(Set.of(f.pickup, f.feeder, f.indexer), driverCommand.getRequirements());
      boolean[] held = {false};
      new Trigger(() -> held[0]).whileTrue(driverCommand);
      f.scheduler.run();
      held[0] = true;
      f.scheduler.run();
      assertTrue(driverCommand.isScheduled());
      assertEquals(IndexerState.kIntaking, f.indexer.getTargetState());
      assertEquals(IntakeState.kDeployed, f.intake.getTargetState());
      held[0] = false;
      f.scheduler.run();
      assertFalse(driverCommand.isScheduled());
      assertEquals(IndexerState.kIdle, f.indexer.getTargetState());
      assertEquals(0, f.pickupIO.velocityRps);
      assertEquals(0, f.feederIO.velocityRps);
      assertEquals(0, f.pivotIO.angleCommands);
      assertEquals(IntakeState.kDeployed, f.intake.getTargetState());
      var autoCommand = f.superstructure.intakeFuel().ignoringDisable(true);
      assertEquals(Set.of(f.intake, f.indexer), autoCommand.getRequirements());
      autoCommand.schedule();
      f.scheduler.run();
      assertEquals(IntakeState.kIntaking, f.intake.getTargetState());
      autoCommand.cancel();
      assertEquals(IntakeState.kStowed, f.intake.getTargetState());
    }
  }

  @Test
  void releaseSuppressesPreviouslyLatchedIntakingUntilAnotherExplicitStateCommand() {
    try (var f = new Fixture()) {
      f.intake.setPersistent(IntakeState.kIntaking).initialize();
      var pivotTarget = f.intake.getTargetPivotAngle();
      var command = f.intake.runRollers().ignoringDisable(true);
      command.schedule();
      f.scheduler.run();
      command.cancel();
      var pickupDefault = f.pickup.getDefaultCommand();
      var feederDefault = f.feeder.getDefaultCommand();
      pickupDefault.initialize();
      feederDefault.initialize();
      pickupDefault.execute();
      feederDefault.execute();
      assertEquals(IntakeState.kIntaking, f.intake.getTargetState());
      assertEquals(pivotTarget, f.intake.getTargetPivotAngle());
      assertEquals(0, f.pickupIO.velocityRps);
      assertEquals(0, f.feederIO.velocityRps);
      assertEquals(0, f.pivotIO.angleCommands);
      f.intake.setPersistent(IntakeState.kIntaking).initialize();
      pickupDefault.execute();
      feederDefault.execute();
      assertEquals(
          IntakeConstants.kIntakeRollerIntaking.in(RotationsPerSecond), f.pickupIO.velocityRps);
      assertEquals(IntakeConstants.kFeederIntaking.in(RotationsPerSecond), f.feederIO.velocityRps);
      pickupDefault.end(true);
      feederDefault.end(true);
    }
  }
}
