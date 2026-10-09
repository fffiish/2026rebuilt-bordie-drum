package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.commands.RobotSuperstructure;
import frc.robot.constants.intake.IntakeConstants;
import frc.robot.lib.subsystem.angular.AngularIO;
import frc.robot.lib.subsystem.angular.AngularSubsystem;
import frc.robot.lib.subsystem.angular.AngularSubsystemConfig;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerState;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class IntakeRollerCommandTest {
  static final class RecordingIO implements AngularIO {
    double velocityRps;
    int angleCommands;
    int stops;
    int refreshes;
    double requestedAngleRadians = Double.NaN;
    double measuredDegrees = IntakeConstants.kPivotStowed.in(Degrees);
    final List<Double> currentLimits = new ArrayList<>();

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
    public void setCurrentLimit(Current current) {
      currentLimits.add(current.in(Amps));
    }

    @Override
    public void updateInputs(AngularIOInputs inputs) {
      refreshes++;
      inputs.angle = Degrees.of(measuredDegrees);
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
      SimHooks.pauseTiming();
      DriverStationSim.resetData();
      DriverStation.refreshData();
      Constants.kTuningMode = false;
      scheduler.cancelAll();
      scheduler.getDefaultButtonLoop().clear();
      LiveWindow.setEnabled(false);
      scheduler.enable();
      pivot = angular(pivotIO, "RollerCommandPivot");
      pickup = angular(pickupIO, "RollerCommandPickup");
      feeder = angular(feederIO, "RollerCommandFeeder");
      indexerRollers = angular(new RecordingIO(), "RollerCommandIndexer");
      intake = new Intake(pivot, pickup, feeder);
      indexer = new Indexer(indexerRollers, null, null);
      superstructure = new RobotSuperstructure(intake, indexer, null);
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setEnabled(true);
      tick();
    }

    void ticks(int count) {
      DriverStationSim.notifyNewData();
      for (int i = 0; i < count; i++) {
        SimHooks.stepTiming(0.02);
        scheduler.run();
      }
    }

    void tick() {
      ticks(3);
    }

    private AngularSubsystem angular(RecordingIO io, String key) {
      return new AngularSubsystem(io, AngularSubsystemConfig.builder().logKey(key).build());
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

  @Test
  void rollerAndPivotCommandsRunIndependentlyAndReleaseStopsBothRollerGroups() {
    try (var f = new Fixture()) {
      for (var state :
          new IntakePivotState[] {IntakePivotState.kStowed, IntakePivotState.kDeployed}) {
        f.intake.setPersistentPivot(state).initialize();
        var pivotCommand = f.intake.setPivot(IntakePivotState.kRaised);
        var rollerCommand = f.superstructure.intakeFuel();
        assertTrue(
            java.util.Collections.disjoint(
                pivotCommand.getRequirements(), rollerCommand.getRequirements()));
        pivotCommand.schedule();
        rollerCommand.schedule();
        f.tick();
        assertTrue(pivotCommand.isScheduled());
        assertTrue(rollerCommand.isScheduled());
        assertEquals(IntakePivotState.kRaised, f.intake.getPivotState());
        assertEquals(
            IntakeConstants.kIntakeRollerIntaking.in(RotationsPerSecond), f.pickupIO.velocityRps);
        assertEquals(
            IntakeConstants.kFeederIntaking.in(RotationsPerSecond), f.feederIO.velocityRps);
        assertEquals(IndexerState.kIdle, f.indexer.getTargetState());
        int pickupStops = f.pickupIO.stops;
        int feederStops = f.feederIO.stops;
        rollerCommand.cancel();
        assertEquals(pickupStops + 1, f.pickupIO.stops);
        assertEquals(feederStops + 1, f.feederIO.stops);
        assertEquals(0, f.pickupIO.velocityRps);
        assertEquals(0, f.feederIO.velocityRps);
        assertTrue(pivotCommand.isScheduled());
        f.tick();
        assertEquals(0, f.pickupIO.velocityRps);
        assertEquals(0, f.feederIO.velocityRps);
        pivotCommand.cancel();
        assertEquals(state, f.intake.getPivotState());
      }
    }
  }

  @Test
  void outtakeInterruptsIntakeReversesIndexerAndLeavesPivotPositionAlone() {
    try (var f = new Fixture()) {
      f.intake.setPersistentPivot(IntakePivotState.kDeployed).initialize();
      var pickup = f.superstructure.intakeFuel();
      pickup.schedule();
      f.tick();
      var outtake = f.superstructure.outtake();
      outtake.schedule();
      f.tick();
      assertFalse(pickup.isScheduled());
      assertEquals(IntakePivotState.kDeployed, f.intake.getPivotState());
      assertEquals(IntakeRollerState.kEjecting, f.intake.getRollerState());
      assertEquals(IndexerState.kUnjamming, f.indexer.getTargetState());
      assertTrue(f.pickupIO.velocityRps < 0);
      assertTrue(f.feederIO.velocityRps < 0);
      outtake.cancel();
      f.tick();
      assertEquals(IntakeRollerState.kOff, f.intake.getRollerState());
      assertEquals(IndexerState.kIdle, f.indexer.getTargetState());
      assertEquals(0, f.pickupIO.velocityRps);
      assertEquals(0, f.feederIO.velocityRps);
    }
  }

  @Test
  void diagnosticAccessAndDirectStopRefreshBothRollerGroupsWithoutScheduling() {
    try (var f = new Fixture()) {
      var pickup = f.superstructure.intakeFuel();
      pickup.schedule();
      f.tick();
      f.scheduler.cancelAll();
      assertSame(f.pivot, f.intake.getDiagnosticPivot());
      int pickupRefreshes = f.pickupIO.refreshes;
      int feederRefreshes = f.feederIO.refreshes;
      f.intake.diagnosticStopRollers();
      assertEquals(pickupRefreshes + 1, f.pickupIO.refreshes);
      assertEquals(feederRefreshes + 1, f.feederIO.refreshes);
      assertEquals(0, f.pickupIO.velocityRps);
      assertEquals(0, f.feederIO.velocityRps);
      int pivotStops = f.pivotIO.stops;
      f.intake.stopImmediately();
      assertEquals(pivotStops + 1, f.pivotIO.stops);
    }
  }
}
