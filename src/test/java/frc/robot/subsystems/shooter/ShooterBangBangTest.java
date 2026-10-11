package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.constants.shooter.ShooterConstants;
import frc.robot.lib.subsystem.angular.AngularIO.AngularIOInputs;
import org.junit.jupiter.api.Test;

class ShooterBangBangTest {
  private static double targetRps(ShooterState state) {
    return state.getFlywheelVelocity().in(RotationsPerSecond);
  }

  @Test
  void presetsMatchPreviousShotsAndHardwareUsesCoastWithoutRampOrVelocityPid() {
    assertEquals(0, ShooterState.kIdle.getFlywheelVelocity().in(RadiansPerSecond));
    assertEquals(
        6.4 / 0.0184, ShooterState.kShootingNear.getFlywheelVelocity().in(RadiansPerSecond));
    assertEquals(
        5.7 / 0.0184, ShooterState.kShootingSoft.getFlywheelVelocity().in(RadiansPerSecond));
    assertEquals(1.25 / 0.0184, ShooterState.kEjecting.getFlywheelVelocity().in(RadiansPerSecond));
    assertEquals(targetRps(ShooterState.kShootingNear), targetRps(ShooterState.kShootingFar));
    var config = ShooterConstants.kFlywheelSparkFlexConfig;
    assertEquals(NeutralModeValue.Coast, config.getNeutralMode());
    assertEquals(0, config.getRampRateSeconds());
    assertEquals(0, config.getKP());
    assertEquals(0, config.getKI());
    assertEquals(0, config.getKD());
    assertEquals(0, config.getKV());
    assertEquals(37, config.getSmartCurrentLimit().in(Amps));
    assertEquals(60, config.getSecondaryCurrentLimit().in(Amps));
    assertEquals(39, config.getMasterId());
    assertEquals(java.util.List.of(34, 26, 29), config.getFollowerIds());
    assertFalse(config.isFollowerOpposed(34));
    assertTrue(config.isFollowerOpposed(26));
    assertTrue(config.isFollowerOpposed(29));
    assertEquals(NeutralModeValue.Coast, ShooterConstants.kFlywheelSimConfig.getNeutralMode());
    assertEquals(
        targetRps(ShooterState.kShootingNear),
        ShooterConstants.kFlywheelMaximumSpeed.in(RotationsPerSecond));
  }

  @Test
  void allActivePresetsSwitchAtTargetAndNeverCallRevVelocityControl() {
    try (var f = new ShooterDriverBindingTest.Fixture(false)) {
      for (var state : ShooterState.values()) {
        f.shooter.setPersistent(state).schedule();
        f.flywheelIO.measuredRps = targetRps(state) - 0.001;
        f.advanceTicks(1);
        assertEquals(state == ShooterState.kIdle ? 0 : 12, f.flywheelIO.requestedVolts);
        f.flywheelIO.measuredRps = targetRps(state);
        f.advanceTicks(1);
        assertEquals(0, f.flywheelIO.requestedVolts, "Exactly at target must coast");
        f.flywheelIO.measuredRps = targetRps(state) + 1;
        f.advanceTicks(1);
        assertEquals(0, f.flywheelIO.requestedVolts, "Overspeed must never reverse/brake");
      }
      assertEquals(0, f.flywheelIO.velocityRequests);
    }
  }

  @Test
  void readinessStartsAtEightyPercentAndRejectsOneSlowFollower() {
    try (var f = new ShooterDriverBindingTest.Fixture(false)) {
      double floor = targetRps(ShooterState.kShootingNear) * 0.8;
      f.axis(0.9);
      f.flywheelIO.measuredRps = floor - 0.001;
      f.advanceTicks(1);
      assertFalse(f.shooter.atSpeed().getAsBoolean());
      f.flywheelIO.measuredRps = floor;
      f.flywheelIO.motorMeasuredRps = new double[] {floor, floor, -floor, -floor};
      f.advanceTicks(1);
      assertTrue(f.shooter.atSpeed().getAsBoolean());
      f.flywheelIO.motorMeasuredRps[2] = -(floor - 0.001);
      f.advanceTicks(1);
      assertFalse(f.shooter.atSpeed().getAsBoolean());
    }
  }

  @Test
  void unhealthySamplesStopOutputAndHealthySamplesRecover() {
    try (var f = new ShooterDriverBindingTest.Fixture(false)) {
      f.axis(0.9);
      assertEquals(12, f.flywheelIO.requestedVolts);
      for (int fault = 0; fault < 7; fault++) {
        switch (fault) {
          case 0 -> f.flywheelIO.configReady = false;
          case 1 -> f.flywheelIO.configPending = true;
          case 2 -> f.flywheelIO.allConnected = false;
          case 3 -> f.flywheelIO.measuredRps = Double.NaN;
          case 4 -> f.flywheelIO.sampleAge = 0.101;
          case 5 -> f.flywheelIO.sampleAge = Double.NaN;
          case 6 ->
              f.flywheelIO.motorMeasuredRps = new double[] {0, 0, Double.POSITIVE_INFINITY, 0};
        }
        f.advanceTicks(1);
        assertEquals(0, f.flywheelIO.requestedVolts, "Fault " + fault);
        assertFalse(f.shooter.atSpeed().getAsBoolean());
        f.flywheelIO.configReady = true;
        f.flywheelIO.configPending = false;
        f.flywheelIO.allConnected = true;
        f.flywheelIO.measuredRps = 0;
        f.flywheelIO.sampleAge = 0;
        f.flywheelIO.motorMeasuredRps = new double[] {0, 0, 0, 0};
        f.advanceTicks(1);
        assertEquals(12, f.flywheelIO.requestedVolts, "Recover " + fault);
      }
    }
  }

  @Test
  void aButtonChangesTargetWhileHeldAndReleaseStops() {
    try (var f = new ShooterDriverBindingTest.Fixture(false)) {
      f.flywheelIO.measuredRps =
          (targetRps(ShooterState.kShootingNear) + targetRps(ShooterState.kShootingSoft)) / 2;
      f.axis(0.9);
      assertEquals(12, f.flywheelIO.requestedVolts);
      f.xboxSim.setAButton(true);
      f.tick();
      assertEquals(ShooterState.kShootingSoft, f.shooter.getTargetState());
      assertEquals(0, f.flywheelIO.requestedVolts);
      f.xboxSim.setAButton(false);
      f.tick();
      assertEquals(ShooterState.kShootingNear, f.shooter.getTargetState());
      assertEquals(12, f.flywheelIO.requestedVolts);
      f.axis(0);
      assertEquals(0, f.flywheelIO.requestedVolts);
    }
  }

  @Test
  void autonomousPersistentSpinupDisableAndImmediateStopAreSafe() {
    try (var f = new ShooterDriverBindingTest.Fixture(false)) {
      DriverStationSim.setAutonomous(true);
      DriverStationSim.notifyNewData();
      DriverStation.refreshData();
      f.shooter.setPersistent(ShooterState.kShootingNear).schedule();
      f.tick();
      assertEquals(12, f.flywheelIO.requestedVolts);
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      DriverStation.refreshData();
      f.advanceTicks(1);
      assertEquals(0, f.flywheelIO.requestedVolts);
      DriverStationSim.setAutonomous(false);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      DriverStation.refreshData();
      f.tick();
      assertEquals(12, f.flywheelIO.requestedVolts);
      f.shooter.stopImmediately();
      assertEquals(0, f.flywheelIO.requestedVolts);
      f.tick();
      assertEquals(ShooterState.kIdle, f.shooter.getTargetState());
      assertEquals(0, f.flywheelIO.requestedVolts);
    }
  }

  /**
   * Test-only first-order plant: powered time constant 0.5 s, coast drag time constant 2 s. Zero
   * voltage means zero motor current; drag alone decelerates the flywheel. Holding speed follows
   * the measured kV. This models control feedback, not REV current limits or real inertia.
   */
  private static final class CoastPlantIO extends ShooterDriverBindingTest.RecordingIO {
    final double busVolts;
    double appliedVolts;

    CoastPlantIO(double busVolts) {
      this.busVolts = busVolts;
    }

    @Override
    public void updateInputs(AngularIOInputs inputs) {
      appliedVolts = Math.min(busVolts, Math.max(0, requestedVolts));
      double steadyRps = appliedVolts / 0.0184 / (2 * Math.PI);
      double timeConstant = appliedVolts == 0 ? 2.0 : 0.5;
      measuredRps = steadyRps + (measuredRps - steadyRps) * Math.exp(-0.02 / timeConstant);
      motorMeasuredRps = new double[] {measuredRps, measuredRps, -measuredRps, -measuredRps};
      super.updateInputs(inputs);
      inputs.busVolts = Volts.of(busVolts);
      inputs.appliedVolts = Volts.of(appliedVolts);
    }
  }

  @Test
  void coastPlantReachesHoldsAndRecoversNormalAndSoftTargetsAtNominalAndReducedVoltage() {
    for (double bus : new double[] {12, 8}) {
      for (var state :
          new ShooterState[] {ShooterState.kShootingNear, ShooterState.kShootingSoft}) {
        var plant = new CoastPlantIO(bus);
        try (var f = new ShooterDriverBindingTest.Fixture(false, plant)) {
          double target = targetRps(state);
          f.shooter.setPersistent(state).schedule();
          int spinupTicks = 0;
          while (!f.shooter.atSpeed().getAsBoolean() && spinupTicks < 100) {
            f.advanceTicks(1);
            spinupTicks++;
          }
          assertTrue(f.shooter.atSpeed().getAsBoolean(), "Reach 80% within 2 seconds");
          // Allow the remainder of the two-second startup window before measuring stability.
          f.advanceTicks(100 - spinupTicks);
          double min = Double.POSITIVE_INFINITY;
          double max = 0;
          for (int i = 0; i < 250; i++) {
            f.advanceTicks(1);
            min = Math.min(min, plant.measuredRps);
            max = Math.max(max, plant.measuredRps);
            assertTrue(plant.measuredRps >= target * 0.9 && plant.measuredRps <= target * 1.1);
            assertTrue(plant.appliedVolts >= 0 && plant.appliedVolts <= bus);
          }
          plant.measuredRps *= 0.8;
          int recoveryTicks = 0;
          while (plant.measuredRps < target * 0.9 && recoveryTicks < 50) {
            f.advanceTicks(1);
            recoveryTicks++;
          }
          assertTrue(plant.measuredRps >= target * 0.9, "Recover within 1 second");
          double speedBeforeStop = plant.measuredRps;
          f.shooter.stopImmediately();
          f.advanceTicks(1);
          assertEquals(0, plant.requestedVolts);
          assertEquals(0, plant.appliedVolts);
          assertEquals(speedBeforeStop * Math.exp(-0.02 / 2), plant.measuredRps, 1e-9);
          assertEquals(0, plant.velocityRequests);
          System.out.printf(
              "BangBang plant %s bus=%.1f: ready=%.2fs range=%.1f..%.1f RPM recovery=%.2fs%n",
              state, bus, spinupTicks * 0.02, min * 60, max * 60, recoveryTicks * 0.02);
        }
      }
    }
  }
}
