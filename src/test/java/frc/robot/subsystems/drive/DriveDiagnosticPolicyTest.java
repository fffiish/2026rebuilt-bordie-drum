package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class DriveDiagnosticPolicyTest {
  @Test
  void wrapsCrossBoundaryWithoutLongRotation() {
    assertEquals(Math.toRadians(2.0), DriveDiagnosticPolicy.wrapRadians(Math.toRadians(-358)), 1e-12);
    assertEquals(Math.toRadians(-2.0), DriveDiagnosticPolicy.wrapRadians(Math.toRadians(358)), 1e-12);
    assertEquals(Math.PI, Math.abs(DriveDiagnosticPolicy.wrapRadians(3 * Math.PI)), 1e-12);
    assertTrue(Double.isNaN(DriveDiagnosticPolicy.wrapRadians(Double.NaN)));
  }

  @Test
  void timingRequiresEnoughFreshFastSamples() {
    var timing = new DriveDiagnosticPolicy.TimingWindow();
    for (int i = 1; i <= 99; i++) {
      timing.add(20, i * 0.02);
    }
    assertFalse(timing.ready(1.98));
    timing.add(20, 2.0);
    assertTrue(timing.ready(2.0));
    assertFalse(timing.ready(2.101));
    assertFalse(timing.ready(1.99));
    assertEquals(20, timing.medianMs());
    assertEquals(20, timing.p95Ms());
    timing.add(Double.NaN, 2.02);
    assertFalse(timing.ready(2.02));
    assertEquals(0, timing.sampleCount());
  }

  @Test
  void timingRejectsBothSlowMedianAndTailAndRecoversWithRollingWindow() {
    var timing = new DriveDiagnosticPolicy.TimingWindow();
    for (int i = 1; i <= 100; i++) {
      timing.add(i <= 94 ? 20 : 40, i * 0.02);
    }
    assertEquals(20, timing.medianMs());
    assertEquals(40, timing.p95Ms());
    assertFalse(timing.ready(2));
    for (int i = 101; i <= 350; i++) {
      timing.add(25, i * 0.02);
    }
    assertFalse(timing.ready(7));
    for (int i = 351; i <= 600; i++) {
      timing.add(20, i * 0.02);
    }
    assertTrue(timing.ready(12));
    assertEquals(250, timing.sampleCount());
  }

  @Test
  void heldButtonNeverStartsAcrossEnableOrFaultRecovery() {
    var gate = new DriveDiagnosticPolicy.DeadmanGate();
    assertFalse(gate.update(false, false, true));
    assertFalse(gate.update(true, true, true));
    assertFalse(gate.update(true, false, true));
    assertTrue(gate.update(true, true, true));
    assertTrue(gate.update(true, true, true));
    assertFalse(gate.update(true, true, false));
    assertFalse(gate.update(true, true, true));
    assertFalse(gate.update(true, false, true));
    assertTrue(gate.update(true, true, true));
    assertFalse(gate.update(false, true, true));
    assertFalse(gate.update(true, true, true));
  }

  @Test
  void cancellationRequiresReleaseBeforeRestart() {
    var gate = new DriveDiagnosticPolicy.DeadmanGate();
    gate.update(true, false, true);
    assertTrue(gate.update(true, true, true));
    gate.cancel();
    assertFalse(gate.update(true, true, true));
    assertFalse(gate.update(true, false, true));
    assertTrue(gate.update(true, true, true));
  }

  @Test
  void pulseStopsAtEitherLimitAndInvalidData() {
    assertFalse(DriveDiagnosticPolicy.steerPulseComplete(0.149, Math.toRadians(1.99)));
    assertTrue(DriveDiagnosticPolicy.steerPulseComplete(0.150, 0));
    assertTrue(DriveDiagnosticPolicy.steerPulseComplete(0.01, Math.toRadians(-2)));
    assertTrue(DriveDiagnosticPolicy.steerPulseComplete(-0.01, 0));
    assertTrue(DriveDiagnosticPolicy.steerPulseComplete(0.01, Double.NaN));
    assertFalse(DriveDiagnosticPolicy.sensorsHealthy(true, true, false, true));
    assertFalse(DriveDiagnosticPolicy.sensorsHealthy(true, true, true, false));
    assertTrue(DriveDiagnosticPolicy.sensorsHealthy(true, true, true, true));
    assertFalse(DriveDiagnosticPolicy.operatorEligible(false, true));
    assertFalse(DriveDiagnosticPolicy.operatorEligible(true, false));
  }

  @Test
  void oscillationIgnoresSmallNoiseButTripsOnFourLargeCrossingsAndLatches() {
    var guard = new DriveDiagnosticPolicy.OscillationGuard();
    guard.reset(0, Math.toRadians(5));
    for (int i = 0; i < 20; i++) {
      assertFalse(guard.update(i * 0.01, Math.toRadians(i % 2 == 0 ? 0.9 : -0.9)));
    }
    assertFalse(guard.update(0.2, Math.toRadians(2)));
    assertFalse(guard.update(0.3, Math.toRadians(-2)));
    assertFalse(guard.update(0.4, Math.toRadians(2)));
    assertFalse(guard.update(0.5, Math.toRadians(-2)));
    assertTrue(guard.update(0.6, Math.toRadians(2)));
    assertTrue(guard.update(3, 0));
  }

  @Test
  void oldCrossingsExpireAndGrowthOrBadTimeTrips() {
    var guard = new DriveDiagnosticPolicy.OscillationGuard();
    guard.reset(0, Math.toRadians(5));
    assertFalse(guard.update(0.0, Math.toRadians(2)));
    assertFalse(guard.update(0.6, Math.toRadians(-2)));
    assertFalse(guard.update(1.2, Math.toRadians(2)));
    assertFalse(guard.update(1.8, Math.toRadians(-2)));
    assertFalse(guard.update(2.4, Math.toRadians(2)));
    assertTrue(guard.update(2.5, Math.toRadians(7.1)));
    guard.reset(3, Math.toRadians(5));
    assertFalse(guard.update(3, Math.toRadians(7)));
    assertTrue(guard.update(2.9, 0));
  }
}
