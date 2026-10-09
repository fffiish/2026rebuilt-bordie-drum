package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.subsystems.drive.DriveDiagnosticPolicy;
import frc.robot.subsystems.drive.RemoteDiagnosticPolicy;
import frc.robot.subsystems.intake.IntakeDiagnosticPolicy.Mode;
import frc.robot.subsystems.intake.IntakeDiagnosticPolicy.Reference;
import frc.robot.subsystems.intake.IntakeDiagnosticPolicy.Request;
import frc.robot.subsystems.intake.IntakeDiagnosticPolicy.Sample;
import org.junit.jupiter.api.Test;

class IntakeDiagnosticPolicyTest {
  private static final Reference RANGE = new Reference(Math.toRadians(95), 0, Math.toRadians(95));
  private static final Request PULSE =
      new Request(Mode.FEEDBACK, .1, Math.toRadians(1), .1, .03, 10, 45);
  private static final Request STEP =
      new Request(Mode.POSITION, .1, Math.toRadians(1), .75, .03, 10, 45);

  private static Sample sample(
      double timestamp, double angle, double current, boolean connected, boolean ready) {
    return new Sample(timestamp, angle, 0, .1, current, 25, connected, ready, .03);
  }

  @Test
  void enabledChangesCannotReleaseOrSelectIsolation() {
    var latch = new IntakeDiagnosticPolicy.SelectionLatch();
    assertFalse(latch.update(false, true));
    assertTrue(latch.update(true, true));
    assertTrue(latch.update(false, false));
    assertTrue(latch.update(false, true));
    assertFalse(latch.update(true, false));
    assertFalse(latch.update(false, true));
  }

  @Test
  void explicitStopStaysLatchedAcrossDisableUntilClearedWhileDisabled() {
    var latch = new IntakeDiagnosticPolicy.StopLatch();
    assertFalse(latch.update(true, false));
    assertTrue(latch.update(false, true));
    assertTrue(latch.update(false, false));
    assertTrue(latch.update(true, true));
    assertTrue(latch.update(false, false));
    assertTrue(
        latch.update(true, false), "An enabled clear cannot silently take effect upon disabling");
    assertTrue(latch.update(true, true));
    assertFalse(latch.update(true, false));
  }

  @Test
  void startupSeedIsNotAnInteriorPositionStartingPoint() {
    assertTrue(RANGE.valid());
    assertFalse(RANGE.interior(RANGE.referenceRad()));
    assertFalse(
        IntakeDiagnosticPolicy.motionFault(
                RANGE, STEP, RANGE.referenceRad(), RANGE.referenceRad(), RANGE.referenceRad())
            .isEmpty());
    assertFalse(new Reference(Double.NaN, 0, Math.toRadians(95)).valid());
    assertFalse(new Reference(0, 0, 0).valid());
    assertFalse(new Reference(0, -Math.PI, Math.PI).valid());
  }

  @Test
  void requestsEnforceAllIndependentHardLimits() {
    assertTrue(PULSE.valid());
    assertTrue(STEP.valid());
    assertFalse(new Request(Mode.OFF, .1, .01, .1, .03, 10, 45).valid());
    assertFalse(new Request(Mode.FEEDBACK, .251, .01, .1, .03, 10, 45).valid());
    assertFalse(new Request(Mode.FEEDBACK, .1, .01, .151, .03, 10, 45).valid());
    assertFalse(new Request(Mode.POSITION, .1, Math.toRadians(2.01), .75, .03, 10, 45).valid());
    assertFalse(new Request(Mode.POSITION, .1, .01, 1.01, .03, 10, 45).valid());
    assertFalse(new Request(Mode.POSITION, .1, .01, .75, .051, 10, 45).valid());
    assertFalse(new Request(Mode.POSITION, .1, .01, .75, .03, 15.01, 45).valid());
    assertFalse(new Request(Mode.POSITION, .1, .01, .75, .03, 10, 55.01).valid());
    assertFalse(new Request(Mode.POSITION, .1, Double.NaN, .75, .03, 10, 45).valid());
  }

  @Test
  void freshnessConnectionConfigurationAndCurrentAreRequired() {
    double angle = Math.toRadians(40);
    assertEquals(
        "", IntakeDiagnosticPolicy.healthFault(5, sample(4.99, angle, 1, true, true), PULSE));
    assertFalse(
        IntakeDiagnosticPolicy.healthFault(5, sample(4.8, angle, 1, true, true), PULSE).isEmpty());
    assertFalse(
        IntakeDiagnosticPolicy.healthFault(5, sample(5.01, angle, 1, true, true), PULSE).isEmpty());
    assertFalse(
        IntakeDiagnosticPolicy.healthFault(5, sample(4.99, angle, 1, false, true), PULSE)
            .isEmpty());
    assertFalse(
        IntakeDiagnosticPolicy.healthFault(5, sample(4.99, angle, 1, true, false), PULSE)
            .isEmpty());
    assertFalse(
        IntakeDiagnosticPolicy.healthFault(5, sample(4.99, angle, 10, true, true), PULSE)
            .isEmpty());
    assertFalse(
        IntakeDiagnosticPolicy.healthFault(5, sample(4.99, Double.NaN, 1, true, true), PULSE)
            .isEmpty());
    assertFalse(
        IntakeDiagnosticPolicy.healthFault(
                5, new Sample(4.99, angle, 0, .1, 1, 25, true, true, .05), PULSE)
            .isEmpty());
    assertFalse(
        IntakeDiagnosticPolicy.healthFault(
                5, new Sample(4.99, angle, 0, .1, 1, 45, true, true, .03), PULSE)
            .isEmpty());
    assertFalse(
        IntakeDiagnosticPolicy.healthFault(
                5, new Sample(4.99, angle, 0, .4, 1, 25, true, true, .03), PULSE)
            .isEmpty());
  }

  @Test
  void pulseStopsAtTravelLimitAndOnOppositeDirection() {
    double start = Math.toRadians(40);
    assertEquals(
        "",
        IntakeDiagnosticPolicy.motionFault(RANGE, PULSE, start, start, start + Math.toRadians(.5)));
    assertFalse(
        IntakeDiagnosticPolicy.motionFault(RANGE, PULSE, start, start, start + Math.toRadians(2))
            .isEmpty());
    assertFalse(
        IntakeDiagnosticPolicy.motionFault(RANGE, PULSE, start, start, start - Math.toRadians(.21))
            .isEmpty());
    assertFalse(
        IntakeDiagnosticPolicy.motionFault(
                RANGE,
                new Request(Mode.FEEDBACK, -.1, .01, .1, .03, 10, 45),
                Math.toRadians(4),
                Math.toRadians(4),
                Math.toRadians(4))
            .isEmpty());
  }

  @Test
  void verifiedHardstopAllowsOnlyBoundedInwardFeedback() {
    var inward = new Request(Mode.FEEDBACK, -.1, .01, .1, .03, 10, 45);
    double upper = RANGE.maxRad();
    assertEquals("", IntakeDiagnosticPolicy.motionFault(RANGE, inward, upper, upper, upper));
    assertEquals(
        "",
        IntakeDiagnosticPolicy.motionFault(
            RANGE, inward, upper + 1e-7, upper + 1e-7, upper + 1e-7));
    assertEquals(
        "",
        IntakeDiagnosticPolicy.motionFault(
            RANGE, inward, upper, upper, upper - Math.toRadians(.5)));
    assertFalse(IntakeDiagnosticPolicy.motionFault(RANGE, PULSE, upper, upper, upper).isEmpty());
    assertFalse(
        IntakeDiagnosticPolicy.motionFault(RANGE, inward, upper, upper, upper + Math.toRadians(.1))
            .isEmpty());
    assertFalse(
        IntakeDiagnosticPolicy.motionFault(RANGE, inward, upper, upper, upper - Math.toRadians(2.1))
            .isEmpty());
    assertEquals("", IntakeDiagnosticPolicy.motionFault(RANGE, PULSE, 0, 0, Math.toRadians(.5)));
    assertFalse(IntakeDiagnosticPolicy.motionFault(RANGE, inward, 0, 0, 0).isEmpty());
    var pocket = new Reference(upper, upper - Math.toRadians(3), upper);
    assertTrue(pocket.valid());
    assertEquals(
        "",
        IntakeDiagnosticPolicy.motionFault(
            pocket, inward, upper, upper, upper - Math.toRadians(.5)));
  }

  @Test
  void stepsCannotTargetOrOvershootOutsideVerifiedInterior() {
    double start = Math.toRadians(40);
    double target = start + STEP.stepRad();
    assertEquals("", IntakeDiagnosticPolicy.motionFault(RANGE, STEP, start, target, target));
    assertFalse(
        IntakeDiagnosticPolicy.motionFault(RANGE, STEP, start, target, start + Math.toRadians(2.1))
            .isEmpty());
    assertFalse(
        IntakeDiagnosticPolicy.motionFault(RANGE, STEP, start, target, start - Math.toRadians(.51))
            .isEmpty());
    assertFalse(
        IntakeDiagnosticPolicy.motionFault(
                RANGE, STEP, Math.toRadians(90), Math.toRadians(91), Math.toRadians(90))
            .isEmpty());
  }

  @Test
  void watchdogStopsWithoutWaitingForMainLoop() {
    assertEquals("", IntakeDiagnosticPolicy.watchdogFault(5, 4.99, 5.1, true, true, true, true));
    assertEquals(
        "Main-loop heartbeat stale",
        IntakeDiagnosticPolicy.watchdogFault(5, 4.9, 5.1, true, true, true, true));
    assertEquals(
        "Deadline reached",
        IntakeDiagnosticPolicy.watchdogFault(5.1, 5.09, 5.1, true, true, true, true));
    assertFalse(
        IntakeDiagnosticPolicy.watchdogFault(5, 4.99, 5.1, false, true, true, true).isEmpty());
    assertFalse(
        IntakeDiagnosticPolicy.watchdogFault(5, 4.99, 5.1, true, false, true, true).isEmpty());
    assertFalse(
        IntakeDiagnosticPolicy.watchdogFault(5, 4.99, 5.1, true, true, false, true).isEmpty());
    assertFalse(
        IntakeDiagnosticPolicy.watchdogFault(5, 4.99, 5.1, true, true, true, false).isEmpty());
    assertFalse(
        IntakeDiagnosticPolicy.watchdogFault(4.8, 4.99, 5.1, true, true, true, true).isEmpty());
  }

  @Test
  void oldCalibrationOrRemoteRequestCannotStartAfterEnable() {
    var gate = new RemoteDiagnosticPolicy.RequestGate();
    assertFalse(gate.consume(1, false));
    assertFalse(gate.consume(1, true));
    assertTrue(gate.consume(2, true));
    assertFalse(gate.consume(2, true));
    assertFalse(gate.consume(1, true));
    assertFalse(RemoteDiagnosticPolicy.heartbeatFresh(5, 4.7));
    assertFalse(RemoteDiagnosticPolicy.heartbeatFresh(5, Double.NaN));
  }

  @Test
  void deadmanHeldBeforePreparationCannotStartAndCancellationRequiresRelease() {
    var gate = new DriveDiagnosticPolicy.DeadmanGate();
    assertFalse(gate.update(false, true, true));
    assertFalse(gate.update(true, true, true));
    assertFalse(gate.update(true, false, true));
    assertTrue(gate.update(true, true, true));
    gate.cancel();
    assertFalse(gate.update(true, true, true));
    assertFalse(gate.update(true, false, true));
    assertTrue(gate.update(true, true, true));
  }

  @Test
  void timingMustRequalifyFollowingModeOrClockDiscontinuity() {
    var window = new DriveDiagnosticPolicy.TimingWindow();
    for (int i = 1; i <= 100; i++) window.add(20, i * .02);
    assertTrue(window.ready(2));
    window.clear();
    assertFalse(window.ready(2));
    window.add(20, 3);
    window.add(20, 2.9);
    assertEquals(0, window.sampleCount());
  }
}
