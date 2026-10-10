package frc.robot.lib.subsystem.angular;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import frc.robot.constants.intake.IntakeConstants;
import frc.robot.constants.shooter.ShooterConstants;
import java.util.List;
import org.junit.jupiter.api.Test;

class HardwareMergeConfigurationTest {
  @Test
  void intakeControlCurrentLimitsPreserveCalibratedHardwareAndGains() {
    var pivot = IntakeConstants.kPivotSparkFlexConfig;
    assertEquals(21, pivot.getMasterId());
    assertEquals(60.0, pivot.getMotorRotationsPerOutputRotations(), 1e-12);
    assertEquals(118.0, pivot.getResetAngle().in(Degrees), 1e-12);
    assertEquals(118.0, pivot.getSoftMaxAngle().in(Degrees), 1e-12);
    assertEquals(0.0, pivot.getSoftMinAngle().in(Degrees), 1e-12);
    assertEquals(40.0, pivot.getSmartCurrentLimit().in(Amps), 1e-12);
    assertEquals(80.0, IntakeConstants.kPivotDeployCurrentLimit.in(Amps), 1e-12);
    assertEquals(120.0, pivot.getSecondaryCurrentLimit().in(Amps), 1e-12);
    assertEquals(2.0, pivot.getKP(), 1e-12);
    assertEquals(0.5, pivot.getKV(), 1e-12);
    assertEquals(0.35, pivot.getKG(), 1e-12);
  }

  @Test
  void mergedAgitationStaysWithinCalibratedPivotTravel() {
    assertEquals(0.0, IntakeConstants.kPivotAgitateLow.in(Degrees), 1e-12);
    assertEquals(30.0, IntakeConstants.kPivotAgitateHigh.in(Degrees), 1e-12);
    assertEquals(90.0, IntakeConstants.kPivotRaised.in(Degrees), 1e-12);
    assertTrue(IntakeConstants.kPivotRaised.lt(IntakeConstants.kPivotStowed));
    assertEquals(3, IntakeConstants.kAgitateCycles);
    assertEquals(0.35, IntakeConstants.kAgitateDwell.in(Seconds), 1e-12);
    assertEquals(2.0, IntakeConstants.kPivotDeployTimeout.in(Seconds), 1e-12);
    assertEquals(5.0, IntakeConstants.kPivotArrivalTolerance.in(Degrees), 1e-12);
  }

  @Test
  void shooterRetainsMirroredFollowersAndBringupSettings() {
    var flywheel = ShooterConstants.kFlywheelSparkFlexConfig;
    assertEquals(39, flywheel.getMasterId());
    assertEquals(List.of(34, 26, 29), flywheel.getFollowerIds());
    assertFalse(flywheel.isFollowerOpposed(34));
    assertTrue(flywheel.isFollowerOpposed(26));
    assertTrue(flywheel.isFollowerOpposed(29));
    assertEquals(40.0, flywheel.getSmartCurrentLimit().in(Amps), 1e-12);
    assertEquals(60.0, flywheel.getSecondaryCurrentLimit().in(Amps), 1e-12);
    assertEquals(10, flywheel.getEncoderMeasurementPeriodMs());
    assertEquals(2, flywheel.getEncoderAverageDepth());
    assertEquals(0.01 / 12.0, flywheel.getKP(), 1e-12);
    assertEquals(0.11 / (2.0 * Math.PI), flywheel.getKV(), 1e-12);
  }
}
