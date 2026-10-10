package frc.robot.lib.subsystem.angular;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import com.revrobotics.spark.SparkBase;
import org.junit.jupiter.api.Test;

class AngularSparkVelocityModeTest {
  @Test
  void anUnconfiguredRollerUsesPlainVelocityControl() {
    var config = AngularIOSparkFlexConfig.builder().masterId(2).build();
    assertEquals(SparkBase.ControlType.kVelocity, AngularIOSparkFlex.velocityControlType(config));
  }

  @Test
  void positiveAccelerationEnablesMaxMotionWithoutRequiringCruiseVelocity() {
    var config =
        AngularIOSparkFlexConfig.builder()
            .masterId(2)
            .acceleration(RotationsPerSecondPerSecond.of(5))
            .build();
    assertEquals(0, config.getCruiseVelocity().in(RotationsPerSecond));
    assertEquals(
        SparkBase.ControlType.kMAXMotionVelocityControl,
        AngularIOSparkFlex.velocityControlType(config));
  }

  @Test
  void zeroAccelerationUsesPlainVelocityDespitePositiveCruiseVelocity() {
    var config =
        AngularIOSparkFlexConfig.builder()
            .masterId(2)
            .cruiseVelocity(RotationsPerSecond.of(50))
            .build();
    assertEquals(SparkBase.ControlType.kVelocity, AngularIOSparkFlex.velocityControlType(config));
  }

  @Test
  void negativeAndNonfiniteAccelerationAreRejected() {
    for (double acceleration :
        new double[] {-1, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
      var config =
          AngularIOSparkFlexConfig.builder()
              .masterId(2)
              .cruiseVelocity(RotationsPerSecond.of(50))
              .acceleration(RadiansPerSecondPerSecond.of(acceleration))
              .build();
      assertThrows(
          IllegalArgumentException.class,
          () -> AngularIOSparkFlex.velocityControlType(config),
          "acceleration=" + acceleration);
    }
  }

  @Test
  void plainVelocityReferenceIsTheGoalAndNeverReadsMaxMotionTelemetry() {
    var goal = RotationsPerSecond.of(-5);
    var reference =
        AngularIOSparkFlex.velocityReference(
            SparkBase.ControlType.kVelocity,
            goal,
            () -> {
              throw new AssertionError("Plain velocity must not read a stale MAXMotion reference");
            },
            2 * Math.PI);
    assertEquals(-5, reference.in(RotationsPerSecond), 1e-12);
  }

  @Test
  void profileChangesDoNotChangeReferenceUntilAnotherVelocityCommandSelectsTheMode() {
    var config = AngularIOSparkFlexConfig.builder().masterId(2).build();
    var commandedType = AngularIOSparkFlex.velocityControlType(config);
    config.setAcceleration(RotationsPerSecondPerSecond.of(5));
    assertEquals(
        SparkBase.ControlType.kMAXMotionVelocityControl,
        AngularIOSparkFlex.velocityControlType(config));
    var reference =
        AngularIOSparkFlex.velocityReference(
            commandedType,
            RotationsPerSecond.of(5),
            () -> {
              throw new AssertionError(
                  "Changing config must not relabel the already-issued plain velocity command");
            },
            2 * Math.PI);
    assertEquals(5, reference.in(RotationsPerSecond), 1e-12);
    commandedType = AngularIOSparkFlex.velocityControlType(config);
    config.setAcceleration(RotationsPerSecondPerSecond.of(0));
    reference =
        AngularIOSparkFlex.velocityReference(
            commandedType, RotationsPerSecond.of(5), () -> 1.5, 2 * Math.PI);
    assertEquals(1.5, reference.in(RotationsPerSecond), 1e-12);
  }
}
