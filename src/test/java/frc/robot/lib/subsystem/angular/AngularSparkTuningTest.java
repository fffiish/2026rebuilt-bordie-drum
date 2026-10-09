package frc.robot.lib.subsystem.angular;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.spark.config.SparkFlexConfig;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class AngularSparkTuningTest {
  @Test
  void convertedFirmwareGainsIncludeStaticFrictionAndGravityInsideOutputRange() {
    var config =
        AngularIOSparkFlexConfig.builder()
            .masterId(21)
            .motorRotationsPerOutputRotations(60)
            .outputAnglePerOutputRotation(Radians.of(Math.PI))
            .kP(.03)
            .kI(.002)
            .kD(.7)
            .kS(.25)
            .kV(.1)
            .kG(.4)
            .gravityType(Optional.of(GravityTypeValue.Arm_Cosine))
            .cruiseVelocity(RadiansPerSecond.of(3 * Math.PI))
            .acceleration(RadiansPerSecondPerSecond.of(4 * Math.PI))
            .build();
    SparkFlexConfig expected = new SparkFlexConfig();
    // P/I/D deliberately retain duty-based output and firmware millisecond time units.
    expected.closedLoop.pid(.03 * Math.PI, .002 * Math.PI, .7 * Math.PI);
    expected.closedLoop.feedForward.kV(.1 * Math.PI).kS(.25).kCos(.4).kCosRatio(.5).kG(0);
    expected.closedLoop.outputRange(-.12, .12);
    expected.closedLoop.maxMotion.cruiseVelocity(3).maxAcceleration(4);
    assertEquals(expected.flatten(), AngularIOSparkFlex.buildTuningConfig(config, .12).flatten());
  }

  @Test
  void staticGravityAndNoGravityDoNotEnableAnArmCosine() {
    for (var gravity :
        new Optional[] {Optional.empty(), Optional.of(GravityTypeValue.Elevator_Static)}) {
      @SuppressWarnings("unchecked")
      Optional<GravityTypeValue> type = gravity;
      var config = AngularIOSparkFlexConfig.builder().masterId(21).kG(.4).gravityType(type).build();
      SparkFlexConfig expected = new SparkFlexConfig();
      expected.closedLoop.pid(0, 0, 0);
      expected
          .closedLoop
          .feedForward
          .kV(0)
          .kS(0)
          .kCos(0)
          .kCosRatio(1)
          .kG(type.isPresent() ? .4 : 0);
      expected.closedLoop.outputRange(-1, 1);
      expected.closedLoop.maxMotion.cruiseVelocity(0).maxAcceleration(0);
      assertEquals(expected.flatten(), AngularIOSparkFlex.buildTuningConfig(config, 1).flatten());
    }
  }

  @Test
  void completeGainApiPreservesLegacyIoCompatibility() {
    class LegacyIO implements AngularIO {
      double[] gains;

      @Override
      public void setPIDVG(double p, double i, double d, double v, double g) {
        gains = new double[] {p, i, d, v, g};
      }
    }
    var io = new LegacyIO();
    io.setGains(1, 2, 3, 4, 5, 6);
    assertArrayEquals(new double[] {1, 2, 3, 5, 6}, io.gains);
  }
}
