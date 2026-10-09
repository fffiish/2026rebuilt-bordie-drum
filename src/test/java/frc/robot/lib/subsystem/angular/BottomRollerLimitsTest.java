package frc.robot.lib.subsystem.angular;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import com.revrobotics.spark.SparkBase;
import frc.robot.constants.indexer.IndexerConstants;
import frc.robot.constants.intake.IntakeConstants;
import frc.robot.constants.shooter.ShooterConstants;
import java.util.HashSet;
import java.util.List;
import org.junit.jupiter.api.Test;

class BottomRollerLimitsTest {
  @Test
  void followerConversionPreservesMeasuredEncoderPolarity() {
    var config = ShooterConstants.kFlywheelSparkFlexConfig;
    assertEquals(
        30,
        AngularIOSparkFlex.followerVelocityRadiansPerSecond(config, 34, 1800) / (2 * Math.PI),
        1e-9);
    assertEquals(
        -30,
        AngularIOSparkFlex.followerVelocityRadiansPerSecond(config, 26, -1800) / (2 * Math.PI),
        1e-9);
    assertEquals(
        -30,
        AngularIOSparkFlex.followerVelocityRadiansPerSecond(config, 29, -1800) / (2 * Math.PI),
        1e-9);
    assertEquals(
        30,
        AngularIOSparkFlex.followerVelocityRadiansPerSecond(config, 26, 1800) / (2 * Math.PI),
        1e-9);
    assertEquals(
        30,
        AngularIOSparkFlex.followerVelocityRadiansPerSecond(config, 29, 1800) / (2 * Math.PI),
        1e-9);
    assertEquals(
        0.5,
        ShooterConstants.kFlywheelSubsystemConfigReal.getVelocityTolerance().in(RotationsPerSecond),
        1e-9);
  }

  @Test
  void mainDrumAndItsThreeFollowersAreCappedAtThirtyRps() {
    var config = ShooterConstants.kFlywheelSparkFlexConfig;
    assertEquals(39, config.getMasterId());
    assertEquals(List.of(34, 26, 29), config.getFollowerIds());
    assertEquals(30, config.getMaximumMotorVelocity().in(RotationsPerSecond), 1e-9);
    assertEquals(30, ShooterConstants.kFlywheelShooting.in(RotationsPerSecond), 1e-9);
    assertEquals(30, ShooterConstants.kFlywheelShootingFar.in(RotationsPerSecond), 1e-9);
    assertEquals(20, ShooterConstants.kFlywheelEjecting.in(RotationsPerSecond), 1e-9);
    assertEquals(
        30,
        AngularIOSparkFlex.limitMotorVelocity(config, RotationsPerSecond.of(95))
            .in(RotationsPerSecond),
        1e-9);
    assertEquals(
        -30,
        AngularIOSparkFlex.limitMotorVelocity(config, RotationsPerSecond.of(-95))
            .in(RotationsPerSecond),
        1e-9);
    for (var subsystem :
        List.of(
            ShooterConstants.kFlywheelSubsystemConfigReal,
            ShooterConstants.kFlywheelSubsystemConfigSim)) {
      assertEquals(
          30, subsystem.limitVelocity(RotationsPerSecond.of(95)).in(RotationsPerSecond), 1e-9);
    }
  }

  @Test
  void mechanismMotorIdsHaveExactlyOneControllerOwner() {
    var ids = new HashSet<Integer>();
    for (var config :
        List.of(
            IntakeConstants.kPivotSparkFlexConfig,
            IntakeConstants.kIntakeRollerSparkFlexConfig,
            IntakeConstants.kFeederSparkFlexConfig,
            IndexerConstants.kSparkFlexConfig,
            ShooterConstants.kFlywheelSparkFlexConfig)) {
      assertTrue(ids.add(config.getMasterId()), "Duplicate leader CAN ID");
      for (int id : config.getFollowerIds()) assertTrue(ids.add(id), "Duplicate follower CAN ID");
    }
    assertTrue(ids.containsAll(List.of(22, 36, 28, 35)));
  }

  @Test
  void bottomRollersUseSixtyAmpsAndThirtyTwoRps() {
    var config = IndexerConstants.kSparkFlexConfig;
    assertEquals(36, config.getMasterId());
    assertEquals(List.of(22), config.getFollowerIds());
    assertEquals(60, config.getSmartCurrentLimit().in(Amps));
    assertEquals(60, config.getSecondaryCurrentLimit().in(Amps));
    assertEquals(1920, config.getMaximumMotorVelocity().in(RPM), 1e-9);
    assertEquals(SparkBase.ControlType.kVelocity, AngularIOSparkFlex.velocityControlType(config));
    assertTrue(config.getKP() > 0);
    assertTrue(config.isFollowerOpposed(22));
    assertEquals(12, config.getKV() * RPM.of(6784).in(RadiansPerSecond), 1e-9);
    assertEquals(32, IndexerConstants.kFeeding.in(RotationsPerSecond), 1e-9);
    assertEquals(32, IndexerConstants.kIntaking.in(RotationsPerSecond), 1e-9);
    assertEquals(
        32,
        AngularIOSparkFlex.limitMotorVelocity(config, IndexerConstants.kFeeding)
            .in(RotationsPerSecond),
        1e-9);
    assertEquals(-1356.8, IndexerConstants.kUnjamming.in(RPM), 1e-9);
    var feeder = IntakeConstants.kFeederSparkFlexConfig;
    assertEquals(28, feeder.getMasterId());
    assertEquals(List.of(35), feeder.getFollowerIds());
    assertEquals(60, feeder.getSmartCurrentLimit().in(Amps));
    assertEquals(60, feeder.getSecondaryCurrentLimit().in(Amps));
    assertTrue(feeder.getKP() > 0);
    assertTrue(feeder.isFollowerOpposed(35));
    assertEquals(15, IntakeConstants.kFeederIntaking.in(RotationsPerSecond), 1e-9);
  }

  @Test
  void hardwareClampsBothDirectionsAndConvertsMotorSpeedThroughGearing() {
    var config =
        AngularIOSparkFlexConfig.builder()
            .masterId(36)
            .maximumMotorVelocity(IndexerConstants.kMaximumSpeed)
            .motorRotationsPerOutputRotations(2)
            .build();
    assertEquals(960, AngularIOSparkFlex.limitMotorVelocity(config, RPM.of(6000)).in(RPM), 1e-9);
    assertEquals(-960, AngularIOSparkFlex.limitMotorVelocity(config, RPM.of(-6000)).in(RPM), 1e-9);
    assertEquals(300, AngularIOSparkFlex.limitMotorVelocity(config, RPM.of(300)).in(RPM), 1e-9);
    assertEquals(0, AngularIOSparkFlex.limitMotorVelocity(config, RPM.of(Double.NaN)).in(RPM));
    var unlimited = AngularIOSparkFlexConfig.builder().masterId(39).build();
    assertEquals(
        5000, AngularIOSparkFlex.limitMotorVelocity(unlimited, RPM.of(5000)).in(RPM), 1e-9);
  }

  @Test
  void subsystemCapAppliesToRealAndSim() {
    for (var config :
        List.of(IndexerConstants.kSubsystemConfigReal, IndexerConstants.kSubsystemConfigSim)) {
      assertEquals(1920, config.limitVelocity(RPM.of(6000)).in(RPM), 1e-9);
      assertEquals(-1920, config.limitVelocity(RPM.of(-6000)).in(RPM), 1e-9);
      assertEquals(0, config.limitVelocity(RPM.of(0)).in(RPM));
    }
  }
}
