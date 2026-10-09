package frc.robot.lib.subsystem.angular;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.constants.intake.IntakeConstants;
import frc.robot.lib.sim.CurrentDrawCalculatorSim;
import frc.robot.lib.sim.PivotSim;
import java.util.Optional;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class AngularIOSimTest {
  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @BeforeEach
  void resetBattery() {
    RoboRioSim.setVInVoltage(12.0);
  }

  private AngularIOSimConfig.AngularIOSimConfigBuilder roller() {
    return AngularIOSimConfig.builder()
        .motor(DCMotor.getNeoVortex(1))
        .moi(KilogramSquareMeters.of(0.004))
        .cruiseVelocity(RotationsPerSecond.of(5.0))
        .acceleration(RotationsPerSecondPerSecond.of(5.0));
  }

  private AngularIOSim sim(AngularIOSimConfig config) {
    return new AngularIOSim(config, new CurrentDrawCalculatorSim());
  }

  @Test
  void currentLimitedAppliedVoltageDiffersFromRequestedVoltage() {
    DCMotor motor = DCMotor.getNeoVortex(1);
    AngularIOSim io = sim(roller().statorCurrentLimit(Amps.of(1.0)).build());
    AngularIO.AngularIOInputs inputs = new AngularIO.AngularIOInputs();
    io.setOpenLoop(Volts.of(12.0));
    io.updateInputs(inputs);
    assertEquals(12.0, inputs.requestedVolts.in(Volts), 1e-9);
    assertEquals(motor.rOhms, inputs.appliedVolts.in(Volts), 1e-9);
    assertEquals(1.0, inputs.statorCurrent.in(Amps), 1e-9);
    assertTrue(inputs.configReady);
  }

  @Test
  void reverseOutputReportsPositiveBatteryDrawAndRespectsBatteryAndDutyLimits() {
    AngularIOSim io = sim(roller().build());
    AngularIO.AngularIOInputs inputs = new AngularIO.AngularIOInputs();
    RoboRioSim.setVInVoltage(9.0);
    assertTrue(io.diagnosticSetOutputLimit(0.1));
    io.setOpenLoop(Volts.of(-12.0));
    io.updateInputs(inputs);
    assertEquals(-12.0, inputs.requestedVolts.in(Volts), 1e-9);
    assertEquals(-0.9, inputs.appliedVolts.in(Volts), 1e-9);
    assertTrue(inputs.supplyCurrent.in(Amps) > 0.0);
    assertEquals(0.1, inputs.readbackOutputLimit, 1e-9);
    assertFalse(io.diagnosticSetOutputLimit(Double.NaN));
    assertFalse(io.diagnosticSetOutputLimit(1.1));
  }

  @Test
  void zeroBatteryDoesNotProduceNanCurrentOrVoltage() {
    AngularIOSim io = sim(roller().build());
    AngularIO.AngularIOInputs inputs = new AngularIO.AngularIOInputs();
    RoboRioSim.setVInVoltage(0.0);
    io.setOpenLoop(Volts.of(12.0));
    io.updateInputs(inputs);
    assertEquals(0.0, inputs.appliedVolts.in(Volts), 1e-9);
    assertEquals(0.0, inputs.supplyCurrent.in(Amps), 1e-9);
    assertTrue(Double.isFinite(inputs.angle.in(Radians)));
  }

  @Test
  void stopReportsZeroAppliedOutputAfterRunningWithCurrentLimits() {
    AngularIOSim io = sim(roller().statorCurrentLimit(Amps.of(1.0)).build());
    AngularIO.AngularIOInputs inputs = new AngularIO.AngularIOInputs();
    io.setOpenLoop(Volts.of(12.0));
    for (int i = 0; i < 100; i++) io.updateInputs(inputs);
    assertTrue(inputs.velocity.in(RadiansPerSecond) > 0.0);
    io.stop();
    io.updateInputs(inputs);
    assertEquals(0.0, inputs.requestedVolts.in(Volts), 1e-9);
    assertEquals(0.0, inputs.appliedVolts.in(Volts), 1e-9);
  }

  @Test
  void rollerProfileAcceleratesAndLiveStaticFeedforwardChangesDirection() {
    AngularIOSim io = sim(roller().kV(0.02).build());
    AngularIO.AngularIOInputs inputs = new AngularIO.AngularIOInputs();
    io.setGains(0.0, 0.0, 0.0, 0.5, 0.02, 0.0);
    io.setVelocity(RotationsPerSecond.of(5.0));
    io.updateInputs(inputs);
    assertTrue(inputs.referenceVel.in(RotationsPerSecond) > 0.0);
    assertTrue(inputs.referenceVel.in(RotationsPerSecond) <= 0.101);
    assertTrue(inputs.requestedVolts.in(Volts) > 0.5);
    for (int i = 0; i < 20; i++) io.updateInputs(inputs);
    assertTrue(inputs.velocity.in(RotationsPerSecond) > 0.0);
    io.stop();
    assertTrue(io.diagnosticCalibrateReference(Radians.of(0.0)));
    io.setVelocity(RotationsPerSecond.of(-5.0));
    io.updateInputs(inputs);
    assertTrue(inputs.requestedVolts.in(Volts) < -0.5);
    assertEquals(0.5, inputs.readbackKS, 1e-9);
  }

  @Test
  void gravityEnabledArmFallsAndReplacingAngleSupplierChangesGravityDirection() {
    AngularIOSim io =
        sim(
            roller()
                .moi(PivotSim.estimateMOI(Inches.of(14.0), Pounds.of(8.0)))
                .armLengthSupplier(Optional.of(() -> Inches.of(14.0)))
                .realAngleFromSubsystemAngleZeroSupplier(Optional.of(() -> Rotation2d.kZero))
                .build());
    AngularIO.AngularIOInputs inputs = new AngularIO.AngularIOInputs();
    io.updateInputs(inputs);
    assertTrue(inputs.velocity.in(RadiansPerSecond) < 0.0);
    assertTrue(io.diagnosticCalibrateReference(Radians.of(0.0)));
    io.setRealAngleFromSubsystemAngleZeroSupplier(() -> Rotation2d.kPi);
    io.updateInputs(inputs);
    assertTrue(inputs.velocity.in(RadiansPerSecond) > 0.0);
  }

  @Test
  void gravityRejectsMissingLengthAndIntakeSimHasExplicitGeometryAndRollerProfiles() {
    assertThrows(
        IllegalArgumentException.class,
        () ->
            sim(
                roller()
                    .realAngleFromSubsystemAngleZeroSupplier(Optional.of(() -> Rotation2d.kZero))
                    .build()));
    assertTrue(IntakeConstants.kPivotSimConfig.getArmLengthSupplier().isPresent());
    assertTrue(
        IntakeConstants.kPivotSimConfig.getRealAngleFromSubsystemAngleZeroSupplier().isPresent());
    assertTrue(
        IntakeConstants.kFeederSimConfig.getAcceleration().in(RadiansPerSecondPerSecond) > 0.0);
    assertTrue(
        IntakeConstants.kIntakeRollerSimConfig.getAcceleration().in(RadiansPerSecondPerSecond)
            > 0.0);
  }
}
