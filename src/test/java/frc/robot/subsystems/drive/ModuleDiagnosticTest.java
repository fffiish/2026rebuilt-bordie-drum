package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class ModuleDiagnosticTest {
  private static final double WHEEL_RADIUS = 0.05;

  private static class RecordingIO implements ModuleIO {
    double driveVolts;
    double turnVolts;
    double driveVelocity;
    Rotation2d target = Rotation2d.kZero;
    Rotation2d measuredAngle = Rotation2d.fromDegrees(170);
    boolean limits;

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
      inputs.turnPosition = measuredAngle;
    }

    @Override
    public void setDriveOpenLoop(double volts) {
      driveVolts = volts;
    }

    @Override
    public void setTurnOpenLoop(double volts) {
      turnVolts = volts;
    }

    @Override
    public void setDriveVelocity(double radiansPerSec) {
      driveVelocity = radiansPerSec;
    }

    @Override
    public void setTurnPosition(Rotation2d angle) {
      target = angle;
    }

    @Override
    public void setDiagnosticLimits(boolean active) {
      limits = active;
    }
  }

  private Module module(RecordingIO io) {
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        constants = new SwerveModuleConstants<>();
    constants.WheelRadius = WHEEL_RADIUS;
    Module module = new Module(io, 0, constants);
    module.updateInputs();
    return module;
  }

  @Test
  void equalDiagnosticVelocityBypassesOptimizationAndCosineScaling() {
    RecordingIO io = new RecordingIO();
    Module module = module(io);
    Rotation2d requested = Rotation2d.fromDegrees(-5);
    module.runDiagnosticPosition(requested, 0.5);
    // A 175 degree difference would reverse/scale speed through the normal setpoint path.
    assertEquals(10.0, io.driveVelocity, 1e-12);
    assertEquals(-5.0, io.target.getDegrees(), 1e-12);
  }

  @Test
  void steeringOnlyAndStopExplicitlyZeroDriveAndBothOpenLoopOutputs() {
    RecordingIO io = new RecordingIO();
    Module module = module(io);
    module.runDiagnostic(1.5, 0.25);
    module.runDiagnosticPosition(Rotation2d.fromDegrees(179), 0);
    assertEquals(0, io.driveVolts);
    assertEquals(179, io.target.getDegrees(), 1e-12);
    module.stop();
    assertEquals(0, io.driveVolts);
    assertEquals(0, io.turnVolts);
  }

  @Test
  void voltageRequestsAndLimitConfigurationForwardWithoutPolarityChanges() {
    RecordingIO io = new RecordingIO();
    Module module = module(io);
    module.runDiagnostic(0.5, -0.25);
    assertEquals(0.5, io.driveVolts);
    assertEquals(-0.25, io.turnVolts);
    module.setDiagnosticLimits(true);
    assertEquals(true, io.limits);
    module.setDiagnosticLimits(false);
    assertEquals(false, io.limits);
  }
}
