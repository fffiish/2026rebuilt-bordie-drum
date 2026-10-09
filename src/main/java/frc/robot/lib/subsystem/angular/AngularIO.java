package frc.robot.lib.subsystem.angular;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Amps;
import static frc.robot.lib.subsystem.angular.AngularIOOutputMode.kNeutral;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.lib.subsystem.DeviceConnectedStatus;
import org.littletonrobotics.junction.AutoLog;

public interface AngularIO {
  default void updateInputs(AngularIOInputs inputs) {}

  @AutoLog
  class AngularIOInputs {
    // Separate from AngularSubsystemOutputMode.
    public AngularIOOutputMode IOOutputMode = kNeutral;

    public Angle angle = Radians.of(0.0);

    public Voltage appliedVolts = Volts.of(0.0);
    public Voltage requestedVolts = Volts.of(0.0);
    public Voltage busVolts = Volts.of(0.0);
    public Current supplyCurrent = Amps.of(0.0);
    public Current statorCurrent = Amps.of(0.0);
    public AngularVelocity velocity = RadiansPerSecond.of(0.0);
    public AngularAcceleration acceleration = RadiansPerSecondPerSecond.of(0.0);

    public NeutralModeValue neutralMode = NeutralModeValue.Brake;

    public double[] motorTemperatures = {};
    public DeviceConnectedStatus[] deviceConnectedStatuses = {};

    public Angle goalPos = Radians.of(0.0);
    public AngularVelocity goalVel = RadiansPerSecond.of(0.0);
    public Angle referencePos = Radians.of(0.0);
    public AngularVelocity referenceVel = RadiansPerSecond.of(0.0);

    /** FPGA time of the last complete, healthy telemetry sample, not just the last poll. */
    public double sampleTimestampSeconds = Double.NaN;

    public boolean configReady = false;
    public boolean configPending = false;
    public String configError = "Configuration not verified";
    public boolean referenceValid = false;
    public long configurationGeneration = 0;
    public double readbackKP = Double.NaN;
    public double readbackKI = Double.NaN;
    public double readbackKD = Double.NaN;
    public double readbackKS = Double.NaN;
    public double readbackKV = Double.NaN;
    public double readbackKG = Double.NaN;
    public double readbackCruiseVelocityRadiansPerSecond = Double.NaN;
    public double readbackAccelerationRadiansPerSecondPerSecond = Double.NaN;
    public double readbackOutputLimit = Double.NaN;
    public double readbackMinimumAngleRadians = Double.NaN;
    public double readbackMaximumAngleRadians = Double.NaN;
  }

  default void setAngle(Angle angle) {}

  default void setAngle(Angle angle, Voltage feedforward) {}

  default void setOpenLoop(Voltage voltage) {}

  default void setVelocity(AngularVelocity velocity) {}

  default void stop() {}

  default void resetAngle() {}

  default void resetAngle(Angle angle) {}

  default void setPIDVG(double kP, double kI, double kD, double kV, double kG) {}

  /** Complete gain update; old IO implementations retain their existing PIDVG behavior. */
  default void setGains(double kP, double kI, double kD, double kS, double kV, double kG) {
    setPIDVG(kP, kI, kD, kV, kG);
  }

  /** Apply and verify a diagnostic duty-cycle cap. Caller and IO both require disabled state. */
  default boolean diagnosticSetOutputLimit(double duty) {
    return false;
  }

  /** Assign a physically confirmed reference and verify the encoder readback while disabled. */
  default boolean diagnosticCalibrateReference(Angle angle) {
    return false;
  }

  default void setConstraints(AngularVelocity cruiseVelocity, AngularAcceleration acceleration) {}

  default void setNeutralMode(NeutralModeValue neutralMode) {}

  /** Change the runtime current limit (SPARK motor phase current). No-op where unsupported. */
  default void setCurrentLimit(Current limit) {}

  default void setLogKey(String logKey) {}
}
