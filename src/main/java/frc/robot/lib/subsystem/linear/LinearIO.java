package frc.robot.lib.subsystem.linear;

import static edu.wpi.first.units.Units.*;
import static frc.robot.lib.subsystem.linear.LinearIOOutputMode.kNeutral;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.lib.subsystem.DeviceConnectedStatus;
import org.littletonrobotics.junction.AutoLog;

public interface LinearIO {
  default void updateInputs(LinearIOInputs inputs) {}

  @AutoLog
  class LinearIOInputs {
    // Separate from LinearSubsystemOutputMode.
    public LinearIOOutputMode IOOutputMode = kNeutral;

    public Distance length = Meters.of(0.0);

    public Voltage appliedVolts = Volts.of(0.0);
    public Current supplyCurrent = Amps.of(0.0);
    public Current statorCurrent = Amps.of(0.0);
    public LinearVelocity velocity = MetersPerSecond.of(0.0);
    public LinearAcceleration acceleration = MetersPerSecondPerSecond.of(0.0);

    public NeutralModeValue neutralMode = NeutralModeValue.Brake;

    public double[] motorTemperatures = {};
    public DeviceConnectedStatus[] deviceConnectedStatuses = {};

    public Distance goal = Meters.of(0.0);
    public Distance reference = Meters.of(0.0);
  }

  default void setLength(Distance length) {}

  default void setOpenLoop(Voltage voltage) {}

  default void stop() {}

  default void resetLength() {}

  default void resetLength(Distance length) {}

  default void setPIDG(double kP, double kI, double kD, double kG) {}

  default void setConstraints(LinearVelocity cruiseVelocity, LinearAcceleration acceleration) {}

  default void setNeutralMode(NeutralModeValue neutralMode) {}

  default void setLogKey(String logKey) {}
}
