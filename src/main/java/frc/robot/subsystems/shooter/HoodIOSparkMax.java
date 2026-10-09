package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.constants.shooter.ShooterConstants;
import frc.robot.lib.subsystem.SparkConnectionMonitor;

/**
 * Hood IO for a SPARK MAX on CAN driving the pair of PA-14P actuators, with their potentiometers on
 * roboRIO analog inputs.
 *
 * <p><b>Brushed mode.</b> A PA-14 is a 2-wire brushed DC motor, so the controller is constructed
 * with {@link MotorType#kBrushed}. A SPARK MAX in brushless mode will not drive it, and brushed
 * mode means there is no controller-side encoder — position comes entirely from the potentiometers.
 *
 * <p>Running on CAN rather than PWM buys three things this mechanism actually wants: a real current
 * limit on a motor that stalls against its own end-stops every time it finishes travelling, current
 * reporting, and a device-presence check. {@link HoodIOPWM} and {@link HoodIORelay} remain for the
 * other wirings but give up all three.
 *
 * <p>Both actuators are driven from this one controller. They are plain two-wire PA-14s with no
 * potentiometer, so there is nothing to read back — {@link Hood} times the travel, which makes
 * {@link ShooterConstants#kHoodTravelTime} load-bearing rather than a fallback. Measure it
 * worst-case.
 */
public class HoodIOSparkMax implements HoodIO {
  private final SparkMax actuators;
  private final SparkConnectionMonitor controllerConnection;

  private boolean extendCommanded = false;

  public HoodIOSparkMax() {
    actuators = new SparkMax(ShooterConstants.kHoodSparkMaxId, MotorType.kBrushed);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit((int) ShooterConstants.kHoodCurrentLimit.in(Amps));

    actuators.clearFaults();
    actuators.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);
    controllerConnection = new SparkConnectionMonitor(actuators);

    setExtended(false);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // No position sensor on a two-wire PA-14, so Hood times the travel.
    inputs.connected = false;
    inputs.positionNormalized = 0.0;
    inputs.sensorVolts = 0.0;

    controllerConnection.beginCycle();
    inputs.extendCommanded = extendCommanded;
    double dutyCycle = actuators.getAppliedOutput();
    controllerConnection.checkLastError();
    double busVoltage = actuators.getBusVoltage();
    controllerConnection.checkLastError();
    inputs.appliedVolts = dutyCycle * busVoltage;
    inputs.currentAmps = actuators.getOutputCurrent();
    controllerConnection.checkLastError();
    inputs.controllerConnected = controllerConnection.isConnected();
  }

  @Override
  public void setExtended(boolean extended) {
    extendCommanded = extended;
    actuators.set(extended ? 1.0 : -1.0);
  }

  @Override
  public void stop() {
    actuators.stopMotor();
  }
}
