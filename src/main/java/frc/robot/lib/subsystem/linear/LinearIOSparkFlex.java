package frc.robot.lib.subsystem.linear;

import static edu.wpi.first.units.Units.*;
import static frc.robot.lib.subsystem.linear.LinearIOOutputMode.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.subsystem.DeviceConnectedStatus;
import frc.robot.lib.subsystem.SparkConnectionMonitor;
import java.util.List;
import java.util.Optional;

/**
 * NEO Vortex / SPARK Flex implementation of {@link LinearIO}, mirroring {@link LinearIOTalonFX}.
 *
 * <p>Carries the same four hardware-forced differences documented on {@link
 * frc.robot.lib.subsystem.angular.AngularIOSparkFlex}: firmware unit conversion, hand-computed
 * gravity feedforward, differentiated acceleration, and estimated supply current.
 *
 * <p>Like the TalonFX linear IO, this is position and open-loop only — there is no velocity control
 * mode on the linear side.
 */
public class LinearIOSparkFlex implements LinearIO {
  private static final ClosedLoopSlot kSlot = ClosedLoopSlot.kSlot0;

  private final SparkFlex master;
  private final List<SparkFlex> followers;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  private final SparkConnectionMonitor masterConnection;
  private final List<SparkConnectionMonitor> followerConnections;

  private final LinearIOSparkFlexConfig deviceConfig;

  private final Alert configurationsNotAppliedAlert =
      new Alert("Configurations for LinearSubsystem not applied!", Alert.AlertType.kError);

  private LinearIOOutputMode outputMode = kNeutral;
  private Optional<Distance> goal = Optional.empty();

  private double lastVelocityMetersPerSec = 0.0;
  private double lastVelocityTimestamp = 0.0;
  private double accelerationMetersPerSecSq = 0.0;

  public LinearIOSparkFlex(LinearIOSparkFlexConfig config) {
    this.deviceConfig = config;

    master = new SparkFlex(config.getMasterId(), MotorType.kBrushless);
    followers =
        config.getFollowerIds().stream()
            .map(id -> new SparkFlex(id, MotorType.kBrushless))
            .toList();

    encoder = master.getEncoder();
    controller = master.getClosedLoopController();

    boolean ok = applyConfig(master, buildMasterConfig());
    for (SparkFlex follower : followers) {
      SparkFlexConfig followerConfig = new SparkFlexConfig();
      followerConfig
          .idleMode(idleMode(config.getNeutralMode()))
          .follow(config.getMasterId(), config.isOpposeMaster());
      if (config.getSmartCurrentLimit() != null) {
        followerConfig.smartCurrentLimit((int) config.getSmartCurrentLimit().in(Amps));
      }
      followerConfig.signals.motorTemperatureAlwaysOn(true).motorTemperaturePeriodMs(100);
      ok &= applyConfig(follower, followerConfig);
    }
    configurationsNotAppliedAlert.set(!ok);

    encoder.setPosition(toOutputRotations(config.getResetLength()));
    masterConnection = new SparkConnectionMonitor(master);
    followerConnections = followers.stream().map(SparkConnectionMonitor::new).toList();
    lastVelocityTimestamp = Timer.getFPGATimestamp();
  }

  private static boolean applyConfig(SparkFlex spark, SparkBaseConfig config) {
    // REV request settings are global; earlier IO constructors may already have selected runtime
    // nonblocking reads. Restore acknowledged startup writes before configuring this device.
    spark.setCANTimeout(100);
    spark.setCANMaxRetries(5);
    spark.clearFaults();
    return spark.configure(
            config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters)
        == com.revrobotics.REVLibError.kOk;
  }

  private static SparkBaseConfig.IdleMode idleMode(NeutralModeValue neutralMode) {
    return neutralMode == NeutralModeValue.Coast
        ? SparkBaseConfig.IdleMode.kCoast
        : SparkBaseConfig.IdleMode.kBrake;
  }

  /** Metres of travel per output rotation — the scale factor on every gain. */
  private double distancePerRotation() {
    return deviceConfig.getOutputDistancePerOutputRotation().in(Meters);
  }

  private double toOutputRotations(Distance length) {
    return length.in(Meters) / distancePerRotation();
  }

  private SparkFlexConfig buildMasterConfig() {
    SparkFlexConfig configuration = new SparkFlexConfig();

    configuration
        .inverted(deviceConfig.isInverted())
        .idleMode(idleMode(deviceConfig.getNeutralMode()));

    if (deviceConfig.getSmartCurrentLimit() != null) {
      configuration.smartCurrentLimit((int) deviceConfig.getSmartCurrentLimit().in(Amps));
    }
    if (deviceConfig.getSecondaryCurrentLimit() != null) {
      configuration.secondaryCurrentLimit(deviceConfig.getSecondaryCurrentLimit().in(Amps));
    }

    double gearing = deviceConfig.getMotorRotationsPerOutputRotations();
    configuration.encoder.positionConversionFactor(1.0 / gearing);
    configuration.encoder.velocityConversionFactor(1.0 / (gearing * 60.0));
    configuration
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputAlwaysOn(true)
        .appliedOutputPeriodMs(20)
        .busVoltageAlwaysOn(true)
        .outputCurrentAlwaysOn(true)
        .motorTemperatureAlwaysOn(true)
        .maxMotionSetpointPositionAlwaysOn(true)
        .maxMotionSetpointPositionPeriodMs(20);

    configuration.closedLoop.pid(
        deviceConfig.getKP() * distancePerRotation(),
        deviceConfig.getKI() * distancePerRotation(),
        deviceConfig.getKD() * distancePerRotation(),
        kSlot);

    configuration.closedLoop.maxMotion.maxVelocity(
        deviceConfig.getCruiseVelocity().in(MetersPerSecond) / distancePerRotation(), kSlot);
    configuration.closedLoop.maxMotion.maxAcceleration(
        deviceConfig.getAcceleration().in(MetersPerSecondPerSecond) / distancePerRotation(), kSlot);

    // Conditions the right way round, unlike the TalonFX path — see docs/lib-subsystem.md §11.
    if (Double.isFinite(deviceConfig.getSoftMaxLength().in(Meters))) {
      configuration
          .softLimit
          .forwardSoftLimit(toOutputRotations(deviceConfig.getSoftMaxLength()))
          .forwardSoftLimitEnabled(true);
    }
    if (Double.isFinite(deviceConfig.getSoftMinLength().in(Meters))) {
      configuration
          .softLimit
          .reverseSoftLimit(toOutputRotations(deviceConfig.getSoftMinLength()))
          .reverseSoftLimitEnabled(true);
    }

    return configuration;
  }

  @Override
  public void updateInputs(LinearIOInputs inputs) {
    masterConnection.beginCycle();
    double outputRotations = encoder.getPosition();
    masterConnection.checkLastError(outputRotations);
    double outputRotationsPerSec = encoder.getVelocity();
    masterConnection.checkLastError(outputRotationsPerSec);

    inputs.length = Meters.of(outputRotations * distancePerRotation());
    inputs.velocity = MetersPerSecond.of(outputRotationsPerSec * distancePerRotation());

    double now = Timer.getFPGATimestamp();
    double dt = now - lastVelocityTimestamp;
    if (dt > 1e-6) {
      accelerationMetersPerSecSq =
          (inputs.velocity.in(MetersPerSecond) - lastVelocityMetersPerSec) / dt;
      lastVelocityMetersPerSec = inputs.velocity.in(MetersPerSecond);
      lastVelocityTimestamp = now;
    }
    inputs.acceleration = MetersPerSecondPerSecond.of(accelerationMetersPerSecSq);

    double dutyCycle = master.getAppliedOutput();
    masterConnection.checkLastError(dutyCycle);
    double outputCurrent = master.getOutputCurrent();
    masterConnection.checkLastError(outputCurrent);
    double busVoltage = master.getBusVoltage();
    masterConnection.checkLastError(busVoltage);
    inputs.appliedVolts = Volts.of(dutyCycle * busVoltage);
    inputs.statorCurrent = Amps.of(outputCurrent);
    // Estimated: a SPARK reports no separate supply current.
    inputs.supplyCurrent = Amps.of(outputCurrent * Math.abs(dutyCycle));

    int deviceCount = followers.size() + 1;
    inputs.motorTemperatures = new double[deviceCount];
    inputs.motorTemperatures[0] = master.getMotorTemperature();
    masterConnection.checkLastError(inputs.motorTemperatures[0]);
    for (int i = 0; i < followers.size(); i++) {
      followerConnections.get(i).beginCycle();
      inputs.motorTemperatures[i + 1] = followers.get(i).getMotorTemperature();
      followerConnections.get(i).checkLastError(inputs.motorTemperatures[i + 1]);
    }

    if (inputs.deviceConnectedStatuses.length != deviceCount) {
      inputs.deviceConnectedStatuses = new DeviceConnectedStatus[deviceCount];
    }
    inputs.neutralMode = deviceConfig.getNeutralMode();
    inputs.IOOutputMode = this.outputMode;
    inputs.goal = this.goal.orElse(Meters.of(0.0));
    if (this.outputMode == kClosedLoop) {
      inputs.reference =
          Meters.of(controller.getMAXMotionSetpointPosition() * distancePerRotation());
      masterConnection.checkLastError(inputs.reference.in(Meters));
    } else {
      inputs.reference = Meters.of(0.0);
    }
    setConnected(inputs, 0, masterConnection.isConnected(), deviceConfig.getMasterId());
    for (int i = 0; i < followers.size(); i++) {
      setConnected(
          inputs,
          i + 1,
          followerConnections.get(i).isConnected(),
          deviceConfig.getFollowerIds().get(i));
    }
  }

  private static void setConnected(
      LinearIOInputs inputs, int index, boolean connected, int deviceId) {
    if (inputs.deviceConnectedStatuses[index] == null) {
      inputs.deviceConnectedStatuses[index] = new DeviceConnectedStatus(connected, deviceId);
    } else {
      inputs.deviceConnectedStatuses[index].setConnected(connected);
    }
  }

  @Override
  public void setLength(Distance length) {
    controller.setReference(
        toOutputRotations(length),
        SparkBase.ControlType.kMAXMotionPositionControl,
        kSlot,
        deviceConfig.getKG(),
        SparkClosedLoopController.ArbFFUnits.kVoltage);
    goal = Optional.of(length);
    outputMode = kClosedLoop;
  }

  @Override
  public void setOpenLoop(Voltage voltage) {
    master.setVoltage(voltage.in(Volts));
    goal = Optional.empty();
    outputMode = kOpenLoop;
  }

  @Override
  public void stop() {
    master.stopMotor();
    outputMode = kNeutral;
  }

  @Override
  public void resetLength() {
    resetLength(deviceConfig.getResetLength());
  }

  @Override
  public void resetLength(Distance length) {
    encoder.setPosition(toOutputRotations(length));
  }

  @Override
  public void setPIDG(double kP, double kI, double kD, double kG) {
    deviceConfig.setKP(kP);
    deviceConfig.setKI(kI);
    deviceConfig.setKD(kD);
    deviceConfig.setKG(kG);
    reapplyMasterAsync();
  }

  @Override
  public void setConstraints(LinearVelocity cruiseVelocity, LinearAcceleration acceleration) {
    deviceConfig.setCruiseVelocity(cruiseVelocity);
    deviceConfig.setAcceleration(acceleration);
    reapplyMasterAsync();
  }

  @Override
  public void setNeutralMode(NeutralModeValue neutralMode) {
    deviceConfig.setNeutralMode(neutralMode);
    SparkFlexConfig idleOnly = new SparkFlexConfig();
    idleOnly.idleMode(idleMode(neutralMode));
    master.configureAsync(
        idleOnly,
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);
    followers.forEach(
        follower ->
            follower.configureAsync(
                idleOnly,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters));
  }

  private void reapplyMasterAsync() {
    master.configureAsync(
        buildMasterConfig(),
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);
  }

  @Override
  public void setLogKey(String logKey) {
    configurationsNotAppliedAlert.setText(
        String.format("Configurations for SPARK Flex %s not applied!", logKey));
  }
}
