package frc.robot.lib.subsystem.angular;

import static edu.wpi.first.units.Units.*;
import static frc.robot.lib.subsystem.angular.AngularIOOutputMode.*;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.REVLibError;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.subsystem.DeviceConnectedStatus;
import frc.robot.lib.subsystem.SparkConnectionMonitor;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * NEO Vortex / SPARK Flex implementation of {@link AngularIO}, mirroring {@link AngularIOTalonFX}.
 *
 * <p>Everything above the IO boundary — {@link AngularSubsystem}, the state machines, logging,
 * replay — is unchanged. Four behaviours differ because the hardware differs, and each is a thing
 * to know rather than a thing to fix:
 *
 * <ol>
 *   <li><b>Units are converted in firmware.</b> Position and velocity conversion factors make the
 *       encoder report <em>output rotations</em> and <em>output rotations per second</em>, matching
 *       what {@code SensorToMechanismRatio} does on a TalonFX. PID output units still differ: SPARK
 *       P/I/D use duty cycle with a millisecond time base, not Talon voltage gains.
 *   <li><b>Gravity feedforward uses REV's voltage feedforward parameters.</b> Arm cosine uses the
 *       measured encoder angle and explicitly configured rotations-to-angle ratio.
 *   <li><b>Acceleration is differentiated.</b> There is no acceleration signal, so it comes from
 *       successive velocity samples. Expect it to be noisier than a TalonFX's.
 *   <li><b>Supply current is estimated.</b> A SPARK reports only output (stator) current, so supply
 *       current is approximated as output current times duty cycle. Anything thresholding on supply
 *       current — {@code CurrentSensorSubsystem}, for instance — is reading an estimate here.
 * </ol>
 *
 * <p>There is also no batched signal refresh: SPARKs are polled individually over the RIO bus
 * rather than joining {@link SignalIOManager}'s per-bus refresh.
 */
public class AngularIOSparkFlex implements AngularIO {
  private static final ClosedLoopSlot kSlot = ClosedLoopSlot.kSlot0;

  private final SparkFlex master;
  private final List<SparkFlex> followers;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  private final SparkConnectionMonitor masterConnection;
  private final List<SparkConnectionMonitor> followerConnections;

  private final AngularIOSparkFlexConfig deviceConfig;
  private final AngularConfigurationState configurationState = new AngularConfigurationState();
  private boolean startupConfigurationHealthy;
  private boolean skipStartupResetOnce;
  private double outputLimit = 1.0;
  private Voltage requestedVolts = Volts.of(0);
  private final AngularIOInputs verifiedConfiguration = new AngularIOInputs();

  private final Alert configurationsNotAppliedAlert =
      new Alert("Configurations for AngularSubsystem not applied!", Alert.AlertType.kError);

  private AngularIOOutputMode outputMode = kNeutral;
  private SparkBase.ControlType lastVelocityControlType = SparkBase.ControlType.kVelocity;
  private Optional<Angle> goalPos = Optional.empty();
  private Optional<AngularVelocity> goalVel = Optional.empty();

  // Acceleration is differentiated from velocity; SPARK exposes no acceleration signal.
  private double lastVelocityRadPerSec = 0.0;
  private double lastVelocityTimestamp = 0.0;
  private double accelerationRadPerSecSq = 0.0;

  public AngularIOSparkFlex(AngularIOSparkFlexConfig config) {
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
      followerConfig
          .signals
          .faultsAlwaysOn(true)
          .faultsPeriodMs(20)
          .warningsAlwaysOn(true)
          .warningsPeriodMs(20);
      configureFollowerTelemetry(followerConfig);
      ok &= applyConfig(follower, followerConfig);
    }
    configurationsNotAppliedAlert.set(!ok);
    startupConfigurationHealthy = ok;

    // A one-shot marker preserves the existing controller coordinate during a warm redeploy.
    // With no marker (including a fresh boot), retain the configured hard-stop startup seed.
    try {
      skipStartupResetOnce =
          Files.deleteIfExists(
              Path.of("/tmp/bordie-angular-keep-reference-" + config.getMasterId()));
    } catch (IOException exception) {
      throw new IllegalStateException(
          "Could not consume warm-redeploy reference marker", exception);
    }
    if (!skipStartupResetOnce) encoder.setPosition(toOutputRotations(config.getResetAngle()));
    Logger.recordOutput(
        "AngularControllers/" + config.getMasterId() + "/StartupReferencePreserved",
        skipStartupResetOnce);
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

  /** Radians of mechanism travel per output rotation — the scale factor on every gain. */
  private double anglePerRotation() {
    return deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
  }

  private double toOutputRotations(Angle angle) {
    return angle.in(Radians) / anglePerRotation();
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

    // Report position in output rotations and velocity in output rotations/sec, so the gains below
    // carry the same meaning they do on a TalonFX with SensorToMechanismRatio set.
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
        .faultsAlwaysOn(true)
        .faultsPeriodMs(20)
        .warningsAlwaysOn(true)
        .warningsPeriodMs(20)
        .maxMotionSetpointPositionAlwaysOn(true)
        .maxMotionSetpointPositionPeriodMs(20)
        .maxMotionSetpointVelocityAlwaysOn(true)
        .maxMotionSetpointVelocityPeriodMs(20);

    configuration.apply(buildTuningConfig(deviceConfig, outputLimit));

    // Unlike the TalonFX path, these conditions are the right way round, so soft limits actually
    // engage. See docs/lib-subsystem.md section 11.
    if (Double.isFinite(deviceConfig.getSoftMaxAngle().in(Radians))) {
      configuration
          .softLimit
          .forwardSoftLimit(toOutputRotations(deviceConfig.getSoftMaxAngle()))
          .forwardSoftLimitEnabled(true);
    }
    if (Double.isFinite(deviceConfig.getSoftMinAngle().in(Radians))) {
      configuration
          .softLimit
          .reverseSoftLimit(toOutputRotations(deviceConfig.getSoftMinAngle()))
          .reverseSoftLimitEnabled(true);
    }

    return configuration;
  }

  /**
   * Preserve existing gain scaling: P is duty/rad, I duty/(rad*ms), D duty*ms/rad; V is
   * volts/(rad/s), S/G volts. Simulation uses volts and seconds for its PID gains.
   */
  static SparkFlexConfig buildTuningConfig(AngularIOSparkFlexConfig config, double dutyLimit) {
    double scale = config.getOutputAnglePerOutputRotation().in(Radians);
    SparkFlexConfig tuning = new SparkFlexConfig();
    tuning.closedLoop.pid(
        config.getKP() * scale, config.getKI() * scale, config.getKD() * scale, kSlot);
    // In pinned REVLib 2026.0.5 velocityFF is an alias of feedForward.kV (volts).
    tuning.closedLoop.feedForward.kV(config.getKV() * scale, kSlot);
    tuning.closedLoop.feedForward.kS(config.getKS(), kSlot);
    boolean gravityEnabled = config.getGravityType().isPresent();
    boolean arm = gravityEnabled && config.getGravityType().get() == GravityTypeValue.Arm_Cosine;
    tuning.closedLoop.feedForward.kCos(arm ? config.getKG() : 0.0, kSlot);
    tuning.closedLoop.feedForward.kCosRatio(scale / (2.0 * Math.PI), kSlot);
    tuning.closedLoop.feedForward.kG(gravityEnabled && !arm ? config.getKG() : 0.0, kSlot);
    tuning.closedLoop.outputRange(-dutyLimit, dutyLimit, kSlot);
    tuning.closedLoop.maxMotion.cruiseVelocity(
        config.getCruiseVelocity().in(RadiansPerSecond) / scale, kSlot);
    tuning.closedLoop.maxMotion.maxAcceleration(
        config.getAcceleration().in(RadiansPerSecondPerSecond) / scale, kSlot);
    return tuning;
  }

  @Override
  public void updateInputs(AngularIOInputs inputs) {
    serviceConfiguration();
    masterConnection.beginCycle();
    double outputRotations = encoder.getPosition();
    masterConnection.checkLastError(outputRotations);
    double outputRotationsPerSec = encoder.getVelocity();
    masterConnection.checkLastError(outputRotationsPerSec);

    inputs.angle = Radians.of(outputRotations * anglePerRotation());
    inputs.velocity = RadiansPerSecond.of(outputRotationsPerSec * anglePerRotation());

    double now = Timer.getFPGATimestamp();
    double dt = now - lastVelocityTimestamp;
    if (dt > 1e-6) {
      accelerationRadPerSecSq = (inputs.velocity.in(RadiansPerSecond) - lastVelocityRadPerSec) / dt;
      lastVelocityRadPerSec = inputs.velocity.in(RadiansPerSecond);
      lastVelocityTimestamp = now;
    }
    inputs.acceleration = RadiansPerSecondPerSecond.of(accelerationRadPerSecSq);

    double dutyCycle = master.getAppliedOutput();
    masterConnection.checkLastError(dutyCycle);
    double outputCurrent = master.getOutputCurrent();
    masterConnection.checkLastError(outputCurrent);
    double busVoltage = master.getBusVoltage();
    masterConnection.checkLastError(busVoltage);
    inputs.appliedVolts = Volts.of(dutyCycle * busVoltage);
    inputs.busVolts = Volts.of(busVoltage);
    inputs.statorCurrent = Amps.of(outputCurrent);
    // Estimated: a SPARK reports no separate supply current.
    inputs.supplyCurrent = Amps.of(outputCurrent * Math.abs(dutyCycle));

    int deviceCount = followers.size() + 1;
    inputs.motorTemperatures = new double[deviceCount];
    inputs.motorTemperatures[0] = master.getMotorTemperature();
    masterConnection.checkLastError(inputs.motorTemperatures[0]);
    if (deviceConfig.isLogFollowerTelemetry()) {
      double gearing = deviceConfig.getMotorRotationsPerOutputRotations();
      logMotorTelemetry(
          deviceConfig.getMasterId(),
          outputRotations * gearing,
          outputRotationsPerSec * gearing * 60.0,
          outputCurrent,
          dutyCycle,
          busVoltage,
          inputs.motorTemperatures[0]);
    }
    for (int i = 0; i < followers.size(); i++) {
      followerConnections.get(i).beginCycle();
      inputs.motorTemperatures[i + 1] = followers.get(i).getMotorTemperature();
      followerConnections.get(i).checkLastError(inputs.motorTemperatures[i + 1]);
      if (deviceConfig.isLogFollowerTelemetry()) {
        logFollowerTelemetry(
            followers.get(i),
            followerConnections.get(i),
            deviceConfig.getFollowerIds().get(i),
            inputs.motorTemperatures[i + 1]);
      }
    }

    if (inputs.deviceConnectedStatuses.length != deviceCount) {
      inputs.deviceConnectedStatuses = new DeviceConnectedStatus[deviceCount];
    }
    inputs.neutralMode = deviceConfig.getNeutralMode();
    inputs.IOOutputMode = this.outputMode;
    inputs.goalPos = this.goalPos.orElse(Radians.of(0.0));
    inputs.goalVel = this.goalVel.orElse(RadiansPerSecond.of(0.0));

    if (this.outputMode == kVelocity) {
      inputs.referenceVel =
          velocityReference(
              lastVelocityControlType,
              this.goalVel.orElse(RadiansPerSecond.of(0.0)),
              controller::getMAXMotionSetpointVelocity,
              anglePerRotation());
      if (lastVelocityControlType == SparkBase.ControlType.kMAXMotionVelocityControl) {
        masterConnection.checkLastError(inputs.referenceVel.in(RadiansPerSecond));
      }
      inputs.referencePos = Radians.of(0.0);
    } else if (this.outputMode == kClosedLoop) {
      inputs.referencePos = inputs.goalPos;
      inputs.referenceVel = RadiansPerSecond.of(0.0);
    } else {
      inputs.referencePos = Radians.of(0.0);
      inputs.referenceVel = RadiansPerSecond.of(0.0);
    }
    boolean controllerFault = controllerFault(master, masterConnection);
    boolean healthy = masterConnection.isConnected();
    setConnected(inputs, 0, healthy, deviceConfig.getMasterId());
    for (int i = 0; i < followers.size(); i++) {
      controllerFault |= controllerFault(followers.get(i), followerConnections.get(i));
      boolean followerHealthy = followerConnections.get(i).isConnected();
      healthy &= followerHealthy;
      setConnected(inputs, i + 1, followerHealthy, deviceConfig.getFollowerIds().get(i));
    }
    if (!healthy || controllerFault) {
      stop();
      configurationState.invalidateController(
          "Controller reset, fault, brownout or unhealthy telemetry: reconfigure disabled and confirm physical reference");
    }
    if (healthy && !controllerFault) inputs.sampleTimestampSeconds = now;
    inputs.requestedVolts = requestedVolts;
    copyConfigurationInputs(inputs);
  }

  private void configureFollowerTelemetry(SparkFlexConfig config) {
    if (!deviceConfig.isLogFollowerTelemetry()) return;
    // Followers report raw motor rotations and RPM, independent of the master's output gearing.
    config.encoder.positionConversionFactor(1.0).velocityConversionFactor(1.0);
    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputAlwaysOn(true)
        .appliedOutputPeriodMs(20)
        .busVoltageAlwaysOn(true)
        .busVoltagePeriodMs(20)
        .outputCurrentAlwaysOn(true)
        .outputCurrentPeriodMs(20);
  }

  private void logFollowerTelemetry(
      SparkFlex spark, SparkConnectionMonitor connection, int id, double temperature) {
    RelativeEncoder followerEncoder = spark.getEncoder();
    double motorRotations = followerEncoder.getPosition();
    connection.checkLastError(motorRotations);
    double motorRPM = followerEncoder.getVelocity();
    connection.checkLastError(motorRPM);
    double current = spark.getOutputCurrent();
    connection.checkLastError(current);
    double duty = spark.getAppliedOutput();
    connection.checkLastError(duty);
    double bus = spark.getBusVoltage();
    connection.checkLastError(bus);
    logMotorTelemetry(id, motorRotations, motorRPM, current, duty, bus, temperature);
  }

  private void logMotorTelemetry(
      int id,
      double motorRotations,
      double motorRPM,
      double current,
      double duty,
      double bus,
      double temperature) {
    String key = "AngularControllers/" + id + "/";
    Logger.recordOutput(key + "LeaderCANID", deviceConfig.getMasterId());
    Logger.recordOutput(key + "MotorRotations", motorRotations);
    Logger.recordOutput(key + "MotorRPM", motorRPM);
    Logger.recordOutput(key + "StatorCurrentAmps", current);
    Logger.recordOutput(key + "AppliedDutyCycle", duty);
    Logger.recordOutput(key + "BusVolts", bus);
    Logger.recordOutput(key + "AppliedVolts", duty * bus);
    Logger.recordOutput(key + "TemperatureCelsius", temperature);
  }

  private boolean controllerFault(SparkFlex spark, SparkConnectionMonitor connection) {
    var faults = spark.getFaults();
    connection.checkLastError(faults.rawBits);
    var stickyFaults = spark.getStickyFaults();
    connection.checkLastError(stickyFaults.rawBits);
    var warnings = spark.getWarnings();
    connection.checkLastError(warnings.rawBits);
    var stickyWarnings = spark.getStickyWarnings();
    connection.checkLastError(stickyWarnings.rawBits);
    if (deviceConfig.isLogFollowerTelemetry()) {
      String key = "AngularControllers/" + spark.getDeviceId() + "/";
      Logger.recordOutput(key + "FaultBits", faults.rawBits);
      Logger.recordOutput(key + "StickyFaultBits", stickyFaults.rawBits);
      Logger.recordOutput(key + "WarningBits", warnings.rawBits);
      Logger.recordOutput(key + "StickyWarningBits", stickyWarnings.rawBits);
      Logger.recordOutput(key + "Connected", connection.isConnected());
      Logger.recordOutput(key + "PollTimestampSeconds", Timer.getFPGATimestamp());
    }
    return faults.rawBits != 0
        || stickyFaults.rawBits != 0
        || warnings.hasReset
        || warnings.brownout
        || stickyWarnings.hasReset
        || stickyWarnings.brownout;
  }

  private static void setConnected(
      AngularIOInputs inputs, int index, boolean connected, int deviceId) {
    if (inputs.deviceConnectedStatuses[index] == null) {
      inputs.deviceConnectedStatuses[index] = new DeviceConnectedStatus(connected, deviceId);
    } else {
      inputs.deviceConnectedStatuses[index].setConnected(connected);
    }
  }

  @Override
  public void setAngle(Angle angle) {
    setAngle(angle, Volts.of(0.0));
  }

  @Override
  public void setAngle(Angle angle, Voltage feedforward) {
    if (!configurationState.ready()) {
      stop();
      return;
    }
    // The intake pivot uses direct position PID; do not depend on a MAXMotion profile.
    REVLibError result =
        controller.setReference(
            toOutputRotations(angle),
            SparkBase.ControlType.kPosition,
            kSlot,
            feedforward.in(Volts),
            SparkClosedLoopController.ArbFFUnits.kVoltage);
    Logger.recordOutput(
        "AngularControllers/" + deviceConfig.getMasterId() + "/PositionCommandResult",
        result.toString());
    goalPos = Optional.of(angle);
    goalVel = Optional.empty();
    outputMode = kClosedLoop;
  }

  @Override
  public void setVelocity(AngularVelocity angVel) {
    if (!configurationState.ready()) {
      stop();
      return;
    }
    try {
      lastVelocityControlType = velocityControlType(deviceConfig);
    } catch (IllegalArgumentException ex) {
      configurationState.reject(ex.getMessage());
      stop();
      return;
    }
    controller.setReference(
        angVel.in(RadiansPerSecond) / anglePerRotation(),
        lastVelocityControlType,
        kSlot,
        0.0,
        SparkClosedLoopController.ArbFFUnits.kVoltage);
    goalPos = Optional.empty();
    goalVel = Optional.of(angVel);
    outputMode = kVelocity;
  }

  /** MAXMotion velocity requires positive acceleration; an unspecified profile uses plain PID. */
  static SparkBase.ControlType velocityControlType(AngularIOSparkFlexConfig config) {
    double acceleration = config.getAcceleration().in(RadiansPerSecondPerSecond);
    if (!Double.isFinite(acceleration) || acceleration < 0.0) {
      throw new IllegalArgumentException("Velocity acceleration must be finite and nonnegative");
    }
    return acceleration > 0.0
        ? SparkBase.ControlType.kMAXMotionVelocityControl
        : SparkBase.ControlType.kVelocity;
  }

  /** Use the mode actually commanded, so later profile edits cannot mislabel the reference. */
  static AngularVelocity velocityReference(
      SparkBase.ControlType controlType,
      AngularVelocity goal,
      DoubleSupplier maxMotionRotationsPerSecond,
      double radiansPerRotation) {
    return controlType == SparkBase.ControlType.kMAXMotionVelocityControl
        ? RadiansPerSecond.of(maxMotionRotationsPerSecond.getAsDouble() * radiansPerRotation)
        : goal;
  }

  @Override
  public void setOpenLoop(Voltage voltage) {
    requestedVolts = voltage;
    if (!configurationState.ready()) {
      stop();
      return;
    }
    double busVoltage = master.getBusVoltage();
    if (master.getLastError() != REVLibError.kOk
        || !Double.isFinite(busVoltage)
        || busVoltage <= 0
        || !Double.isFinite(voltage.in(Volts))) {
      stop();
      return;
    }
    double cap = busVoltage * outputLimit;
    master.setVoltage(Math.max(-cap, Math.min(cap, voltage.in(Volts))));
    goalPos = Optional.empty();
    goalVel = Optional.empty();
    outputMode = kOpenLoop;
  }

  @Override
  public void stop() {
    master.stopMotor();
    requestedVolts = Volts.of(0);
    outputMode = kNeutral;
  }

  @Override
  public void resetAngle() {
    // AngularSubsystem requests one reset during construction; keep the same warm reference.
    if (skipStartupResetOnce) {
      skipStartupResetOnce = false;
      return;
    }
    resetAngle(deviceConfig.getResetAngle());
  }

  @Override
  public void resetAngle(Angle angle) {
    encoder.setPosition(toOutputRotations(angle));
  }

  @Override
  public void setPIDVG(double kP, double kI, double kD, double kV, double kG) {
    setGains(kP, kI, kD, deviceConfig.getKS(), kV, kG);
  }

  @Override
  public void setGains(double kP, double kI, double kD, double kS, double kV, double kG) {
    if (!allFinite(kP, kI, kD, kS, kV, kG)) {
      configurationState.reject("Nonfinite gain request");
      return;
    }
    if (kP == deviceConfig.getKP()
        && kI == deviceConfig.getKI()
        && kD == deviceConfig.getKD()
        && kS == deviceConfig.getKS()
        && kV == deviceConfig.getKV()
        && kG == deviceConfig.getKG()
        && (configurationState.ready() || configurationState.pending())) return;
    deviceConfig.setKP(kP);
    deviceConfig.setKI(kI);
    deviceConfig.setKD(kD);
    deviceConfig.setKS(kS);
    deviceConfig.setKV(kV);
    deviceConfig.setKG(kG);
    configurationState.request();
  }

  @Override
  public void setConstraints(AngularVelocity cruiseVelocity, AngularAcceleration acceleration) {
    double velocity = cruiseVelocity.in(RadiansPerSecond);
    double accel = acceleration.in(RadiansPerSecondPerSecond);
    if (!allFinite(velocity, accel) || velocity <= 0 || accel <= 0) {
      configurationState.reject("Motion constraints must be finite and positive");
      return;
    }
    if (cruiseVelocity.equals(deviceConfig.getCruiseVelocity())
        && acceleration.equals(deviceConfig.getAcceleration())
        && (configurationState.ready() || configurationState.pending())) return;
    deviceConfig.setCruiseVelocity(cruiseVelocity);
    deviceConfig.setAcceleration(acceleration);
    configurationState.request();
  }

  @Override
  public void setNeutralMode(NeutralModeValue neutralMode) {
    if (neutralMode == deviceConfig.getNeutralMode()) return;
    deviceConfig.setNeutralMode(neutralMode);
    configurationState.request();
  }

  private static boolean allFinite(double... values) {
    for (double value : values) if (!Double.isFinite(value)) return false;
    return true;
  }

  private boolean matches(double actual, double expected) {
    return master.getLastError() == REVLibError.kOk
        && Double.isFinite(actual)
        && Math.abs(actual - expected) <= Math.max(1e-5, Math.abs(expected) * 1e-5);
  }

  private boolean verifyConfiguration() {
    boolean ok = true;
    double scale = anglePerRotation();
    double actual = master.configAccessor.closedLoop.getP(kSlot);
    ok &= matches(actual, deviceConfig.getKP() * scale);
    verifiedConfiguration.readbackKP = actual / scale;
    actual = master.configAccessor.closedLoop.getI(kSlot);
    ok &= matches(actual, deviceConfig.getKI() * scale);
    verifiedConfiguration.readbackKI = actual / scale;
    actual = master.configAccessor.closedLoop.getD(kSlot);
    ok &= matches(actual, deviceConfig.getKD() * scale);
    verifiedConfiguration.readbackKD = actual / scale;
    actual = master.configAccessor.closedLoop.feedForward.getkV(kSlot);
    ok &= matches(actual, deviceConfig.getKV() * scale);
    verifiedConfiguration.readbackKV = actual / scale;
    actual = master.configAccessor.closedLoop.feedForward.getkS(kSlot);
    ok &= matches(actual, deviceConfig.getKS());
    verifiedConfiguration.readbackKS = actual;
    boolean gravityEnabled = deviceConfig.getGravityType().isPresent();
    boolean arm =
        gravityEnabled && deviceConfig.getGravityType().get() == GravityTypeValue.Arm_Cosine;
    actual = master.configAccessor.closedLoop.feedForward.getkCos(kSlot);
    ok &= matches(actual, arm ? deviceConfig.getKG() : 0.0);
    double gravityReadback = arm ? actual : 0.0;
    actual = master.configAccessor.closedLoop.feedForward.getkG(kSlot);
    ok &= matches(actual, gravityEnabled && !arm ? deviceConfig.getKG() : 0.0);
    if (gravityEnabled && !arm) gravityReadback = actual;
    verifiedConfiguration.readbackKG = gravityReadback;
    actual = master.configAccessor.closedLoop.feedForward.getkCosRatio(kSlot);
    ok &= matches(actual, scale / (2.0 * Math.PI));
    actual = master.configAccessor.closedLoop.maxMotion.getCruiseVelocity(kSlot);
    ok &= matches(actual, deviceConfig.getCruiseVelocity().in(RadiansPerSecond) / scale);
    verifiedConfiguration.readbackCruiseVelocityRadiansPerSecond = actual * scale;
    actual = master.configAccessor.closedLoop.maxMotion.getMaxAcceleration(kSlot);
    ok &= matches(actual, deviceConfig.getAcceleration().in(RadiansPerSecondPerSecond) / scale);
    verifiedConfiguration.readbackAccelerationRadiansPerSecondPerSecond = actual * scale;
    actual = master.configAccessor.closedLoop.getMaxOutput(kSlot);
    ok &= matches(actual, outputLimit);
    verifiedConfiguration.readbackOutputLimit = actual;
    ok &= matches(master.configAccessor.closedLoop.getMinOutput(kSlot), -outputLimit);
    ok &=
        matches(
            master.configAccessor.encoder.getPositionConversionFactor(),
            1.0 / deviceConfig.getMotorRotationsPerOutputRotations());
    ok &=
        matches(
            master.configAccessor.encoder.getVelocityConversionFactor(),
            1.0 / (60.0 * deviceConfig.getMotorRotationsPerOutputRotations()));
    boolean inversion = master.configAccessor.getInverted();
    ok &= master.getLastError() == REVLibError.kOk && inversion == deviceConfig.isInverted();
    if (Double.isFinite(deviceConfig.getSoftMinAngle().in(Radians))) {
      actual = master.configAccessor.softLimit.getReverseSoftLimit();
      ok &= matches(actual, toOutputRotations(deviceConfig.getSoftMinAngle()));
      verifiedConfiguration.readbackMinimumAngleRadians = actual * scale;
      boolean enabled = master.configAccessor.softLimit.getReverseSoftLimitEnabled();
      ok &= master.getLastError() == REVLibError.kOk && enabled;
    }
    if (Double.isFinite(deviceConfig.getSoftMaxAngle().in(Radians))) {
      actual = master.configAccessor.softLimit.getForwardSoftLimit();
      ok &= matches(actual, toOutputRotations(deviceConfig.getSoftMaxAngle()));
      verifiedConfiguration.readbackMaximumAngleRadians = actual * scale;
      boolean enabled = master.configAccessor.softLimit.getForwardSoftLimitEnabled();
      ok &= master.getLastError() == REVLibError.kOk && enabled;
    }
    return ok;
  }

  private void serviceConfiguration() {
    configurationState.service(
        DriverStation.isDisabled(),
        Timer.getFPGATimestamp(),
        () -> {
          stop();
          try {
            master.setCANTimeout(20);
            master.setCANMaxRetries(0);
            // Never erase a possibly unseen reset while an old physical reference is qualified.
            // Fault recovery already invalidated it; ordinary gain edits preserve sticky evidence.
            boolean clearSticky = !configurationState.referenceValid();
            boolean ok = !clearSticky || master.clearFaults() == REVLibError.kOk;
            ok &=
                master.configure(
                        buildMasterConfig(),
                        SparkBase.ResetMode.kNoResetSafeParameters,
                        SparkBase.PersistMode.kNoPersistParameters)
                    == REVLibError.kOk;
            ok &= verifyConfiguration();
            // Reassert follower relationship after reset/brownout as well as idle/current settings.
            SparkFlexConfig followerConfig = new SparkFlexConfig();
            followerConfig
                .idleMode(idleMode(deviceConfig.getNeutralMode()))
                .follow(deviceConfig.getMasterId(), deviceConfig.isOpposeMaster());
            if (deviceConfig.getSmartCurrentLimit() != null)
              followerConfig.smartCurrentLimit((int) deviceConfig.getSmartCurrentLimit().in(Amps));
            followerConfig
                .signals
                .motorTemperatureAlwaysOn(true)
                .motorTemperaturePeriodMs(100)
                .faultsAlwaysOn(true)
                .faultsPeriodMs(20)
                .warningsAlwaysOn(true)
                .warningsPeriodMs(20);
            configureFollowerTelemetry(followerConfig);
            for (SparkFlex follower : followers) {
              try {
                follower.setCANTimeout(20);
                follower.setCANMaxRetries(0);
                if (clearSticky) ok &= follower.clearFaults() == REVLibError.kOk;
                ok &=
                    follower.configure(
                            followerConfig,
                            SparkBase.ResetMode.kNoResetSafeParameters,
                            SparkBase.PersistMode.kNoPersistParameters)
                        == REVLibError.kOk;
              } finally {
                follower.setCANTimeout(0);
                follower.setCANMaxRetries(0);
              }
            }
            if (ok && startupConfigurationHealthy) {
              return true;
            }
            return false;
          } finally {
            master.setCANTimeout(0);
            master.setCANMaxRetries(0);
          }
        });
    configurationsNotAppliedAlert.set(!configurationState.ready());
  }

  private void copyConfigurationInputs(AngularIOInputs inputs) {
    inputs.configReady = configurationState.ready();
    inputs.configPending = configurationState.pending();
    inputs.configError = configurationState.error();
    inputs.referenceValid = configurationState.referenceValid();
    inputs.configurationGeneration = configurationState.generation();
    inputs.readbackKP = verifiedConfiguration.readbackKP;
    inputs.readbackKI = verifiedConfiguration.readbackKI;
    inputs.readbackKD = verifiedConfiguration.readbackKD;
    inputs.readbackKS = verifiedConfiguration.readbackKS;
    inputs.readbackKV = verifiedConfiguration.readbackKV;
    inputs.readbackKG = verifiedConfiguration.readbackKG;
    inputs.readbackCruiseVelocityRadiansPerSecond =
        verifiedConfiguration.readbackCruiseVelocityRadiansPerSecond;
    inputs.readbackAccelerationRadiansPerSecondPerSecond =
        verifiedConfiguration.readbackAccelerationRadiansPerSecondPerSecond;
    inputs.readbackOutputLimit = verifiedConfiguration.readbackOutputLimit;
    inputs.readbackMinimumAngleRadians = verifiedConfiguration.readbackMinimumAngleRadians;
    inputs.readbackMaximumAngleRadians = verifiedConfiguration.readbackMaximumAngleRadians;
  }

  @Override
  public boolean diagnosticSetOutputLimit(double duty) {
    if (!DriverStation.isDisabled() || !Double.isFinite(duty) || duty <= 0 || duty > 1)
      return false;
    if (outputLimit != duty) {
      outputLimit = duty;
      configurationState.request();
    }
    serviceConfiguration();
    return configurationState.ready();
  }

  @Override
  public boolean diagnosticCalibrateReference(Angle angle) {
    if (!DriverStation.isDisabled()
        || !configurationState.ready()
        || !Double.isFinite(angle.in(Radians))
        || angle.in(Radians) < deviceConfig.getSoftMinAngle().in(Radians)
        || angle.in(Radians) > deviceConfig.getSoftMaxAngle().in(Radians)) return false;
    stop();
    try {
      master.setCANTimeout(20);
      if (encoder.setPosition(toOutputRotations(angle)) != REVLibError.kOk) return false;
      boolean confirmed = matches(encoder.getPosition(), toOutputRotations(angle));
      if (confirmed) configurationState.confirmReference();
      return confirmed;
    } finally {
      master.setCANTimeout(0);
      master.setCANMaxRetries(0);
    }
  }

  @Override
  public void setLogKey(String logKey) {
    configurationsNotAppliedAlert.setText(
        String.format("Configurations for SPARK Flex %s not applied!", logKey));
  }
}
