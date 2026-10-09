package frc.robot.lib.subsystem.angular;

import static edu.wpi.first.units.Units.*;
import static frc.robot.lib.subsystem.angular.AngularIOOutputMode.*;

import com.ctre.phoenix6.signals.GravityTypeValue;
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
 * NEO Vortex / SPARK Flex implementation of {@link AngularIO}, mirroring {@link AngularIOTalonFX}.
 *
 * <p>Everything above the IO boundary — {@link AngularSubsystem}, the state machines, logging,
 * replay — is unchanged. Four behaviours differ because the hardware differs, and each is a thing
 * to know rather than a thing to fix:
 *
 * <ol>
 *   <li><b>Units are converted in firmware.</b> Position and velocity conversion factors make the
 *       encoder report <em>output rotations</em> and <em>output rotations per second</em>, matching
 *       what {@code SensorToMechanismRatio} does on a TalonFX, so the gain scaling is identical.
 *   <li><b>Gravity feedforward is computed here.</b> A SPARK has no firmware {@code kG}, so this
 *       class applies it as an arbitrary feedforward, cosine-scaled for an arm.
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

  private final Alert configurationsNotAppliedAlert =
      new Alert("Configurations for AngularSubsystem not applied!", Alert.AlertType.kError);

  private AngularIOOutputMode outputMode = kNeutral;
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
      ok &= applyConfig(follower, followerConfig);
    }
    configurationsNotAppliedAlert.set(!ok);

    encoder.setPosition(toOutputRotations(config.getResetAngle()));
    masterConnection = new SparkConnectionMonitor(master);
    followerConnections = followers.stream().map(SparkConnectionMonitor::new).toList();
    lastVelocityTimestamp = Timer.getFPGATimestamp();
  }

  private static boolean applyConfig(SparkFlex spark, SparkBaseConfig config) {
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

    configuration.closedLoop.pid(
        deviceConfig.getKP() * anglePerRotation(),
        deviceConfig.getKI() * anglePerRotation(),
        deviceConfig.getKD() * anglePerRotation(),
        kSlot);
    configuration.closedLoop.velocityFF(deviceConfig.getKV() * anglePerRotation(), kSlot);

    configuration.closedLoop.maxMotion.maxVelocity(
        deviceConfig.getCruiseVelocity().in(RadiansPerSecond) / anglePerRotation(), kSlot);
    configuration.closedLoop.maxMotion.maxAcceleration(
        deviceConfig.getAcceleration().in(RadiansPerSecondPerSecond) / anglePerRotation(), kSlot);

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

  /** Gravity feedforward in volts, since the SPARK cannot compute one itself. */
  private double gravityFeedforwardVolts() {
    if (deviceConfig.getGravityType().isEmpty()) {
      return 0.0;
    }
    double kG = deviceConfig.getKG();
    if (deviceConfig.getGravityType().get() == GravityTypeValue.Arm_Cosine) {
      return kG * Math.cos(encoder.getPosition() * anglePerRotation());
    }
    return kG;
  }

  @Override
  public void updateInputs(AngularIOInputs inputs) {
    masterConnection.beginCycle();
    double outputRotations = encoder.getPosition();
    masterConnection.checkLastError();
    double outputRotationsPerSec = encoder.getVelocity();
    masterConnection.checkLastError();

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
    masterConnection.checkLastError();
    double outputCurrent = master.getOutputCurrent();
    masterConnection.checkLastError();
    inputs.appliedVolts = Volts.of(dutyCycle * master.getBusVoltage());
    masterConnection.checkLastError();
    inputs.statorCurrent = Amps.of(outputCurrent);
    // Estimated: a SPARK reports no separate supply current.
    inputs.supplyCurrent = Amps.of(outputCurrent * Math.abs(dutyCycle));

    int deviceCount = followers.size() + 1;
    inputs.motorTemperatures = new double[deviceCount];
    inputs.motorTemperatures[0] = master.getMotorTemperature();
    masterConnection.checkLastError();
    for (int i = 0; i < followers.size(); i++) {
      followerConnections.get(i).beginCycle();
      inputs.motorTemperatures[i + 1] = followers.get(i).getMotorTemperature();
      followerConnections.get(i).checkLastError();
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
          RadiansPerSecond.of(controller.getMAXMotionSetpointVelocity() * anglePerRotation());
      masterConnection.checkLastError();
      inputs.referencePos = Radians.of(0.0);
    } else {
      inputs.referencePos =
          Radians.of(controller.getMAXMotionSetpointPosition() * anglePerRotation());
      masterConnection.checkLastError();
      inputs.referenceVel =
          RadiansPerSecond.of(controller.getMAXMotionSetpointVelocity() * anglePerRotation());
      masterConnection.checkLastError();
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
    controller.setReference(
        toOutputRotations(angle),
        SparkBase.ControlType.kMAXMotionPositionControl,
        kSlot,
        feedforward.in(Volts) + gravityFeedforwardVolts(),
        SparkClosedLoopController.ArbFFUnits.kVoltage);
    goalPos = Optional.of(angle);
    goalVel = Optional.empty();
    outputMode = kClosedLoop;
  }

  @Override
  public void setVelocity(AngularVelocity angVel) {
    controller.setReference(
        angVel.in(RadiansPerSecond) / anglePerRotation(),
        SparkBase.ControlType.kMAXMotionVelocityControl,
        kSlot,
        deviceConfig.getKS() * Math.signum(angVel.in(RadiansPerSecond)) + gravityFeedforwardVolts(),
        SparkClosedLoopController.ArbFFUnits.kVoltage);
    goalPos = Optional.empty();
    goalVel = Optional.of(angVel);
    outputMode = kVelocity;
  }

  @Override
  public void setOpenLoop(Voltage voltage) {
    master.setVoltage(voltage.in(Volts));
    goalPos = Optional.empty();
    goalVel = Optional.empty();
    outputMode = kOpenLoop;
  }

  @Override
  public void stop() {
    master.stopMotor();
    outputMode = kNeutral;
  }

  @Override
  public void resetAngle() {
    resetAngle(deviceConfig.getResetAngle());
  }

  @Override
  public void resetAngle(Angle angle) {
    encoder.setPosition(toOutputRotations(angle));
  }

  @Override
  public void setPIDVG(double kP, double kI, double kD, double kV, double kG) {
    deviceConfig.setKP(kP);
    deviceConfig.setKI(kI);
    deviceConfig.setKD(kD);
    deviceConfig.setKV(kV);
    deviceConfig.setKG(kG);
    reapplyMasterAsync();
  }

  @Override
  public void setConstraints(AngularVelocity cruiseVelocity, AngularAcceleration acceleration) {
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

  /**
   * Live re-tuning. Async so a dashboard edit never blocks the 20 ms loop, and without a parameter
   * reset so the follower relationship survives.
   */
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
