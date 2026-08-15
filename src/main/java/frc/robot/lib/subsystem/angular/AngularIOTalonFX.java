package frc.robot.lib.subsystem.angular;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.RobotConstants.kMaxTimeoutMS;
import static frc.robot.lib.subsystem.angular.AngularIOOutputMode.*;
import static frc.robot.lib.utils.PhoenixUtils.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.lib.subsystem.DeviceConnectedStatus;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;

public class AngularIOTalonFX implements AngularIO {
  // Hardware
  private final TalonFX master;
  private final List<TalonFX> followers;
  private Optional<CANcoder> cancoder;

  // Config
  private final TalonFXConfiguration masterConfig;
  private final TalonFXConfiguration followerConfig;

  // Status Signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Double> referencePosition;
  private final StatusSignal<Double> referenceVelocity;
  private final List<BaseStatusSignal> motorTemperatures;

  private final MotionMagicVoltage motionMagicPos;
  private final MotionMagicVelocityVoltage motionMagicVel;
  private final VoltageOut voltageOut;

  private final AngularIOTalonFXConfig deviceConfig;

  private final Alert configurationsNotAppliedAlert =
      new Alert("Configurations for AngularSubsystem not applied!", Alert.AlertType.kError);

  private AngularIOOutputMode outputMode = kNeutral;
  private Optional<Angle> goalPos = Optional.empty();
  private Optional<AngularVelocity> goalVel = Optional.empty();

  public AngularIOTalonFX(AngularIOTalonFXConfig config) {
    this.deviceConfig = config;

    master = new TalonFX(config.getMasterId(), config.getBus());
    followers =
        config.getFollowerIds().stream().map(id -> new TalonFX(id, config.getBus())).toList();

    // Clear sticky faults.
    master.clearStickyFaults(kMaxTimeoutMS);
    followers.forEach(talonFX -> talonFX.clearStickyFaults(kMaxTimeoutMS));

    // Set follower control.
    followers.forEach(
        talonFX ->
            talonFX.setControl(
                new Follower(
                    config.getMasterId(),
                    config.isOpposeMaster()
                        ? MotorAlignmentValue.Opposed
                        : MotorAlignmentValue.Aligned)));

    // Apply motor configs.
    masterConfig = getMasterConfig();
    followerConfig = getMasterConfig();
    followerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    followerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    AtomicBoolean applySuccess =
        new AtomicBoolean(
            tryUntilOk(() -> master.getConfigurator().apply(masterConfig, kMaxTimeoutMS)));
    followers.forEach(
        talonFX ->
            applySuccess.set(
                applySuccess.get()
                    & tryUntilOk(
                        () -> talonFX.getConfigurator().apply(followerConfig, kMaxTimeoutMS))));
    configurationsNotAppliedAlert.set(!applySuccess.get());

    motionMagicPos =
        new MotionMagicVoltage(
            Rotations.of(
                config.getResetAngle().in(Radians)
                    / config.getOutputAnglePerOutputRotation().in(Radians)));
    motionMagicVel = new MotionMagicVelocityVoltage(RotationsPerSecond.of(0.0));
    voltageOut = new VoltageOut(0.0);

    // Set signals.
    position = master.getPosition();
    appliedVolts = master.getMotorVoltage();
    statorCurrent = master.getStatorCurrent();
    supplyCurrent = master.getSupplyCurrent();
    velocity = master.getVelocity();
    acceleration = master.getAcceleration();
    referencePosition = master.getClosedLoopReference();
    referenceVelocity = master.getClosedLoopReferenceSlope();
    motorTemperatures = new ArrayList<>();
    motorTemperatures.add(master.getDeviceTemp());
    motorTemperatures.addAll(followers.stream().map(CoreTalonFX::getDeviceTemp).toList());
    String busName = config.getBus().getName();

    // Add to io manager
    SignalIOManager.addSignals(
        busName,
        position,
        appliedVolts,
        statorCurrent,
        supplyCurrent,
        velocity,
        acceleration,
        referencePosition,
        referenceVelocity);
    SignalIOManager.addSignals(busName, motorTemperatures);

    // Update logging frequency
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        position,
        velocity,
        acceleration,
        appliedVolts,
        supplyCurrent,
        statorCurrent,
        referencePosition,
        referenceVelocity);

    // Update status frame rate
    master
        .getMotorVoltage()
        .setUpdateFrequency(
            config.getStatusFrameUpdateRate()); // So that the followers update at a higher rate
  }

  private TalonFXConfiguration getMasterConfig() {
    final TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.Slot0.kP =
        deviceConfig.getKP() * deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
    configuration.Slot0.kI =
        deviceConfig.getKI() * deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
    configuration.Slot0.kD =
        deviceConfig.getKD() * deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
    configuration.Slot0.kS = deviceConfig.getKS();
    configuration.Slot0.kV =
        deviceConfig.getKV() * deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
    configuration.Slot0.kA =
        deviceConfig.getKA() * deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
    if (deviceConfig.getGravityType().isPresent()) {
      configuration.Slot0.GravityType = deviceConfig.getGravityType().get();
      configuration.Slot0.kG = deviceConfig.getKG();
    }

    configuration.MotionMagic.MotionMagicCruiseVelocity =
        deviceConfig.getCruiseVelocity().in(RadiansPerSecond)
            / deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
    configuration.MotionMagic.MotionMagicAcceleration =
        deviceConfig.getAcceleration().in(RadiansPerSecondPerSecond)
            / deviceConfig.getOutputAnglePerOutputRotation().in(Radians);

    configuration.CurrentLimits.SupplyCurrentLimit = deviceConfig.getSupplyCurrentLimit().in(Amps);
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;

    if (deviceConfig.getSupplyCurrentLowerTime() != null) {
      configuration.CurrentLimits.SupplyCurrentLowerTime =
          deviceConfig.getSupplyCurrentLowerTime().in(Seconds);
    }

    if (deviceConfig.getSupplyCurrentLower() != null) {
      configuration.CurrentLimits.SupplyCurrentLowerLimit =
          deviceConfig.getSupplyCurrentLower().in(Amps);
    }

    configuration.CurrentLimits.StatorCurrentLimit = deviceConfig.getStatorCurrentLimit().in(Amps);
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;

    configuration.Feedback.SensorToMechanismRatio =
        deviceConfig.getMotorRotationsPerOutputRotations();

    // For remote CANCoder
    if (deviceConfig.getSensorId().isPresent()) {
      configuration.Feedback.FeedbackRemoteSensorID = deviceConfig.getSensorId().get();
      configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
      configuration.Feedback.RotorToSensorRatio = deviceConfig.getRotorRotationsPerSensorRotation();
      cancoder = Optional.of(new CANcoder(deviceConfig.getSensorId().get(), deviceConfig.getBus()));
    } else {
      cancoder = Optional.empty();
    }

    configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        deviceConfig.getSoftMaxAngle().in(Radians)
            / deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
    configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable =
        deviceConfig.getSoftMaxAngle().gte(Radians.of(Double.POSITIVE_INFINITY));
    configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        deviceConfig.getSoftMinAngle().in(Radians)
            / deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
    configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable =
        deviceConfig.getSoftMinAngle().lte(Radians.of(Double.NEGATIVE_INFINITY));

    configuration.MotorOutput.NeutralMode = deviceConfig.getNeutralMode();
    configuration.MotorOutput.Inverted = deviceConfig.getInverted();
    return configuration;
  }

  @Override
  public void updateInputs(AngularIOInputs inputs) {
    inputs.angle =
        Radians.of(
            position.getValueAsDouble()
                * deviceConfig.getOutputAnglePerOutputRotation().in(Radians));
    inputs.appliedVolts = appliedVolts.getValue();
    inputs.supplyCurrent = supplyCurrent.getValue();
    inputs.statorCurrent = statorCurrent.getValue();
    inputs.velocity =
        RadiansPerSecond.of(
            velocity.getValueAsDouble()
                * deviceConfig.getOutputAnglePerOutputRotation().in(Radians));
    inputs.acceleration =
        RadiansPerSecondPerSecond.of(
            acceleration.getValueAsDouble()
                * deviceConfig.getOutputAnglePerOutputRotation().in(Radians));

    inputs.motorTemperatures = new double[motorTemperatures.size()];
    int mIdx = 0;
    for (var signal : motorTemperatures) {
      inputs.motorTemperatures[mIdx++] = signal.getValueAsDouble();
    }

    int size = followers.size() + 1;
    if (inputs.deviceConnectedStatuses.length != size)
      inputs.deviceConnectedStatuses = new DeviceConnectedStatus[size];
    if (inputs.deviceConnectedStatuses[0] == null) {
      inputs.deviceConnectedStatuses[0] =
          new DeviceConnectedStatus(
              BaseStatusSignal.isAllGood(
                  position,
                  appliedVolts,
                  supplyCurrent,
                  statorCurrent,
                  acceleration,
                  referencePosition,
                  referenceVelocity,
                  motorTemperatures.get(0)),
              deviceConfig.getMasterId());
    } else {
      inputs.deviceConnectedStatuses[0].setConnected(
          BaseStatusSignal.isAllGood(
              position,
              appliedVolts,
              supplyCurrent,
              statorCurrent,
              acceleration,
              referencePosition,
              referenceVelocity,
              motorTemperatures.get(0)));
    }

    for (int i = 0; i < followers.size(); i++) {
      if (inputs.deviceConnectedStatuses[i + 1] == null) {
        inputs.deviceConnectedStatuses[i + 1] =
            new DeviceConnectedStatus(
                BaseStatusSignal.isAllGood(motorTemperatures.get(i + 1)),
                deviceConfig.getFollowerIds().get(i));
      } else {
        inputs.deviceConnectedStatuses[i + 1].setConnected(
            BaseStatusSignal.isAllGood(motorTemperatures.get(i + 1)));
      }
    }

    inputs.neutralMode = deviceConfig.getNeutralMode();

    inputs.IOOutputMode = this.outputMode;
    inputs.goalPos = this.goalPos.orElse(Radians.of(0.0));
    inputs.goalVel = this.goalVel.orElse(RadiansPerSecond.of(0.0));
    if (this.outputMode == kVelocity) {
      inputs.referenceVel =
          RadiansPerSecond.of(
              referencePosition.getValueAsDouble()
                  * deviceConfig.getOutputAnglePerOutputRotation().in(Radians));
      inputs.referencePos = Radians.of(0.0);
    } else {
      inputs.referencePos =
          Radians.of(
              referencePosition.getValueAsDouble()
                  * deviceConfig.getOutputAnglePerOutputRotation().in(Radians));
      inputs.referenceVel =
          RadiansPerSecond.of(
              referenceVelocity.getValueAsDouble()
                  * deviceConfig.getOutputAnglePerOutputRotation().in(Radians));
    }
  }

  @Override
  public void setAngle(Angle angle) {
    setAngle(angle, Volts.of(0.0));
  }

  @Override
  public void setAngle(Angle angle, Voltage feedforward) {
    master.setControl(
        motionMagicPos
            .withPosition(
                (angle.in(Radians) + this.deviceConfig.getSensorOffset().in(Radians))
                    / deviceConfig.getOutputAnglePerOutputRotation().in(Radians))
            .withFeedForward(feedforward));
    goalPos = Optional.of(angle);
    goalVel = Optional.empty();
    outputMode = kClosedLoop;
  }

  @Override
  public void setVelocity(AngularVelocity angVel) {
    master.setControl(
        motionMagicVel.withVelocity(
            angVel.in(RadiansPerSecond)
                / deviceConfig.getOutputAnglePerOutputRotation().in(Radians)));
    goalPos = Optional.empty();
    goalVel = Optional.of(angVel);
    outputMode = kVelocity;
  }

  @Override
  public void setOpenLoop(Voltage voltage) {
    master.setControl(voltageOut.withOutput(voltage));
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
    if (masterConfig.Feedback.FeedbackSensorSource == FeedbackSensorSourceValue.RemoteCANcoder) {
      // Reset to the absolute position
      cancoder.get().setPosition(cancoder.get().getAbsolutePosition().getValueAsDouble());
      return;
    }
    master.setPosition(
        Rotations.of(
            angle.in(Radians) / deviceConfig.getOutputAnglePerOutputRotation().in(Radians)));
  }

  @Override
  public void setPIDVG(double kP, double kI, double kD, double kV, double kG) {
    deviceConfig.setKP(kP);
    deviceConfig.setKI(kI);
    deviceConfig.setKD(kD);
    deviceConfig.setKV(kV);
    deviceConfig.setKG(kG);

    masterConfig.Slot0.kP = kP * deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
    masterConfig.Slot0.kI = kI * deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
    masterConfig.Slot0.kD = kD * deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
    masterConfig.Slot0.kV = kV * deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
    masterConfig.Slot0.kG = kG * deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
    master.getConfigurator().apply(masterConfig, 0.0);
  }

  @Override
  public void setConstraints(AngularVelocity cruiseVelocity, AngularAcceleration acceleration) {
    deviceConfig.setCruiseVelocity(cruiseVelocity);
    deviceConfig.setAcceleration(acceleration);

    masterConfig.MotionMagic.MotionMagicCruiseVelocity =
        cruiseVelocity.in(RadiansPerSecond)
            / deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
    masterConfig.MotionMagic.MotionMagicAcceleration =
        acceleration.in(RadiansPerSecondPerSecond)
            / deviceConfig.getOutputAnglePerOutputRotation().in(Radians);
    master.getConfigurator().apply(masterConfig, 0.0);
  }

  @Override
  public void setNeutralMode(NeutralModeValue neutralMode) {
    deviceConfig.setNeutralMode(neutralMode);
    master.setNeutralMode(neutralMode, 0.0);
    followers.forEach(talonFX -> talonFX.setNeutralMode(neutralMode, 0.0));
  }

  @Override
  public void setLogKey(String logKey) {
    configurationsNotAppliedAlert.setText(
        String.format("Configurations for TalonFX %s not applied!", logKey));
  }
}
