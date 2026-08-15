package frc.robot.lib.subsystem.linear;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.RobotConstants.kMaxTimeoutMS;
import static frc.robot.lib.subsystem.linear.LinearIOOutputMode.*;
import static frc.robot.lib.utils.PhoenixUtils.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.lib.subsystem.DeviceConnectedStatus;
import frc.robot.lib.subsystem.angular.SignalIOManager;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;

public class LinearIOTalonFX implements LinearIO {
  // Hardware
  private final TalonFX master;
  private final List<TalonFX> followers;

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
  private final List<BaseStatusSignal> motorTemperatures;

  private final MotionMagicVoltage motionMagic;
  private final VoltageOut voltageOut;

  private final LinearIOTalonFXConfig deviceConfig;

  private final Alert configurationsNotAppliedAlert =
      new Alert("Configurations for LinearSubsystem not applied!", Alert.AlertType.kError);

  private LinearIOOutputMode outputMode = kNeutral;
  private Optional<Distance> goal = Optional.empty();

  public LinearIOTalonFX(LinearIOTalonFXConfig config) {
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

    motionMagic =
        new MotionMagicVoltage(
            Rotations.of(
                config.getResetLength().in(Meters)
                    / config.getOutputDistancePerOutputRotation().in(Meters)));
    voltageOut = new VoltageOut(0.0);

    // Set signals.
    position = master.getPosition();
    appliedVolts = master.getMotorVoltage();
    statorCurrent = master.getStatorCurrent();
    supplyCurrent = master.getSupplyCurrent();
    velocity = master.getVelocity();
    acceleration = master.getAcceleration();
    referencePosition = master.getClosedLoopReference();
    motorTemperatures = new ArrayList<>();
    motorTemperatures.add(master.getDeviceTemp());
    motorTemperatures.addAll(followers.stream().map(CoreTalonFX::getDeviceTemp).toList());
    String busName = config.getBus().getName();

    // Set update frequency
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        position,
        velocity,
        acceleration,
        appliedVolts,
        supplyCurrent,
        statorCurrent,
        referencePosition);
    SignalIOManager.addSignals(
        busName,
        position,
        velocity,
        acceleration,
        appliedVolts,
        supplyCurrent,
        statorCurrent,
        referencePosition);
    SignalIOManager.addSignals(busName, motorTemperatures);
  }

  private TalonFXConfiguration getMasterConfig() {
    final TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.Slot0.kP = deviceConfig.getKP();
    configuration.Slot0.kI = deviceConfig.getKI();
    configuration.Slot0.kD = deviceConfig.getKD();
    configuration.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    configuration.Slot0.kG = deviceConfig.getKG();

    configuration.MotionMagic.MotionMagicCruiseVelocity =
        deviceConfig.getCruiseVelocity().in(MetersPerSecond)
            / deviceConfig.getOutputDistancePerOutputRotation().in(Meters);
    configuration.MotionMagic.MotionMagicAcceleration =
        deviceConfig.getAcceleration().in(MetersPerSecondPerSecond)
            / deviceConfig.getOutputDistancePerOutputRotation().in(Meters);

    configuration.CurrentLimits.SupplyCurrentLimit = deviceConfig.getSupplyCurrentLimit().in(Amps);
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    configuration.CurrentLimits.StatorCurrentLimit = deviceConfig.getStatorCurrentLimit().in(Amps);
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;

    configuration.Feedback.SensorToMechanismRatio =
        deviceConfig.getMotorRotationsPerOutputRotations();

    configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        deviceConfig.getSoftMaxLength().in(Meters)
            / deviceConfig.getOutputDistancePerOutputRotation().in(Meters);
    configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable =
        deviceConfig.getSoftMaxLength().gte(Meters.of(Double.POSITIVE_INFINITY));
    configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        deviceConfig.getSoftMinLength().in(Meters)
            / deviceConfig.getOutputDistancePerOutputRotation().in(Meters);
    configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable =
        deviceConfig.getSoftMinLength().lte(Meters.of(Double.NEGATIVE_INFINITY));

    configuration.MotorOutput.NeutralMode = deviceConfig.getNeutralMode();
    configuration.MotorOutput.Inverted = deviceConfig.getInverted();
    return configuration;
  }

  @Override
  public void updateInputs(LinearIOInputs inputs) {
    inputs.length =
        Meters.of(
            position.getValueAsDouble()
                * deviceConfig.getOutputDistancePerOutputRotation().in(Meters));
    inputs.appliedVolts = appliedVolts.getValue();
    inputs.supplyCurrent = supplyCurrent.getValue();
    inputs.statorCurrent = statorCurrent.getValue();
    inputs.velocity =
        MetersPerSecond.of(
            velocity.getValueAsDouble()
                * deviceConfig.getOutputDistancePerOutputRotation().in(Meters));
    inputs.acceleration =
        MetersPerSecondPerSecond.of(
            acceleration.getValueAsDouble()
                * deviceConfig.getOutputDistancePerOutputRotation().in(Meters));

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
    inputs.goal = this.goal.orElse(Meters.of(0.0));
    inputs.reference =
        Meters.of(
            referencePosition.getValueAsDouble()
                * deviceConfig.getOutputDistancePerOutputRotation().in(Meters));
  }

  @Override
  public void setLength(Distance length) {
    master.setControl(
        motionMagic.withPosition(
            length.in(Meters) / deviceConfig.getOutputDistancePerOutputRotation().in(Meters)));
    goal = Optional.of(length);
    outputMode = kClosedLoop;
  }

  @Override
  public void setOpenLoop(Voltage voltage) {
    master.setControl(voltageOut.withOutput(voltage));
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
    System.out.println(
        "Reset Length: "
            + length.in(Meters) / deviceConfig.getOutputDistancePerOutputRotation().in(Meters));
    master.setPosition(
        Rotations.of(
            length.in(Meters) / deviceConfig.getOutputDistancePerOutputRotation().in(Meters)));
  }

  @Override
  public void setPIDG(double kP, double kI, double kD, double kG) {
    deviceConfig.setKP(kP);
    deviceConfig.setKI(kI);
    deviceConfig.setKD(kD);
    deviceConfig.setKG(kG);

    masterConfig.Slot0.kP = kP;
    masterConfig.Slot0.kI = kI;
    masterConfig.Slot0.kD = kD;
    masterConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    masterConfig.Slot0.kG = kG;
    master.getConfigurator().apply(masterConfig, 0.0);
  }

  @Override
  public void setConstraints(LinearVelocity cruiseVelocity, LinearAcceleration acceleration) {
    deviceConfig.setCruiseVelocity(cruiseVelocity);
    deviceConfig.setAcceleration(acceleration);

    masterConfig.MotionMagic.MotionMagicCruiseVelocity =
        cruiseVelocity.in(MetersPerSecond)
            / deviceConfig.getOutputDistancePerOutputRotation().in(Meters);
    masterConfig.MotionMagic.MotionMagicAcceleration =
        acceleration.in(MetersPerSecondPerSecond)
            / deviceConfig.getOutputDistancePerOutputRotation().in(Meters);
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
