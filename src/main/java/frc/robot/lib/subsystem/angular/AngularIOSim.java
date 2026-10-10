package frc.robot.lib.subsystem.angular;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.RobotConstants.kDt;
import static frc.robot.lib.subsystem.angular.AngularIOOutputMode.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.sim.CurrentDrawCalculatorSim;
import frc.robot.lib.sim.PivotSim;
import frc.robot.lib.subsystem.DeviceConnectedStatus;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;

public class AngularIOSim implements AngularIO {
  private final PivotSim pivot;
  private final ProfiledPIDController posController;
  private final ProfiledPIDController velController;

  private final AngularIOSimConfig deviceConfig;

  // Ambient-only placeholder: this plant does not model motor heating.
  private final double[] motorTemperatures;
  private final DeviceConnectedStatus[] deviceConnectedStatuses = new DeviceConnectedStatus[] {};

  private AngularIOOutputMode outputMode = kNeutral;
  private Optional<Angle> goalPos = Optional.empty();
  private Optional<AngularVelocity> goalVel = Optional.empty();
  private Optional<Voltage> openLoopVolts = Optional.empty();
  private Voltage feedforward = Volts.of(0);

  private Current supplyCurrent = Amps.of(0.0);
  private double diagnosticOutputLimit = 1.0;
  private boolean diagnosticReferenceValid;

  private Optional<Supplier<Rotation2d>> realAngleFromSubsystemAngleZero = Optional.empty();
  private Optional<Supplier<Distance>> armLength = Optional.empty();

  private AngularVelocity velocity = RadiansPerSecond.of(0.0);

  public AngularIOSim(
      AngularIOSimConfig config, CurrentDrawCalculatorSim currentDrawCalculatorSim) {
    this.deviceConfig = config;
    motorTemperatures = new double[config.getNumMotors()];
    Arrays.fill(motorTemperatures, 25.0);

    this.realAngleFromSubsystemAngleZero = config.getRealAngleFromSubsystemAngleZeroSupplier();
    this.armLength = config.getArmLengthSupplier();

    // Hardware
    DCMotor motor = config.getMotor();
    pivot =
        new PivotSim(
            motor,
            config.getMotorRotationsPerOutputRotations(),
            config.getMoi().in(KilogramSquareMeters),
            armLength.orElse(() -> Meters.of(0.0)).get().in(Meters),
            config.getPhysicalMinAngle().in(Radians),
            config.getPhysicalMaxAngle().in(Radians),
            realAngleFromSubsystemAngleZero,
            config.getResetAngle().in(Radians));
    posController =
        new ProfiledPIDController(
            config.getKP(),
            config.getKI(),
            config.getKD(),
            new TrapezoidProfile.Constraints(
                config.getCruiseVelocity().in(RadiansPerSecond),
                config.getAcceleration().in(RadiansPerSecondPerSecond)));
    velController =
        new ProfiledPIDController(
            config.getKP(),
            config.getKI(),
            config.getKD(),
            new TrapezoidProfile.Constraints(
                config.getAcceleration().in(RadiansPerSecondPerSecond), 1e9));

    currentDrawCalculatorSim.registerCurrentDraw(() -> supplyCurrent);
  }

  @Override
  public void updateInputs(AngularIOInputs inputs) {
    inputs.goalPos = goalPos.orElse(Radians.of(0.0));
    inputs.goalVel = goalVel.orElse(RadiansPerSecond.of(0.0));
    inputs.sampleTimestampSeconds = Timer.getFPGATimestamp();
    inputs.configReady = true;
    inputs.referenceValid = diagnosticReferenceValid;
    inputs.configPending = false;
    inputs.configError = "";
    inputs.readbackKP = deviceConfig.getKP();
    inputs.readbackKI = deviceConfig.getKI();
    inputs.readbackKD = deviceConfig.getKD();
    inputs.readbackKS = deviceConfig.getKS();
    inputs.readbackKV = deviceConfig.getKV();
    inputs.readbackKG = deviceConfig.getKG();
    inputs.readbackOutputLimit = diagnosticOutputLimit;
    inputs.readbackCruiseVelocityRadiansPerSecond =
        deviceConfig.getCruiseVelocity().in(RadiansPerSecond);
    inputs.readbackAccelerationRadiansPerSecondPerSecond =
        deviceConfig.getAcceleration().in(RadiansPerSecondPerSecond);
    inputs.readbackMinimumAngleRadians = deviceConfig.getPhysicalMinAngle().in(Radians);
    inputs.readbackMaximumAngleRadians = deviceConfig.getPhysicalMaxAngle().in(Radians);
    armLength.ifPresent(length -> pivot.setArmLength(length.get()));

    Optional<Angle> posSet = Optional.empty();
    Optional<AngularVelocity> velSet = Optional.empty();
    double requestedVolts = 0.0;
    switch (outputMode) {
      case kClosedLoop -> {
        double currentAngle = pivot.getAngleRads();
        double correction =
            posController.calculate(currentAngle, goalPos.orElse(Radians.of(0.0)).in(Radians));
        double profileVelocity = posController.getSetpoint().velocity;
        requestedVolts =
            correction
                + profileVelocity * deviceConfig.getKV()
                + Math.signum(profileVelocity) * deviceConfig.getKS()
                + gravityFeedforwardVolts()
                + feedforward.in(Volts);
        posSet = Optional.of(Radians.of(posController.getSetpoint().position));
        velSet = Optional.of(RadiansPerSecond.of(posController.getSetpoint().velocity));
      }
      case kOpenLoop -> requestedVolts = openLoopVolts.orElse(Volts.of(0.0)).in(Volts);
      case kVelocity -> {
        double goalVelValue = goalVel.orElse(RadiansPerSecond.of(0.0)).in(RadiansPerSecond);
        double correction = velController.calculate(pivot.getVelocityRadPerSec(), goalVelValue);
        double profileVelocity = velController.getSetpoint().position;
        requestedVolts =
            correction
                + profileVelocity * deviceConfig.getKV()
                + Math.signum(profileVelocity) * deviceConfig.getKS()
                + gravityFeedforwardVolts();
        velSet = Optional.of(RadiansPerSecond.of(velController.getSetpoint().position));
      }
      case kNeutral -> requestedVolts = 0.0;
    }
    inputs.requestedVolts = Volts.of(requestedVolts);
    double batteryVoltage = Math.max(0.0, RobotController.getBatteryVoltage());
    double voltageLimit = Math.min(12.0, batteryVoltage) * diagnosticOutputLimit;
    double limitedVolts = MathUtil.clamp(requestedVolts, -voltageLimit, voltageLimit);

    // Current limiting by Nishant
    DCMotor motor = deviceConfig.getMotor();
    double backemf =
        pivot.getVelocityRadPerSec()
            * deviceConfig.getMotorRotationsPerOutputRotations()
            / motor.KvRadPerSecPerVolt; // Volts
    double desiredI = (limitedVolts - backemf) / motor.rOhms; // Amps

    // Stator current limit
    if (Math.abs(desiredI)
        > deviceConfig.getStatorCurrentLimit().in(Amps) * deviceConfig.getNumMotors()) {
      desiredI =
          Math.signum(desiredI)
              * deviceConfig.getStatorCurrentLimit().in(Amps)
              * deviceConfig.getNumMotors();
    }

    // Supply current limit
    // supplyCurrent = desiredI * applV / Vbat
    //   = desiredI * (backemf + desiredI * rOhms) / Vbat = supplyLimit
    // quadratic sol rOhms * I^2 + backemf * I - supplyLimit * Vbat = 0
    double supplyLimit =
        deviceConfig.getSupplyCurrentLimit().in(Amps) * deviceConfig.getNumMotors();
    double Vbat = batteryVoltage;
    double maxStatorFromSupply =
        (-backemf
                + Math.signum(desiredI)
                    * Math.sqrt(backemf * backemf + 4 * motor.rOhms * supplyLimit * Vbat))
            / (2 * motor.rOhms);
    if (Math.abs(desiredI) > Math.abs(maxStatorFromSupply)) {
      desiredI = maxStatorFromSupply;
    }

    // Calculate applied voltage from desired current
    double applV = MathUtil.clamp(backemf + desiredI * motor.rOhms, -voltageLimit, voltageLimit);
    // A stopped controller reports zero output even while the plant coasts/brakes under gravity.
    if (outputMode == kNeutral) applV = 0.0;
    inputs.appliedVolts = Volts.of(applV);
    double mag = Vbat > 0.0 ? Math.abs(applV / Vbat) : 0.0;

    pivot.setInput(applV);

    inputs.statorCurrent = Amps.of(Math.abs(pivot.getCurrentDrawAmps()));
    inputs.supplyCurrent = Amps.of(inputs.statorCurrent.in(Amps) * mag);
    this.supplyCurrent = inputs.supplyCurrent;

    double priorVelocity = pivot.getVelocityRadPerSec();
    pivot.update(kDt);

    inputs.referencePos = posSet.orElse(Radians.of(0.0));
    inputs.referenceVel = velSet.orElse(RadiansPerSecond.of(0.0));

    inputs.angle = Radians.of(pivot.getAngleRads());

    inputs.velocity = RadiansPerSecond.of(pivot.getVelocityRadPerSec());
    this.velocity = inputs.velocity;
    inputs.acceleration =
        RadiansPerSecondPerSecond.of((inputs.velocity.in(RadiansPerSecond) - priorVelocity) / kDt);

    inputs.motorTemperatures = this.motorTemperatures;
    inputs.deviceConnectedStatuses = this.deviceConnectedStatuses;

    inputs.neutralMode = deviceConfig.getNeutralMode();
    inputs.IOOutputMode = this.outputMode;
  }

  @Override
  public void setAngle(Angle angle) {
    setAngle(angle, Volts.of(0.0));
  }

  @Override
  public void setAngle(Angle angle, Voltage feedforward) {
    this.goalPos = Optional.of(angle);
    this.goalVel = Optional.empty();
    this.openLoopVolts = Optional.empty();
    this.feedforward = feedforward;

    if (outputMode
        != kClosedLoop) { // If the output mode was already closed loop, then the controller has
      // been updated with the measured state
      posController.reset(pivot.getAngleRads(), velocity.in(RadiansPerSecond));
    }
    outputMode = kClosedLoop;
  }

  @Override
  public void setVelocity(AngularVelocity angVel) {
    this.goalPos = Optional.empty();
    this.goalVel = Optional.of(angVel);
    this.openLoopVolts = Optional.empty();
    if (outputMode != kVelocity) {
      velController.reset(this.velocity.in(RadiansPerSecond));
    }
    outputMode = kVelocity;
  }

  @Override
  public void setOpenLoop(Voltage openLoopVoltage) {
    this.goalPos = Optional.empty();
    this.goalVel = Optional.empty();
    this.openLoopVolts = Optional.of(openLoopVoltage);
    outputMode = kOpenLoop;
  }

  @Override
  public void stop() {
    setOpenLoop(Volts.of(0.0));
    outputMode = kNeutral;
  }

  @Override
  public void setPIDVG(double kP, double kI, double kD, double kV, double kG) {
    setGains(kP, kI, kD, deviceConfig.getKS(), kV, kG);
  }

  @Override
  public void setGains(double kP, double kI, double kD, double kS, double kV, double kG) {
    deviceConfig.setKP(kP);
    deviceConfig.setKI(kI);
    deviceConfig.setKD(kD);
    deviceConfig.setKS(kS);
    deviceConfig.setKV(kV);
    deviceConfig.setKG(kG);

    posController.setPID(kP, kI, kD);
    velController.setPID(kP, kI, kD);
  }

  @Override
  public void setConstraints(AngularVelocity cruiseVelocity, AngularAcceleration acceleration) {
    deviceConfig.setCruiseVelocity(cruiseVelocity);
    deviceConfig.setAcceleration(acceleration);

    posController.setConstraints(
        new TrapezoidProfile.Constraints(
            cruiseVelocity.in(RadiansPerSecond), acceleration.in(RadiansPerSecondPerSecond)));
    velController.setConstraints(
        new TrapezoidProfile.Constraints(acceleration.in(RadiansPerSecondPerSecond), 1e9));
  }

  @Override
  public void setNeutralMode(NeutralModeValue neutralMode) {
    deviceConfig.setNeutralMode(neutralMode);
  }

  public void setRealAngleFromSubsystemAngleZeroSupplier(
      Supplier<Rotation2d> realAngleFromSubsystemAngleZero) {
    Optional<Supplier<Rotation2d>> supplier = Optional.ofNullable(realAngleFromSubsystemAngleZero);
    pivot.setRealAngleFromSubsystemAngleZeroSupplier(supplier);
    this.realAngleFromSubsystemAngleZero = supplier;
  }

  public void setArmLengthSupplier(Supplier<Distance> length) {
    pivot.setArmLength(length.get());
    this.armLength = Optional.of(length);
  }

  private double gravityFeedforwardVolts() {
    if (!deviceConfig.isKgArm()) {
      return deviceConfig.getKG();
    }
    double horizontalOffset =
        realAngleFromSubsystemAngleZero.map(supplier -> supplier.get().getRadians()).orElse(0.0);
    return deviceConfig.getKG() * Math.cos(pivot.getAngleRads() + horizontalOffset);
  }

  @Override
  public boolean diagnosticSetOutputLimit(double duty) {
    if (!Double.isFinite(duty) || duty < 0.0 || duty > 1.0) {
      return false;
    }
    diagnosticOutputLimit = duty;
    return true;
  }

  @Override
  public boolean diagnosticCalibrateReference(Angle angle) {
    double radians = angle.in(Radians);
    if (!Double.isFinite(radians)
        || radians < deviceConfig.getPhysicalMinAngle().in(Radians)
        || radians > deviceConfig.getPhysicalMaxAngle().in(Radians)) {
      return false;
    }
    // Simulation has no independent absolute sensor: place the modeled arm at the reference.
    pivot.setState(radians, 0.0);
    velocity = RadiansPerSecond.of(0.0);
    posController.reset(radians, 0.0);
    velController.reset(0.0);
    stop();
    diagnosticReferenceValid = true;
    return true;
  }
}
