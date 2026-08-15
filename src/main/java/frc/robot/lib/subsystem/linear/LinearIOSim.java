package frc.robot.lib.subsystem.linear;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.RobotConstants.kDt;
import static frc.robot.lib.subsystem.linear.LinearIOOutputMode.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.lib.sim.CurrentDrawCalculatorSim;
import frc.robot.lib.sim.LinearExtensionSim;
import frc.robot.lib.subsystem.DeviceConnectedStatus;
import java.util.Optional;
import java.util.function.Supplier;

public class LinearIOSim implements LinearIO {
  private final LinearExtensionSim linearExtension;
  private final ProfiledPIDController controller;

  private final LinearIOSimConfig deviceConfig;

  private final double[] motorTemperatures = new double[] {};
  private final DeviceConnectedStatus[] deviceConnectedStatuses = new DeviceConnectedStatus[] {};

  private LinearIOOutputMode outputMode = kNeutral;
  private Optional<Distance> goal = Optional.empty();
  private Optional<Voltage> openLoopVolts = Optional.empty();

  private Current supplyCurrent = Amps.of(0.0);

  private Optional<Supplier<Rotation2d>> angle = Optional.empty();

  private LinearVelocity velocity = MetersPerSecond.of(0.0);

  public LinearIOSim(LinearIOSimConfig config, CurrentDrawCalculatorSim currentDrawCalculatorSim) {
    this.deviceConfig = config;

    // Hardware
    DCMotor motor = config.getMotor();
    linearExtension =
        new LinearExtensionSim(
            motor,
            config.getMotorRotationsPerOutputRotations(),
            config.getCarriageMass().in(Kilograms),
            config.getOutputDistancePerOutputRotation().div(Math.PI).in(Meters),
            config.getPhysicalMinLength().in(Meters),
            config.getPhysicalMaxLength().in(Meters),
            angle,
            config.getResetLength().in(Meters));
    controller =
        new ProfiledPIDController(
            config.getKP(),
            config.getKI(),
            config.getKD(),
            new TrapezoidProfile.Constraints(
                config.getCruiseVelocity().in(MetersPerSecond),
                config.getAcceleration().in(MetersPerSecondPerSecond)));

    currentDrawCalculatorSim.registerCurrentDraw(() -> supplyCurrent);
  }

  @Override
  public void updateInputs(LinearIOInputs inputs) {
    inputs.goal = this.goal.orElse(Meters.of(0.0));
    Distance goal = Meters.of(0.0);
    switch (outputMode) {
      case kClosedLoop -> {
        inputs.appliedVolts =
            Volts.of(
                MathUtil.clamp(
                    controller.calculate(
                                linearExtension.getPositionMeters(),
                                this.goal.orElse(Meters.of(0.0)).in(Meters))
                            * RobotController.getBatteryVoltage()
                        + deviceConfig.getKG(),
                    -12.0,
                    12.0));
        goal = Meters.of(controller.getSetpoint().position);
      }
      case kOpenLoop ->
          inputs.appliedVolts =
              Volts.of(MathUtil.clamp(openLoopVolts.orElse(Volts.of(0.0)).in(Volts), -12.0, 12.0));
      case kNeutral -> inputs.appliedVolts = Volts.of(0.0);
    }
    linearExtension.setInput(inputs.appliedVolts.in(Volts));

    // Current limiting by Nishant
    DCMotor motor = deviceConfig.getMotor();
    double backemf =
        linearExtension.getMotorVelocityRadPerSec() / motor.KvRadPerSecPerVolt; // Volts
    double desiredI = (inputs.appliedVolts.in(Volts) - backemf) / motor.rOhms; // Amps

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
    double Vbat = RobotController.getBatteryVoltage();
    double maxStatorFromSupply =
        (-backemf
                + Math.signum(desiredI)
                    * Math.sqrt(backemf * backemf + 4 * motor.rOhms * supplyLimit * Vbat))
            / (2 * motor.rOhms);
    if (Math.abs(desiredI) > Math.abs(maxStatorFromSupply)) {
      desiredI = maxStatorFromSupply;
    }

    // Calculate applied voltage from desired current
    double applV = backemf + desiredI * motor.rOhms;
    if (applV > 12.0) {
      applV = 12.0;
    } else if (applV < -12.0) {
      applV = -12.0;
    }
    double mag = applV / Vbat;

    linearExtension.setInput(applV);

    inputs.statorCurrent = Amps.of(linearExtension.getCurrentDrawAmps());
    inputs.supplyCurrent = Amps.of(linearExtension.getCurrentDrawAmps() * mag);
    this.supplyCurrent = inputs.supplyCurrent;

    linearExtension.update(kDt);

    inputs.length = Meters.of(linearExtension.getPositionMeters());
    inputs.reference = goal;

    inputs.velocity = MetersPerSecond.of(linearExtension.getVelocityMetersPerSecond());
    this.velocity = inputs.velocity;
    inputs.acceleration = MetersPerSecondPerSecond.of(0.0);

    inputs.motorTemperatures = this.motorTemperatures;
    inputs.deviceConnectedStatuses = this.deviceConnectedStatuses;

    inputs.neutralMode = deviceConfig.getNeutralMode();
    inputs.IOOutputMode = this.outputMode;
  }

  @Override
  public void setLength(Distance length) {
    if (outputMode
        != kClosedLoop) { // If the output mode was already closed loop, then the controller has
      // been updated with the measured state
      controller.reset(linearExtension.getPositionMeters(), velocity.in(MetersPerSecond));
    }
    this.goal = Optional.of(length);
    this.openLoopVolts = Optional.empty();
    outputMode = kClosedLoop;
  }

  @Override
  public void setOpenLoop(Voltage openLoopVoltage) {
    this.goal = Optional.empty();
    this.openLoopVolts = Optional.of(openLoopVoltage);
    outputMode = kOpenLoop;
  }

  @Override
  public void stop() {
    setOpenLoop(Volts.of(0.0));
    outputMode = kNeutral;
  }

  @Override
  public void setPIDG(double kP, double kI, double kD, double kG) {
    deviceConfig.setKP(kP);
    deviceConfig.setKI(kI);
    deviceConfig.setKD(kD);
    deviceConfig.setKG(kG);

    controller.setPID(kP, kI, kD);
  }

  @Override
  public void setConstraints(LinearVelocity cruiseVelocity, LinearAcceleration acceleration) {
    deviceConfig.setCruiseVelocity(cruiseVelocity);
    deviceConfig.setAcceleration(acceleration);

    controller.setConstraints(
        new TrapezoidProfile.Constraints(
            cruiseVelocity.in(MetersPerSecond), acceleration.in(MetersPerSecondPerSecond)));
  }

  @Override
  public void setNeutralMode(NeutralModeValue neutralMode) {
    deviceConfig.setNeutralMode(neutralMode);
  }

  public void setAngleSupplier(Supplier<Rotation2d> angleSupplier) {
    this.angle = Optional.ofNullable(angleSupplier);
  }
}
