package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;

/**
 * Swerve module IO for NEO Vortex drive and steer motors on SPARK Flex controllers, with a CTRE
 * CANcoder for absolute steering position.
 *
 * <p>Geometry, CAN ids and gear ratios still come from {@link SwerveModuleConstants} — those fields
 * are plain data, so {@code TunerConstants} stays useful as the single place module layout is
 * described even though nothing here talks to a TalonFX.
 *
 * <p>Two differences from the TalonFX module IO this replaces that matter:
 *
 * <ol>
 *   <li><b>No fused absolute encoder.</b> A TalonFX can fuse a remote CANcoder in firmware ({@code
 *       FusedCANcoder}); a SPARK cannot. Instead the CANcoder is read over CAN and seeded into the
 *       steer motor's relative encoder once at construction, after which the SPARK closes the loop
 *       on its own encoder. If a module is ever re-zeroed mid-match, call {@link
 *       #seedTurnPosition()}.
 *   <li><b>No high-rate odometry thread.</b> {@link PhoenixOdometryThread} depends on Phoenix
 *       signal timestamps and CANivore timesync, neither of which exists here, so odometry is
 *       sampled once per main loop. Expect 50 Hz pose updates rather than 250 Hz, which degrades
 *       pose accuracy during fast motion but is otherwise correct.
 * </ol>
 */
public class ModuleIOSpark implements ModuleIO {
  private static final ClosedLoopSlot kSlot = ClosedLoopSlot.kSlot0;

  private final SparkFlex driveSpark;
  private final SparkFlex turnSpark;
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  private final CANcoder cancoder;
  private final StatusSignal<Angle> turnAbsolutePosition;

  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  public ModuleIOSpark(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.constants = constants;

    driveSpark = new SparkFlex(constants.DriveMotorId, MotorType.kBrushless);
    turnSpark = new SparkFlex(constants.SteerMotorId, MotorType.kBrushless);
    driveEncoder = driveSpark.getEncoder();
    turnEncoder = turnSpark.getEncoder();
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    cancoder = new CANcoder(constants.EncoderId);
    turnAbsolutePosition = cancoder.getAbsolutePosition();

    configure(driveSpark, buildDriveConfig());
    configure(turnSpark, buildTurnConfig());

    seedTurnPosition();
  }

  private static void configure(SparkFlex spark, SparkBaseConfig config) {
    spark.clearFaults();
    spark.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);
  }

  /**
   * Copies the CANcoder's absolute reading into the steer motor's relative encoder. A SPARK has no
   * firmware sensor fusion, so this is what makes closed-loop steering absolute.
   */
  public final void seedTurnPosition() {
    turnAbsolutePosition.refresh();
    double absoluteRotations = turnAbsolutePosition.getValueAsDouble() - constants.EncoderOffset;
    turnEncoder.setPosition(Units.rotationsToRadians(absoluteRotations));
  }

  private SparkFlexConfig buildDriveConfig() {
    SparkFlexConfig config = new SparkFlexConfig();
    config
        .inverted(constants.DriveMotorInverted)
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit((int) constants.SlipCurrent);

    // Report wheel radians and radians/sec, which is what ModuleIO's contract expects.
    double radiansPerMotorRotation = 2.0 * Math.PI / constants.DriveMotorGearRatio;
    config.encoder.positionConversionFactor(radiansPerMotorRotation);
    config.encoder.velocityConversionFactor(radiansPerMotorRotation / 60.0);

    // TODO(bringup): tune against the real drivetrain; feedforward comes from characterization.
    config.closedLoop.pid(0.1, 0.0, 0.0, kSlot);
    config.closedLoop.velocityFF(0.0, kSlot);
    return config;
  }

  private SparkFlexConfig buildTurnConfig() {
    SparkFlexConfig config = new SparkFlexConfig();
    config
        .inverted(constants.SteerMotorInverted)
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(30);

    double radiansPerMotorRotation = 2.0 * Math.PI / constants.SteerMotorGearRatio;
    config.encoder.positionConversionFactor(radiansPerMotorRotation);
    config.encoder.velocityConversionFactor(radiansPerMotorRotation / 60.0);

    // Steering is continuous: let the controller take the short way round rather than unwinding.
    config.closedLoop.positionWrappingEnabled(true);
    config.closedLoop.positionWrappingInputRange(-Math.PI, Math.PI);
    // TODO(bringup): tune against the real modules.
    config.closedLoop.pid(1.0, 0.0, 0.0, kSlot);
    return config;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveConnected = driveSpark.getFirmwareVersion() != 0;
    inputs.drivePositionRad = driveEncoder.getPosition();
    inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
    inputs.driveAppliedVolts = driveSpark.getAppliedOutput() * driveSpark.getBusVoltage();
    inputs.driveCurrentAmps = driveSpark.getOutputCurrent();

    inputs.turnConnected = turnSpark.getFirmwareVersion() != 0;
    inputs.turnEncoderConnected = turnAbsolutePosition.refresh().getStatus().isOK();
    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble() - constants.EncoderOffset);
    inputs.turnPosition = new Rotation2d(turnEncoder.getPosition());
    inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
    inputs.turnAppliedVolts = turnSpark.getAppliedOutput() * turnSpark.getBusVoltage();
    inputs.turnCurrentAmps = turnSpark.getOutputCurrent();

    // One sample per loop — see the class note on odometry rate.
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveController.setReference(velocityRadPerSec, SparkBase.ControlType.kVelocity, kSlot);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnController.setReference(rotation.getRadians(), SparkBase.ControlType.kPosition, kSlot);
  }
}
