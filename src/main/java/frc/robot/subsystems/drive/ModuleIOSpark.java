package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.revrobotics.REVLibError;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

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
 *       #seedTurnPosition(double)}.
 *   <li><b>No high-rate odometry thread.</b> {@link PhoenixOdometryThread} depends on Phoenix
 *       signal timestamps and CANivore timesync, neither of which exists here, so odometry is
 *       sampled once per main loop. Expect 50 Hz pose updates rather than 250 Hz, which degrades
 *       pose accuracy during fast motion but is otherwise correct.
 * </ol>
 */
public class ModuleIOSpark implements ModuleIO {
  private static final ClosedLoopSlot kSlot = ClosedLoopSlot.kSlot0;

  // Live-tunable so the modules can be tuned at the robot without a redeploy. Shared statically:
  // all four modules are mechanically identical and want the same gains.
  //
  // Starting points are deliberately low. The steer loop previously ran kP = 1.0 with no damping,
  // which oscillates: in radians, a 1 rad error commands full output, so a few degrees of error is
  // already a hard shove through a 21.4:1 reduction. Raise kP until the module tracks crisply,
  // then back off before it buzzes; add kD only if it overshoots at a kP you otherwise want.
  private static final LoggedTunableNumber turnKp =
      new LoggedTunableNumber("Drive/Module/TurnKp", 0.3);
  private static final LoggedTunableNumber turnKd =
      new LoggedTunableNumber("Drive/Module/TurnKd", 0.0);

  // Velocity control on pure P oscillates by nature: with no feedforward, P has to generate the
  // entire output from error, so it overshoots and reverses. kV is what Drive Simple FF
  // Characterization measures; set it first and kP only trims what is left.
  private static final LoggedTunableNumber driveKp =
      new LoggedTunableNumber("Drive/Module/DriveKp", 0.0);
  private static final LoggedTunableNumber driveKv =
      new LoggedTunableNumber("Drive/Module/DriveKv", 0.0);

  private final SparkFlex driveSpark;
  private final SparkFlex turnSpark;
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  private final CANcoder cancoder;
  private final StatusSignal<Angle> turnAbsolutePosition;

  private static final int kBootSeedAttempts = 5;
  private static final double kBootSeedTimeoutSec = 0.1;
  private static final int kDisabledReseedLoops = 50; // ~1 s at 50 Hz
  private static final double kReseedMaxVelocityRadPerSec = 0.1;

  private boolean turnSeeded = false;
  private int loopsSinceSeed = 0;

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

    // Record the gains just applied so the first updateInputs doesn't see them as "changed" and
    // fire a second, asynchronous configure at the SPARK after we have already seeded it.
    primeTunables();

    // At boot the CANcoder may not have published a reading yet, and a failed read still hands
    // back 0, which would seed every module to -offset. Block briefly for a real sample; if none
    // arrives, updateInputs keeps retrying until one does.
    for (int attempt = 0; attempt < kBootSeedAttempts && !turnSeeded; attempt++) {
      turnSeeded = seedTurnPosition(kBootSeedTimeoutSec);
    }
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
   *
   * @param timeoutSec how long to wait for a fresh CANcoder sample; 0 uses the latest one received
   * @return true only if the CANcoder reading was valid and the SPARK accepted the new position
   */
  public final boolean seedTurnPosition(double timeoutSec) {
    if (timeoutSec > 0) {
      turnAbsolutePosition.waitForUpdate(timeoutSec);
    } else {
      turnAbsolutePosition.refresh();
    }
    if (!turnAbsolutePosition.getStatus().isOK()) {
      return false;
    }
    double absoluteRotations = turnAbsolutePosition.getValueAsDouble() - constants.EncoderOffset;
    return turnEncoder.setPosition(Units.rotationsToRadians(absoluteRotations)) == REVLibError.kOk;
  }

  /**
   * Retries a seed that failed at boot, and re-seeds periodically while disabled so any drift or a
   * bad boot read is corrected before the match. Never re-seeds while enabled: jumping the encoder
   * under an active position loop would kick the module.
   */
  private void maintainTurnSeed() {
    if (!turnSeeded) {
      turnSeeded = seedTurnPosition(0);
      loopsSinceSeed = 0;
      return;
    }
    if (DriverStation.isDisabled()
        && ++loopsSinceSeed >= kDisabledReseedLoops
        && Math.abs(turnEncoder.getVelocity()) < kReseedMaxVelocityRadPerSec) {
      seedTurnPosition(0);
      loopsSinceSeed = 0;
    }
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

    // A SPARK derives velocity from the hall sensors over a measurement window and then averages
    // several windows. The defaults (~32 ms x 8) hand the control loop a number 50-100 ms stale,
    // and a velocity loop closed around a measurement that old oscillates with growing amplitude.
    // 10 ms x 2 cuts the lag to roughly 20 ms, which a 50 Hz loop can cope with. Position control
    // is unaffected by this, which is why steering behaves while drive does not.
    config.encoder.uvwMeasurementPeriod(10);
    config.encoder.uvwAverageDepth(2);

    config.closedLoop.pid(0.005, 0.0, 0.0, kSlot);
    config.closedLoop.velocityFF(0.112, kSlot);
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
    config.closedLoop.pid(0.5, 0.0, turnKd.get(), kSlot);
    return config;
  }

  private void primeTunables() {
    LoggedTunableNumber.ifChanged(hashCode(), () -> {}, driveKp, driveKv);
    LoggedTunableNumber.ifChanged(hashCode() + 1, () -> {}, turnKp, turnKd);
  }

  /** Pushes new gains to the controllers when a dashboard value changes. */
  private void updateTunables() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            driveSpark.configureAsync(
                buildDriveConfig(),
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters),
        driveKp,
        driveKv);
    LoggedTunableNumber.ifChanged(
        hashCode() + 1,
        () ->
            turnSpark.configureAsync(
                buildTurnConfig(),
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters),
        turnKp,
        turnKd);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    updateTunables();
    maintainTurnSeed();
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

    // A timestamped snapshot also publishes when the wheel angle is unchanged. This lets a
    // read-only calibration client distinguish a stationary wheel from stale CAN data.
    // Fields: FPGA seconds, raw absolute rotations, CAN sample age seconds, timestamp valid,
    // encoder connected, drive connected, turn connected, zeroed absolute radians,
    // relative turn radians, drive volts, turn volts, disabled, turn encoder seeded.
    if (DriverStation.isDisabled()) {
      Logger.recordOutput(
          "Drive/Calibration/CANcoder" + constants.EncoderId,
          new double[] {
            Timer.getFPGATimestamp(),
            turnAbsolutePosition.getValueAsDouble(),
            turnAbsolutePosition.getTimestamp().getLatency(),
            turnAbsolutePosition.getTimestamp().isValid() ? 1.0 : 0.0,
            inputs.turnEncoderConnected ? 1.0 : 0.0,
            inputs.driveConnected ? 1.0 : 0.0,
            inputs.turnConnected ? 1.0 : 0.0,
            inputs.turnAbsolutePosition.getRadians(),
            inputs.turnPosition.getRadians(),
            inputs.driveAppliedVolts,
            inputs.turnAppliedVolts,
            1.0,
            turnSeeded ? 1.0 : 0.0
          });
    }

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
