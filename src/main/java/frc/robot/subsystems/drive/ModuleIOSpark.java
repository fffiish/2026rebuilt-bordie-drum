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
import edu.wpi.first.math.MathUtil;
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
 *       steer motor's relative encoder while disabled, after which the SPARK closes the loop on its
 *       own encoder. {@link #seedTurnPosition(double)} refuses reseeding while enabled.
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
  // Operator-selected steering gains: P = 1.0, I = 0, D = 0. Steering position units are
  // radians, with continuous wrapping configured below from -pi to pi.
  private static final LoggedTunableNumber turnKp =
      new LoggedTunableNumber("Drive/Module/TurnKp", 1.0);
  private static final LoggedTunableNumber turnKd =
      new LoggedTunableNumber("Drive/Module/TurnKd", 0.0);

  // Preserve the previous applied drive defaults; expose the same values through the dashboard.
  private static final LoggedTunableNumber driveKp =
      new LoggedTunableNumber("Drive/Module/DriveKp", 0.005);
  private static final LoggedTunableNumber driveKv =
      new LoggedTunableNumber("Drive/Module/DriveKv", 0.112);

  private final SparkFlex driveSpark;
  private final SparkFlex turnSpark;
  private static SparkFlex startupTransport;
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  private final CANcoder cancoder;
  private final StatusSignal<Angle> turnAbsolutePosition;

  private static final int kBootSeedAttempts = 5;
  private static final double kBootSeedTimeoutSec = 0.1;
  private static final double kDisabledReseedIntervalSec = 1.0;
  private static final double kReseedMaxVelocityRadPerSec = 0.1;
  private static final SeedAttemptBudget seedAttemptBudget = new SeedAttemptBudget();

  /** Keeps acknowledged encoder writes from accumulating across modules in one 20 ms loop. */
  static final class SeedAttemptBudget {
    private double lastAttemptSec = Double.NEGATIVE_INFINITY;

    boolean acquire(double now) {
      if (now - lastAttemptSec < 0.020) return false;
      lastAttemptSec = now;
      return true;
    }
  }

  /** Hardware-independent scheduling; attempted writes count even when seeding fails. */
  static final class TurnSeedCadence {
    private final SeedAttemptBudget budget;
    private double lastAttemptSec = Double.NEGATIVE_INFINITY;

    TurnSeedCadence(SeedAttemptBudget budget) {
      this.budget = budget;
    }

    void recordAttempt(double now) {
      lastAttemptSec = now;
    }

    boolean acquire(
        double now,
        boolean disabled,
        boolean configured,
        boolean statusHealthy,
        boolean seeded,
        double velocityRadPerSec) {
      if (!Double.isFinite(now)
          || !disabled
          || !configured
          || !statusHealthy
          || now - lastAttemptSec < kDisabledReseedIntervalSec
          || (seeded && !(Math.abs(velocityRadPerSec) < kReseedMaxVelocityRadPerSec))) {
        return false;
      }
      if (!budget.acquire(now)) return false;
      recordAttempt(now);
      return true;
    }
  }

  private boolean turnSeeded = false;
  private final TurnSeedCadence turnSeedCadence = new TurnSeedCadence(seedAttemptBudget);
  private boolean driveConfigurationHealthy;
  private boolean turnConfigurationHealthy;
  private boolean baseConfigurationHealthy;
  private boolean driveStatusHealthy;
  private boolean turnStatusHealthy;
  private boolean diagnosticLimits;
  private boolean diagnosticLimitsApplied;
  private double appliedDriveKp = Double.NaN;
  private double appliedDriveKv = Double.NaN;
  private double appliedTurnKp = Double.NaN;
  private double appliedTurnKd = Double.NaN;
  private boolean appliedDriveInverted;
  private boolean appliedTurnInverted;
  private double lastGainAttemptSec = Double.NEGATIVE_INFINITY;
  private double requestedDriveKp;
  private double requestedDriveKv;
  private double requestedTurnKp;
  private double requestedTurnKd;
  private boolean requestedLimitsMode;

  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  public ModuleIOSpark(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.constants = constants;

    driveSpark = createSparkForStartup(constants.DriveMotorId);
    turnSpark = createSparkForStartup(constants.SteerMotorId);
    driveEncoder = driveSpark.getEncoder();
    turnEncoder = turnSpark.getEncoder();
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    cancoder = new CANcoder(constants.EncoderId);
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnAbsolutePosition.setUpdateFrequency(100.0);
    // Parameter requests may block at boot, but ordinary telemetry uses cached status frames.
    driveSpark.setCANMaxRetries(0);
    turnSpark.setCANMaxRetries(0);
    driveSpark.setPeriodicFrameTimeout(100);
    turnSpark.setPeriodicFrameTimeout(100);
    try {
      driveSpark.setCANTimeout(100);
      driveConfigurationHealthy =
          DriverStation.isDisabled() && validGains()
              && configure(driveSpark, buildDriveConfig(), constants.DriveMotorInverted);
      turnConfigurationHealthy =
          DriverStation.isDisabled() && validGains()
              && configure(turnSpark, buildTurnConfig(), constants.SteerMotorInverted);
      driveConfigurationHealthy &= verifyDriveConfiguration();
      turnConfigurationHealthy &= verifyTurnConfiguration();
      baseConfigurationHealthy = driveConfigurationHealthy && turnConfigurationHealthy;
      rememberRequestedConfiguration();
    } finally {
      restoreNonblockingCAN();
    }

    // At boot the CANcoder may not have published a reading yet, and a failed read still hands
    // back 0, which would seed every module to -offset. Block briefly for a real sample; if none
    // arrives, updateInputs keeps retrying until one does.
    for (int attempt = 0; attempt < kBootSeedAttempts && !turnSeeded; attempt++) {
      turnSeeded = seedTurnPosition(kBootSeedTimeoutSec);
    }
  }

  private static boolean configure(SparkFlex spark, SparkBaseConfig config, boolean inverted) {
    boolean previousInverted = spark.configAccessor.getInverted();
    boolean persistPolarity = spark.getLastError() != REVLibError.kOk || previousInverted != inverted;
    spark.clearFaults();
    // Persist a polarity correction once. Ordinary startup and live PID writes avoid rewriting
    // controller flash; live tuning below explicitly reasserts the same module polarity.
    return spark.configure(
            config,
            SparkBase.ResetMode.kResetSafeParameters,
            persistPolarity ? SparkBase.PersistMode.kPersistParameters
                : SparkBase.PersistMode.kNoPersistParameters)
        == REVLibError.kOk;
  }

  private void restoreNonblockingCAN() {
    // REVLib stores these transport settings globally, across all SPARK controllers. Temporary
    // acknowledged writes must not leave mechanism telemetry waiting for responses afterward.
    driveSpark.setCANTimeout(0);
    driveSpark.setCANMaxRetries(0);
  }

  /**
   * Copies the CANcoder's absolute reading into the steer motor's relative encoder. A SPARK has no
   * firmware sensor fusion, so this is what makes closed-loop steering absolute.
   *
   * @param timeoutSec how long to wait for a fresh CANcoder sample; 0 uses the latest one received
   * @return true only if the CANcoder reading was valid and the SPARK accepted the new position
   */
  public final boolean seedTurnPosition(double timeoutSec) {
    if (DriverStation.isEnabled() || !turnConfigurationHealthy) return false;
    turnSeedCadence.recordAttempt(Timer.getFPGATimestamp());
    if (timeoutSec > 0) {
      turnAbsolutePosition.waitForUpdate(timeoutSec);
    } else {
      turnAbsolutePosition.refresh();
    }
    if (!turnAbsolutePosition.getStatus().isOK()
        || !turnAbsolutePosition.getTimestamp().isValid()
        || turnAbsolutePosition.getTimestamp().getLatency() > 0.1) {
      return false;
    }
    double absoluteRotations = turnAbsolutePosition.getValueAsDouble() - constants.EncoderOffset;
    if (!Double.isFinite(absoluteRotations)) return false;
    try {
      turnSpark.setCANTimeout(10);
      return turnEncoder.setPosition(
              MathUtil.angleModulus(Units.rotationsToRadians(absoluteRotations)))
          == REVLibError.kOk;
    } finally {
      restoreNonblockingCAN();
    }
  }

  /**
   * Retries a seed that failed at boot, and re-seeds periodically while disabled so any drift or a
   * bad boot read is corrected before the match. Never re-seeds while enabled: jumping the encoder
   * under an active position loop would kick the module.
   */
  private void maintainTurnSeed(double velocityRadPerSec) {
    // Even with a fresh CANcoder sample, a missing SPARK can make setPosition wait for its
    // acknowledgement. Space both successful reseeds and failed attempts by elapsed time so a
    // disconnected steering controller cannot impose that wait on every disabled loop.
    if (!turnSeedCadence.acquire(
        Timer.getFPGATimestamp(),
        DriverStation.isDisabled(),
        turnConfigurationHealthy,
        turnStatusHealthy,
        turnSeeded,
        velocityRadPerSec)) return;
    turnSeeded = seedTurnPosition(0);
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

    // Preserve the existing velocity measurement settings while recording timing for diagnosis.
    config.encoder.uvwMeasurementPeriod(10);
    config.encoder.uvwAverageDepth(2);

    config.signals.primaryEncoderPositionAlwaysOn(true).primaryEncoderPositionPeriodMs(20);
    config.signals.primaryEncoderVelocityAlwaysOn(true).primaryEncoderVelocityPeriodMs(20);
    config.signals.appliedOutputAlwaysOn(true).appliedOutputPeriodMs(20);
    config.signals.busVoltageAlwaysOn(true).busVoltagePeriodMs(20);
    config.signals.outputCurrentAlwaysOn(true).outputCurrentPeriodMs(20);
    config.apply(buildDriveTuningConfig(
        constants.DriveMotorInverted, driveKp.get(), driveKv.get(), diagnosticLimits));
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
    config.signals.primaryEncoderPositionAlwaysOn(true).primaryEncoderPositionPeriodMs(20);
    config.signals.primaryEncoderVelocityAlwaysOn(true).primaryEncoderVelocityPeriodMs(20);
    config.signals.appliedOutputAlwaysOn(true).appliedOutputPeriodMs(20);
    config.signals.busVoltageAlwaysOn(true).busVoltagePeriodMs(20);
    config.signals.outputCurrentAlwaysOn(true).outputCurrentPeriodMs(20);
    config.apply(buildTurnTuningConfig(
        constants.SteerMotorInverted, turnKp.get(), turnKd.get(), diagnosticLimits));
    return config;
  }

  private static SparkFlex createSparkForStartup(int canId) {
    // REV transport settings are global. The preceding module restores a nonblocking timeout;
    // restore acknowledged requests before the next constructor asks for model/firmware data.
    if (startupTransport != null) {
      startupTransport.setCANTimeout(100);
      startupTransport.setCANMaxRetries(2);
    }
    SparkFlex spark = new SparkFlex(canId, MotorType.kBrushless);
    spark.setCANTimeout(100);
    spark.setCANMaxRetries(2);
    startupTransport = spark;
    return spark;
  }

  // Every PID/output-range write reasserts the module's fixed polarity. PID tuning must never
  // infer inversion from observed wheel spin: the optimizer can reverse speed at a half-turn.
  static SparkFlexConfig buildDriveTuningConfig(
      boolean inverted, double kp, double kv, boolean limits) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.inverted(inverted);
    config.closedLoop.pid(kp, 0.0, 0.0, kSlot);
    // REV 2026 feedforward uses volts per configured velocity unit (wheel radians/second).
    config.closedLoop.feedForward.kV(kv, kSlot);
    config.closedLoop.outputRange(limits ? -0.15 : -1.0, limits ? 0.15 : 1.0, kSlot);
    return config;
  }

  static SparkFlexConfig buildTurnTuningConfig(
      boolean inverted, double kp, double kd, boolean limits) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.inverted(inverted);
    config.closedLoop.pid(kp, 0.0, kd, kSlot);
    config.closedLoop.outputRange(limits ? -0.1 : -1.0, limits ? 0.1 : 1.0, kSlot);
    return config;
  }

  private boolean validGains() {
    return Double.isFinite(driveKp.get())
        && driveKp.get() >= 0.0
        && driveKp.get() <= 1.0
        && Double.isFinite(driveKv.get())
        && driveKv.get() >= 0.0
        && driveKv.get() <= 2.0
        && Double.isFinite(turnKp.get())
        && turnKp.get() >= 0.0
        && turnKp.get() <= 1.0
        && Double.isFinite(turnKd.get())
        && turnKd.get() >= 0.0
        && turnKd.get() <= 1.0;
  }

  /** Only send partial PID/output configs while disabled; never reset an already seeded encoder. */
  private void updateTunables() {
    if (!DriverStation.isDisabled()) return;
    if (!validGains()) {
      driveConfigurationHealthy = false;
      turnConfigurationHealthy = false;
      rememberRequestedConfiguration();
      return;
    }
    // A failed request remains inhibited. Do not repeatedly block the loop retrying a missing
    // controller; a deliberate gain/limit change or robot-program restart is required to retry.
    boolean changed =
        requestedDriveKp != driveKp.get()
            || requestedDriveKv != driveKv.get()
            || requestedTurnKp != turnKp.get()
            || requestedTurnKd != turnKd.get()
            || requestedLimitsMode != diagnosticLimits;
    double now = Timer.getFPGATimestamp();
    if (!changed || now - lastGainAttemptSec < 1.0) return;
    lastGainAttemptSec = now;
    rememberRequestedConfiguration();
    driveSpark.stopMotor();
    turnSpark.stopMotor();
    driveConfigurationHealthy = false;
    turnConfigurationHealthy = false;
    diagnosticLimitsApplied = false;
    try {
      driveSpark.setCANTimeout(20);
      SparkFlexConfig driveConfig = buildDriveTuningConfig(
          constants.DriveMotorInverted, driveKp.get(), driveKv.get(), diagnosticLimits);
      SparkFlexConfig turnConfig = buildTurnTuningConfig(
          constants.SteerMotorInverted, turnKp.get(), turnKd.get(), diagnosticLimits);
      driveConfigurationHealthy =
          driveSpark.configure(
                  driveConfig,
                  SparkBase.ResetMode.kNoResetSafeParameters,
                  SparkBase.PersistMode.kNoPersistParameters)
              == REVLibError.kOk;
      turnConfigurationHealthy =
          turnSpark.configure(
                  turnConfig,
                  SparkBase.ResetMode.kNoResetSafeParameters,
                  SparkBase.PersistMode.kNoPersistParameters)
              == REVLibError.kOk;
      driveConfigurationHealthy &= verifyDriveConfiguration();
      turnConfigurationHealthy &= verifyTurnConfiguration();
      driveConfigurationHealthy &= baseConfigurationHealthy;
      turnConfigurationHealthy &= baseConfigurationHealthy;
      diagnosticLimitsApplied =
          diagnosticLimits && driveConfigurationHealthy && turnConfigurationHealthy;
    } finally {
      restoreNonblockingCAN();
    }
  }

  private void rememberRequestedConfiguration() {
    requestedDriveKp = driveKp.get();
    requestedDriveKv = driveKv.get();
    requestedTurnKp = turnKp.get();
    requestedTurnKd = turnKd.get();
    requestedLimitsMode = diagnosticLimits;
  }

  private boolean matches(SparkFlex spark, double actual, double expected) {
    return spark.getLastError() == REVLibError.kOk
        && Double.isFinite(actual)
        && Math.abs(actual - expected) <= 1e-5;
  }

  private boolean verifyDriveConfiguration() {
    boolean ok = true;
    appliedDriveKp = driveSpark.configAccessor.closedLoop.getP(kSlot);
    ok &= matches(driveSpark, appliedDriveKp, driveKp.get());
    ok &= matches(driveSpark, driveSpark.configAccessor.closedLoop.getI(kSlot), 0);
    ok &= matches(driveSpark, driveSpark.configAccessor.closedLoop.getD(kSlot), 0);
    appliedDriveKv = driveSpark.configAccessor.closedLoop.feedForward.getkV(kSlot);
    ok &= matches(driveSpark, appliedDriveKv, driveKv.get());
    ok &=
        matches(
            driveSpark,
            driveSpark.configAccessor.closedLoop.getMinOutput(kSlot),
            diagnosticLimits ? -.15 : -1);
    ok &=
        matches(
            driveSpark,
            driveSpark.configAccessor.closedLoop.getMaxOutput(kSlot),
            diagnosticLimits ? .15 : 1);
    appliedDriveInverted = driveSpark.configAccessor.getInverted();
    ok &=
        driveSpark.getLastError() == REVLibError.kOk
            && appliedDriveInverted == constants.DriveMotorInverted;
    ok &=
        matches(
            driveSpark,
            driveSpark.configAccessor.encoder.getPositionConversionFactor(),
            2 * Math.PI / constants.DriveMotorGearRatio);
    ok &=
        matches(
            driveSpark,
            driveSpark.configAccessor.encoder.getVelocityConversionFactor(),
            2 * Math.PI / constants.DriveMotorGearRatio / 60);
    return ok;
  }

  private boolean verifyTurnConfiguration() {
    boolean ok = true;
    appliedTurnKp = turnSpark.configAccessor.closedLoop.getP(kSlot);
    ok &= matches(turnSpark, appliedTurnKp, turnKp.get());
    ok &= matches(turnSpark, turnSpark.configAccessor.closedLoop.getI(kSlot), 0);
    appliedTurnKd = turnSpark.configAccessor.closedLoop.getD(kSlot);
    ok &= matches(turnSpark, appliedTurnKd, turnKd.get());
    ok &=
        matches(
            turnSpark,
            turnSpark.configAccessor.closedLoop.getMinOutput(kSlot),
            diagnosticLimits ? -.1 : -1);
    ok &=
        matches(
            turnSpark,
            turnSpark.configAccessor.closedLoop.getMaxOutput(kSlot),
            diagnosticLimits ? .1 : 1);
    appliedTurnInverted = turnSpark.configAccessor.getInverted();
    ok &=
        turnSpark.getLastError() == REVLibError.kOk
            && appliedTurnInverted == constants.SteerMotorInverted;
    ok &=
        matches(
            turnSpark,
            turnSpark.configAccessor.encoder.getPositionConversionFactor(),
            2 * Math.PI / constants.SteerMotorGearRatio);
    ok &=
        matches(
            turnSpark,
            turnSpark.configAccessor.encoder.getVelocityConversionFactor(),
            2 * Math.PI / constants.SteerMotorGearRatio / 60);
    boolean wrappingEnabled = turnSpark.configAccessor.closedLoop.getPositionWrappingEnabled();
    ok &= turnSpark.getLastError() == REVLibError.kOk && wrappingEnabled;
    ok &=
        matches(
            turnSpark, turnSpark.configAccessor.closedLoop.getPositionWrappingMinInput(), -Math.PI);
    ok &=
        matches(
            turnSpark, turnSpark.configAccessor.closedLoop.getPositionWrappingMaxInput(), Math.PI);
    return ok;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    updateTunables();
    boolean driveOk = true;
    inputs.drivePositionRad = driveEncoder.getPosition();
    driveOk &= driveSpark.getLastError() == REVLibError.kOk;
    inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
    driveOk &= driveSpark.getLastError() == REVLibError.kOk;
    double driveAppliedOutput = driveSpark.getAppliedOutput();
    driveOk &= driveSpark.getLastError() == REVLibError.kOk;
    double driveBusVoltage = driveSpark.getBusVoltage();
    driveOk &= driveSpark.getLastError() == REVLibError.kOk;
    inputs.driveAppliedVolts = driveAppliedOutput * driveBusVoltage;
    inputs.driveCurrentAmps = driveSpark.getOutputCurrent();
    driveOk &= driveSpark.getLastError() == REVLibError.kOk;
    driveOk &=
        Double.isFinite(inputs.drivePositionRad)
            && Double.isFinite(inputs.driveVelocityRadPerSec)
            && Double.isFinite(inputs.driveAppliedVolts)
            && Double.isFinite(inputs.driveCurrentAmps);
    driveStatusHealthy = driveOk;
    inputs.driveConnected = driveOk;

    boolean turnOk = true;
    turnAbsolutePosition.refresh();
    inputs.absoluteSensorAgeSec = turnAbsolutePosition.getTimestamp().getLatency();
    inputs.turnEncoderConnected =
        turnAbsolutePosition.getStatus().isOK()
            && turnAbsolutePosition.getTimestamp().isValid()
            && inputs.absoluteSensorAgeSec <= .1
            && Double.isFinite(turnAbsolutePosition.getValueAsDouble());
    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble() - constants.EncoderOffset);
    inputs.turnPosition = new Rotation2d(turnEncoder.getPosition());
    turnOk &= turnSpark.getLastError() == REVLibError.kOk;
    inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
    turnOk &= turnSpark.getLastError() == REVLibError.kOk;
    double turnAppliedOutput = turnSpark.getAppliedOutput();
    turnOk &= turnSpark.getLastError() == REVLibError.kOk;
    double turnBusVoltage = turnSpark.getBusVoltage();
    turnOk &= turnSpark.getLastError() == REVLibError.kOk;
    inputs.turnAppliedVolts = turnAppliedOutput * turnBusVoltage;
    inputs.turnCurrentAmps = turnSpark.getOutputCurrent();
    turnOk &= turnSpark.getLastError() == REVLibError.kOk;
    turnOk &=
        Double.isFinite(inputs.turnPosition.getRadians())
            && Double.isFinite(inputs.turnVelocityRadPerSec)
            && Double.isFinite(inputs.turnAppliedVolts)
            && Double.isFinite(inputs.turnCurrentAmps);
    turnStatusHealthy = turnOk;
    maintainTurnSeed(inputs.turnVelocityRadPerSec);
    inputs.turnConnected = turnOk;
    inputs.driveStatusHealthy = driveStatusHealthy;
    inputs.turnStatusHealthy = turnStatusHealthy;
    inputs.driveMotorRpm =
        inputs.driveVelocityRadPerSec * constants.DriveMotorGearRatio * 60 / (2 * Math.PI);
    inputs.turnMotorRpm =
        inputs.turnVelocityRadPerSec * constants.SteerMotorGearRatio * 60 / (2 * Math.PI);
    inputs.turnSeeded = turnSeeded;
    inputs.configurationHealthy = driveConfigurationHealthy && turnConfigurationHealthy;
    inputs.diagnosticLimitsApplied = diagnosticLimitsApplied;
    inputs.appliedDriveKp = appliedDriveKp;
    inputs.appliedDriveKv = appliedDriveKv;
    inputs.appliedTurnKp = appliedTurnKp;
    inputs.appliedTurnKd = appliedTurnKd;
    inputs.appliedDriveInverted = appliedDriveInverted;
    inputs.appliedTurnInverted = appliedTurnInverted;
    inputs.snapshotTimestampSec = Timer.getFPGATimestamp();
    if (!outputsReady()) {
      driveSpark.stopMotor();
      turnSpark.stopMotor();
    }

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
    if (!outputsReady() || !Double.isFinite(output)) {
      driveSpark.stopMotor();
      return;
    }
    driveSpark.setVoltage(diagnosticLimits ? MathUtil.clamp(output, -1.5, 1.5) : output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    if (!outputsReady() || !Double.isFinite(output)) {
      turnSpark.stopMotor();
      return;
    }
    turnSpark.setVoltage(diagnosticLimits ? MathUtil.clamp(output, -.25, .25) : output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    if (!outputsReady() || !Double.isFinite(velocityRadPerSec)) {
      driveSpark.stopMotor();
      return;
    }
    driveController.setReference(velocityRadPerSec, SparkBase.ControlType.kVelocity, kSlot);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    if (!outputsReady() || !Double.isFinite(rotation.getRadians())) {
      turnSpark.stopMotor();
      return;
    }
    turnController.setReference(rotation.getRadians(), SparkBase.ControlType.kPosition, kSlot);
  }

  private boolean outputsReady() {
    return DriverStation.isEnabled()
        && turnSeeded
        && driveConfigurationHealthy
        && turnConfigurationHealthy
        && driveStatusHealthy
        && turnStatusHealthy
        && turnAbsolutePosition.getStatus().isOK()
        && turnAbsolutePosition.getTimestamp().isValid()
        && Double.isFinite(turnAbsolutePosition.getValueAsDouble())
        && turnAbsolutePosition.getTimestamp().getLatency() <= .1;
  }

  @Override
  public void setDiagnosticLimits(boolean active) {
    if (!DriverStation.isDisabled() || diagnosticLimits == active) return;
    diagnosticLimits = active;
    diagnosticLimitsApplied = false;
    lastGainAttemptSec = Double.NEGATIVE_INFINITY;
    updateTunables();
  }
}
