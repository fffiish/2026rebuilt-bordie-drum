// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.RobotSuperstructure;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.indexer.IndexerConstants;
import frc.robot.constants.intake.IntakeConstants;
import frc.robot.constants.shooter.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.LoggedInterpolatingTableManager;
import frc.robot.lib.alliancecolor.AllianceChecker;
import frc.robot.lib.controller.Joysticks;
import frc.robot.lib.sim.CurrentDrawCalculatorSim;
import frc.robot.lib.subsystem.angular.AngularIO;
import frc.robot.lib.subsystem.angular.AngularIOSim;
import frc.robot.lib.subsystem.angular.AngularIOSparkFlex;
import frc.robot.lib.subsystem.angular.AngularSubsystem;
import frc.robot.lib.subsystem.sensor.canrange.CANRangeIO;
import frc.robot.lib.subsystem.sensor.canrange.CANRangeIOCANRange;
import frc.robot.lib.subsystem.sensor.canrange.CANRangeSubsystem;
import frc.robot.lib.subsystem.sensor.currentsensor.CurrentSensorSubsystem;
import frc.robot.lib.subsystem.sensor.currentsensor.CurrentSensorSubsystemConfig;
import frc.robot.subsystems.SuperstructureVisualizer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveDiagnostics;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.HoodIO;
import frc.robot.subsystems.shooter.HoodIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 *
 * <p>TEMPLATE NOTE: this container is stripped down to drive + vision. Every place a new mechanism
 * subsystem needs to be wired in is marked with a {@code TODO(template)} comment. There are four of
 * them, and they must all be filled in for a subsystem to come alive:
 *
 * <ol>
 *   <li>Declare the field (see {@link #drive})
 *   <li>Construct it in the REAL branch, gated on a {@code Constants.*HardwareExists} flag
 *   <li>Construct it in the SIM branch with the {@code *IOSim} implementation
 *   <li>Construct a no-op version in the REPLAY branch (blank IO, so logs replay deterministically)
 * </ol>
 *
 * <p>Then bind it in {@link #configureButtonBindings()} and, if it participates in autos, register
 * its commands in {@link RobotSuperstructure#registerAutoCommands()}.
 */
public class RobotContainer {
  // Controller
  private final Joysticks driverController = new Joysticks(0);
  private final Joysticks operatorController = new Joysticks(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final Field2d field = new Field2d();

  private final Drive drive;
  private final DriveDiagnostics driveDiagnostics;
  private final Vision vision;

  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;

  private final RobotSuperstructure superstructure;

  @SuppressWarnings("FieldCanBeLocal")
  private final AllianceChecker allianceChecker = new AllianceChecker();

  @SuppressWarnings("FieldCanBeLocal")
  private final LoggedInterpolatingTableManager tableManager =
      new LoggedInterpolatingTableManager();

  @SuppressWarnings("FieldCanBeLocal")
  private final CurrentDrawCalculatorSim currentDrawCalculatorSim = new CurrentDrawCalculatorSim();

  /** Overlay these two in AdvantageScope to see mechanism tracking error. */
  @SuppressWarnings("FieldCanBeLocal")
  private final SuperstructureVisualizer measuredSuperstructureState;

  @SuppressWarnings("FieldCanBeLocal")
  private final SuperstructureVisualizer targetSuperstructureState;

  private final Alert autoAlert = new Alert("No auto selected!", Alert.AlertType.kWarning);
  private final Alert controllerOneAlert =
      new Alert("Controller 1 is unplugged!", Alert.AlertType.kWarning);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   *
   * @param isAutonomous supplied by {@link Robot}. Currently unused — pass it to any subsystem that
   *     needs to behave differently in auto (e.g. an intake that only auto-oscillates in teleop).
   */
  public RobotContainer(BooleanSupplier isAutonomous) {
    // Vision pose estimates are rejected more aggressively while a mechanism is "aimed" at a
    // target. With no aiming mechanism in the template this is always false.
    // TODO(template): replace with e.g. shooter::isTurretAimed once that subsystem exists.
    BooleanSupplier aimed = () -> false;

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        if (Constants.driveHardwareExists) {
          drive =
              new Drive(
                  new GyroIONavX(),
                  new ModuleIOSpark(TunerConstants.FrontLeft),
                  new ModuleIOSpark(TunerConstants.FrontRight),
                  new ModuleIOSpark(TunerConstants.BackLeft),
                  new ModuleIOSpark(TunerConstants.BackRight));
        } else {
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {});
        }

        if (Constants.visionHardwareExists) {
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  aimed,
                  new VisionIOLimelight(camera0Name, drive::getRotation),
                  new VisionIOLimelight(camera1Name, drive::getRotation));
        } else {
          vision = new Vision(drive::addVisionMeasurement, aimed, new VisionIO() {});
        }

        if (Constants.intakeHardwareExists) {
          intake =
              new Intake(
                  new AngularSubsystem(
                      new AngularIOSparkFlex(IntakeConstants.kPivotSparkFlexConfig),
                      IntakeConstants.kPivotSubsystemConfigReal),
                  new AngularSubsystem(
                      new AngularIOSparkFlex(IntakeConstants.kIntakeRollerSparkFlexConfig),
                      IntakeConstants.kIntakeRollerSubsystemConfigReal),
                  new AngularSubsystem(
                      new AngularIOSparkFlex(IntakeConstants.kFeederSparkFlexConfig),
                      IntakeConstants.kFeederSubsystemConfigReal));
        } else {
          intake = blankIntake();
        }

        if (Constants.indexerHardwareExists) {
          indexer =
              buildIndexer(
                  new AngularSubsystem(
                      new AngularIOSparkFlex(IndexerConstants.kSparkFlexConfig),
                      IndexerConstants.kSubsystemConfigReal),
                  new CANRangeIOCANRange(IndexerConstants.kCANRangeIOConfig));
        } else {
          indexer = blankIndexer();
        }

        if (Constants.shooterHardwareExists) {
          shooter =
              new Shooter(
                  new AngularSubsystem(
                      new AngularIOSparkFlex(ShooterConstants.kFlywheelSparkFlexConfig),
                      ShooterConstants.kFlywheelSubsystemConfigReal),
                  new Hood(new HoodIOSparkMax()));
        } else {
          shooter = blankShooter();
        }
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft, currentDrawCalculatorSim),
                new ModuleIOSim(TunerConstants.FrontRight, currentDrawCalculatorSim),
                new ModuleIOSim(TunerConstants.BackLeft, currentDrawCalculatorSim),
                new ModuleIOSim(TunerConstants.BackRight, currentDrawCalculatorSim));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                aimed,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

        intake =
            new Intake(
                new AngularSubsystem(
                    new AngularIOSim(IntakeConstants.kPivotSimConfig, currentDrawCalculatorSim),
                    IntakeConstants.kPivotSubsystemConfigSim),
                new AngularSubsystem(
                    new AngularIOSim(
                        IntakeConstants.kIntakeRollerSimConfig, currentDrawCalculatorSim),
                    IntakeConstants.kIntakeRollerSubsystemConfigSim),
                new AngularSubsystem(
                    new AngularIOSim(IntakeConstants.kFeederSimConfig, currentDrawCalculatorSim),
                    IntakeConstants.kFeederSubsystemConfigSim));

        indexer =
            buildIndexer(
                new AngularSubsystem(
                    new AngularIOSim(IndexerConstants.kSimConfig, currentDrawCalculatorSim),
                    IndexerConstants.kSubsystemConfigSim),
                new CANRangeIO() {});

        shooter =
            new Shooter(
                new AngularSubsystem(
                    new AngularIOSim(ShooterConstants.kFlywheelSimConfig, currentDrawCalculatorSim),
                    ShooterConstants.kFlywheelSubsystemConfigSim),
                new Hood(new HoodIO() {}));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, aimed, new VisionIO() {});

        intake = blankIntake();
        indexer = blankIndexer();
        shooter = blankShooter();
        break;
    }

    driveDiagnostics = new DriveDiagnostics(drive);
    stopOtherMechanisms();
    superstructure = new RobotSuperstructure(intake, indexer, shooter);
    superstructure.registerAutoCommands();

    // Overlay these two in AdvantageScope: the gap between them is the tracking error.
    measuredSuperstructureState =
        new SuperstructureVisualizer(
            drive::getPose,
            "Measured",
            RobotConstants.kMeasuredStateColor,
            intake::getMeasuredPivotAngle,
            shooter::getHoodMeasuredTravel);
    targetSuperstructureState =
        new SuperstructureVisualizer(
            drive::getPose,
            "Target",
            RobotConstants.kTargetStateColor,
            intake::getTargetPivotAngle,
            shooter::getHoodTargetTravel);

    // TODO(template): register any subsystem that implements AllianceUpdatedObserver, e.g.
    // allianceChecker.registerObservers(shooter, intake);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    logInit();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    boolean sim = Constants.currentMode == Constants.simMode;

    /* DRIVE COMMANDS
    - Left joystick: translate
    - Right joystick: turn
    - Hold X (real only): stop and move modules to X pattern to resist push
    - Hold left bumper: turbo (see RobotSuperstructure#getDriveMultiplier)
    - Hold A: dynamically align heading & X position with the trench, you control forward speed
     */
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () ->
                driverController.getLeftStickY()
                    * superstructure.getDriveMultiplier(false, driverController.leftBumper),
            () ->
                -driverController.getLeftStickX()
                    * superstructure.getDriveMultiplier(false, driverController.leftBumper),
            () ->
                -driverController.getRightStickX()
                    * superstructure.getDriveMultiplier(true, driverController.leftBumper)));

    if (!sim) {
      driverController.buttonX.whileTrue(Commands.runOnce(drive::stopWithX, drive));
    }

    driverController.buttonA.whileTrue(
        DriveCommands.joystickDriveThroughTrench(
            drive,
            () ->
                driverController.getLeftStickY()
                    * superstructure.getDriveMultiplier(false, driverController.leftBumper),
            drive::getPose));

    /* DRIVER (single-controller scheme)
    - Left stick: translate            - Right stick: rotate
    - Left trigger: intake             - Right trigger: spin up and shoot
    - Right bumper: outtake            - Left bumper: turbo
    - A: trench align                  - X (real robot only): X-lock the wheels
     */
    driverController.leftTrigger.whileTrue(superstructure.intakeFuel());
    driverController.rightTrigger.whileTrue(superstructure.shoot());
    driverController.rightBumper.whileTrue(superstructure.outtake());

    // Rumble the driver when a ball reaches the throat, so they know to stop chasing it.
    indexer.staged().onTrue(driverController.rumble.rumble(0.5, 0.25));
  }

  private void logInit() {
    SmartDashboard.putData("Field", field);

    Logger.recordOutput(
        "Poses/AprilTagField", VisionConstants.kAprilTagField.values().toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Poses/WeldedAprilTagField",
        VisionConstants.kWeldedAprilTagField.values().toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Poses/AndyMarkAprilTagField",
        VisionConstants.kAndyMarkAprilTagField.values().toArray(new Pose3d[0]));

    Logger.recordOutput("Drive/TrenchDrive/TrenchY", 0.0);
    Logger.recordOutput("Drive/TrenchDrive/YError", 0.0);
  }

  public void periodic() {
    field.setRobotPose(drive.getPose());

    autoAlert.set(autoChooser.get() == null);
    controllerOneAlert.set(!DriverStation.isJoystickConnected(0));
  }

  public void stopOtherMechanisms() {
    intake.stopImmediately();
    indexer.stopImmediately();
    shooter.stopImmediately();
  }

  /** Normal scheduler and bindings are not polled anywhere in this path. */
  public void diagnosticPeriodic(double now, double loopPeriodMs) {
    if (DriverStation.isTest()) {
      stopOtherMechanisms();
      drive.periodic();
      Logger.recordOutput("Drive/Diagnostics/OtherMechanismsInhibited", true);
    } else {
      Logger.recordOutput("Drive/Diagnostics/OtherMechanismsInhibited", false);
    }
    driveDiagnostics.periodic(edu.wpi.first.wpilibj.Timer.getFPGATimestamp(), loopPeriodMs);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  // -------------------------------------------------------------------------
  // Blank-IO builders. `new AngularIO() {}` is a complete no-op implementation, so these give a
  // fully functional subsystem that simply never moves. Used by REPLAY (so logs replay
  // deterministically) and by the REAL branch when a Constants.*HardwareExists flag is false, which
  // is how the rest of the robot runs on a partially-assembled chassis.
  // -------------------------------------------------------------------------

  private static Intake blankIntake() {
    return new Intake(
        new AngularSubsystem(new AngularIO() {}, IntakeConstants.kPivotSubsystemConfigReal),
        new AngularSubsystem(new AngularIO() {}, IntakeConstants.kIntakeRollerSubsystemConfigReal),
        new AngularSubsystem(new AngularIO() {}, IntakeConstants.kFeederSubsystemConfigReal));
  }

  private static Indexer blankIndexer() {
    return buildIndexer(
        new AngularSubsystem(new AngularIO() {}, IndexerConstants.kSubsystemConfigReal),
        new CANRangeIO() {});
  }

  private static Shooter blankShooter() {
    return new Shooter(
        new AngularSubsystem(new AngularIO() {}, ShooterConstants.kFlywheelSubsystemConfigReal),
        new Hood(new HoodIO() {}));
  }

  /**
   * The jam detector derives from the indexer rollers' own logged current, so the rollers must
   * exist before the sensor can be built — hence one builder shared by all three runtime modes.
   */
  private static Indexer buildIndexer(AngularSubsystem rollers, CANRangeIO canRangeIO) {
    return new Indexer(
        rollers,
        new CANRangeSubsystem(canRangeIO, IndexerConstants.kCANRangeSubsystemConfig),
        new CurrentSensorSubsystem(
            CurrentSensorSubsystemConfig.fromAngularSubsystem(
                rollers,
                IndexerConstants.kJamCurrentThreshold,
                IndexerConstants.kJamDebounce,
                "IndexerJam")));
  }
}
