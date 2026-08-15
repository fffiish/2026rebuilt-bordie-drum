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
import frc.robot.generated.TunerConstants;
import frc.robot.lib.LoggedInterpolatingTableManager;
import frc.robot.lib.alliancecolor.AllianceChecker;
import frc.robot.lib.controller.Joysticks;
import frc.robot.lib.sim.CurrentDrawCalculatorSim;
import frc.robot.subsystems.SuperstructureVisualizer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
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
  private final Vision vision;

  // TODO(template) 1/4: declare your mechanism subsystems here, e.g.
  // private final Intake intake;
  // private final Shooter shooter;

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
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(TunerConstants.FrontLeft),
                  new ModuleIOTalonFX(TunerConstants.FrontRight),
                  new ModuleIOTalonFX(TunerConstants.BackLeft),
                  new ModuleIOTalonFX(TunerConstants.BackRight));
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
                  new VisionIOLimelight(camera1Name, drive::getRotation),
                  new VisionIOLimelight(camera2Name, drive::getRotation),
                  new VisionIOLimelight(camera3Name, drive::getRotation));
        } else {
          vision = new Vision(drive::addVisionMeasurement, aimed, new VisionIO() {});
        }

        // TODO(template) 2/4: construct mechanism subsystems here, e.g.
        // if (Constants.intakeHardwareExists) {
        //   intake =
        //       new Intake(
        //           new AngularSubsystem(
        //               new AngularIOTalonFX(RollerConstants.kTalonFXConfig),
        //               RollerConstants.kSubsystemConfigReal),
        //           ...);
        // } else {
        //   intake = new Intake(); // blank-IO constructor
        // }
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
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose),
                new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, drive::getPose),
                new VisionIOPhotonVisionSim(camera3Name, robotToCamera3, drive::getPose));

        // TODO(template) 3/4: construct mechanism subsystems with *IOSim implementations, e.g.
        // intake =
        //     new Intake(
        //         new AngularSubsystem(
        //             new AngularIOSim(RollerConstants.kSimConfig, currentDrawCalculatorSim),
        //             RollerConstants.kSubsystemConfigSim),
        //         ...);
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

        // TODO(template) 4/4: construct blank-IO mechanism subsystems here so replay is
        // deterministic, e.g. intake = new Intake();
        break;
    }

    superstructure = new RobotSuperstructure();
    superstructure.registerAutoCommands();

    // TODO(template): as subsystems appear, pass their getMeasuredState/getTargetState suppliers
    // into these two visualizers.
    measuredSuperstructureState =
        new SuperstructureVisualizer(
            drive::getPose, "Measured", RobotConstants.kMeasuredStateColor);
    targetSuperstructureState =
        new SuperstructureVisualizer(drive::getPose, "Target", RobotConstants.kTargetStateColor);

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
    - Hold right trigger: turbo (see RobotSuperstructure#getDriveMultiplier)
    - Hold A: dynamically align heading & X position with the trench, you control forward speed
     */
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () ->
                driverController.getLeftStickY()
                    * superstructure.getDriveMultiplier(false, driverController.rightTrigger),
            () ->
                -driverController.getLeftStickX()
                    * superstructure.getDriveMultiplier(false, driverController.rightTrigger),
            () ->
                -driverController.getRightStickX()
                    * superstructure.getDriveMultiplier(true, driverController.rightTrigger)));

    if (!sim) {
      driverController.buttonX.whileTrue(Commands.runOnce(drive::stopWithX, drive));
    }

    driverController.buttonA.whileTrue(
        DriveCommands.joystickDriveThroughTrench(
            drive,
            () ->
                driverController.getLeftStickY()
                    * superstructure.getDriveMultiplier(false, driverController.rightTrigger),
            drive::getPose));

    // TODO(template): bind your mechanism subsystems here.
    //
    // The house style is a state machine per subsystem: each subsystem owns a *State class and
    // exposes `set(State)` as a command factory. Bindings should read as
    //     driverController.leftTrigger.whileTrue(intake.set(IntakeState.kIntaking));
    // rather than poking motors directly. See docs/lib-subsystem.md.
    //
    // `operatorController` is unused until then; controls are conventionally split as
    // driver = drivetrain + intake, operator = scoring mechanisms.
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
