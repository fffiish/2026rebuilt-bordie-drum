package frc.robot.lib.controller;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.lib.RumbleControl;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

/** Joysticks wrapper to provide easier access to buttons, triggers and sticks. */
public class Joysticks {
  public final Trigger leftTrigger;
  public final Trigger rightTrigger;

  public final Trigger rightJoyStickPress;
  public final Trigger leftJoyStickPress;
  public final Trigger leftMidButton;
  public final Trigger rightMidButton;

  public final Trigger rightBumper;
  public final Trigger leftBumper;
  public final Trigger buttonA;
  public final Trigger buttonB;
  public final Trigger buttonY;
  public final Trigger buttonX;
  public final Trigger dPadDown;
  public final Trigger dPadLeft;
  public final Trigger dPadRight;
  public final Trigger dPadUp;

  public final Trigger rightStickMoved;
  public final Trigger leftStickMoved;
  public final Trigger leftStickHeldUp;
  public final Trigger leftStickHeldDown;
  public final Trigger leftStickHeldRight;
  public final Trigger leftStickHeldLeft;
  public final Trigger rightStickHeldUp;
  public final Trigger rightStickHeldDown;
  public final Trigger rightStickHeldRight;
  public final Trigger rightStickHeldLeft;
  public final Trigger povPressed;

  public final RumbleControl rumble = new RumbleControl(this::rumble);

  /** XboxController Object for Controller; contains all Xbox Controller Functions */
  private final CommandXboxController controller;

  private final int port;

  public Joysticks(int port) {
    this.port = port;
    controller =
        !Constants.currentMode.equals(Constants.Mode.SIM)
            ? new CommandXboxController(port)
            : new CommandXboxControllerSim(port);

    leftTrigger = controller.leftTrigger(0.5);
    rightTrigger = controller.rightTrigger(0.5);

    rightJoyStickPress = controller.rightStick();
    leftJoyStickPress = controller.leftStick();
    leftMidButton = controller.back();
    rightMidButton = controller.start();

    rightBumper = controller.rightBumper();
    leftBumper = controller.leftBumper();
    buttonA = controller.a();
    buttonB = controller.b();
    buttonY = controller.y();
    buttonX = controller.x();

    dPadDown = controller.povDown();
    dPadLeft = controller.povLeft();
    dPadRight = controller.povRight();
    dPadUp = controller.povUp();

    rightStickMoved = new Trigger(() -> Math.hypot(getRightStickX(), getRightStickY()) > 0.1);
    leftStickMoved = new Trigger(() -> Math.hypot(getLeftStickX(), getRightStickY()) > 0.1);
    leftStickHeldUp = new Trigger(() -> getLeftStickY() > 0.5);
    leftStickHeldDown = new Trigger(() -> getLeftStickY() < -0.5);
    leftStickHeldRight = new Trigger(() -> getLeftStickX() > 0.5);
    leftStickHeldLeft = new Trigger(() -> getLeftStickX() < -0.5);
    rightStickHeldUp = new Trigger(() -> getRightStickY() > 0.5);
    rightStickHeldDown = new Trigger(() -> getRightStickY() < -0.5);
    rightStickHeldRight = new Trigger(() -> getRightStickX() > 0.5);
    rightStickHeldLeft = new Trigger(() -> getRightStickX() < -0.5);

    povPressed = new Trigger(() -> getPOV() >= 0);
  }

  public Translation2d getLeftStickDirection() {
    return new Translation2d(getLeftStickY(), -getLeftStickX());
  }

  public double getLeftStickX() {
    return controller.getLeftX();
  }

  public double getLeftStickY() {
    return -controller.getLeftY();
  }

  public double getRightStickX() {
    return controller.getRightX();
  }

  public double getRightStickY() {
    return -controller.getRightY();
  }

  public double getRightTriggerValue() {
    return controller.getRightTriggerAxis();
  }

  public double getLeftTriggerValue() {
    return controller.getLeftTriggerAxis();
  }

  public boolean anyStickMoved() {
    return Math.abs(getLeftStickX()) > 0.15
        || Math.abs(getLeftStickY()) > 0.15
        || Math.abs(getRightStickX()) > 0.15;
  }

  public boolean anyStickMovedFast() {
    return Math.abs(getLeftStickX()) > 0.5
        || Math.abs(getLeftStickY()) > 0.5
        || Math.abs(getRightStickX()) > 0.5;
  }

  public boolean anyStickMovedStiff() {
    return Math.abs(getLeftStickX()) > 0.15
        || Math.abs(getLeftStickY()) > 0.15
        || Math.abs(getRightStickX()) > 0.15;
  }

  /**
   * Returns 0.0 if the given value is within the specified range around zero. The remaining range
   * between the deadband and 1.0 is scaled from 0.0 to 1.0.
   *
   * @param value value to clip
   * @param deadband range around zero
   */
  private double applyDeadband(double value, double deadband) {
    if (Math.abs(value) < deadband) {
      return 0.0;
    }

    if (value > 0.0) {
      return (value - deadband) / (1.0 - deadband);
    }
    return (value + deadband) / (1.0 - deadband);
  }

  /**
   * Returns 0.0 if the given value is within the specified range around zero. The remaining range
   * between the deadband and 1.0 is scaled from 0.0 to 1.0.
   *
   * @param value value to clip
   */
  private double applyDeadband(double value) {
    return applyDeadband(value, 0.02);
  }

  public int getPort() {
    return port;
  }

  public void rumble(RumbleType type, double intensity) {
    this.controller.getHID().setRumble(type, intensity);
  }

  public static Function<Double, Double> scale(boolean inverted, DoubleSupplier scaler) {
    return (Double input) -> {
      double x1 = 0.5, y1 = 1;
      double x2 = 1.8, y2 = 0.5;

      double x = scaler.getAsDouble();
      // Interpolate based on elevator height.
      double y = y1 + (y2 - y1) * ((x - x1) / (x2 - x1));

      return input * (inverted ? -1 : 1) * Math.min(y, 1);
    };
  }

  public int getPOV() {
    return controller.getHID().getPOV();
  }

  public CommandXboxController getController() {
    return controller;
  }
}
