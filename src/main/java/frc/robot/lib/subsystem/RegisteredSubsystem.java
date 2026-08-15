package frc.robot.lib.subsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** Constructs a subsystem with a periodic callback. Defines a real subsystem on the robot. */
public abstract class RegisteredSubsystem implements Subsystem {
  protected RegisteredSubsystem() {
    this.register();
  }
}
