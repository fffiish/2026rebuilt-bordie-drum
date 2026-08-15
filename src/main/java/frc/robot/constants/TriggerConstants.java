package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerConstants {
  public static final Trigger kAlwaysTrue = new Trigger(() -> true);
  public static final Trigger kAlwaysFalse = new Trigger(() -> false);
}
