package frc.robot.lib.command;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandsUtil {
  public static Command repeatTimes(Command command, int times) {
    return new RepeatCommandWithEnd(command, times);
  }
}
