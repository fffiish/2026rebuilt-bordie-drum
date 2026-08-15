package frc.robot.lib.utils;

import static frc.robot.constants.RobotConstants.kMaxAttempts;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

public class PhoenixUtils {
  /** Attempts to run the command 5 times until no error is produced. */
  public static boolean tryUntilOk(Supplier<StatusCode> command) {
    return tryUntilOk(kMaxAttempts, command);
  }

  /** Attempts to run the command until no error is produced. */
  public static boolean tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) return true;
    }
    return false;
  }
}
