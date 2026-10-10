package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.lib.ShuffleboardNumbers;
import org.junit.jupiter.api.Test;

class DriveShuffleboardTest {
  private static double read(String tab, String title) {
    return NetworkTableInstance.getDefault()
        .getTable("Shuffleboard")
        .getSubTable(tab)
        .getEntry(title)
        .getDouble(Double.NaN);
  }

  @Test
  void targetAndActualSpeedsAppearOnDriveTab() {
    assertTrue(HAL.initialize(500, 0));

    Drive.publishSpeeds(new ChassisSpeeds(3.0, 4.0, Math.PI), new ChassisSpeeds(0.6, 0.8, -1.0));

    assertEquals(5.0, read("Drive", "Target Linear m/s"), 1e-9);
    assertEquals(180.0, read("Drive", "Target Angular deg/s"), 1e-9);
    assertEquals(1.0, read("Drive", "Actual Linear m/s"), 1e-9);
    assertEquals(Math.toDegrees(-1.0), read("Drive", "Actual Angular deg/s"), 1e-9);

    // A second publish (e.g. a second Drive instance) reuses the widgets instead of throwing.
    Drive.publishSpeeds(new ChassisSpeeds(), new ChassisSpeeds());
    assertEquals(0.0, read("Drive", "Target Linear m/s"), 1e-9);
  }

  @Test
  void motorVoltageTitleAppearsOnShooterTab() {
    assertTrue(HAL.initialize(500, 0));

    ShuffleboardNumbers.set("Shooter", "Flywheel 39 V", 6.4);
    ShuffleboardNumbers.set("Shooter", "Flywheel 39 V", 6.7);

    assertEquals(6.7, read("Shooter", "Flywheel 39 V"), 1e-9);
  }
}
