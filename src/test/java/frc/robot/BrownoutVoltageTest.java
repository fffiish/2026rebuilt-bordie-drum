package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.RobotConstants;
import org.junit.jupiter.api.Test;

class BrownoutVoltageTest {
  @Test
  void robotInitLowersBrownoutToFiveAndAHalfVolts() {
    assertTrue(HAL.initialize(500, 0));
    assertEquals(5.5, RobotConstants.kBrownoutVoltage, 1e-9);

    Robot.configureBrownoutVoltage();

    assertEquals(5.5, RobotController.getBrownoutVoltage(), 1e-9);
  }
}
